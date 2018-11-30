#include "deca_device_api.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_spis.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "port_platform.h"
#include "sdk_config.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

//-----------------dw1000----------------------------
/*DW1000 config function*/
static dwt_config_t config = {
    5,		     /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,	     /* Preamble acquisition chunk size. Used in RX only. */
    10,		     /* TX preamble code. Used in TX only. */
    10,		     /* RX preamble code. Used in RX only. */
    0,		     /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

#define PRECISION 100

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 3 below. */
#define RX_RESP_TO_UUS 5000

/* Buffer to store received frame. See NOTE 4 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

#define SPIS_INSTANCE 2							 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE); /**< SPIS instance. */

static uint8_t m_tx_buf[FRAME_LEN_MAX];
static uint8_t m_rx_buf[FRAME_LEN_MAX];
static const uint8_t m_length = sizeof(m_tx_buf);

static uint8_t beacon[FRAME_LEN_MAX];
static uint8_t estimated[FRAME_LEN_MAX];

// The minimum chord length expressed as meters
float threshold = 5.0;

#define N_DEVS 10

static uint8_t m_tx_est_buf[N_DEVS*6];

uint32 device_map[] = {
    0xc3f58103, // 00
    0xc4351729, // 01
    0xc3f50d8a, // 02
    0xc435021b, // 03
    0xc3f50f85, // 04
    0xc3f44b92, // 05
    0xc3f40789, // 06
    0xc3f55783, // 07
    0xc435028e, // 08
    0xc440912c, // 09
};

static volatile bool spis_xfer_done; /**< Flag used to indicate that SPIS instance completed the transfer. */

/**
 * @brief SPIS user event handler.
 */
void spis_event_handler(nrf_drv_spis_event_t event) {
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE) {
        spis_xfer_done = true;

        beacon[0] = '0xc5';      // multipurpose frame

        beacon[1] = m_rx_buf[0]; // 0xaa (TX -> RX), 0xcc (TX -> GW)

        beacon[2] = m_rx_buf[1]; // sgnx
        beacon[3] = m_rx_buf[2]; // intx
        beacon[4] = m_rx_buf[3]; // decx

        beacon[5] = m_rx_buf[4]; // sgny
        beacon[6] = m_rx_buf[5]; // inty
        beacon[7] = m_rx_buf[6]; // decy
    }
}

int main(void) {

    /* Reset DW1000 */
    reset_DW1000();

    /* Set SPI clock to 2MHz */
    port_set_dw1000_slowrate();

    /* Init the DW1000 */
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        //Init of DW1000 Failed
        while (1) {
        };
    }

    // Set SPI to 8MHz clock
    //port_set_dw1000_fastrate();

    /* Configure DW1000. */
    dwt_configure(&config);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

    /* Set delay to turn reception on after transmission of the frame. See NOTE 2 below. */
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

    /* Set response frame timeout. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);

    LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);
    LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);

    int sgnx, intx, decx, sgny, inty, decy;
    float posx, posy;
    int timeout = 0;

    /*
     * 0: TX (1 device)
     * 1: RX (n_devs devices, from 00 to ...)
     */
    int device = 0;

    if (device == 0) {
        LEDS_ON(BSP_LED_0_MASK);

        // Enable the constant latency sub power mode to minimize the time it takes
        // for the SPIS peripheral to become active after the CSN line is asserted
        // (when the CPU is in sleep mode).
        NRF_POWER->TASKS_CONSTLAT = 1;

        // bsp_board_leds_init();

        nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
        spis_config.csn_pin = 3;  // GPIO8/SPI_CE0 on RPi
        spis_config.miso_pin = 7; // GPIO9/SPI_MISO on RPi
        spis_config.mosi_pin = 6; // GPIO10/SPI_MOSI on RPi
        spis_config.sck_pin = 4;  // GPIO11/SPI_SCLK on RPi
        spis_config.mode = NRF_DRV_SPIS_MODE_1;

        nrf_drv_spis_init(&spis, &spis_config, spis_event_handler);

        // TX phase
        SEGGER_RTT_printf(0, "TX Starting\n");
        bool tx = true;

        while (tx) {
            memset(m_rx_buf, 0, m_length);
            spis_xfer_done = false;

            nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length);

            if (beacon[1] == 0xcc) {
                tx = false; // TX becomes GW and sends 0xcc many times (1000)

                for (int i = 0; i < 1000; i++) {
                    dwt_writetxdata(sizeof(beacon), beacon, 0);
                    dwt_writetxfctrl(sizeof(beacon), 0, 0);

                    dwt_starttx(DWT_START_TX_IMMEDIATE);

                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                }

                break;
            }

            while (!spis_xfer_done && tx) {
                dwt_writetxdata(sizeof(beacon), beacon, 0);
                dwt_writetxfctrl(sizeof(beacon), 0, 0);

                dwt_starttx(DWT_START_TX_IMMEDIATE);

                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                __WFE();
            }

            bsp_board_led_invert(BSP_BOARD_LED_0);
        }

        // GW phase
        SEGGER_RTT_printf(0, "GW Starting\n");
        bool gw = true;

        LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);
        LEDS_ON(BSP_LED_1_MASK);

        float est_pos[N_DEVS][2];
        bool est_pos_bool[N_DEVS];
        for (int i = 0; i < N_DEVS; i++) {
            est_pos_bool[i] = false;
            est_pos[i][0] = 0.0;
            est_pos[i][1] = 0.0;
        }

        while (gw && timeout < 1000) {
            for (int i = 0; i < FRAME_LEN_MAX; i++) {
                rx_buffer[i] = 0;
            }

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

            if (status_reg & SYS_STATUS_RXFCG) {
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= FRAME_LEN_MAX) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }

                if (rx_buffer[1] == 0xbb) {
                    // RX -> GW

                    timeout++;

                    sgnx = (int) rx_buffer[2];
                    intx = (int) rx_buffer[3];
                    decx = (int) rx_buffer[4];

                    sgny = (int) rx_buffer[5];
                    inty = (int) rx_buffer[6];
                    decy = (int) rx_buffer[7];

                    posx = intx + decx / 100.;
                    posx = (sgnx == 0 ? posx : -posx);
                    posy = inty + decy / 100.;
                    posy = (sgny == 0 ? posy : -posy);

                    int id = (int) rx_buffer[8];

                    est_pos[id][0] = posx;
                    est_pos[id][1] = posy;

                    est_pos_bool[id] = true;

                    // check when est_pos_bool[i] is full true
                    bool sum = true;
                    for (int i = 0; i < N_DEVS; i++) {
                        sum = (sum && est_pos_bool[i]);
                    }
                    gw = !sum;
                }

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            }
        }

        // GW -> RPi (collecting phase)
        SEGGER_RTT_printf(0, "Collecting Starting\n");
        LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);
        LEDS_ON(BSP_LED_2_MASK);

        for (int i = 0; i < N_DEVS; i++) {
            float est_x = est_pos[i][0];
            float est_y = est_pos[i][1];

            sgnx = (est_x >= 0 ? 0 : 1);
            est_x = fabs(est_x);
            intx = (int) est_x;
            decx = (est_x - intx) * 100;

            sgny = (est_y >= 0 ? 0 : 1);
            est_y = fabs(est_y);
            inty = (int) est_y;
            decy = (est_y - inty) * 100;

            m_tx_est_buf[6*i] = (char) sgnx;
            m_tx_est_buf[6*i+1] = (char) intx;
            m_tx_est_buf[6*i+2] = (char) decx;

            m_tx_est_buf[6*i+3] = (char) sgny;
            m_tx_est_buf[6*i+4] = (char) inty;
            m_tx_est_buf[6*i+5] = (char) decy;

            SEGGER_RTT_printf(0, "ID %d: (%d) %d %d, (%d) %d %d\n", i, sgnx, intx, decx, sgny, inty, decy);
        }

        // Send data to RPi through SPI
        nrf_drv_spis_buffers_set(&spis, m_tx_est_buf, sizeof(m_tx_est_buf), m_rx_buf, m_length);

    } else if (device == 1) {
        // Get device's ID
        uint32 _id;
        dwt_otpread(0x006, &_id, 1);

        int device_id = -1;
        for (int i = 0; i < N_DEVS; i++) {
            if (device_map[i] == _id) {
                device_id = i;
            }
        }

        bool first = true;
        int n_chords = 0;
        float chords[3][2];
        bool rx = true;

        while (rx) {

            for (int i = 0; i < FRAME_LEN_MAX; i++) {
                rx_buffer[i] = 0;
            }

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

            if (status_reg & SYS_STATUS_RXFCG) {
                // Reset timeout
                timeout = 0;

                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= FRAME_LEN_MAX) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }

                if (rx_buffer[1] == 0xaa) {
                    // TX -> RX
                    sgnx = (int) rx_buffer[2];
                    intx = (int) rx_buffer[3];
                    decx = (int) rx_buffer[4];

                    sgny = (int) rx_buffer[5];
                    inty = (int) rx_buffer[6];
                    decy = (int) rx_buffer[7];

                    posx = intx + decx / 100.;
                    posx = (sgnx == 0 ? posx : -posx);
                    posy = inty + decy / 100.;
                    posy = (sgny == 0 ? posy : -posy);

                    if (first && n_chords < 2) {
                        if (n_chords == 0) {
                            chords[0][0] = posx;
                            chords[0][1] = posy;
                            LEDS_ON(BSP_LED_0_MASK);
                        } else {
                            chords[2][0] = posx;
                            chords[2][1] = posy;
                            LEDS_ON(BSP_LED_1_MASK);
                        }
                        first = false;
                        SEGGER_RTT_printf(0, "(%d) %d %d, (%d) %d %d\n", sgnx, intx, decx, sgny, inty, decy);
                    } else {
                        // SEGGER_RTT_printf(0, "-> (%d.%d, %d.%d)\n", intx, decx, inty, decy);
                    }
                } else if (rx_buffer[1] == 0xcc) {
                    // Exit
                    rx = false;
                }

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            } else if (status_reg & SYS_STATUS_ALL_RX_TO) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);

                timeout++;

                if (timeout > 1000 && !first && n_chords == 0) {
                    // Check chord length
                    float length = sqrt(pow(posx-chords[0][0], 2) + pow(posy-chords[0][1], 2));
                    if (length >= threshold) {
                        chords[1][0] = posx;
                        chords[1][1] = posy;

                        SEGGER_RTT_printf(0, "(%d) %d %d, (%d) %d %d\n", sgnx, intx, decx, sgny, inty, decy);
                        first = true;
                        n_chords++;
                        LEDS_ON(BSP_LED_2_MASK);

                        // Reset timeout
                        timeout = 0;
                    } else {
                        SEGGER_RTT_printf(0, "Too much short\n");
                    }
                }

                //SEGGER_RTT_printf(0, "Timeout: %d\n", timeout);
            } else {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            }
        }

        // RX computes its position
        LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);
        LEDS_ON(BSP_LED_1_MASK);

        float xi = chords[0][0];
        float yi = chords[0][1];
        float xj = chords[1][0];
        float yj = chords[1][1];
        float xk = chords[2][0];
        float yk = chords[2][1];

        float aij = xj - xi;
        float bij = yj - yi;
        float cij = (xj - xi)*((xi + xj)/2) + (yj - yi)*((yi + yj)/2);

        float ajk = xk - xj;
        float bjk = yk - yj;
        float cjk = (xk - xj)*((xj + xk)/2) + (yk - yj)*((yj + yk)/2);

        float est_x = fabs(cij*bjk - cjk*bij) / fabs(aij*bjk - ajk*bij);
        float est_y = fabs(aij*cjk - ajk*cij) / fabs(aij*bjk - ajk*bij);

        sgnx = (est_x >= 0 ? 0 : 1);
        est_x = fabs(est_x);
        intx = (int) est_x;
        decx = (est_x - intx) * 100;

        sgny = (est_y >= 0 ? 0 : 1);
        est_y = fabs(est_y);
        inty = (int) est_y;
        decy = (est_y - inty) * 100;

        estimated[0] = '0xc5';  // multipurpose frame

        estimated[1] = 0xbb; // 0xbb RX -> GW

        estimated[2] = (char) sgnx; // sgnx
        estimated[3] = (char) intx; // intx
        estimated[4] = (char) decx; // decx

        estimated[5] = (char) sgny; // sgny
        estimated[6] = (char) inty; // inty
        estimated[7] = (char) decy; // decy

        estimated[8] = (char) device_id; // id

        while (1) {
            dwt_writetxdata(sizeof(estimated), estimated, 0);
            dwt_writetxfctrl(sizeof(estimated), 0, 0);

            dwt_starttx(DWT_START_TX_IMMEDIATE);

            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            __WFE();
        }
    } else {

    }
}