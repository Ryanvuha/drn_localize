// gcc -o spi spi.c -l bcm2835
// sudo ./spi
//

#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PIN RPI_V2_GPIO_P1_24

#define RADIUS 100
#define N_DEVS 10
#define MAX_CHORDS 6

int main(int argc, char **argv)
{
    // If you call this, it will not actually access the GPIO
    // Use for testing
    // bcm2835_set_debug(1);

    if (!bcm2835_init()) {
        printf("bcm2835_init failed. Are you running as root??\n");
        return 1;
    }

    if (!bcm2835_spi_begin()) {
        printf("bcm2835_spi_begin failed. Are you running as root??\n");
        return 1;
    }

    bcm2835_spi_begin();

    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(PIN, LOW);

    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

    if (argc == 4) {
        float beaconx = atof(argv[1]);
        float beacony = atof(argv[2]);
        int scan = atoi(argv[3]);

        int sgnx = (beaconx >= 0 ? 0 : 1);
        beaconx = fabs(beaconx);
        int intx = (int) beaconx;
        int decx = (beaconx - intx) * 100;

        int sgny = (beacony >= 0 ? 0 : 1);
        beacony = fabs(beacony);
        int inty = (int) beacony;
        int decy = (beacony - inty) * 100;

        char buf[] = {
            0xaa,
            (char) sgnx, (char) intx, (char) decx,
            (char) sgny, (char) inty, (char) decy,
            (char) scan
        };
        bcm2835_spi_transfern(buf, sizeof(buf));
    } else if (strcmp(argv[1], "gw") == 0) {
        char buf[] = { 0xcc, 0x00, 0x00, 0x00, 0x00 };
        bcm2835_spi_transfern(buf, sizeof(buf));
    } else if (strcmp(argv[1], "cl") == 0) {
        char buf[61];
        buf[0] = 0xdd;
        bcm2835_spi_transfern(buf, sizeof(buf));
        for (int i = 0; i < N_DEVS; i++) {
            int sgnx = (int) buf[6*i];
            int intx = (int) buf[6*i+1];
            int decx = (int) buf[6*i+2];

            int sgny = (int) buf[6*i+3];
            int inty = (int) buf[6*i+4];
            int decy = (int) buf[6*i+5];

            float posx = intx + decx / 100.;
            posx = (sgnx == 0 ? posx : -posx);
            float posy = inty + decy / 100.;
            posy = (sgny == 0 ? posy : -posy);

            if (abs(posx) < 0.01 && abs(posy) < 0.01) {
                printf("ID=%d Estimated=No data\n", i);
            } else {
                printf("ID=%d Estimated=(%.2f, %.2f)\n", i, posx, posy);
            }
        }
    } else if (strcmp(argv[1], "t1") == 0) {
        char buf[2*RADIUS];
        bcm2835_spi_transfern(buf, sizeof(buf));
        for (int i = 0; i < 2*RADIUS; i++) {
            printf("%d ", (int) buf[i]);
        }
        printf("\n");
    } else if (strcmp(argv[1], "t2") == 0) {
        char buf[2*MAX_CHORDS*6];
        bcm2835_spi_transfern(buf, sizeof(buf));

        for (int j = 0; j < 2*MAX_CHORDS; j++) {
            int sgnx = (int) buf[j*6+0];
            int intx = (int) buf[j*6+1];
            int decx = (int) buf[j*6+2];
            int sgny = (int) buf[j*6+3];
            int inty = (int) buf[j*6+4];
            int decy = (int) buf[j*6+5];
            float posx = intx + decx / 100.;
            posx = (sgnx == 0 ? posx : -posx);
            float posy = inty + decy / 100.;
            posy = (sgny == 0 ? posy : -posy);

            printf(" (%.2f, %.2f) ", posx, posy);
            if (j % 2 == 1) {
                printf("\n");
            }
        }

        printf("\n");
    }

    bcm2835_gpio_write(PIN, HIGH);
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
