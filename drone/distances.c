#include "dwm_api.h"
#include "hal.h"

int main(void)
{
    dwm_init();

    dwm_loc_data_t loc;
    dwm_pos_t pos;
    loc.p_pos = &pos;
    while(1) {
        if(dwm_loc_get(&loc) == RV_OK) {
            for (int i = 0; i < loc.anchors.dist.cnt; ++i) {
                //HAL_Print("0x%llx", loc.anchors.dist.addr[i]);
                float fdist = loc.anchors.dist.dist[i]/1000.0;
                HAL_Print("%.3f\n", fdist);
            }
        }
    }

    return 0;   
}
