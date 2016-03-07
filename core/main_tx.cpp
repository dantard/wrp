#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <unistd.h>
#include <libconfig.h>
#include "layer_1/include/layer_1.h"
#include "layer_1/include/types.h"
#include "../utils/timespec_utils.h"
#include <pthread.h>
#include <string.h>


int main(int argc, char * argv[]) {
    config_t cfg;
    config_init(&cfg);
    config_setting_t * settings_l1 = NULL;

    char filename[256];
    sprintf(filename,"%s/.%s/%s.cfg", getenv("HOME"), "wrp", "wrp");

    /* config file is like:
    
    layer_1: {
        interface = "wlan0";
        use_mon = true;
    }
    
    */

    printf("Reading configuration file %s...", filename);

    if (config_read_file(&cfg, filename)){
        settings_l1 = config_lookup(&cfg, "layer_1");
        printf("ok.\n");

    }else{
        printf("not found.\n");
    }

    char data[2342];
    struct WRPPacketInfo pi;

    /* L1_setup(protocol, use_monitor, settings) */
    L1_setup(0x6868, 1, settings_l1);
    
    while (1) {

        if (argc == 1){
            /* L1_receive(timeout, data, packet_info) */
            int res = L1_receive(10000, data, &pi);
            if (res == 0){
                fprintf(stderr, "Len:%d Rate:%d RSSI: %d Data: %s\n", pi.len, pi.rate, pi.rssi, data);
            }
        }else{
            fprintf(stderr,"Sending %s\n", argv[1]);
            L1_send(argv[1],strlen(argv[1]) + 1);
            usleep(500000);
        }
    }
}
