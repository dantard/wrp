/*------------------------------------------------------------------------
 *---------------------               WRP             --------------------
 *------------------------------------------------------------------------
 *
 *  Authors: Danilo Tardioli
 *
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2015, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#include <stdarg.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>     /* the L2 protocols */
#include <asm/types.h>
#include <time.h>
#include <libconfig.h>
#include <netinet/if_ether.h>
#include "radiotap/radiotap.h"
#include "radiotap/radiotap_iter.h"
#include "include/bridge.h"
#include "include/frames.h"
#include "../../utils/timespec_utils.h"
#include "include/types.h"

#define WMP_MSG fprintf
#define MAX_PACKET_LEN 2342

static int tx, use_monitor = 1, L1_sock_rx, protocol, rxb_data_len;
static struct sockaddr_ll broadcast;
static struct L1Header * rxb_head, *txb_head;
static char txb[65535], L1_buf_rx[65536], *txb_data, *rxb_data;
static unsigned char bcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static int sock_raw_init(const char * DEVICE, int protocol, int * sock, int * if_idx, unsigned char src_mac[6]) {
    int s, ifindex, i;
    struct ifreq ifr;

    s = socket(PF_PACKET, SOCK_RAW, htons(protocol));
    if (s == -1) {
        perror("socket():");
        return 1;
    }

    strncpy(ifr.ifr_name, DEVICE, IFNAMSIZ);
    if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
        perror("SIOCGIFINDEX");
        return 1;
    }
    ifindex = ifr.ifr_ifindex;

    if (ioctl(s, SIOCGIFHWADDR, &ifr) == -1) {
        perror("SIOCGIFINDEX");
        return 1;
    }

    for (i = 0; i < 6; i++) {
        src_mac[i] = ifr.ifr_hwaddr.sa_data[i];
    }

    /* bind socket to interface to receive frame ONLY from that interface */
    struct sockaddr_ll sll;
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifindex;
    sll.sll_protocol = htons(protocol);
    if ((bind(s, (struct sockaddr *) &sll, sizeof(sll))) == -1) {
        perror("bind: ");
        return 1;
    }

    (*sock) = s;
    (*if_idx) = ifindex;
    return 0;
}

static void parse_radiotap(struct ieee80211_radiotap_header * buf, struct WRPPacketInfo * pi) {
    //    int pkt_rate_100kHz = 0, antenna = 0, pwr = 0;
    //    char rssi_dbm = 0, noise_dbm = 0;
    struct ieee80211_radiotap_iterator iterator;

    int ret = ieee80211_radiotap_iterator_init(&iterator, buf, buf->it_len, 0);
    // WMP_MSG(stderr, "ret1: %d buf->it_len:%d \n", ret, buf->it_len);
    while (!ret) {
        ret = ieee80211_radiotap_iterator_next(&iterator);       

        if (ret) {
            continue;
        }

        switch (iterator.this_arg_index) {
        case IEEE80211_RADIOTAP_RATE:
            pi->rate = (*iterator.this_arg) * 5;
            break;
        case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
            pi->rssi = (*iterator.this_arg);
            break;
        case IEEE80211_RADIOTAP_CHANNEL:
            pi->channel = (*iterator.this_arg);
            break;
        case IEEE80211_RADIOTAP_ANTENNA:
            pi->antenna = (*iterator.this_arg);
            break;
        case IEEE80211_RADIOTAP_DBM_TX_POWER:
            pi->pwr = *iterator.this_arg;
            break;
        case IEEE80211_RADIOTAP_DB_ANTNOISE:
            pi->noise_dbm = *iterator.this_arg;
            break;
        default:
            break;
        }
        pi->valid = 1;
    }
}

int L1_receive(int timeout, char * buf, struct WRPPacketInfo * pi) {

    int rlen, valid = 1;

    if (timeout == 0) {
        rlen = recvfrom(L1_sock_rx, L1_buf_rx, MAX_PACKET_LEN, 0, 0, 0);

    } else {
        struct timeval tv;
        fd_set fd_rx;
        tv.tv_sec = 0;
        tv.tv_usec = 1000 * timeout;
        FD_ZERO(&fd_rx);
        FD_SET(L1_sock_rx, &fd_rx);
        int r = select(FD_SETSIZE, &fd_rx, NULL, NULL, &tv);
        if (r) {
            rlen = recvfrom(L1_sock_rx, L1_buf_rx, MAX_PACKET_LEN, 0, 0, 0);
        } else {
            return ETIMEDOUT;
        }
    }

    if (use_monitor) {
        struct ieee80211_radiotap_header * rte = (struct ieee80211_radiotap_header *) L1_buf_rx;
        struct ieee80211_frame * eh = (struct ieee80211_frame *) (L1_buf_rx + rte->it_len);
        struct llc * p = (struct llc *) (L1_buf_rx + rte->it_len + sizeof(struct ieee80211_frame));
        rxb_head = (struct L1Header *) (L1_buf_rx + rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc));
        rxb_data = (char *) (L1_buf_rx + rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc) + sizeof(struct L1Header));
        rxb_data_len = rlen - (rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc) + sizeof(struct L1Header));

        if (eh->fcs != 8) {
            //fprintf(stderr, "Discarding (NOT Data frame)...\n");
            valid = 0;
        }

        if (p->proto != protocol && protocol != 0) {
            //fprintf(stderr, "Discarding (NOT expected protocol %x)...\n", protocol);
            valid = 0;
        }

        parse_radiotap(rte, pi);

    } else {
        struct ethhdr * eh = (struct ethhdr *) (L1_buf_rx);
        rxb_head = (struct L1Header *) (L1_buf_rx + ETH_HLEN);
        rxb_data = (char *) (L1_buf_rx + ETH_HLEN + sizeof(struct L1Header));
        rxb_data_len  = rlen - (ETH_HLEN + sizeof(struct L1Header));
        if (eh->h_proto != protocol) {
            valid = 0;
        }
    }

    if (!valid){
        return EINVAL;
    }

    memcpy(buf, rxb_data, rxb_data_len);
    pi->len = rxb_data_len;
    pi->protocol = protocol;
    return 0;
}

int L1_setup(int proto, int use_mon, config_setting_t * settings) {
    char interface[64] = {"lo"};
    char essid[64] = {"robotics"};
    char ip[64] = {0};

    int wait_join = 1, configure_card = 1;

    protocol = proto;
    use_monitor = use_mon;

    int l1_freq = 2462, l1_rate = 6;

    if (settings != NULL) {
        const char * str2;
        if (config_setting_lookup_string(settings, "interface", &str2)){
            sprintf(interface, "%s", str2);
        }
        if (config_setting_lookup_string(settings, "essid", &str2)){
            sprintf(essid, "%s", str2);
        }
        if (config_setting_lookup_string(settings, "ip", &str2)){
            sprintf(ip, "%s", str2);
        }
        config_setting_lookup_int(settings, "freq", &l1_freq);
        config_setting_lookup_int(settings, "rate", &l1_rate);
        config_setting_lookup_int(settings, "wait_joining", &wait_join);
        config_setting_lookup_int(settings, "use_mon", &use_monitor);
        config_setting_lookup_int(settings, "configure_card", &configure_card);
    }

    int is_loopback = strcmp(interface, "lo") == 0;

    if (is_loopback){
        use_monitor = 0;
        printf("L1 Config >> interface:%s\n", interface);
    }else{
        printf("L1 Config >> interface:%s, use_mon:%s, essid:%s, freq:%d MHz, rate:%d Mbps\n", interface, use_monitor?"true":"false", essid, l1_freq, l1_rate);
    }

    int txi, rxi, res = 0;
    unsigned char rx_mac[6], tx_mac[6];


    if (is_loopback){
        res |= sock_raw_init(interface, proto, &L1_sock_rx, &rxi, rx_mac);
    }else{
        int ns;
        if (use_monitor) {
            ns = create_monitor(interface, "mon0");
            if (ns !=0){
                fprintf(stderr,"Unable to create monitor interface (error %d)\n", ns);
                if (ns == -2){
                    fprintf(stderr,"RF KILL?\n");
                }
                exit(1);
            }
            res |= sock_raw_init("mon0", ETH_P_ALL, &L1_sock_rx, &rxi, rx_mac);
        }else{
            res |= sock_raw_init(interface, ETH_P_ALL, &L1_sock_rx, &rxi, rx_mac);
        }

        if (configure_card){
            ns = set_network(interface,essid,l1_freq,l1_rate);
            if (ns!=0){
                fprintf(stderr,"Unable to set network configuration (error %d)\n", ns);
                exit(1);
            }


            if (ip[0]){
                set_ip(interface, ip);
            }

            if (wait_join){
                ns = wait_joining(interface,essid,l1_freq);
                if (ns!=0){
                    fprintf(stderr,"Unable to join network (error %d)\n", ns);
                    exit(1);
                }
            }                        
        }



    }

    res |= sock_raw_init(interface, protocol, &tx, &txi, tx_mac);


    /* TX */
    txb_head = (struct L1Header *) (txb + ETH_HLEN);
    txb_data = (char *) (txb + ETH_HLEN + sizeof(struct L1Header));
    struct ethhdr * eh = (struct ethhdr *) txb;
    eh->h_proto = htons(protocol);
    memcpy((void *) eh->h_dest, (void*) bcast_mac, ETH_ALEN);
    memcpy((void *) eh->h_source, (void*) tx_mac, ETH_ALEN);

    broadcast.sll_family = PF_PACKET;
    broadcast.sll_protocol = htons(protocol);
    broadcast.sll_ifindex = txi;
    broadcast.sll_pkttype = PACKET_BROADCAST;
    broadcast.sll_halen = ETH_ALEN;

    return res;
}

int L1_send(char * buf, unsigned int len) {
    usleep(25000);
    memcpy(txb_data, buf, len);
    int size = len + ETHER_HDR_LEN + sizeof(struct L1Header);
    int res = sendto(tx, txb, size, 0, (struct sockaddr*) &broadcast, sizeof(broadcast));
    //TODOOOOOOOOOOOO: recvfrom(L1_sock_rx, 0, MAX_PACKET_LEN, 0, 0, 0);
    return (res == -1);
}



