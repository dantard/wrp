#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_tun.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <stdint.h>
#include <arpa/inet.h>

#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <pthread.h>
#include <semaphore.h>

#include "../core/queues/include/queues.h"
#include "../utils/bits.h"

static int tap1, stop = 0, num_of_nodes, node_id;

sem_t sem;

static int tun_alloc_old(char *dev) {
    char tunname[IFNAMSIZ];

    sprintf(tunname, "/dev/%s", dev);
    return open(tunname, O_RDWR);
}

static int tun_alloc(char *dev) {
    struct ifreq    ifr;
    int     fd;
    int     err;

    if ((fd = open("/dev/net/tun", O_RDWR)) < 0)
        return tun_alloc_old(dev);

    memset(&ifr, 0, sizeof(ifr));

    /* Flags: IFF_TUN   - TUN device (no Ethernet headers)
     *        IFF_TAP   - TAP device
     *
     *        IFF_NO_PI - Do not provide packet information
     */
    ifr.ifr_flags = IFF_TAP;
    if (*dev)
        strncpy(ifr.ifr_name, dev, IFNAMSIZ);

    if ((err = ioctl(fd, TUNSETIFF, (void*)&ifr)) < 0) {
        close(fd);
        perror("TUNSETIFF");
        return err;
    }

    strcpy(dev, ifr.ifr_name);
    return fd;
}


//static size_t write2(int fildes, const void *buf, size_t nbyte) {
//    int     ret;
//    size_t  n;

//    n = nbyte;
//    while (n > 0) {
//        ret = write(fildes, buf, nbyte);
//        if (ret < 0)
//            return ret;

//        n -= ret;
//        buf += ret;
//    }
//    return nbyte;
//}


static uint16_t ipcheck(uint16_t *ptr, size_t len) {
    uint32_t    sum;
    uint16_t    answer;

    sum = 0;

    while (len > 1) {
        sum += *ptr++;
        len -= 2;
    }

    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    answer = ~sum;

    return answer;
}

static int set_ip(char *iface_name, const char *ip_addr) {
    if (!iface_name)
        return -1;

    int sockfd;
    struct ifreq ifr;
    struct sockaddr_in sin;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        fprintf(stderr, "Setting IP: could not get socket\n");
        return -1;
    }
    strncpy(ifr.ifr_name, iface_name, IFNAMSIZ);
    if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) < 0) {
        fprintf(stderr, "Setting IP: ifdown failed ");
        perror(ifr.ifr_name);
        return -1;
    }
#ifdef ifr_flags
# define IRFFLAGS       ifr_flags
#else
# define IRFFLAGS       ifr_flagshigh
#endif
    if (!(ifr.IRFFLAGS & IFF_UP)) {
        fprintf(stdout, "Setting IP: device is currently down..setting up -- %u\n", ifr.IRFFLAGS);
        ifr.IRFFLAGS |= IFF_UP;
        if (ioctl(sockfd, SIOCSIFFLAGS, &ifr) < 0) {
            fprintf(stderr, "Setting IP: ifup failed ");
            perror(ifr.ifr_name);
            return -1;
        }
    }

    if (ip_addr == 0) {
        return 0;
    }

    sin.sin_family = AF_INET;
    inet_aton(ip_addr, (struct in_addr *) &sin.sin_addr.s_addr);
    memcpy(&ifr.ifr_addr, &sin, sizeof(struct sockaddr));
    if (ioctl(sockfd, SIOCSIFADDR, &ifr) < 0) {
        fprintf(stderr, "Setting IP: cannot set IP address ");
        perror(ifr.ifr_name);
        return -1;
    }
#undef IRFFLAGS
    return 0;
}

void * receive(void *){
    fd_set fdset;
    int i;
    ssize_t n;
    char  buf[1500];

    while (!stop) {

        FD_ZERO(&fdset);
        FD_SET(tap1, &fdset);

        if (select(tap1+1, &fdset,NULL,NULL,NULL) < 0) {fprintf(stderr,"TUN interface error, continuing...");
            continue;
        }

        if (FD_ISSET(tap1, &fdset)) {
            n = read(tap1, buf, sizeof(buf));
        }

        if (n < 0){
            fprintf(stderr,"TUN interface error, continuing...");
            sleep(1);
        }

        ethhdr * eh = (ethhdr *) (buf + 4);
        iphdr * ip_header = (iphdr *) (buf + 4 + sizeof(ethhdr));

        unsigned char srce_ip_lsb = ntohl(ip_header->saddr) & 0xFF;
        unsigned char dest_ip_lsb = ntohl(ip_header->daddr) & 0xFF;

        fprintf(stderr,"IP:%d %d\n", srce_ip_lsb, dest_ip_lsb);

////      char ipbuf[INET_ADDRSTRLEN];
////      printf("Dest IP address: %s\n", inet_ntop(AF_INET, &ip_header->daddr, ipbuf, sizeof(ipbuf)));
////      printf("Source IP address: %s\n", inet_ntop(AF_INET, &ip_header->saddr, ipbuf, sizeof(ipbuf)));

//        fprintf(stderr,"\n----%d-a---\n", n);
//        for (i = 0; i<n; i++){
//            fprintf(stderr,"%x:%x ", i, buf[i]);
//        }
        //fprintf(stderr,"Pushing? %d.%d %d.%d  size = %d\n",  buf[0x20],  buf[0x21], buf[0x24], (unsigned char) buf[0x25], n);
//        if (srce_ip_lsb == node_id + 1 && dest_ip_lsb > 0 && dest_ip_lsb <= num_of_nodes){
          //if (srce_ip_lsb == node_id + 1){

            unsigned int dest = 1;
            //SET_BIT(dest, dest_ip_lsb - 1);

            if (node_id == 0) dest = 2;
            if (node_id == 1) dest = 1;


//            fprintf(stderr,"PUSHING in QUEUE size: %d\n", n);
            queue_tx_push(&buf[0],n,0,0,node_id, dest);

//            buf[0x26] = 0;
//            int a = buf[0x21];
//            buf[0x21] = buf[0x25];
//            buf[0x25] = a;
//            write(tap1,buf,n);

//                    fprintf(stderr,"\n----%d-a---\n", n);
//                    for (i = 0; i<n; i++){
//                        fprintf(stderr,"%x:%x ", i, buf[i]);
//                    }

       // }

        //sem_post(&sem);
        //push in the queue
    }

    close(tap1);
}

void * transmit(void *){
    while (!stop){
//        sem_wait(&sem);

        //read from queue
        QElement * e = queue_rx_pop(0);

//        buf[0x26] = 0;
//        buf[0x21] = 0x17;
//        buf[0x25] = 0x16;
//        //write(tap1,buf,n);
        char * buf = e->data;

//        buf[0x20] = node_id + 1;
//        buf[0x24] = node_id + 1;
//        buf[0x25] = node_id + 1;

//        buf[0x4] = 0x16;
//        buf[0x5] = 0x73;
//        buf[0x6] = 0x0d;
//        buf[0x7] = 0xdc;
//        buf[0x8] = 0x50;
//        buf[0x9] = 0x03;

//            int i;
//                fprintf(stderr,"\n----%d-a---\n", e->size);
//                for (i = 0; i<e->size; i++){
//                    fprintf(stderr,"%x:%x ", i, buf[i]);
//                }

        fprintf(stderr,"RECEIVED & WRITING %d.%d %d.%d  size = %d\n",  buf[0x20],  buf[0x21], buf[0x24],  buf[0x25], e->size);
     //   write()
        int n = write(tap1, e->data, e->size);

//        char hh[1500];

//        int n1 = read(tap1, hh, 1500);
//        fprintf(stderr,"WRITTEN %d READ:%d\n", n, n1);


//        ethhdr * eh = (ethhdr *) (hh + 4);
//        iphdr * ip_header = (iphdr *) (hh + 4 + sizeof(ethhdr));

//        unsigned char srce_ip_lsb = ntohl(ip_header->saddr) & 0xFF;
//        unsigned char dest_ip_lsb = ntohl(ip_header->daddr) & 0xFF;

//        fprintf(stderr,"IP RESP:%d %d\n", srce_ip_lsb, dest_ip_lsb);

//        if (srce_ip_lsb == node_id + 1 && dest_ip_lsb > 0 && dest_ip_lsb <= num_of_nodes){

//            unsigned int dest = 0;
//            SET_BIT(dest, dest_ip_lsb - 1);

//            fprintf(stderr,"PUSHING RESP in QUEUE size: %d\n", n);
//            queue_push(&hh[0],n1,0,0,node_id, dest, 0);

//        }



    }

}


int tun_init(char * iface, int _node_id, int _num_of_nodes){

    char tunname[IFNAMSIZ], ip[16];
    sprintf(tunname, "%s%d", iface,_node_id);

    if ((tap1 = tun_alloc(tunname)) < 0) {
        fprintf(stderr,"UNABLE to set TUN interface.");
        return EINVAL;
    }

    sprintf(ip, "192.168.44.%d",_node_id+1);
    set_ip(tunname,ip);

    pthread_t th_rx, th_tx;
    pthread_create(&th_rx,0,receive,0);
    pthread_create(&th_tx,0,transmit,0);

    num_of_nodes = _num_of_nodes;
    node_id = _node_id;
    return 0;
}

//int main(){
//    tun_init("tap0", 5);
//    while (1){
//        sleep(1);
//    }
//    sem_init(&sem,0,0);
//}

