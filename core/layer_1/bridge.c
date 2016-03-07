#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>     /* the L2 protocols */
#include <asm/types.h>
#include <time.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <libconfig.h>
#include <unistd.h>

int qmain(int argc, char **argv);

char iw[256];
char ifconfig[256];
char iwconfig[256];
char * mute = "2>/dev/null 1>/dev/null";
char cmd[256];

int locate_commands(void) {
    if(system("ifconfig 2>/dev/null 1>/dev/null") == 0) {
        sprintf(ifconfig, "ifconfig");
    } else if(system("/sbin/ifconfig 2>/dev/null 1>/dev/null") == 0) {
        sprintf(ifconfig, "/sbin/ifconfig");
    } else {
        fprintf(stderr, "Unable to locate 'ifconfig' command");
        return -1;
    }

    if(system("iw 2>/dev/null 1>/dev/null") == 0) {
        sprintf(iw, "iw");
    } else if(system("/sbin/iw 2>/dev/null 1>/dev/null") == 0) {
        sprintf(iw, "/sbin/iw");
    } else {
        fprintf(stderr, "Unable to locate 'iw' command");
        return -2;
    }

    if(system("iwconfig 2>/dev/null 1>/dev/null") == 0) {
        sprintf(iwconfig, "iwconfig");
    } else if(system("/sbin/iwconfig 2>/dev/null 1>/dev/null") == 0) {
        sprintf(iwconfig, "/sbin/iwconfig");
    } else {
        fprintf(stderr, "Unable to locate 'iwconfig' command");
        return -3;
    }
    return 0;
}


int set_iface_state(char * interface, int status) {
    sprintf(cmd, "%s %s %s %s", ifconfig, interface, status ? "up" : "down", mute);
    fprintf(stderr,"Executing %s\n",cmd);
    return system(cmd);
}

int set_iface_mode(char * interface, int status) {
    sprintf(cmd, "%s dev %s set type %s %s", iw, interface, status ? "ibss" : "managed", mute);
    fprintf(stderr,"Executing %s\n",cmd);
    return system(cmd);
}

int set_iface_address(char * interface, char * address) {
    sprintf(cmd, "%s %s %s up %s", ifconfig, interface, address, mute);
    fprintf(stderr,"Executing %s\n",cmd);
    return system(cmd);
}
int set_iface_leave(char * interface) {
    sprintf(cmd, "%s dev %s ibss leave %s", iw, interface, mute);
    fprintf(stderr,"Executing %s\n",cmd);
    return system(cmd);
}

int set_iface_param(char * interface, char * essid, int freq, int rate) {
    sprintf(cmd, "%s dev %s ibss join %s %d fixed-freq 22:34:55:34:21:12 mcast-rate %d %s", iw, interface, essid, freq, rate, "");
    fprintf(stderr,"Executing %s\n",cmd);
    return system(cmd);
}

int delete_interface(char * interface) {
    sprintf(cmd, "%s dev %s del %s", iw, interface, mute);
    fprintf(stderr,"Executing %s\n",cmd);
    return system(cmd);
}

int create_monitor(char * interface, char *name) {
    locate_commands();
    delete_interface(name);
    sprintf(cmd, "%s dev %s interface add %s type monitor %s", iw, interface, name, "");
    fprintf(stderr,"Executing %s\n", cmd);

    if(system(cmd) != 0) {
        return -1;
    }

    int res = set_iface_state(name, 1);

    if(res) {
        return -2;
    }

    return 0;
}

int wait_joining(char * interface, char * name, int freq) {
    sprintf(cmd, "%s dev %s link", iw, interface);
    fprintf(stderr, "Joining network...");
    FILE *fp;
    char result[1024], data[256], cnt = 0, fail = 0;

    while(1) {
        fp = popen(cmd, "r");
        char * len = fgets(result, 1024, fp);

        if(len != 0) {
            if(strstr(result, "Joined")) {
                fgets(result, 1024, fp);

                if(strstr(result, name) == 0) {
                    fail += 1;
                }
                fgets(result, 1024, fp);
                sprintf(data, "%d", freq);

                if(strstr(result, data) == 0) {
                    fail += 2;
                }

                break;
            }
        }

        fprintf(stderr, ".");
        pclose(fp);
        sleep(1);

        if(cnt ++ > 30) {
            return -10;
        }
    }

    if(fail == 0) {
        fprintf(stderr, "done.\n");
    } else if(fail == 1) {
        fprintf(stderr, "joined, but essid is wrong.\n");
    } else if(fail == 2) {
        fprintf(stderr, "joined, but frequency is wrong.\n");
    } else if(fail == 3) {
        fprintf(stderr, "joined, but both frequency and essid are wrong.\n");
    }

    return 0;
}


int set_network(char * interface, char * name, int freq, int rate) {
    int res = locate_commands();

    if(res) {
        return -1;
    }

    res = set_iface_state(interface, 0);

    if(res) {
        return -2;
    }

    res = set_iface_mode(interface, 0);

    if(res) {
        return -3;
    }

    res = set_iface_mode(interface, 1);

    if(res) {
        return -4;
    }

    res = set_iface_state(interface, 1);

    if(res) {
        return -5;
    }

    set_iface_leave(interface);
    res = set_iface_param(interface, name, freq, rate);

    if(res) {
        return -7;
    }

    return 0;
}
int set_ip(char *iface_name, const char *ip_addr) {
    sprintf(cmd, "%s %s %s %s", ifconfig, iface_name, ip_addr, mute);
    fprintf(stderr,"Executing %s\n",cmd);
    return system(cmd);
}

//int set_ip(char *iface_name, const char *ip_addr) {
//    if (!iface_name)
//        return -1;

//    int sockfd;
//    struct ifreq ifr;
//    struct sockaddr_in sin;

//    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//    if (sockfd == -1) {
//        fprintf(stderr, "Setting IP: could not get socket\n");
//        return -1;
//    }
//    memset(&ifr,0,sizeof (struct ifreq));
//    strncpy(ifr.ifr_name, iface_name, IFNAMSIZ);
//    if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) < 0) {
//        fprintf(stderr, "Setting IP: ifdown failed ");
//        perror(ifr.ifr_name);
//        return -1;
//    }

//    if (ip_addr == 0){
//        return 0;
//    }

//#ifdef ifr_flags
//# define IRFFLAGS       ifr_flags
//#else
//# define IRFFLAGS       ifr_flagshigh
//#endif

//    if (!(ifr.IRFFLAGS & IFF_UP)) {
//        //fprintf(stdout, "Setting IP: device is currently down..setting up -- %u\n", ifr.IRFFLAGS);
//        ifr.IRFFLAGS |= IFF_UP;
//        if (ioctl(sockfd, SIOCSIFFLAGS, &ifr) < 0) {
//            fprintf(stderr, "Setting IP: ifup failed ");
//            perror(ifr.ifr_name);
//            return -1;
//        }
//    }

//    if (ip_addr == 1) {
//        return 0;
//    }

//    sin.sin_family = AF_INET;
//    inet_aton(ip_addr, (struct in_addr *) &sin.sin_addr.s_addr);
//    memcpy(&ifr.ifr_addr, &sin, sizeof(struct sockaddr));
//    if (ioctl(sockfd, SIOCSIFADDR, &ifr) < 0) {
//        fprintf(stderr, "Setting IP: cannot set IP address ");
//        perror(ifr.ifr_name);
//        return -1;
//    }
//#undef IRFFLAGS
//    return 0;
//}

//int iw(int num_param, ...) {
//  int i;
//  char * params[num_param + 1];

//  va_list valist;
//  va_start(valist, num_param);
//    params[0] = (char *)malloc(5);
//    sprintf(params[0], "%s", "/sbin/iw");
//  for (i = 1; i <= num_param; i++) {
//        params[i] = (char *) malloc(64);
//        char * arg = va_arg(valist, char*);
//        sprintf(params[i], "%s", arg);
//  }
//  va_end(valist);
//    return qmain(num_param + 1, params);
//}

//int set_network(char * interface, char * name, int freq, int rate) {
//  char f[64], r[64];
//  sprintf(f, "%d", freq);
//  sprintf(r, "%d", rate);


//    fprintf(stderr,"res:%d\n", set_iface_state("hh0", true));
//    exit(0);

//    printf("Setting interface down...\n");
//    if (set_iface_state(interface, false) != 0){
//        return -1;
//    }

//    printf("Setting ad-hoc mode...\n");
//    int res = iw(5, "dev", interface, "set", "type", "managed");
//    if (res!=0){
//        return -2;
//    }

//    res = iw(5, "dev", interface, "set", "type", "ibss");
//    if (res!=0){
//        return -3;
//    }

//    if (set_ip(interface, 1) != 0){
//        return -4;
//    }

////    printf("Leaving ibss...\n");
////    res = iw(4, "dev", interface, "ibss", "leave");
////    if (res != -67){
////        return res;
////    }

//  printf("Joining %s ibss at frequency %d and rate %d Mbps\n", name, freq, rate);
//    res = iw(9, "dev", interface, "ibss", "join", name, f, "fixed-freq", "mcast-rate", r);
//    if (res!=0){
//        return -4;
//    }
//  return res;
//}





