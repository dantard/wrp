
#ifndef BRIDGE_H
#define BRIDGE_H

//int iw(int num_param, ...);
int set_network(char * interface, char * name, int freq, int rate);
//int delete_interface(char * interface);
//int create_monitor(const char * interface, char *name);
//int set_ip(char *iface_name, const char *ip_addr);
//int set_iface_down(char * interface);
int locate_commands(void);
int set_iface_state(char * interface, int status);
int set_iface_mode(char * interface, int status);
int set_iface_address(char * interface, char * address);
int set_iface_param(char * interface, char * essid, int freq, int rate);
int delete_interface(char * interface);
int create_monitor(char * interface, char *name);
int wait_joining(char * interface, char * name, int freq);
int set_ip(char *iface_name, const char *ip_addr);

#endif
