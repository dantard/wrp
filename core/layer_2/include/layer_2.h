#ifndef L2_INTERFACE_H_
#define L2_INTERFACE_H_
#include "../../graphs/include/graphs_common.h"
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

#include <libconfig.h>

#define ANY_NODE -1

struct __attribute__((__packed__)) L2Header {
    unsigned char from;
    unsigned char to;
    unsigned int len;
    unsigned int tx_count;
    unsigned char retries;
    unsigned int serial;
    unsigned char type;
};

struct frame{
    char data[2342];
    int len;
    unsigned char from;
    unsigned char to;
    unsigned char piggybacked;
    unsigned int tx_count;
    unsigned int serial;
    unsigned char type;
    unsigned char retries;

};


enum response{NORMAL, FORWARDED, TIMEDOUT, PIGGYBACKED};
enum type{DATA, DROP, OUT_OF_BAND};


int    L2_setup(int node_id, int num_of_nodes, config_setting_t *settings);
int    L2_send_out_of_band(char * buf, unsigned char from, unsigned char to, unsigned int len, int serial);
int    L2_send(char * buf, unsigned char from, unsigned char to, unsigned int len);
double L2_get_pdr(int id);
int    L2_get_fake_quality(int node_id, int dest);
void   L2_reset_pdr(int id);
int    L2_inc_highest_serial(int serial);
int    L2_set_highest_serial(int serial);
int    L2_send_with_ack(char * buf, unsigned char from, unsigned char to, unsigned int len);
int    L2_oob_receive(int timeout, char *buf, int *len, unsigned char *from, unsigned char *to);
void   L2_oob_queue_clear();
int    L2_oob_get_queue_lenght();
int    L2_receive(int timeout, char *buf, int *len, unsigned char *from, unsigned char *to, unsigned char *retries);
double L2_get_tx_pdr(int id);
double L2_get_max_pdr();
void   L2_set_retry_timeout(int _timeout);
void   L2_set_max_retries(int _retries);
int    L2_set_topology(Matrix &m, bool just_value);


#endif /* L2_INTERFACE_H_ */
