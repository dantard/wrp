#ifndef L1_INTERFACE_H_
#define L1_INTERFACE_H_

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

#include "types.h"
extern "C"{
int L1_receive(int timeout, char * buf, struct WRPPacketInfo * pi);
int L1_setup(int proto, int use_mon, config_setting_t * settings);
int L1_send(char * buf, unsigned int len);
}

#endif /* L1_INTERFACE_H_ */
