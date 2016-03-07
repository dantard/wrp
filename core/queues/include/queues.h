#ifndef QUEUES_H_
#define QUEUES_H_

#include <vector>
#include <sstream>
#include <map>

#include "Queue.h"
#include "QElement.h"

/* QUEUES */

void         queue_init(int ports, int sort_criteria);
void         queue_finish();
int          queue_get_incomplete_messages_count();

Queue *      queue_tx_get();
QElement *   queue_tx_head();
QElement *   queue_tx_pop();
QElement *   queue_tx_get_by_id(unsigned int uid);
int          queue_tx_push(char * data, unsigned int size, unsigned char priority, unsigned char port, unsigned char src, unsigned char dst, unsigned int deadline = 0);
int          queue_tx_get_length();
int          queue_tx_push(QElement * e);
void         queue_tx_pop_by_id(unsigned int uid);


QElement *   queue_rx_pop(int port, int timeout);
QElement *   queue_rx_pop(int port);
int          queue_rx_get_current_size(int port);
int          queue_rx_insert_part(u32 uid, int part, int length, char * data, u32 size, u8 priority, u8 port, u8 src, u8 dst, unsigned short deadline);
void         queue_rx_pop_done(int port, QElement * e);


/* FLOWS */
int          flow_set_period(int port, int period);
int          flow_set_priority(int port, int priority);
int          flow_get_time_to_next(int more_priority_than);
int          flow_add(unsigned int port, unsigned int period, unsigned short priority, unsigned int deadline);
int          flow_push(unsigned int flow_id, char * data, unsigned int size, unsigned char src, unsigned int dst);

#endif
