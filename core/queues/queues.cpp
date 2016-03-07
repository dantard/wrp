#include <vector>
#include <sstream>
#include <map>
#include <climits>

#include "include/Queue.h"
#include "include/QElement.h"


struct Flow{
    unsigned int period;
    unsigned long long last_push;
    unsigned short priority;
    unsigned int deadline;
    unsigned char port;
    unsigned int max_messages_in_queue;

    Flow(unsigned int period, unsigned short priority, unsigned char port, unsigned int deadline){
        this->period = period;
        this->priority = priority;
        this->last_push = 0;
        this->deadline = deadline;
        this->port = port;
    }
};

static std::vector<Flow> flows;
static std::vector<Queue * > rx_queues;
static Queue* tx_queue;
static std::map<u32, QElement * > hm;


int flow_add(unsigned int port, unsigned int period, unsigned short priority, unsigned int deadline){
    if (port < flows.size()){
        Flow flw(period, priority, port, deadline);
        flows[port] = flw;
        return 0;
    }else{
        return -1;
    }
}


int flow_set_period(int port, int period){
    flows[port].period = period;
    return period;
}

int flow_set_priority(int port, int priority){
    flows[port].priority = priority;
    return priority;
}

int flow_push(unsigned int flow_id, char * data, unsigned int size, unsigned char src, unsigned int dst){
    if (flows.size() <= flow_id){
        fprintf(stderr,"*** WARNING: FLOW %d not defined, discarding push\n", flow_id);
        return -1;
    }   
    flows[flow_id].last_push = timespec_timestamp();
    QElement * e = new QElement(data,size,flows[flow_id].priority,flows[flow_id].port, src, dst, flows[flow_id].deadline);
    tx_queue->push(e);
    return 0;
}


unsigned short flow_get_time_to_next(int more_priority_than){
    int time_remaining = UINT16_MAX;
    for (int i = 0; i< flows.size(); i++){

        if (flows[i].period > 0 && flows[i].last_push > 0 && flows[i].priority > more_priority_than){
            long long remaining = (long long) (flows[i].period*1000 - (timespec_timestamp() - flows[i].last_push));
            remaining = remaining > 0 ? remaining/1000 : 0;

            if (remaining < time_remaining){
                time_remaining = remaining;
            }
        }
    }
    return time_remaining;
}


void queue_init(int ports, int sort_criteria){
    tx_queue = new Queue("TX Queue", sort_criteria);
    for (int i = 0; i< ports; i++){
        std::ostringstream oss;
        oss << "RX Queue for port " << i << std::endl;
        rx_queues.push_back(new Queue(oss.str(), sort_criteria));
        Flow f(0,0,i,5000);
        flows.push_back(f);
    }
}

QElement * queue_tx_head(){
    return tx_queue->head();
}

QElement * queue_pop(int port){
    return rx_queues[port]->pop();
}

QElement * queue_tx_pop(){
    return tx_queue->pop();
}

QElement * queue_tx_get_by_id(unsigned int uid){
    return tx_queue->getByUid(uid);
}

void queue_tx_pop_by_id(unsigned int uid){
    tx_queue->popByUid(uid);
}

int queue_tx_get_length(){
    return tx_queue->size();
}

int queue_tx_push(QElement * e){
    tx_queue->push(e);
}


void queue_tx_set_max_for_port(int port, int max){
    tx_queue->setMaxForPort(port,max);
}

Queue * queue_tx_get(){
    return tx_queue;
}

QElement * queue_push(char * data, unsigned int size, unsigned char priority, unsigned char port, unsigned char src, unsigned char dst, unsigned int deadline){
    QElement * e = new QElement(data,size,priority,port, src, dst, deadline);
    queue_tx_push(e);
    return e;
}

void queue_rx_pop_done(int port, QElement * e){
    rx_queues[port]->done(e);
}

void queue_finish(){
    delete tx_queue;
    for (int i = 0; i< rx_queues.size(); i++){
        delete rx_queues[i];
    }
}

QElement * queue_rx_pop(int port){
    return rx_queues[port]->pop();
}

QElement * queue_rx_pop(int port, int timeout){
    return rx_queues[port]->pop(timeout);
}

int queue_rx_get_current_size(int port){
    return rx_queues[port]->size();
}

int queue_rx_insert_part(u32 uid, int part, int plen, char * data, u32 mlen, u8 priority, u8 port, u8 src, u8 dst, unsigned short deadline){
    if (hm.size() > 0){
        for (auto iter = hm.cbegin(); iter != hm.cend(); ) {
            QElement * e = iter->second;
            if (e!=NULL && e->getDeadline() == 0){
                std::cerr << "*** WARNING: Removing incomplete message " << iter->second->uid << " port:" << (int) port <<", priority:"<< (int)priority<<" (still " << hm.size() << " incomplete)" << std::endl;
                hm.erase(iter);
            }
            iter++;
        }
    }
    std::map<u32,QElement * >::iterator it = hm.find(uid);
    QElement * e;
    if(it == hm.end()){
        e = new QElement(mlen,  priority,  port,  src,  dst);
        hm[uid] = e;
    }else{
        e = it->second;
    }

    e->deadline = deadline;
    e->ts = timespec_timestamp();
    e->putPart(part, data, plen);

    if (e->isDone()){
        rx_queues[port]->push(e);
        hm.erase(uid);
        return 1;
    }

    return 0;
}

int queue_get_incomplete_messages_count(){
    return hm.size();
}
