#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <iostream>
#include <libconfig.h>

#include "../layer_1/include/layer_1.h"
#include "include/layer_2.h"
#include "../signal_quality/include/signal_quality.h"
#include "../graphs/include/graphs_common.h"
#include <map>
#include <vector>
#include<semaphore.h>
#include<pthread.h>
#include <time.h>


extern "C" {
#include "../../utils/timespec_utils.h"
}
static char L2_buf_tx[2500];
static char L2_buf_rx[2500];

static L2Header * L2_rx_p = (L2Header * ) L2_buf_rx;
static L2Header * L2_tx_p = (L2Header * ) L2_buf_tx;

static int tx_count = 0;
static int num_of_nodes = 0;
static int highest_serial = 0;
static int node_id;
static int promisc = 1, retry_timeout = 100, max_retries = 2;

static PDR * pdr;
static Matrix topology;

std::vector<frame> frames;
std::vector<frame> oob;
std::vector<unsigned int> rx_count;

pthread_mutex_t mtx;
sem_t frame_received, oob_received;

static double max_pdr = 99.0;
static bool only_values = true;

//void L2_tick(unsigned int id){
//    for (int i = 0; i<num_of_nodes; i++){
//        pdr[i].tick(id);
//    }
//}
#include <iostream>



void * receiver(void *){
    char buf[2342];
    WRPPacketInfo pi;
    L2Header * L2h = (L2Header *) buf;

    while (1){
        L1_receive(0,buf,&pi);


        if (L2h->from == node_id){
            continue;
        }
        assert(L2h->from < num_of_nodes);

        /* Forced Topology */
        if (topology.initialized()){
            if (topology(L2h->from, node_id) < 1e-6){
                continue;
            }
            if ((!only_values) && rand()%100 > topology(L2h->from,node_id)){
                continue;
            }
        }

        pdr[L2h->from].newIncomingFrame(L2h->tx_count);

        if (L2h->type == DATA){
            if (L2h->serial <= highest_serial){
                continue;
            }else{
                highest_serial = L2h->serial;
            }
        }


        //        if (L2h->type == DATA){
        //            if ((L2h->tx_count-L2h->retries) <= rx_count[L2h->from]){
        //                fprintf(stderr,"Frame already treated from:%d serial:%d %d vs %d\n", L2h->from, L2h->serial,L2h->tx_count, rx_count[L2h->from]);
        //                continue;
        //            }else{
        //                rx_count[L2h->from] = L2h->tx_count;
        //            }
        //        }


        frame f;
        f.from = L2h->from;
        f.to = L2h->to;
        f.len = L2h->len;
        f.serial = L2h->serial;
        f.tx_count = L2h->tx_count;
        f.type = L2h->type;
        f.retries = L2h->retries;

        memcpy(f.data, buf + sizeof (L2Header), f.len);

        pthread_mutex_lock(&mtx);
        if (L2h->type == DATA){
            frames.push_back(f);
            pthread_mutex_unlock(&mtx);
            sem_post(&frame_received);
        }else{
            oob.push_back(f);
            pthread_mutex_unlock(&mtx);
            sem_post(&oob_received);
        }
    }
}


void L2_set_retry_timeout(int _timeout){
    retry_timeout = _timeout;
}
void L2_set_max_retries(int _retries){
    max_retries = _retries;
}

int L2_send_with_ack(char * buf, unsigned char from, unsigned char to, unsigned int len){

    struct timespec tic,tac;
    int retries = 0;

    L2_tx_p->from = from;
    L2_tx_p->to = to;
    L2_tx_p->len = len;
    L2_tx_p->serial = ++highest_serial;
    L2_tx_p->type = DATA;

    memcpy(L2_buf_tx + sizeof(L2Header), buf, len);

    while (retries <= max_retries){

        L2_tx_p->tx_count = ++ tx_count;
        L2_tx_p->retries = retries ++;
        pdr[L2_tx_p->to].newOutgoingFrame();

        L1_send(L2_buf_tx, len + sizeof(L2Header));

        timespec_now(&tic);
        while (1){

            int timeout = retry_timeout - timespec_elapsed_ms(&tic);
            timespec_future(&tac, timeout);

            int res = sem_timedwait(&frame_received, &tac);
            if (res != 0){
                pdr[L2_tx_p->to].newTimedoutFrame();
                break;
            }

            pthread_mutex_lock(&mtx);
            frame & f = frames.at(0);

            /* REQUIRES ACK FROM NEXT NODE (if commented any network activity will do)*/
            /* uncommented, gives problem with burst */

            //            if (f.from != to){
            //                frames.pop_back();
            //                continue;
            //            }

            if (f.to == from){
                f.piggybacked = 1;
                pthread_mutex_unlock(&mtx);
                sem_post(&frame_received);
            }else{
                frames.pop_back();
                pthread_mutex_unlock(&mtx);
            }

            return FORWARDED;
        }
    }
    return TIMEDOUT;
}


int L2_oob_receive(int timeout, char *buf, int *len, unsigned char *from, unsigned char *to){
    struct timespec tic,tac;
    int orig_to = timeout;
    timespec_now(&tic);

    while (1){

        timeout = orig_to - timespec_elapsed_ms(&tic);
        timespec_future(&tac, timeout);

        int res;
        if (orig_to > 0){
            sem_timedwait(&oob_received, &tac);
        }else{
            sem_wait(&oob_received);
        }

        if (res == 0){
            pthread_mutex_lock(&mtx);
            frame f = oob.at(0);

            //            if (f.to != node_id){
            //                oob.pop_back();
            //                pthread_mutex_unlock(&mtx);
            //                continue;
            //            }

            memcpy(buf, f.data, f.len);
            *from = f.from;
            *to = f.to;
            *len = f.len;
            oob.pop_back();
            pthread_mutex_unlock(&mtx);
            return 0;
        }else{
            return ETIMEDOUT;
        }
    }
}

void L2_oob_queue_clear(){
    pthread_mutex_lock(&mtx);
    for (int i = 0; i < oob.size(); i++){
        sem_wait(&oob_received);
        oob.pop_back();
    }
    pthread_mutex_unlock(&mtx);
}

int L2_oob_get_queue_lenght( ){
    return oob.size();
}



void L2_set_promisc(int _promisc){
    promisc = _promisc;
}

int L2_receive(int timeout, char *buf, int *len, unsigned char *from, unsigned char *to, unsigned char * retries){

    struct timespec tic,tac;
    int orig_to = timeout;
    timespec_now(&tic);
    while (1){

        timeout = orig_to - timespec_elapsed_ms(&tic);
        timespec_future(&tac, timeout);

        int res = sem_timedwait(&frame_received, &tac);
        if (res == 0){
            pthread_mutex_lock(&mtx);
            frame f = frames.at(0);

            if (!promisc && f.to != node_id){
                frames.pop_back();
                pthread_mutex_unlock(&mtx);
                continue;
            }

            memcpy(buf, f.data, f.len);
            *from = f.from;
            *to = f.to;
            *len = f.len;
            *retries = f.retries;
            frames.pop_back();
            pthread_mutex_unlock(&mtx);

            return 0;
        }else{
            return ETIMEDOUT;
        }
    }
}

int L2_setup(int _node_id, int _num_of_nodes, config_setting_t * settings){
    num_of_nodes = _num_of_nodes;
    node_id = _node_id;

    int has_conn_matrix = 0;
    Matrix m(num_of_nodes, num_of_nodes);


    if (settings != NULL) {
        config_setting_lookup_int(settings, "max_retries", &max_retries);
        config_setting_lookup_int(settings, "retry_timeout", &retry_timeout);

        double val;
        char link[32];
        for (int i = 0; i< num_of_nodes; i++){
            for (int j = 0; j< num_of_nodes; j++){
                sprintf(link,"pdr_%d_to_%d", i, j);
                if (config_setting_lookup_float(settings, link, &val)){
                    m(i,j) = val;
                    has_conn_matrix = 1;
                }
                sprintf(link,"pdr_%d_and_%d", i, j);
                if (config_setting_lookup_float(settings, link, &val)){
                    m(i,j) = val;
                    m(j,i) = val;
                    has_conn_matrix = 1;
                }
            }
        }
    }

    printf("L2 Config >> retry_timeout:%d ms, max_retries:%d\n", retry_timeout, max_retries);

    if (has_conn_matrix){
        m.print_as_int("L2 Config >> connectivity matrix");
        topology = m;
    }

    L2_rx_p->tx_count = 0;

    pdr = new PDR[num_of_nodes];
    rx_count.resize(num_of_nodes);

    for (int i = 0; i<num_of_nodes; i++){
        pdr[i].setNodeId(i);
    }

    sem_init(&frame_received, 0,0);
    sem_init(&oob_received, 0,0);
    pthread_mutex_init(&mtx,0);

    pthread_t th;
    pthread_create(&th,0,receiver,0);

    return 0;
}

int L2_set_topology(Matrix &m, bool just_value){
    topology = m;
    only_values = just_value;
}

int L2_set_highest_serial(int serial){
    highest_serial = serial;

}
int L2_inc_highest_serial(int serial){
    highest_serial += serial;

}

int L2_send_out_of_band(char * buf, unsigned char from, unsigned char to, unsigned int len, int serial){
    tx_count++;
    L2_tx_p->from = from;
    L2_tx_p->to = to;
    L2_tx_p->len = len;
    L2_tx_p->tx_count = ++ tx_count;
    L2_tx_p->serial = serial;
    L2_tx_p->type = OUT_OF_BAND;

    memcpy(L2_buf_tx + sizeof(L2Header), buf, len);
    return L1_send(L2_buf_tx, len + sizeof(L2Header));
}

int L2_send(char * buf, unsigned char from, unsigned char to, unsigned int len){
    tx_count++;
    L2_tx_p->from = from;
    L2_tx_p->to = to;
    L2_tx_p->len = len;
    L2_tx_p->tx_count = tx_count;
    highest_serial ++;
    L2_tx_p->serial = highest_serial;
    memcpy(L2_buf_tx + sizeof(L2Header), buf, len);
    return L1_send(L2_buf_tx, len + sizeof(L2Header));
}

double L2_get_max_pdr(){
    return max_pdr;
}
double L2_get_pdr(int id){


    if (topology.initialized()){
        if (pdr[id].get() > 0){
            return topology(id, node_id);
        }else{
            return 0;
        }
    }

    //        if (r > 75){
    //            return  1;
    //        }else if (r > 50){
    //            return  2;
    //        }else if (r > 25){
    //            return  3;
    //        }else if (r > 0){
    //            return  4;
    //        }else{
    //            return  0;
    //        }
    int r =  pdr[id].get();
    return r > max_pdr ? max_pdr : r;
}

void L2_reset_pdr(int id){
    pdr[id].reset();
}
double L2_get_tx_pdr(int id){
    pdr[id].get_tx_pdr();
}
