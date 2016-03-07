#ifndef QUEUE_H_
#define QUEUE_H_

#include <stdio.h>
#include <string>
#include <vector>
#include <semaphore.h>
#include <pthread.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <assert.h>

#include "QElement.h"


struct ByPriority{
    bool operator () (QElement * i,QElement * j) {
        if (i->getPriority()==j->getPriority()){
            return (i->ts<j->ts);
        }
        return (i->getPriority()<j->getPriority());
    }
};


struct ByDeadline{
    bool operator () (QElement * i,QElement * j) {
        unsigned int rem1 = i->getDeadline();
        unsigned int rem2 = j->getDeadline();

        if (i->getPriority()==j->getPriority()){
            if (rem1 == rem2){
                return i->ts > j->ts;
            }else{
                return rem1 > rem2; //<
            }
        }
        return (i->getPriority()<j->getPriority());
    }
};


class Queue{
    std::vector<QElement * > queue;
    sem_t sem;
    pthread_mutex_t mtx;
    std::string name;
    int sort_criteria;
    std::map<int,int> max_for_port;

public:
    enum sort_criteria_t{PRIORITY_AND_AGE, PRIORITY_AND_DEADLINE};

    Queue( std::string name, int sort_criteria){
        sem_init(&sem, 0,0);
        pthread_mutex_init(&mtx,NULL);
        this->name = name;
        this->sort_criteria = sort_criteria;
    }
    Queue(){
        sem_init(&sem, 0,0);
        pthread_mutex_init(&mtx,NULL);
        this->name = "generic";
        this->sort_criteria = 0;
    }
    ~Queue(){
        while(queue.size()>0){
            QElement * e = queue.back();
            queue.pop_back();
            delete e;
        }
    }

    int getSortCriteria(){
        return sort_criteria;
    }

    void setMaxForPort(int port, int max){
        max_for_port[port] = max;
    }

    QElement * getByUid(unsigned int uid){
        pthread_mutex_lock(&mtx);
        QElement * e = NULL;
        for (int i = 0 ; i< queue.size(); i++){
             if (queue[i]->uid == uid){
                 e = queue[i];
                 break;
             }
        }
        pthread_mutex_unlock(&mtx);
        return e;
    }

    void popByUid(unsigned int uid){
        pthread_mutex_lock(&mtx);
        QElement * e = NULL;
        for (int i = 0 ; i< queue.size(); i++){
             if (queue[i]->uid == uid){
                sem_wait(&sem);
                queue.erase(queue.begin() + i);
                break;
             }
        }
        pthread_mutex_unlock(&mtx);
    }



    void sort(int lock){
        if (lock) pthread_mutex_lock(&mtx);
        if (sort_criteria == PRIORITY_AND_AGE){
            ByPriority cmp_prio;
            std::sort (queue.begin(), queue.end(), cmp_prio);
        }else if (sort_criteria == PRIORITY_AND_DEADLINE){
            ByDeadline cmp_dead;
            std::sort (queue.begin(), queue.end(), cmp_dead);
        }else{
            assert(0);
        }
        if (lock) pthread_mutex_unlock(&mtx);
    }

    void sort(){
        sort(1);
    }

    void push(QElement * e){

        pthread_mutex_lock(&mtx);

        e->ts = timespec_timestamp();
        queue.push_back(e);

        sort(0);

        int should_post = true;

        /* Remove from the queue if there are too much of a specific port */
        int max_for_this_port = -1;
        if (max_for_port.find(e->port) != max_for_port.end()){
            max_for_this_port = max_for_port[e->port];
        }

        if (max_for_this_port > 0){
            int begin = -1, count = 0;
            for (int i = 0; i< queue.size(); i++){
                QElement * m = queue.at(i);
                if (m->port == e->port){
                    begin = begin != -1? begin: i;
                    count++;
                }
            }
            if (count > max_for_this_port){
                queue.erase(queue.begin()+begin);
                std::cerr << "Queue erase size:" << queue.size() << std::endl;
                should_post = false;
            }
        }

        pthread_mutex_unlock(&mtx);

        if (should_post){
            sem_post(&sem);
        }
    }

    QElement * head(){

        pthread_mutex_lock(&mtx);
        QElement * e;

        if (queue.size() > 0){
            e = queue.back();
        }else{
            e = NULL;
        }

        pthread_mutex_unlock(&mtx);
        return e;
    }

    QElement * pop(){
        sem_wait(&sem);
        pthread_mutex_lock(&mtx);
        QElement * e = queue.back();
        queue.pop_back();
        pthread_mutex_unlock(&mtx);
        return e;
    }

    QElement * pop(unsigned int timeout){
        struct timespec ts;
        QElement * e = NULL;

        timespec_future(&ts, timeout);
        int res = sem_timedwait(&sem, &ts);
        if (res == 0){
            pthread_mutex_lock(&mtx);
            e = queue.back();
            queue.pop_back();
            pthread_mutex_unlock(&mtx);
        }
        return e;
    }

    void done(QElement * e){
        delete e;
    }

    int size(){
        return queue.size();
    }

    void print(){
        printf("Queue: %s\n", name.c_str());
        pthread_mutex_lock(&mtx);
        QElement * e;
        for (int i = 0; i <queue.size() ; i++ ){
            e = queue.at(queue.size() -i -1);
            printf("Q[%4d]: ts:%llu prio:%3d src:%2d dst:%3d port:%3d dl:%d rem:%u partial:%d %s\n", i, e->ts, e->priority, e->src, e->dst, e->port, e->deadline, e->getDeadline(),e->isPartial(), e->data);
        }
        pthread_mutex_unlock(&mtx);
    }

};




#endif
