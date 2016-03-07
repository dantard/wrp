#ifndef ELEMENT_H_
#define ELEMENT_H_

typedef unsigned char u8;
typedef unsigned int u32;
typedef unsigned short u16;

typedef  char s8;

#include <stdio.h>
#include <string.h>
extern "C"{
#include "../../../utils/timespec_utils.h"
}
#include <math.h>

#define PART_LENGTH 15

class QElement{
public:

    u32 size;
    u8 priority;
    u8 port;
    u32 uid;
    u8 src;
    u32 dst;
    u32 deadline;
    s8 * data;
    unsigned long long ts;
    u16 nparts;
    s8 * parts;
    unsigned char best_effort;
    unsigned char burst;

    QElement(char * data, u32 size,
             u8 priority,
             u8 port,
             u8 src,
             u32 dst, unsigned short deadline){
        this->size = size;
        this->priority = priority;
        this->port = port;
        this->src = src;
        this->dst = dst;
        this->data = new char[size];
        this->burst = 0;
        this->nparts = ceil(double(size) / double(PART_LENGTH));
        this->parts = new char[nparts];
        memset(this->parts, 0, nparts);

        memcpy(this->data, data, size);
        this->uid = timespec_timestamp();
        this->deadline = deadline;
    }

    QElement(u32 size,
             u8 priority,
             u8 port,
             u8 src,
             u32 dst){
        this->size = size;
        this->priority = priority;
        this->port = port;
        this->src = src;
        this->dst = dst;
        this->data = new char[size];
        this->burst = 0;
        this->nparts = ceil(double(size) / double(PART_LENGTH));
        this->parts = new char[nparts];
        memset(this->parts, 0, nparts);
        this->uid = timespec_timestamp();
    }

    void setBestEffort(){
        best_effort = 1;
    }

    void setBurst(){
        burst = 1;
        best_effort = 1;
    }

    void setDontFragment(){
        burst = 2;
    }

    int isDone(){
        int sum = 0;
        for (int i = 0; i< nparts; i++){
            sum+=(parts[i]>0);
        }
        return sum==nparts;
    }

    int isPartial(){
        int sum = 0;
        for (int i = 0; i< nparts; i++){
            sum+=(parts[i]>0);
        }
        return (sum > 0 && sum < nparts);
    }

    /* Return if it is done or -1 if error*/
    int putPart(int part, s8 * buffer, int length = PART_LENGTH){
        if (length > PART_LENGTH || part > nparts-1){
            return -1;
        }
        char * pnt = data;
        pnt+=part * PART_LENGTH;
        memcpy(pnt, buffer, length);
        parts[part]++;
        return isDone();
    }

    /* Return effective size of the payload */
    int getPart(int part, int set_done, s8 * buffer){
        char * pnt = data;
        pnt+=part * PART_LENGTH;

        int length = size - part * PART_LENGTH;
        length  = length > PART_LENGTH ? PART_LENGTH : length;
        memcpy(buffer, pnt, length);

        if (set_done){
            parts[part]++;
        }
        return length;
    }

    int setPartDone(int part){
        parts[part]++;
        return isDone();
    }

    int getPartLength(){
        return PART_LENGTH;
    }

    /* if no other parts returns -1, else it returns if is done */
    int getNextPart(int set_done, s8 * buffer, int * part = 0){
        for (int i = 0; i< nparts; i++){
            if (parts[i] == 0){
                if (part!=0){
                    *part = i;
                }
                return getPart(i, set_done, buffer);
            }
        }
        return -1;
    }

    int getNumParts(){
        return nparts;
    }

    u8 getPriority(){
        return priority;
    }

    u32 getOriginalDeadline(){
        return deadline;
    }

    u32 getDeadline(){
        long long rem = ((unsigned long long) this->deadline*1000) - (timespec_timestamp() - this->ts);
        rem = rem > 0 ? rem/1000 : 0;
        return (unsigned int) rem;
    }

    ~QElement(){
        delete data;
    }
};


#endif
