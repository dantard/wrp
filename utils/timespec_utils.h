/*
 * timespec_utils.h
 *
 *  Created on: Mar 23, 2015
 *      Author: danilo
 */

#ifndef TIMESPEC_UTILS_H_
#define TIMESPEC_UTILS_H_
#include <time.h>
#include <time.h>

typedef unsigned long long u64b;
typedef long long s64b;

void timespec_addms(struct timespec *ts, long ms);
void timespec_subtract(struct timespec *a, struct timespec *b);
int timespec_milliseconds(struct timespec *a);
void timespec_now(struct timespec *ts);
int timespec_subtract_to_ms(struct timespec *a, struct timespec *b);
int timespec_elapsed_ms(struct timespec *a);
int timespec_elapsed_us(struct timespec *a);
unsigned long long timespec_timestamp(void);
void timespec_future(struct timespec *ts, unsigned int ms);
void timespec_elapsed_print_us(struct timespec *a, char * text);
unsigned int timespec_timestamp_ms(void) ;

#endif /* TIMESPEC_UTILS_H_ */
