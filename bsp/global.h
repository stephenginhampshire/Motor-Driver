/* 
 * File:   global.h
 * Author: Stephen
 *
 * Created on 15 August 2017, 21:16
 */

#ifndef GLOBAL_H
#define	GLOBAL_H

#define true 1
#define false 0

typedef struct {
    unsigned char running : 1;
    unsigned char cmd : 1;
    unsigned char dummy : 6;
} global_flags;

#endif	/* GLOBAL_H */

