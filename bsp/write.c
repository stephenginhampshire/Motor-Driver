/*
 * Redirecting standard output to UART2 module
 */

#include <stdio.h>
#include "uart.h"

int write(int handle, void *buffer, unsigned int len) {
    int i;

    switch (handle) {
        case 0: // stdin
        case 1: // stdout
        case 2: // stderr
            for (i = len; i; --i) {
                vPutCharU2(*(int*) buffer++);
            }
            break;

        default:
            break;
    }
    return (len);
}
