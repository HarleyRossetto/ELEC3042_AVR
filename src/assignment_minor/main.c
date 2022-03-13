#include "avr/io.h"
#include "avr/interrupt.h"

#include "fsm.h"

int initialise() {

}

int main(void) {
    if (!initialise())
        return -1;

    while (1) {
        
    }

    return 0;
}