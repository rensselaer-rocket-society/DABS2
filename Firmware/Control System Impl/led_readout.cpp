#include "led_readout.h"

void ReadoutLED::blinkN(unsigned int n) const {
    if(n==0){
        blinkLong();
    }
    else{
        blinkShort();
        for(unsigned int i = 1; i < n; ++i) {
            delay(LED_BLINK_SPACE_MS);
            blinkShort();
        }
    }
}
// Bulids up stack of digits, blinks on rollback
// Max stack depth is limited to ceil(N*log(2)) for machine with N bit ints
void ReadoutLED::blinkDigits(unsigned int num) const {
    if(num < 10){
        blinkN(num);
    } else {
        blinkDigits(num/10);
        delay(LED_DIGIT_SPACE_MS);
        blinkN(num%10);
    }
}