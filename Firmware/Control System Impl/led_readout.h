#ifndef LED_READOUT_H
#define LED_READOUT_H

#include <Arduino.h>

#define LED_BLINK_SHORT_MS 125
#define LED_BLINK_SPACE_MS 125
#define LED_BLINK_LONG_MS (5*(LED_BLINK_SHORT_MS+LED_BLINK_SPACE_MS))
#define LED_DIGIT_SPACE_MS 1000

#define LED_ACTIVE_HIGH 1
#define LED_ACTIVE_LOW 0

class ReadoutLED {
private:
    int pin;
    bool active;
public:
    ReadoutLED(int p, bool level) : pin(p), active(level) {}

    inline void init() const { pinMode(pin,OUTPUT); }

    inline void on() const { digitalWrite(pin,active); }
    inline void off() const { digitalWrite(pin,!active); }

    void blinkShort() const
    {
        on();
        delay(LED_BLINK_SHORT_MS);
        off();
    }
    void blinkLong() const {
        on();
        delay(LED_BLINK_LONG_MS);
        off();
    }

    void blinkN(unsigned int n) const;
    void blinkDigits(unsigned int num) const;
};

#endif