/*
 * Copyright (C) 2012 adrian_007, adrian-007 on o2 point pl
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/**
 * ATMega32-based digital clock with double two-digit 7-segment display modules.
 *
 * Functionality:
 * 1. Telling time, obviously
 * 2. Alarm (via buzzer)
 * 3. Drift calibration. Even with clock XTAL, clocks tends to drift few seconds a month. In order to mitigate this effect,
      there's an algorithm that adds one second every 10th minute multiplied by selected value. Example:
      calibration value set to 6; 10th minute multiplied by 6 is 60 minutes, so every hour clock would count one extra second forward.
      If value would be negative, it would skip one second every hour.
 * 4. Four buttons to configure it (mode, +, -, set alarm)
 */

/**
 * By design, clock is using external 16 MHz XTAL
 */
#ifndef F_CPU
# define F_CPU 16000000
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define CALIB_EEPROM_ADDR    (reinterpret_cast<uint32_t*>(0x0A))
#define ALARM_EEPROM_ADDR    (reinterpret_cast<uint32_t*>(0x1A))

#define DDR_SEGMENTS        DDRD
#define DDR_MULTIPLEX       DDRC
#define DDR_BLINK_LEDS      DDRA
#define DDR_BUTTONS         DDRA
#define DDR_BUZZER          DDRA

#define PORT_SEGMENTS       PORTD
#define PORT_MULTIPLEX      PORTC
#define PORT_BLINK_LEDS     PORTA
#define PORT_BUTTONS        PORTA
#define PORT_BUZZER         PORTA

#define PIN_SEGMENTS        PIND
#define PIN_MULTIPLEX       PINC
#define PIN_BLINK_LEDS      PINA
#define PIN_BUTTONS         PINA
#define PIN_BUZZER          PINA

#define DISP1_PIN           PINC0
#define DISP2_PIN           PINC1
#define DISP3_PIN           PINC2
#define DISP4_PIN           PINC3

#define BLINK_LED1_PIN      PINA6
#define BLINK_LED2_PIN      PINA7

#define BUTTON1_PIN         PINA3
#define BUTTON2_PIN         PINA2
#define BUTTON3_PIN         PINA1
#define BUTTON4_PIN         PINA0

#define BUZZER_PIN          PINA4

#define SEGMENT_A_PIN       PIND0
#define SEGMENT_B_PIN       PIND1
#define SEGMENT_C_PIN       PIND2
#define SEGMENT_D_PIN       PIND3
#define SEGMENT_E_PIN       PIND4
#define SEGMENT_F_PIN       PIND5
#define SEGMENT_G_PIN       PIND6
#define SEGMENT_DOT_PIN     PIND7

#define DIGIT_0             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_E_PIN | 1 << SEGMENT_F_PIN))
#define DIGIT_1             (~(1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN))
#define DIGIT_2             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_B_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_E_PIN | 1 << SEGMENT_G_PIN))
#define DIGIT_3             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_G_PIN))
#define DIGIT_4             (~(1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_F_PIN | 1 << SEGMENT_G_PIN))
#define DIGIT_5             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_F_PIN | 1 << SEGMENT_G_PIN))
#define DIGIT_6             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_E_PIN | 1 << SEGMENT_F_PIN | 1 << SEGMENT_G_PIN))
#define DIGIT_7             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN))
#define DIGIT_8             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_E_PIN | 1 << SEGMENT_F_PIN | 1 << SEGMENT_G_PIN))
#define DIGIT_9             (~(1 << SEGMENT_A_PIN | 1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_F_PIN | 1 << SEGMENT_G_PIN))

class Display
{
public:
    Display()
    {
        digits[0]  = DIGIT_0;
        digits[1]  = DIGIT_1;
        digits[2]  = DIGIT_2;
        digits[3]  = DIGIT_3;
        digits[4]  = DIGIT_4;
        digits[5]  = DIGIT_5;
        digits[6]  = DIGIT_6;
        digits[7]  = DIGIT_7;
        digits[8]  = DIGIT_8;
        digits[9]  = DIGIT_9;
        digits[10] = ~(1 << SEGMENT_G_PIN);       // "minus"
        digits[11] = 0xFF;                        // off
    }

    void init()
    {
        DDR_SEGMENTS    |=   1 << SEGMENT_A_PIN  | 1 << SEGMENT_B_PIN | 1 << SEGMENT_C_PIN | 1 << SEGMENT_D_PIN | 1 << SEGMENT_E_PIN | 1 << SEGMENT_F_PIN | 1 << SEGMENT_G_PIN | 1 << SEGMENT_DOT_PIN;
        DDR_BLINK_LEDS  |=   1 << BLINK_LED1_PIN | 1 << BLINK_LED2_PIN;
        PORT_BLINK_LEDS &= ~(1 << BLINK_LED1_PIN | 1 << BLINK_LED2_PIN);
        set(-1);

    }

    void set(int16_t digit, bool dot = false)
    {
        if(digit < 0)
        {
            digit = 10;
        }
        else if(digit > 9)
        {
            digit = 11;
        }

        if(dot)
        {
            PORT_SEGMENTS = digits[static_cast<uint32_t>(digit)] & ~(1 << SEGMENT_DOT_PIN);
        }
        else
        {
            PORT_SEGMENTS = digits[static_cast<uint32_t>(digit)] |  (1 << SEGMENT_DOT_PIN);
        }
    }

private:
    volatile uint8_t digits[12];
};

class Config
{
public:
    enum
    {
        BUTTON_INCREMENT    = 0x01,
        BUTTON_DECREMENT    = 0x02,
        BUTTON_MODE         = 0x04,
        BUTTON_ALARM        = 0x08
    };

    enum
    {
        MODE_NONE            = 0,
        MODE_SET_MINUTES     = 1,
        MODE_SET_HOURS       = 2,
        MODE_LAST            = 3
    };

    Config()
    {
        mMode = MODE_NONE;
    }

    void init()
    {
        DDR_BUTTONS  |= ~(1 << BUTTON1_PIN | 1 << BUTTON2_PIN | 1 << BUTTON3_PIN | 1 << BUTTON4_PIN);
        PORT_BUTTONS |=  (1 << BUTTON1_PIN | 1 << BUTTON2_PIN | 1 << BUTTON3_PIN | 1 << BUTTON4_PIN); // pull-up on buttons
    }

    uint8_t testButtons(const bool updateMode = true)
    {
        uint8_t flags = 0;
        if(isButtonPressed(&PIN_BUTTONS, BUTTON1_PIN))
        {
            flags |= BUTTON_MODE;

            if(updateMode)
            {
                mMode = ++mMode % MODE_LAST;
            }
        }

        if(isButtonPressed(&PIN_BUTTONS, BUTTON2_PIN))
        {
            flags |= BUTTON_INCREMENT;
        }

        if(isButtonPressed(&PIN_BUTTONS, BUTTON3_PIN))
        {
            flags |= BUTTON_DECREMENT;
        }

        if(isButtonPressed(&PIN_BUTTONS, BUTTON4_PIN))
        {
            flags |= BUTTON_ALARM;
        }

        return flags;
    }

    uint8_t mode() const
    {
        return mMode;
    }

    void setMode(uint8_t mode)
    {
        mMode = mode;
    }

private:
    uint8_t     mMode;

    bool isButtonPressed(volatile uint8_t* pin, uint8_t pinNumber)
    {
        if(bit_is_clear(*pin, pinNumber))
        {
            _delay_ms(20);
            return bit_is_clear(*pin, pinNumber);
        }
        return false;
    }
};

class Clock
{
public:
    Clock()
    {
        mSeconds = 0;
        mPause = true;
    }

    void init()
    {
        // Timer2 is using external 32 kHz "clock" XTAL
        ASSR  |= 1 << AS2;          // async mode
        TCCR2 |= 1 << CS22;         // prescaler 64 (which gives interrupts every 0.5s)
        TIMSK |= 1 << TOIE2;

        while(ASSR & (1 << TCR2UB));
    }

    bool isPaused() const
    {
        return mPause;
    }

    void pause(const bool p)
    {
        mPause = p;
    }

    void set(int32_t val)
    {
        mSeconds = val;
    }

    int32_t ticks() const
    {
        return mSeconds;
    }

    virtual void increment(int32_t val = 1)
    {
        mSeconds += val;

        if(mSeconds >= 86400 || mSeconds < 0)
        {
            mSeconds = 0;
        }
    }

    virtual int16_t digit(uint8_t pos, bool& dot, bool& blink, uint8_t mode) const
    {
        blink = false;

        switch(pos)
        {
            case 0:
            {
                if(mode & Config::MODE_SET_MINUTES)
                {
                    blink = true;
                }
                return minutes() % 10;
            }
            case 1:
            {
                if(mode & Config::MODE_SET_MINUTES)
                {
                    blink = true;
                }
                return minutes() / 10;
            }
            case 2:
            {
                if(mode & Config::MODE_SET_HOURS)
                {
                    blink = true;
                }
                return hours() % 10;
            }
            case 3:
            {
                uint8_t h = hours() / 10;
                if(h > 0)
                {
                    if(mode & Config::MODE_SET_HOURS)
                    {
                        blink = true;
                    }
                    return h;
                }
                return 0xFF;
            }
        }

        return 0xFF;
    }

protected:
    volatile int32_t mSeconds;
    volatile bool    mPause;

    uint8_t seconds() const
    {
        return static_cast<uint8_t>(mSeconds % 60);
    }

    uint8_t minutes() const
    {
        return static_cast<uint8_t>((mSeconds / 60) % 60);
    }

    uint8_t hours() const
    {
        return static_cast<uint8_t>((mSeconds / (60*60)) % 24);
    }
};

class Alarm : public Clock
{
public:
    void init()
    {
        DDR_BUZZER  |=   1 << BUZZER_PIN;
        PORT_BUZZER &= ~(1 << BUZZER_PIN);

        Clock::init();
        mActive = false;
    }

    bool active() const
    {
        return mActive;
    }

    void setActive(const bool active)
    {
        mActive = active;
    }

protected:
    volatile bool mActive;
};

class Calibration : public Clock
{
public:
    void init()
    {
        Clock::init();
        correction = 0;
        mSeconds = 0;
    }

    int8_t checkCorrection()
    {
        if(mSeconds != 0 && ++correction >= absoluteValue())
        {
            correction = 0;

            if(mSeconds < 0)
            {
                return -1;
            }
            else
            {
                return 1;
            }
        }

        return 0;
    }

    void increment(int32_t val = 1)
    {
        if(val == 0)
        {
            return;
        }

        (val > 0) ? val = 1 : val = -1;

        mSeconds += val;

        if(mSeconds < -999 || mSeconds > 999)
        {
            mSeconds -= val;
        }
    }

    int16_t digit(uint8_t pos, bool& dot, bool& blink, uint8_t mode) const
    {
        if(mode & Config::MODE_SET_HOURS)
        {
            blink = true;
        }

        int32_t ret = absoluteValue();

        switch(pos)
        {
            case 0:
            {
                ret %= 10;
                break;
            }
            case 1:
            {
                ret /= 10;
                ret %= 10;
                break;
            }
            case 2:
            {
                ret /= 100;
                break;
            }
            case 3:
            {
                if(mSeconds < 0)
                {
                    return -1;
                }
                else
                {
                    return 0xFF;
                }
            }
        }

        return ret;
    }

private:
    volatile int32_t correction;

    int32_t absoluteValue() const
    {
        if(mSeconds < 0)
        {
            return mSeconds*(-1);
        }
        return mSeconds;
    }
};

Config      config;
Display     display;
Clock       clock;
Calibration calibration;
Alarm       alarm;
Clock*      currentClock = 0;

int main()
{
    // Disable anything that is not needed / used
    {
        UCSRB &= ~(1 << TXEN | 1 << RXEN);      // Just in case, to make sure PD0:PD1 works
        DDRB  =   0x00;
        PORTB =   0xFF;                         // Enable pull-up for not connected ports
    }

    // Init objects
    {
        display.init();
        config.init();
        clock.init();
        calibration.init();
        alarm.init();
    }

    // Init multiplexer
    {
        TCCR0         |= 1 << CS02;             // prescaler 256
        TIFR          |= 1 << TOV0;
        TIMSK         |= 1 << TOIE0;
        DDR_MULTIPLEX |= 1 << DISP1_PIN | 1 << DISP2_PIN | 1 << DISP3_PIN | 1 << DISP4_PIN;
    }

    // Sleep a bit to let power source stabilize
    _delay_ms(100);

    // Read calibration value
    {
        eeprom_busy_wait();
        int32_t calib = static_cast<int32_t>(eeprom_read_dword(CALIB_EEPROM_ADDR));
        calibration.set(calib);
    }

    // Read last alarm value
    {
        eeprom_busy_wait();
        int32_t alarmTicks = static_cast<int32_t>(eeprom_read_dword(ALARM_EEPROM_ADDR));
        alarm.set(alarmTicks);
    }

    uint8_t buttons = 0;
    bool wantSave = false;
    clock.pause(true);

    // Enable interrupts and start the fun
    sei();

    do
    {
        if((buttons = config.testButtons()) != 0)
        {
            clock.pause(true);

            if(currentClock == 0)
            {
                currentClock = &clock;
            }

            if(buttons & Config::BUTTON_ALARM)
            {
                if(config.mode() == Config::MODE_NONE)
                {
                    alarm.setActive(!alarm.active());
                }
                else
                {
                    if(currentClock == &clock)
                    {
                        currentClock = &alarm;
                    }
                    else
                    {
                        currentClock = &clock;
                    }
                    config.setMode(Config::MODE_SET_MINUTES);
                }
            }
            else if((buttons & Config::BUTTON_INCREMENT) && (buttons & Config::BUTTON_DECREMENT) && config.mode() == Config::MODE_NONE)
            {
                currentClock = &calibration;

                do // until all buttons are released
                {
                    buttons = config.testButtons(false);
                }
                while(buttons != 0);

                _delay_ms(500);
                config.setMode(Config::MODE_SET_HOURS);
            }

            int8_t upValue = 0;

            if(buttons & Config::BUTTON_INCREMENT)
            {
                upValue = 1;
            }
            else if(buttons & Config::BUTTON_DECREMENT)
            {
                upValue = -1;
            }

            switch(config.mode())
            {
                case Config::MODE_SET_MINUTES:
                {
                    wantSave = true;
                    currentClock->increment(upValue * 60);
                    break;
                }
                case Config::MODE_SET_HOURS:
                {
                    wantSave = true;
                    currentClock->increment(upValue * 3600);
                    break;
                }
                case Config::MODE_NONE:
                {
                    currentClock = &clock;

                    if(wantSave == true)
                    {
                        cli();

                        int32_t ticks;
                        eeprom_busy_wait();
                        ticks = calibration.ticks();
                        eeprom_write_dword(CALIB_EEPROM_ADDR, ticks);

                        eeprom_busy_wait();
                        ticks = alarm.ticks();
                        eeprom_write_dword(ALARM_EEPROM_ADDR, ticks);

                        wantSave = false;

                        sei();
                    }

                    clock.pause(false);
                    break;
                }
            }

            // 4 x 20ms = 80ms, +120 = 200ms, constantly pressed button = 5 units/sec
            _delay_ms(120);
        }
    }
    while(true);

    return 0;
}

volatile uint8_t activeSegment = 0;
volatile bool    halfSecond    = false;

ISR(TIMER0_OVF_vect)
{
    TCNT0 = 222;             // sth like 100 times per second... +/- ...

    int8_t  pin    = -1;
    int16_t digit  = -1;
    bool    dot    = false;
    bool    blink  = false;

    if(currentClock != 0)
    {
        digit = currentClock->digit(activeSegment, dot, blink, config.mode());
    }
    else
    {
        blink = true;
    }

    if(blink == false || halfSecond == true)
    {
        switch(activeSegment)
        {
            case 0:
            {
                dot = alarm.active();
                pin = DISP1_PIN;
                break;
            }
            case 1:
            {
                pin = DISP2_PIN;
                break;
            }
            case 2:
            {
                pin = DISP3_PIN;
                break;
            }
            case 3:
            {
                pin = DISP4_PIN;
                break;
            }
        }
    }

    PORT_MULTIPLEX |= 1 << DISP1_PIN | 1 << DISP2_PIN | 1 << DISP3_PIN | 1 << DISP4_PIN;

    display.set(digit, dot);

    if(pin >= 0)
    {
        PORT_MULTIPLEX &= ~(1 << pin);
    }

    activeSegment = activeSegment + 1;
    activeSegment %= 4;
}

ISR(TIMER2_OVF_vect)
{
    // Disable interrupts
    cli();

    if(clock.isPaused() == false)
    {
        if(halfSecond == false)
        {
            int8_t n = 1;

            // Every 10 minutes
            if((clock.ticks() % 600) == 0)
            {
                n += calibration.checkCorrection();
            }

            clock.increment(n);
        }

        PORT_BLINK_LEDS ^= 1 << BLINK_LED1_PIN | 1 << BLINK_LED2_PIN;
    }
    else
    {
        PORT_BLINK_LEDS |= 1 << BLINK_LED1_PIN | 1 << BLINK_LED2_PIN;
    }

    if(alarm.active())
    {
        uint32_t ticks          = clock.ticks();
        uint32_t alarmTicks     = alarm.ticks();

        if(ticks >= alarmTicks && ticks < (alarmTicks + 3600))
        {
            PORT_BUZZER ^= 1 << BUZZER_PIN;
        }
        else
        {
            PORT_BUZZER &= ~(1 << BUZZER_PIN);
        }
    }
    else
    {
        PORT_BUZZER &= ~(1 << BUZZER_PIN);
    }

    halfSecond = !halfSecond;

    // Enable interrupts
    sei();
}
