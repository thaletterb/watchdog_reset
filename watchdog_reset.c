/****************************************************************************
Title:    Watchdog Timer Reset 
Author:   Elegantcircuits.com
File:     $Id: watchdog_reset.c
Software: AVR-GCC 3.3
Hardware: Atmega328P AVR 

Description:

An example to illustrate the AVR's watchdog timer system reset functinoality. If the pin connected from PD7 to ground is not switched on before all 4 LEDs turn off, the watchdog timer will trigger a system reset.

HW Description:

LED -> PD0
LED -> PD1
LED -> PD2
LED -> PD3

Switch connected from PD7 to GND. 
PD7 connected to internal pullup resistor

Reference:
http://electronics.stackexchange.com/questions/74840/use-avr-watchdog-like-normal-isr
*****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

// Function Prototypes
void init_io(void);
void check_wdt(void);
void setup_wdt(void);
void timer1_init(void);
void control_leds(uint8_t led_count);

// Global Variables
volatile char tick_flag = 0;                    // if non-zero, a tick has elapsed */
volatile uint8_t led_count= 0;

int main(void){
    uint8_t switch_status = 1;
    init_io();

    cli();
    check_wdt();
    setup_wdt();
    timer1_init();
    sei();                                      // Enables interrupts


    for(;;){                                    // Event Loop
        if(tick_flag){
            tick_flag = 0;
            control_leds(led_count);
            switch_status = PIND & _BV(PD7);
            if(switch_status == 0){
                wdt_reset();
            }
        }
    }
}

void control_leds(uint8_t led_count){
    switch(led_count){
        case 1:
            PORTD = 0xF0;                                           // Turn all LEDs off, and connect all pullup resistors
            PORTD |= _BV(PD0) | _BV(PD1) | _BV(PD2) | _BV(PD3);     // (LED's high & on)
            break;
        case 2:
            PORTD = 0xF0;                                           // Turn all LEDs off, and connect all pullup resistors
            PORTD |= _BV(PD0) | _BV(PD1) | _BV(PD2);                // (LED's high & on)
            break;
        case 3:
            PORTD = 0xF0;                                           // Turn all LEDs off, and connect all pullup resistors
            PORTD |= _BV(PD0) | _BV(PD1);                           // (LED's high & on)
            break;
        case 4:
            PORTD = 0xF0;                                           // Turn all LEDs off, and connect all pullup resistors
            PORTD |= _BV(PD0);                                      // (LED's high & on)
            break;
    }
}

void check_wdt(void){
    if(MCUSR & _BV(WDRF)){                  // If a reset was caused by the Watchdog Timer...
        MCUSR &= ~_BV(WDRF);                // Clear the WDT reset flag
        WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
        WDTCSR = 0x00;                      // Disable the WDT
    }
}
void init_io(void){
// Initialize PortD[3:0] as Output and PortD[7:4] as Input
    DDRD = 0x00;
    DDRD  |= _BV(PD3) | _BV(PD2) | _BV(PD1) | _BV(PD0);      // Use PortD[3:0] pins for output, PortD[7:4] are input 
    PORTD = 0xF0;                                           // Turn all LEDs off, and connect all pullup resistors
    
}

void setup_wdt(void){
// Set up Watch Dog Timer for Inactivity
    WDTCSR |= (_BV(WDCE) | _BV(WDE));                // Enable the WD Change Bit
                                                     // Enable WDT interrupt - WDIE Bit 
    WDTCSR =   _BV(WDE) |  _BV(WDP3) | _BV(WDP0);   // Set Timeout to ~8 seconds and watchdog enable 
}

void timer1_init(void) {
// Setup Timer 1 (16 bits)
    cli();                                          // Disable global interrupts
    TCCR1B |= 1<<CS11 | 1<<CS10;                    // Divide by 64
    OCR1A = 65535;                                  // Count 15624 cycles for 1 second interrupt
    TCCR1B |= 1<<WGM12;                             // Put Timer/Counter1 in CTC mode
    TIMSK1 |= 1<<OCIE1A;                            // Enable timer compare interrupt
    sei();                                          // Enable global interrupts
}

ISR(WDT_vect){
// WDT has overflowed
    
    //sleep_disable();
    PORTD ^= _BV(PD0);
    //sleep_enable();
}

ISR(TIMER1_COMPA_vect){
// Timer 1 has overflowed
    led_count++;
    tick_flag = 1;
}

