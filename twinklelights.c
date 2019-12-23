/*   twinklelights.c
 *   Copyright (C) 2019  Josh Boudreau
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#define F_CPU 1000000

#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define LATCH_OUT   0
#define CLOCK       1
#define DATA_OUT    2
#define OUT_ENA     3

#define NUMBER_OF_CHIPS 1
#define DATA_WIDTH (NUMBER_OF_CHIPS * 8)

#define CLOCK_PULSE_DELAY 5 //us

#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)

enum Mode{NORM, SPEC};

#define NUM_FRAMES 8
uint8_t pattern[NUM_FRAMES] = {
  1 << 0,
  1 << 1,
  1 << 2,
  1 << 3,
  1 << 4,
  1 << 5,
  1 << 6,
  1 << 7,
};

void blink_lights(uint8_t *byte);
void shuffle(uint8_t *byte);
void go2sleep(void);
void shift_8_bits(uint8_t byte);
void setup();
void pulse(uint8_t pin);
void switch_mode(enum Mode mode);
void send_config(uint32_t config);
void show_pattern();

ISR(WDT_vect){
  WDTCR |= (1 << WDIE) | (1 << WDE);
}

int main(){
  setup();
  uint8_t seed = 0x55;
  while(1){
    blink_lights(&seed);
    go2sleep();
    show_pattern();
    go2sleep();
  }
}

void blink_lights(uint8_t *byte){
  shift_8_bits(0);
  PORTB &= ~(1 << OUT_ENA);
  for(uint8_t i = 0; i < NUM_FRAMES; i++){
    shuffle(byte);
    shift_8_bits(*byte);
    _delay_ms(200);
  }
  PORTB |= (1 << OUT_ENA);
}

void shuffle(uint8_t *byte){
  uint8_t value = *byte;
  value ^= value << 3;
  value ^= value >> 1;
  value ^= value << 5;
  *byte = value;
}

void go2sleep(){
  wdt_reset();
  WDTCR |= (1 << WDIE) | (1 << WDE);
  sleep_enable();
  sleep_cpu();
}

void setup(){
  //  CLOCK, LATCH_OUT, DATA_OUT, P_LOAD_IN all outputs, rest input
  DDRB = (1 << CLOCK) | (1 << LATCH_OUT) | (1 << DATA_OUT) | (1 << OUT_ENA);
  // set pullups on remaining pins
  PORTB |= ~((1 << CLOCK) | (1 << LATCH_OUT) | (1 << DATA_OUT) | (1 << OUT_ENA));
  // init outputs low
  PORTB &= ~(1 << CLOCK) & ~(1 << LATCH_OUT) & ~(1 << OUT_ENA);
  adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  switch_mode(SPEC);
  shift_8_bits(0b00000000); // lowest brightness
  switch_mode(NORM);
  cli();
  WDTCR |= (1 << WDCE); // WD change enable
  WDTCR |= (1 << WDP3) | (1 << WDP0); // 8 second timeout
  WDTCR |= (1 << WDIE) | (1 << WDE); // enable WD interrupt, no reset
  sei();
}

void pulse(uint8_t pin){
  _delay_us(CLOCK_PULSE_DELAY);
  PORTB ^= (1 << pin);
  _delay_us(CLOCK_PULSE_DELAY);
  PORTB ^= (1 << pin);
}

void switch_mode(enum Mode mode){
  uint8_t PORTB_old = PORTB;
  PORTB = (1 << OUT_ENA) | (0 << CLOCK) | (0 << LATCH_OUT);
  pulse(CLOCK);
  PORTB &= ~(1 << OUT_ENA);
  pulse(CLOCK);
  PORTB |= (1 << OUT_ENA);
  pulse(CLOCK);
  switch(mode){
  case SPEC:
    PORTB |= (1 << LATCH_OUT);
    break;
  default:
  case NORM:
    break;
  }
  pulse(CLOCK);
  PORTB &= ~(1 << LATCH_OUT);
  pulse(CLOCK);
  PORTB = PORTB_old;
}

void shift_8_bits(uint8_t byte){
  for(uint8_t i = 0; i < 8; i++){
    // set SDO
    if(byte & 0x01){
      PORTB |= (1 << DATA_OUT);
    }else{
      PORTB &= ~(1 << DATA_OUT);
    }
    pulse(CLOCK);
    byte = byte >> 1;
  }
  // latch output
  pulse(LATCH_OUT);
}

void show_pattern(){
  shift_8_bits(0);
  PORTB &= ~(1 << OUT_ENA);
  for(uint8_t i = 0; i < NUM_FRAMES; i++){
    shift_8_bits(pattern[i]);
    _delay_ms(200);
  }
  PORTB |= (1 << OUT_ENA);
}
