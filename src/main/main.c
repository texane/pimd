/* pulse the mosfet gate driver every PIMD_PULSE_OFF_MS */
/* for PIMD_PULSE_ON_US. note that the mosfet is turned */
/* off when the pin is set to a logic one. when the mosfet */
/* is turned off, the pulse decays and a back emf is created */
/* between the coil. the purpose of pulse induction based */
/* metal detection is to measure the time for the back emf */
/* to decay, as this time varies with the presence of metal. */
/* to do so, ADC can not be used as the required sampling */
/* is too high. instead, a timer is started and is captured */
/* when the voltage on PIMD_IO_BACKEMF reaches a threshold */
/* PIMD_BACKEMF_THRESH. for this purpose, the compare and */
/* capture hardware logic is used. metal is detected with */
/* variation of the captured timer value relative to what */
/* it is when no metal is present. */
/* the circuit largely relies on this one, but some stages */
/* have been replaced using the microcontroller hardware: */
/* http://oldradiobuilder.com/mdetsp.html */


#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./uart.c"


#define PIMD_PULSE_OFF_MS 1
#define PIMD_PULSE_ON_US 50

#define PIMD_IO_PULSE_DDR DDRC
#define PIMD_IO_PULSE_PORT PORTC
#define PIMD_IO_PULSE_MASK (1 << 0)

#if 0 /* TODO */
/* calibration button */
#define PIMD_IO_CALIB_DDR DDRC
#define PIMD_IO_CALIB_PORT PORTC
#define PIMD_IO_CALIB_MASK (1 << 0)
#endif /* e */TODO


/* pulse routines */

static inline void pimd_pulse_off(void)
{
  PIMD_IO_PULSE_PORT |= PIMD_IO_PULSE_MASK;
}

static inline void pimd_pulse_on(void)
{
  PIMD_IO_PULSE_PORT &= ~PIMD_IO_PULSE_MASK;
}

static void pimd_setup(void)
{
  PIMD_IO_PULSE_DDR |= PIMD_IO_PULSE_MASK;
  pimd_pulse_off();
}

static void pimd_pulse_do(uint16_t* n)
{
  /* n the captured counter value */

  pimd_pulse_on();
  _delay_us(PIMD_PULSE_ON_US);
  pimd_pulse_off();

  /* start the timer */

  /* wait for the capture or timer overflow */

  _delay_ms(PIMD_PULSE_OFF_MS);
}

int main(void)
{
  uint8_t i;
  uint16_t n;
  uint32_t sum;

  uart_setup();
  pimd_setup();

  while (1)
  {
    /* average 8 times */
    sum = 0;
    for (i = 0; i != 8; ++i)
    {
      pimd_pulse_do(&n);
      sum += n;
    }

    uart_write(uint32_to_string(sum), 8);
    uart_write((uint8_t*)"\r\n", 2);

    /* do it 4 times per second */
    _delay_ms(250);
  }

  return 0;
}
