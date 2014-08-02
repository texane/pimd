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


#define PIMD_PULSE_OFF_MS 5
#define PIMD_PULSE_ON_US 150

#define PIMD_IO_PULSE_DDR DDRC
#define PIMD_IO_PULSE_PORT PORTC
#define PIMD_IO_PULSE_MASK (1 << 0)

#if 0 /* TODO */
/* calibration button */
#define PIMD_IO_CALIB_DDR DDRC
#define PIMD_IO_CALIB_PORT PORTC
#define PIMD_IO_CALIB_MASK (1 << 0)
#endif /* TODO */

/* pcint23, ain1, pd7 */
#define PIMD_IO_BEMF_DDR DDRD
#define PIMD_IO_BEMF_PIN PIND
#define PIMD_IO_BEMF_PORT PORTD
#define PIMD_IO_BEMF_MASK (1 << 7)

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

  /* PIMD_IO_BEMF as an input */
  PIMD_IO_BEMF_DDR &= ~PIMD_IO_BEMF_MASK;
  PIMD_IO_BEMF_PORT |= PIMD_IO_BEMF_MASK;

/* #define CONFIG_ACMP */
#ifdef CONFIG_ACMP
  /* setup analog comparator. datasheet ch22. */
  /* when AIN0(bandgap) > AIN1, then ACO is set */
  /* setup to capture the value of counter1 */
  /* AIN0 is set to bandgap voltage reference (1.81) */
  /* AIN1 is wired to pin PIMD_IO_BEMF */
  ADCSRB = 1 << 6;
  ACSR = (1 << 7) | (1 << 6) | (1 << 2);
  DIDR1 = (1 << 1) | (1 << 0);
#endif /* CONFIG_ACMP */

  /* setup timer */
  /* normal counter */
  /* noise canceler enabled */
  /* inverted capture edge */
  TCCR1A = 0;
  TCCR1B = (1 << 7) | (1 << 6);

#ifdef CONFIG_ACMP
  TIMSK1 = 1 << 5;
#endif /* CONFIG_ACMP */
}

static void pimd_pulse_do(uint16_t* n)
{
  /* n the captured counter value */

  /* clear the ICF1 flag by writing one */
  TIFR1 |= 1 << 5;

  /* actual pulse */
  pimd_pulse_on();
  _delay_us(PIMD_PULSE_ON_US);
  pimd_pulse_off();

  /* set counter to 0 and enable */
  TCNT1 = 0;
  TCCR1B |= 1 << 0;

#if 1
  /* wait for the pin to be 0 */
  while (1)
  {
    if ((PIMD_IO_BEMF_PIN & PIMD_IO_BEMF_MASK) == 0) break ;
  }
#endif

#ifdef CONFIG_ACMP
  /* enable the comparator */
  ACSR &= ~(1 << 7);

  /* wait for the capture or timer top value */
  while (1)
  {
    /* wait event capture by polling ICF1 */
    if (TIFR1 | (1 << 5))
    {
      *n = ICR1;
      break ;
    }
  }
#else
  while (1)
  {
    if (PIMD_IO_BEMF_PIN & PIMD_IO_BEMF_MASK)
    {
      *n = TCNT1;
      break ;
    }
  }
#endif

  /* disable counter */
  TCCR1B &= ~(1 << 0);

#ifdef CONFIG_ACMP
  /* disable the comparator */
  ACSR |= 1 << 7;
#endif /* CONFIG_ACMP */

  /* wait before next pulse */
  _delay_ms(PIMD_PULSE_OFF_MS);
}

int main(void)
{
  uint8_t i;
  uint16_t n;
  uint32_t sum;

  uart_setup();
  pimd_setup();

  uart_write((uint8_t*)"go\r\n", 4);

  sei();

  while (1)
  {
    /* averaging loop */
    sum = 0;
    for (i = 0; i != 64; ++i)
    {
      pimd_pulse_do(&n);
      sum += n;
    }

    uart_write(uint32_to_string(sum / i), 8);
    uart_write((uint8_t*)"\r\n", 2);
  }

  return 0;
}
