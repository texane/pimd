#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./uart.c"


/* pimd implementation */
/* pulse the mosfet gate driver every PIMD_PULSE_OFF_MS */
/* for PIMD_PULSE_ON_US. note that the mosfet is turned */
/* off when the pin is set to a logic one. when the mosfet */
/* is turned off, the pulse decays and a back emf is created */
/* between the coil. the purpose of pulse induction based */
/* metal detection is to measure the time for the back emf */
/* to decay, as this time varies with the presence of metal. */
/* to do so, ADC can not be used as the required sampling */
/* is too high. instead, a counter is started and is captured */
/* when the voltage on PIMD_IO_BACKEMF reaches a digital one */
/* or some maximum value. metal is detected with variation of */
/* the captured counter value relative to what it is when no */
/* metal is present, measured during a calibration phase. */
/* the circuit largely relies on this one, but some stages */
/* have been replaced using the microcontroller hardware: */
/* http://oldradiobuilder.com/mdetsp.html */

/* assumes the frequency is greater or equal to 1MHz */
#define US_TO_TICKS(__us) \
((uint16_t)((__us) * (F_CPU / 1000000L)))

/* maximum tick count for backemf to be 0 and 1 */
static const uint16_t PIMD_WAIT_ZERO_TICKS = US_TO_TICKS(50);
static const uint16_t PIMD_WAIT_ONE_TICKS = US_TO_TICKS(200);

/* pulse on off durations */
#define PIMD_PULSE_OFF_MS 5
#define PIMD_PULSE_ON_US 150

/* mosfet gate control */
#define PIMD_IO_PULSE_DDR DDRC
#define PIMD_IO_PULSE_PORT PORTC
#define PIMD_IO_PULSE_MASK (1 << 0)

/* calibration button */
#define PIMD_IO_CALIB_DDR DDRC
#define PIMD_IO_CALIB_PIN PINC
#define PIMD_IO_CALIB_PORT PORTC
#define PIMD_IO_CALIB_MASK (1 << 0)

/* detect switch */
#define PIMD_IO_DETECT_DDR DDRC
#define PIMD_IO_DETECT_PIN PINC
#define PIMD_IO_DETECT_PORT PORTC
#define PIMD_IO_DETECT_MASK (1 << 0)

/* detect led */
#define PIMD_LED_DETECT_DDR DDRC
#define PIMD_LED_DETECT_PORT PORTC
#define PIMD_LED_DETECT_MASK (1 << 0)

/* not calibrated led */
#define PIMD_LED_CALIB_DDR DDRC
#define PIMD_LED_CALIB_PORT PORTC
#define PIMD_LED_CALIB_MASK (1 << 0)

/* backemf amplified to digital level, icp1 pin */
#define PIMD_IO_BEMF_DDR DDRB
#define PIMD_IO_BEMF_PIN PINB
#define PIMD_IO_BEMF_PORT PORTB
#define PIMD_IO_BEMF_MASK (1 << 0)

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
  /* setup io pins */

  PIMD_IO_PULSE_DDR |= PIMD_IO_PULSE_MASK;
  pimd_pulse_off();

  PIMD_IO_BEMF_DDR &= ~PIMD_IO_BEMF_MASK;
  PIMD_IO_BEMF_PORT |= PIMD_IO_BEMF_MASK;

  /* TODO: setup calib button an pullup */
  /* TODO: setup detect button an pullup */
  /* TODO: setup calib led */
  /* TODO: setup detect led */

  /* disable counter1 (ds, 15) */

  TCCR1B = 0;
  TCCR1A = 0;
  TCCR1C = 0;
  TIMSK1 = (1 << 5) | (1 << 1);
  TIFR1 = 0;
}

static volatile uint8_t pimd_timer1_isr = 0;

static inline void pimd_timer1_vect(void)
{
  pimd_timer1_isr = 1;
}

ISR(TIMER1_COMPA_vect)
{
  pimd_timer1_vect();
}

ISR(TIMER1_CAPT_vect)
{
  pimd_timer1_vect();
}

static void pimd_wait_bemf(uint8_t x, uint16_t* n, uint16_t m)
{
  /* wait the bemf pin to change value */
  /* x the value waited for (0 or 1) */
  /* *n the elapsed tick count */
  /* m the maximum tick count */

  pimd_timer1_isr = 0;

  /* setup counter1 (ds, 15) */
  /* ctc mode, top is ocr1a */
  /* set icr1 to m, in case of no capture */
  /* noise canceler enabled */
  /* inverted capture edge */

  OCR1A = m;
  ICR1 = m;
  TCNT1 = 0;

#define PIMD_TCCR1B_MASK ((1 << 7) | (1 << 3) | (1 << 0))
  if (x)
  {
    /* rising edge triggers capture */
    TCCR1B = PIMD_TCCR1B_MASK | (1 << 6);
  }
  else
  {
    /* falling edge triggers capture */
    TCCR1B = PIMD_TCCR1B_MASK | (0 << 6);
  }

  while (pimd_timer1_isr == 0) ;
  pimd_timer1_isr = 0;

  *n = ICR1;

  /* disable counter */
  TCCR1B &= ~(1 << 0);
}

static void pimd_do_pulse(uint16_t* n)
{
  /* n the captured counter value */

  /* actual pulse */
  pimd_pulse_on();
  _delay_us(PIMD_PULSE_ON_US);
  pimd_pulse_off();

  /* wait for pin to be 0 */
  pimd_wait_bemf(0, n, PIMD_WAIT_ZERO_TICKS);

  /* wait for pin to be 1 */
  pimd_wait_bemf(1, n, PIMD_WAIT_ONE_TICKS);

  /* wait before next pulse */
  _delay_ms(PIMD_PULSE_OFF_MS);
}

static void pimd_do_loop(uint16_t* n, uint8_t* mask)
{
  /* *n the averaged value */

  uint8_t i;
  uint32_t sum32;

#define PIMD_LOOP_MASK_WAIT_ONE (1 << 0)
  *mask = 0;

#define PIMD_AVG_COUNT ((uint8_t)20)
  sum32 = 0;
  for (i = 0; i != PIMD_AVG_COUNT; ++i)
  {
    pimd_do_pulse(n);
    sum32 += *n;

    if ((*n) == PIMD_WAIT_ONE_TICKS)
    {
      /* indicate the wait for 1 was reached */
      *mask |= PIMD_LOOP_MASK_WAIT_ONE;
    }
  }

  *n = (uint16_t)(sum32 / (uint32_t)i);
}

static void pimd_wait_event(uint8_t* mask)
{
#define PIMD_EV_MASK_CALIB (1 << 0)
#define PIMD_EV_MASK_DETECT (1 << 1)
  *mask = 0;

#if 0 /* TODO */

  if (PIMD_IO_CALIB_PIN & PIMD_IO_CALIB_MASK)
    *mask |= PIMD_EV_MASK_CALIB;

  if (PIMD_IO_DETECT_PIN & PIMD_IO_DETECT_MASK)
    *mask |= PIMD_EV_MASK_DETECT;

#else

  *mask |= PIMD_EV_MASK_DETECT;

#endif /* TODO */
}


/* main */

int main(void)
{
  uint8_t mask;
  uint16_t calib = 0;
  uint16_t n;

  uart_setup();
  pimd_setup();

  uart_write((uint8_t*)"go\r\n", 4);

  sei();

  while (1)
  {
    pimd_wait_event(&mask);

    if (mask & PIMD_EV_MASK_CALIB)
    {
      pimd_do_loop(&n, &mask);

      if (mask & PIMD_LOOP_MASK_WAIT_ONE)
      {
	/* calibration failure */
	uart_write((uint8_t*)"calib failed\r\n", 14);
      }
      else
      {
	calib = n;
	uart_write((uint8_t*)"calib ", 6);
	uart_write(uint16_to_string(calib), 4);
	uart_write((uint8_t*)"\r\n", 2);
      }
    }
    else if (mask & PIMD_EV_MASK_DETECT)
    {
      pimd_do_loop(&n, &mask);

      /* compare to calibration value */
      if (n > (calib + 2))
      {
	uart_write((uint8_t*)"detect ", 7);
	uart_write(uint16_to_string(calib), 4);
	uart_write((uint8_t*)" ", 1);
	uart_write(uint16_to_string(n), 4);
	uart_write((uint8_t*)"\r\n", 2);
      }
    }
  }

  return 0;
}
