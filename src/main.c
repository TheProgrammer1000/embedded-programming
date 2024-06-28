// #include "avr8-stub.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>

/*
#define RXC0 7  // Receive Complete
#define TXC0 6  // Transmit Complete
#define UDRE0 5 // USART Data Register Empty
#define FE0 4   // Frame Error
#define DOR0 3  // Data OverRun
#define UPE0 2  // Parity Error
#define U2X0 1  // Double Transmission Speed
#define MPCM0 0 // Multi-processor Communication Mode
*/

char sevenDigit = 7;

char e = '1';
char d = '2';
char g = '3';
char c = '4';
char f = '5';
char b = '6';
char a = '7';

/*
  in RAM we can only use from 0x0100 to 0x08FF in the adressing space

  Temporary on the stack 0x08FF, and as data and addresses are pushed onto the
  stack, it grows downward towards the area where global variables are stored
*/

void setup_digit_pins_output() {

  DDRB |= (1 << PINB0);

  DDRD = DDRD | (1 << DDD2);
  DDRD = DDRD | (1 << DDD3);
  DDRD = DDRD | (1 << DDD4);
  DDRD = DDRD | (1 << DDD5);
  DDRD = DDRD | (1 << DDD6);
  DDRD = DDRD | (1 << DDD7);
}

void UART_init() {

  // Setting up the baud-width to 9600
  UBRR0L = (uint8_t)(103 & 255); // 103 in binary
  UBRR0H = (uint8_t)(103 >> 8);  // 0 in binary

  /*
  Next, the UART transmitter and receiver functions must be enabled. Otherwise,
  the UART rx/tx pins on the microcontroller will behave as standard I/O pins.
  To do this, we need to set the RXEN0 and TXEN0 bits to 1 in the UCSR0B
  register.
  */

  // Enable the UART transmitter and receiver
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
}

// Universal Synchronous and Asynchronous serial Receiver and Transmitter)
void UART_putchar(unsigned char data) {
  // wait for transmit buffer to be empty

  // This will check to see when UDREn is set to one

  // USART Control and Status Register A

  // (1 << UDREn) is setting the UDREn first bit to one
  // and then & in USCRnA is checking if this first bit is set or not
  while (!(UCSR0A & (1 << UDRE0)))
    ;

  /* Put data into buffer, sends the data. So UDRn is the buffer where we put
   * the value we want to send in */
  UDR0 = data;
}

void UART_putstring(char *s) {
  // *s > 0 this is looking for to see if the memory address in *s is 0.
  while (*s > 0)

    // *s++ this increment this memory address with one resulting in the next
    // char?
    UART_putchar(*s++);
}

void UART_puthex8(uint8_t hexValue) {
  // extract upper and lower nibbles from input value
  uint8_t upperNibble = (hexValue & 0xF0) >> 4;
  uint8_t lowerNibble = hexValue & 0x0F;

  // convert nibble to its ASCII hex equivalent
  upperNibble += upperNibble > 9 ? 'A' - 10 : '0';
  lowerNibble += lowerNibble > 9 ? 'A' - 10 : '0';

  // print the characters
  UART_putchar(upperNibble);
  UART_putchar(lowerNibble);
}

void UART_puthex16(uint16_t hexValue) {
  // transmit upper 8 bits
  UART_puthex8((uint8_t)(hexValue >> 8));

  // transmit lower 8 bits
  UART_puthex8((uint8_t)(hexValue & 0x00FF));
}

unsigned char UART_reciever_getchar(void) {
  // Wait until data is available
  while (!(UCSR0A & (1 << RXC0)))
    ; // Wait here until RXC0 becomes 1 (data received)

  // Return received data
  return UDR0;
}

void UART_getLine(char *buf, uint8_t n) {
  uint8_t bufIdx = 0;
  char c;

  // while received character is not carriage return
  // and end of buffer has not been reached
  do {
    // receive character
    c = UART_reciever_getchar();

    // store character in buffer
    buf[bufIdx++] =
        c; // increment the memory address so the next charachter will be read
  } while ((bufIdx < n) && (c != '\r'));

  // ensure buffer is null terminated
  buf[bufIdx] = 0;
}

void digit1() {

  PORTD = PORTD | (1 << PORTD6) | (1 << PORT4);

  UART_putchar('1'); // write to computer from adruino, we sending from andruino
  _delay_ms(1000);

  PORTD = PORTD & ~((1 << PORTD6) |
                    (1 << PORTD4)); // Clear PORTD6 and PORTD4 (set them low)

  _delay_ms(1000);
}

void digit2() {

  _delay_ms(1000);
  PORTD = PORTD | (1 << PORTD7) | (1 << PORTD6) | (1 << PORTD2) | (1 << PORTD3);
  PORTB |= (1 << PINB0);
  UART_putchar('2');

  _delay_ms(1000);
  PORTD =
      PORTD & ~((1 << PORTD7) | (1 << PORTD6) | (1 << PORTD2) | (1 << PORTD3));

  PORTB = PORTB & ~(1 << PINB0);
}

void digit3() {

  _delay_ms(1000);
  PORTD = PORTD | (1 << PORTD7) | (1 << PORTD6) | (1 << PORTD3) |
          (1 << PORTD4) | (1 << PORTD2);

  UART_putchar('3');

  _delay_ms(1000);
  PORTD = PORTD & ~((1 << PORTD7) | (1 << PORTD6) | (1 << PORTD3) |
                    (1 << PORTD4) | (1 << PORTD2));
}

void digit4() {
  _delay_ms(1000);
  PORTD = PORTD | (1 << PORTD5) | (1 << PORTD3) | (1 << PORTD6) | (1 << PORTD4);

  UART_putchar('4');

  _delay_ms(1000);
  PORTD =
      PORTD & ~((1 << PORTD5) | (1 << PORTD3) | (1 << PORTD6) | (1 << PORTD4));
}

void digit5() {

  /*
  char e = '1';
  char d = '2';
  char g = '3';
  char c = '4';
  char f = '5';
  char b = '6';
  char a = '7';
*/

  _delay_ms(1000);
  PORTD = PORTD | (1 << PORTD7) | (1 << PORTD5) | (1 << PORTD3) |
          (1 << PORTD4) | (1 << PORTD2);

  UART_putchar('5');

  _delay_ms(1000);
  PORTD = PORTD & ~((1 << PORTD7) | (1 << PORTD5) | (1 << PORTD3) |
                    (1 << PORTD4) | (1 << PORTD2));
}

void setup() {
  // Set the Data Direction Register for Port D to output pin 1-7

  setup_digit_pins_output();
  UART_init();
}

void loop() {
  while (1) {
    // Set all pins on Port B high
    unsigned char data_recieve_byte;

    data_recieve_byte = UART_reciever_getchar();

    // char *buffer = 10;
    // UART_getLine(buffer, data_recieve_byte);

    if (data_recieve_byte) {
      // Display 1
      if (data_recieve_byte == 49) {
        digit1();
      }

      // Display 2
      if (data_recieve_byte == 50) {
        digit2();
      }

      // Display 3
      if (data_recieve_byte == 51) {
        digit3();
      }

      // Display 4
      if (data_recieve_byte == 52) {
        digit4();
      }

      if (data_recieve_byte == 53) {
        digit5();
      }
    }
  }
}

// Turn on LED
void turnOnLed() { PORTD = PORTD | (1 << PORTD0); }

void turnOffLed() { PORTD = PORTD & ~(1 << PORTD0); }

int main(void) {
  setup();

  loop();
  return 0;
}
