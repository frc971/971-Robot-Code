#ifndef CAPE_UART_COMMON_H_
#define CAPE_UART_COMMON_H_

// Configures it for baud, 1 start bit, 1 stop bit, and 1 even parity bit.
// baud must be between 57KHz and 3.75MHz. It must be something that the USART can
// handle exactly (see
// <http://www.st.com/web/en/resource/technical/document/reference_manual/CD00225773.pdf#page=633>
// or so for choices (fPCLK = 60MHz, OVER8 = 0 for now)).
void uart_common_configure(int baud);

#endif  // CAPE_UART_COMMON_H_
