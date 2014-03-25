#ifndef BBB_EXPORT_UART_H_
#define BBB_EXPORT_UART_H_

namespace bbb {

// Returns a pointer to static memory with the name of the device file for the
// UART.
const char *UartDevice();

// Unexports (if already exported) and then reexports the UART cape slot.
void ExportUart();

}  // namespace bbb

#endif  // BBB_EXPORT_UART_H_
