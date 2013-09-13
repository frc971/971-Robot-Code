#ifndef GYRO_BOARD_USB_DIGITAL_H_
#define GYRO_BOARD_USB_DIGITAL_H_

#define readGPIO(gpio, chan) ((((gpio)->FIOPIN) >> (chan)) & 1)

// These are the actual pin numbers for all of the digital I(/0) pins on the
// board.
//
// GPIO1 P0.4
// GPIO2 P0.5
// GPIO3 P0.6
// GPIO4 P0.7
// GPIO5 P0.8
// GPIO6 P0.9
// GPIO7 P2.0
// GPIO8 P2.1
// GPIO9 P2.2
// GPIO10 P2.3
// GPIO11 P2.4
// GPIO12 P2.5
//
// DIP0 P1.29
// DIP1 P2.13
// DIP2 P0.11
// DIP3 P0.10
//
// ENC0A 1.20
// ENC0B 1.23
// ENC1A 2.11
// ENC1B 2.12
// ENC2A 0.21
// ENC2B 0.22
// ENC3A 0.19
// ENC3B 0.20

void digital_init(void);

int digital(int channel);

int dip_switch(int channel);

// Boolean set by digital_init() which says whether or not this is the 3rd
// robot.
extern int is_bot3;

#endif  // GYRO_BOARD_USB_DIGITAL_H_
