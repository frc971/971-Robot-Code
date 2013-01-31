// ****************************************************************************
// CopyLeft qwerk Robotics unINC. 2010 All Rights Reserved.
// ****************************************************************************

// ****************************************************************************
// **************** IO Pin Setup
// ****************************************************************************

#include "FreeRTOS.h"

void analog_init (void)
{
	// b[1:0] CAN RD1 p0.0
	// b[3:2] CAN TD1 p0.1
	//PINCON->PINSEL0 = 0x00000005;

	// b[29:28] USB_DMIN 	p0.30
	// b[27:26] USB_DPLUS	p0.29
	// b[21:20] AD0.3	p0.26
	// b[19:18] AD0.2	p0.25
	// PINCON->PINSEL1 = 0x14140000;

	// PINCON->PINSEL2 = 0x0;

	// b[31:30] AD0.5	p1.31
	// b[29:28] V_BUS	p1.30
	// b[21:20] MCOB1	p1.26
	// b[19:18] MCOA1	p1.25
	// b[15:14] MCI1	p1.23
	// b[13:12] MCOB0	p1.22
	// b[09:08] MCI0	p1.20
	// b[07:06] MCOA0	p1.19
	// b[05:04] USB_UP_LED	p1.18
	//PINCON->PINSEL3 = 0xE0145150;
	SC->PCONP |= PCONP_PCAD;

	// Enable AD0.0, AD0.1, AD0.2, AD0.3
	PINCON->PINSEL1 &= 0xFFC03FFF;
	PINCON->PINSEL1 |= 0x00D54000;
	ADC->ADCR = 0x00200500;
}

// ****************************************************************************
// **************** ADC Functions
// ****************************************************************************


// **************** macros
// starts convertion [26:24] = 001

// **************** functions
int analog(int channel)
{
	ADC->ADCR = ((ADC->ADCR & 0xF8FFFF00) | (0x01000000 | (1 << channel)));

	// Poll until it is done.
	while(!(ADC->ADGDR & 0x80000000));

	return ((ADC->ADGDR & 0x0000FFF0) >> 4);
}
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

// DIP0 P1.29
// DIP1 P2.13
// DIP2 P0.11
// DIP3 P0.10
#define readGPIO(gpio,chan) ((((gpio)->FIOPIN) >> (chan)) & 1)
inline int readGPIO_inline(int major,int minor){
	switch(major){
		case 0:
			return readGPIO(GPIO0,minor);
		case 1:
			return readGPIO(GPIO1,minor);
		case 2:
			return readGPIO(GPIO2,minor);
		default:
			return -1;
	}
}
int digital(int channel)
{
	if(channel < 1){
		return -1;
	}else if(channel < 7){
		int chan = channel + 3;
		return readGPIO(GPIO0,chan);
	}else if(channel < 13){
		int chan = channel - 7;
		return readGPIO(GPIO2,chan);
	}
	return -1;
}
int dip(int channel)
{
	switch(channel){
		case 0:
			return readGPIO(GPIO1,29);
		case 1:
			return readGPIO(GPIO2,13);
		case 2:
			return readGPIO(GPIO0,11);
		case 3:
			return readGPIO(GPIO0,10);
		default:
			return -1;

	}
}
//ENC0A 1.20
//ENC0B 1.23
//ENC1A 2.11
//ENC1B 2.12
//ENC2A 0.21
//ENC2B 0.22
//ENC3A 0.19
//ENC3B 0.20

#define ENC(gpio,a,b) readGPIO(gpio,a) * 2 + readGPIO(gpio,b)
int encoder_bits(int channel)
{
	switch(channel){
		case 0:
			return ENC(GPIO1,20,23);
		case 1:
			return ENC(GPIO2,11,12);
		case 2:	
			return ENC(GPIO0,21,22);
		case 3:	
			return ENC(GPIO0,19,20);
		default:
			return -1;
	}
	return -1;
}

volatile int32_t encoder1_val;
void encoder_init(void)
{
// port 0
	// Setup the encoder interface.
	SC->PCONP |= PCONP_PCQEI;
	PINCON->PINSEL3 = ((PINCON->PINSEL3 & 0xffff3dff) | 0x00004100);

	// Reset the count and velocity
	QEI->QEICON = 0x00000005;

	QEI->QEICONF = 0x00000004;
	// Wrap back to 0 when we wrap the int...
	QEI->QEIMAXPOS = 0xffffffff;
// port 1
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
	//NVIC_EnableIRQ(EINT3_IRQn);
	//PINSEL4 23/22 0 1
	//PINSEL4 25 24 0 1
	PINCON->PINSEL4 = (PINCON->PINSEL4 & ~(0x3 << 22)) | (0x1 << 22);
	PINCON->PINSEL4 = (PINCON->PINSEL4 & ~(0x3 << 24)) | (0x1 << 24);

	//EXTMODE 1 2 1 1 // all others off
	SC->EXTMODE = 0x6;
	SC->EXTINT = 0x6;
	encoder1_val = 0;
//ports 2 and 3
	

}
void EINT1_IRQHandler(void){
	//ENC1A 2.11
	int stored_val = encoder1_val;
	int fiopin = GPIO2->FIOPIN;
	if(((fiopin >> 1) ^ fiopin) & 0x800){
		stored_val ++;
	}else{
		stored_val --;
	}
	encoder1_val = stored_val;
	SC->EXTPOLAR ^= 0x2;   
	SC->EXTINT = 0x2;
}
void EINT2_IRQHandler(void){
	//ENC1B 2.12
	int stored_val = encoder1_val;
	int fiopin = GPIO2->FIOPIN;
	if(((fiopin >> 1) ^ fiopin) & 0x800){
		stored_val --;
	}else{
		stored_val ++;
	}
	encoder1_val = stored_val;
	SC->EXTPOLAR ^= 0x4;   
	SC->EXTINT = 0x4;
}
void EINT3_IRQHandler(void){
	
}
int32_t encoder_val(int chan)
{
	switch(chan){
		case 0:
			return (int32_t)QEI->QEIPOS;
		case 1:
			return encoder1_val;
		case 2:
		case 3:
		default:
			return -1;
	}
}

