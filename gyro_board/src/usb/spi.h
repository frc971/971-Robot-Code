#ifndef __SPI_H__
#define __SPI_H__

void spi_init (void);
uint16_t transfer_spi_bytes(uint16_t data);
void disable_gyro_csel (void);
void enable_gyro_csel (void);

#endif // __SPI_H__
