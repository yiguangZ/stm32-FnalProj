#include "SPI.h"
#include "SysTimer.h"
#include "accelerometer.h"


void accWrite(uint8_t addr, uint8_t val) {
    SPI_Transfer_Data(((addr & 0x3F) << 8) | val);
}


uint8_t accRead(uint8_t addr) {
    return SPI_Transfer_Data(((addr & 0x3F) | 0x80) << 8);
}


void initAcc(void) {
    // Set to 100 Hz data rate
    accWrite(0x2C, 0x0A);

    // Set to full resolution mode and ±16g range
    accWrite(0x31, 0x0B);

    // Enable measurement
    accWrite(0x2D, 0x08);
}


void readValues(double* x, double* y, double* z) {
    uint8_t x0 = accRead(0x32);
    uint8_t x1 = accRead(0x33);
    uint8_t y0 = accRead(0x34);
    uint8_t y1 = accRead(0x35);
    uint8_t z0 = accRead(0x36);
    uint8_t z1 = accRead(0x37);


    int16_t x_raw = (x1 << 8) | x0;
    int16_t y_raw = (y1 << 8) | y0;
    int16_t z_raw = (z1 << 8) | z0;


    double scale = 0.0039;  // Scale factor in g/LSB for full-resolution mode
    *x = x_raw * scale;
    *y = y_raw * scale;
    *z = z_raw * scale;
}