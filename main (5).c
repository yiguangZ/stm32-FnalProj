/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#include "stm32l476xx.h"
#include "SysClock.h"
#include "SysTimer.h"
#include "LED.h"
#include "DMA.h"
#include "UART.h"
#include "motor.h"
#include "SPI.h"
#include "I2C.h"
#include "accelerometer.h"
#include <stdio.h>
#include <math.h>

static char buffer[IO_SIZE];
uint8_t state = 0; 
static const uint8_t TempLow = 20;
static const uint8_t TempHigh = 24;
// UART input handler
void UART_onInput(char* inputs, uint32_t size) {
    // Clean input by removing trailing \r or \n characters
    while (size > 0 && (inputs[size - 1] == '\r' || inputs[size - 1] == '\n')) {
        inputs[size - 1] = '\0';
        size--;
    }

    // Ensure input is valid for size
    if (size < 1) {
        sprintf(buffer, "Error: Empty command received.\r\n");
        UART_print(buffer);
        return;
    }

    // Use the first character as the command and switch on it
    char command = inputs[0];
    switch (command) {
        case 'O': // Open the door
            if (state == 1) {
                setDire(1);
                sprintf(buffer, "Door opening...\r\n");
                startManual();
                state = 2; // Opening
            } else {
                sprintf(buffer, "Door is already open or in transition.\r\n");
            }
            break;

        case 'C': // Close the door
            if (state == 0) {
                setDire(-1);
                sprintf(buffer, "Door closing...\r\n");
                startManual();
                state = 3; // Closing
            } else {
                sprintf(buffer, "Door is already closed or in transition.\r\n");
            }
            break;

        case 'S': // Stop the motor
            setDire(0);
            sprintf(buffer, "Motor stopped.\r\n");
            startManual();
            state = 4; // Stopped
            break;

        case 'R': // Rotate CW or CCW based on input
            if (size > 1 && inputs[1] == 'W') {
                setDire(1);
                sprintf(buffer, "Rotating clockwise...\r\n");
                startManual();
            } else if (size > 1 && inputs[1] == 'C') {
                setDire(-1);
                sprintf(buffer, "Rotating counterclockwise...\r\n");
                startManual();
            } else {
                sprintf(buffer, "Invalid rotate direction. Use 'RW' or 'RC'.\r\n");
            }
            break;

        default:
            sprintf(buffer, "Unrecognized command: %c\r\n", command);
            break;
    }
    UART_print(buffer);
}

void monitor_temp(uint8_t *last_temp, uint8_t *current_temp, uint8_t *stability_count) {
    uint8_t temp_data;
    uint8_t temp_request = 0x00;
    uint8_t sensor_address = 0x90; // Example I2C address (7-bit address shifted left by 1)

    // Request temperature data
    I2C_SendData(I2C1, sensor_address, &temp_request, 1);

    // Receive temperature data
    I2C_ReceiveData(I2C1, sensor_address, &temp_data, 1);

    // Check if the temperature has changed significantly
    if (temp_data != *last_temp) {
        if (temp_data == *current_temp) {
            // Increment stability count if reading is consistent
            (*stability_count)++;

            if ((*stability_count >= 3 && temp_data < *last_temp) || (*stability_count >= 4 && temp_data > *last_temp)) {
                *last_temp = *current_temp;
                *stability_count = 0;
                sprintf(buffer, "Updated Temperature: %d C\r\n", *last_temp);
                UART_print(buffer);
            }
        } else {
            // Update current temperature if new reading is different
            *current_temp = temp_data;
            *stability_count = 1;
        }
    }

    // Perform actions based on temperature thresholds
    switch (state) {
        case 0: // Closed
            if (*last_temp >= TempHigh) {
                sprintf(buffer, "Temperature exceeded %dC. Opening door...\r\n", TempHigh);
                UART_print(buffer);
                setDire(1); // Start opening
                state = 2; // Transition to opening
            }
            break;

        case 1: // Open
            if (*last_temp <= TempLow) {
                sprintf(buffer, "Temperature dropped below %dC. Closing door...\r\n", TempLow);
                UART_print(buffer);
                setDire(-1); // Start closing
                state = 3; // Transition to closing
            }
            break;

        case 2: // Opening
            // Door is transitioning to open; no temp-based actions
            break;

        case 3: // Closing
            // Door is transitioning to closed; no temp-based actions
            break;

        case 4: // Stopped
            sprintf(buffer, "Motor is stopped. No temperature actions.\r\n");
            UART_print(buffer);
            break;

        default:
            sprintf(buffer, "Unknown state: %d\r\n", state);
            UART_print(buffer);
            break;
    }
}
void monitor_orientation() {
    double accel_x, accel_y, accel_z;

    // Read accelerometer values
    readValues(&accel_x, &accel_y, &accel_z);

    switch (state) {
        case 2: // Opening
            if (fabs(accel_z) > 1.1) {
                sprintf(buffer, "Door fully opened. Transitioning to open state.\r\n");
                UART_print(buffer);
                state = 1; // Set state to open
                setDire(0); // Stop motor
            }
            break;

        case 3: // Closing
            if (fabs(accel_y) > 1.1) {
                sprintf(buffer, "Door fully closed. Transitioning to closed state.\r\n");
                UART_print(buffer);
                state = 0; // Set state to closed
                setDire(0); // Stop motor
            }
            break;

        case 0: // Closed
        case 1: // Open
        case 4: // Stopped
            // No actions needed in these states for orientation
            break;

        default:
            sprintf(buffer, "Invalid state during orientation check: %d\r\n", state);
            UART_print(buffer);
            break;
    }
}

int main(void) {
	// Switch System Clock = 80 MHz
	System_Clock_Init(); 
	Motor_Init();
	SysTick_Init();
	UART2_Init();
	LED_Init();	
	SPI1_GPIO_Init();
	SPI1_Init();
	initAcc();
	I2C_GPIO_Init();
	I2C_Initialization();
	
	sprintf(buffer, "Program Starts.\r\n");
	UART_print(buffer);
	
    uint8_t last_temp = 0;      // Last recorded stable temperature
    uint8_t current_temp = 0;   // Current temperature reading
    uint8_t stability_count = 0; // Stability counter for temperature

    while (1) {
        // Monitor and act on temperature
        monitor_temp(&last_temp, &current_temp, &stability_count);

        // Monitor and act on door orientation
        monitor_orientation();

        // Toggle the LED as a heartbeat indicator
        LED_Toggle();

        // Wait 100ms between checks
        delay(100);
    }
}


