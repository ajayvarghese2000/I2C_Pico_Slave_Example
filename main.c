/**
 * @file main.c
 * @author Ajay Varghese (Team C)
 * @date 21 March 2022
 * @brief The program aims to set-up i2c slave communication on the Raspberry
 * Pi Pico so that it can interface with I2C master controllers to send data
 * when requested of it.
 * 
 * This is part of a collection of programs and codes for the 21WSD001 Team
 * Project run by Loughborough  University
*/


/* *************** [INCLUDES] *************** */

// Standard C library's to use standarc C functions and types
#include <stdio.h>
#include <stdint.h>

// Standard Pi Pico library to initialise Pico's Peripherals 
#include "pico/stdlib.h"

// library to implement I2C communications from Pico SDK
#include "hardware/i2c.h"

// library to handle interrupts from I2C reads and writes
#include "hardware/irq.h"



/* *************** [CONFIGURATION VARIABLES] *************** */ 

// Uncomment this line to enable debugging statements onto USB UART - MUST ENABLE IN MAKEFILE
#define DEBUGGING

// The Address of the I2C Peripheral
#define I2C_ADDR 0x4D   // [WARNING] can only take values between 0x08 - 0x77

// The Specific GP Pins on the Pi Pico to enable I2C on
#define SDA_SLAVE 4
#define SCL_SLAVE 5

// Data Array to hold the information from the registers
// 0 -> The i2c address of the device itself
// 1 -> The data
uint16_t data[2] = {I2C_ADDR, 0};

// Register Variable to Read/Write data too
uint8_t register_accessed;



/* *************** [EXECTUTION CODE] *************** */ 

/**
 * @brief This interrupt is called whenever the i2c0 bus is accessed by the
 * master. It handles what to do on reads and writes as well as resetting
 * interrupt registers
 * 
 */
void i2c0_irq_handler() {

    // Get interrupt status
    uint32_t status = i2c0->hw->intr_stat;

    // Check to see if we have received data from the I2C master (Will always be called on R/W)
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {

        // Parsing the register the master is trying to access
        register_accessed = ((i2c0->hw->data_cmd) << 24) >> 24;

        #ifdef DEBUGGING
            printf("Write acessed %02x\n", register_accessed);
        #endif

        /* 
            As this device will not need to update any internal registars no writes
            will be done. If we did want to write to an internal register to e.g
            setup configurations for this i2c device we would write that code
            below here.
        */
        
    }

    // Check to see if the I2C master is requesting data from us
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {

        #ifdef DEBUGGING
            printf("Read Accessed On Register %02x\n", register_accessed);
        #endif

        uint16_t invalid_data = 0;

        switch (register_accessed)
        {   
            // Register One contains the address of the device
            case 0:
                //data[register_accessed] = I2C_ADDR;
                i2c_write_raw_blocking(i2c0, &data[register_accessed], 2);

                #ifdef DEBUGGING
                    printf("Data Sent -> %02x\n", data[register_accessed]);
                #endif

                break;
            // Register Two Contains the Data from the device
            case 1:
                i2c_write_raw_blocking(i2c0, &data[register_accessed], 2);

                #ifdef DEBUGGING
                    printf("Data Sent -> %02x\n", data[register_accessed]);
                #endif

                break;
            // If An incorrect Register Is Accessed the output is the invalid_data variable
            default:

                i2c_write_raw_blocking(i2c0, &invalid_data, 2);

                #ifdef DEBUGGING
                    printf("Data Sent -> %02x\n", invalid_data);
                #endif

                #ifdef DEBUGGING
                    printf("Incorrect Register\n");
                #endif

                break;
        }

        #ifdef DEBUGGING
            printf("Sent Data\n");
        #endif

        // Clear the interrupt
        i2c0->hw->clr_rd_req;
    }
}



/* *************** [Main Loop] *************** */ 

int main() {

    // Initialize GPIO and Debug Over USB UART - MUST ENABLE IN MAKEFILE
    stdio_init_all();

    // Initializing the I2C0 Controller on the Pi Pico
    i2c_init(i2c0, 10000);

    // Setting the I2C0 Controller as a I2C Slave
    i2c_set_slave_mode(i2c0, true, I2C_ADDR);

    // Enabling I2C Mode on Pins GP4 and GP5
    gpio_set_function(SDA_SLAVE, GPIO_FUNC_I2C);
    gpio_set_function(SCL_SLAVE, GPIO_FUNC_I2C);

    // Enabling the internal Pull Up resistors for I2C to work
    gpio_pull_up(SDA_SLAVE);
    gpio_pull_up(SCL_SLAVE);

    // Enable the interrupts on i2c0 controller on access
    i2c0->hw->intr_mask = (I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);

    // Set up the interrupt handler function to call on an interrupt
    irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);

    // Enable I2C interrupts on the NVIC
    irq_set_enabled(I2C0_IRQ, true);


    // Printing a Debugging statement to the USB UART
    #ifdef DEBUGGING
        printf("I2C Set-Up and Active\n");
    #endif

    // Loop forever
    while (true) {

        tight_loop_contents();
        
    }

    return 0;
}