/******************************************************************************************
 * Example program for use of I2C slave
 * This example acts as a 256byte RAM
******************************************************************************************/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

// define I2C addresses of slave
#define I2C0_SLAVE_ADDR 0x30

// GPIO pins to use for I2C
#define GPIO_SDA0 12
#define GPIO_SCK0 13

// define base address of I2C controller hardware
#define I2C0_BASE 0x40044000

// define the hardware registers used
volatile uint32_t * const I2C0_DATA_CMD       = (volatile uint32_t * const)(I2C0_BASE + 0x10);
volatile uint32_t * const I2C0_INTR_STAT      = (volatile uint32_t * const)(I2C0_BASE + 0x2c);
volatile uint32_t * const I2C0_INTR_MASK      = (volatile uint32_t * const)(I2C0_BASE + 0x30);
volatile uint32_t * const I2C0_CLR_RD_REQ     = (volatile uint32_t * const)(I2C0_BASE + 0x50);

// Declare the bits in the registers we use
#define I2C_DATA_CMD_FIRST_BYTE 0x00000800
#define I2C_DATA_CMD_DATA       0x000000ff
#define I2C_INTR_STAT_READ_REQ  0x00000020
#define I2C_INTR_STAT_RX_FULL   0x00000004
#define I2C_INTR_MASK_READ_REQ  0x00000020
#define I2C_INTR_MASK_RX_FULL   0x00000004

// define address to be used for RAM accesses
// N.B. Address auto increments, as stored in 8 bit value it automatically rolls round when reaches 255
uint8_t ram_addr;
// define storage for data of RAM
uint8_t ram[256];

// Interrupt handler implements the RAM
void i2c0_irq_handler() {
    // Get interrupt status
    uint32_t status = *I2C0_INTR_STAT;
    // Check to see if we have received data from the I2C master
    if (status & I2C_INTR_STAT_RX_FULL) {
        // Read the data (this will clear the interrupt)
        uint32_t value = *I2C0_DATA_CMD;
        // Check if this is the 1st byte we have received
        if (value & I2C_DATA_CMD_FIRST_BYTE) {
            // If so treat it as the address to use
            ram_addr = (uint8_t)(value & I2C_DATA_CMD_DATA);
        } else {
            // If not 1st byte then store the data in the RAM
            // and increment the address to point to next byte
            ram[ram_addr] = (uint8_t)(value & I2C_DATA_CMD_DATA);
            ram_addr++;
        }
    }
    // Check to see if the I2C master is requesting data from us
    if (status & I2C_INTR_STAT_READ_REQ) {
        // Write the data from the current address in RAM
        *I2C0_DATA_CMD = (uint32_t)ram[ram_addr];
        // Clear the interrupt
        *I2C0_CLR_RD_REQ;
        // Increment the address
        ram_addr++;
    }
}

// Main loop - initilises system and then loops while interrupts get on with processing the data
int main() {

    // Setup I2C0
    i2c_init(i2c0, 100 * 1000);
    i2c_set_slave_mode(i2c0, true, I2C0_SLAVE_ADDR);
    gpio_set_function(GPIO_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCK0, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_SDA0);
    gpio_pull_up(GPIO_SCK0);

    // Initialise the address to use for ram to 0
    ram_addr = 0;

    // Enable the interrupts we want
    *I2C0_INTR_MASK = (I2C_INTR_MASK_READ_REQ | I2C_INTR_MASK_RX_FULL);

    // Set up the interrupt handlers
    irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);
    // Enable I2C interrupts
    irq_set_enabled(I2C0_IRQ, true);

    // Do nothing in main loop
    while (true) {
        tight_loop_contents();
    }
    return 0;
}