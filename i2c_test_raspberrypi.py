## [Imports]
from smbus2 import SMBus    # Used to communicate with the i2c bus on pi

# Initialising the I2C Bus
BUS = SMBus(1)

# Address of the i2c device
ADDRESS = 0x4d

# The Number of Blocks of 8 bits to read from the device
BLOCKSIZE = 2   # [WARNING] If set incorrectly the i2c bus will block out

# Simple Function to read from a registar and combine two blocks of data and retrun as a HEX
def read_test(registar_address):
    # Use the Global Variables
    global BLOCKSIZE, ADDRESS, BUS

    # Attempt read from bus
    try:
        # Attempt read from device
        dat = BUS.read_i2c_block_data(ADDRESS, registar_address, BLOCKSIZE)

        # Combine the Values
        combine = dat[1] << 8 | dat[0]

        # Retrun the Hex
        return hex(combine)
    # Returns 1 on failure
    except:
        return 1

# Try reading from registar 1
print(read_test(1))

