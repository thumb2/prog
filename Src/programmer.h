/******************************************************************************
* DEFINES
*/
// Programmer data line bitmasks (programmer I/O port 0)
#define SWCLK       0x80 // PA7
#define SWDIO		0x20 // PA5

#define SET_DIO_INPUT() do { \
        GPIOA->MODER &= ~(0x03 << 10UL);         \
    }while(0)
//! Set programmer DD line as output
#define SET_DIO_OUTPUT() do { \
        GPIOA->MODER &= ~(0x03 << 10UL);          \
        GPIOA->MODER |= (0x01 << 10UL);          \
    }while(0)

/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    Initializes the programmer by switching to 32 MHz XOSC and
*           configuring I/O.
*
* @param    data    Byte to write
*
* @return   None.
******************************************************************************/
void programmer_init(void)
{
    // Switch programmer (a CC2530) to 32 MHz XOSC for max performance
    // CLKCONCMD = 0x80;
    // while (CLKCONSTA != 0x80);

    // Set P0[6:0] as output low, P0[7] (RESET_N) as output high
    // P0 = RESET_N;
    // P0DIR = 0xFF;
//    GPIOA->BSRR = RESET_N;
}

