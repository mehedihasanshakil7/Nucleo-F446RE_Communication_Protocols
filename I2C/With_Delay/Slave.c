/***********************************************************************
*  RAJSHAHI UNIVERSITY, BANGLADESH
*  DEPARTMENT OF COMPUTER SCIENCE & ENGINEERING (CSE)
*
*  File Name           : I2C_Slave.c
*  Applicable Processor: STM32F446RE
*  Last Update		   : 30 March 2025
*  Author's Name       : Mehedi Hasan Shakil
*
*  Processsor Reference: https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
*  Datasheet           : https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html
*
*  Table of Contents   :
*  -----------------
*
*  1. Function-1: I2C1_Init
*  2. Function-2: I2C1_Read
*
************************************************************************/

// List of Register Declarations
#define RCC_BASE 	(0x40023800)
#define GPIOB_BASE  (0x40020400)
#define GPIOA_BASE  (0x40020000)
#define I2C1_BASE 	(0x40005400)
#define TIM2_BASE 	(0x40000000)

#define RCC_AHB1ENR (*(volatile unsigned int *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile unsigned int *)(RCC_BASE + 0x40))
#define RCC_CFGR 	(*(volatile unsigned int *)(RCC_BASE + 08))

#define GPIOA_MODER (*(volatile unsigned int *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER (*(volatile unsigned int *)(GPIOA_BASE + 0x04))
#define GPIOA_ODR 	(*(volatile unsigned int *)(GPIOA_BASE + 0x14))

#define GPIOB_MODER (*(volatile unsigned int *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER (*(volatile unsigned int *)(GPIOB_BASE + 0x04))
#define GPIOB_PUPDR (*(volatile unsigned int *)(GPIOB_BASE + 0x0C))
#define GPIOB_AFRH 	(*(volatile unsigned int *)(GPIOB_BASE + 0x24))

#define I2C1_CR1 	(*(volatile unsigned int *)(I2C1_BASE + 0x00))
#define I2C1_CR2 	(*(volatile unsigned int *)(I2C1_BASE + 0x04))
#define I2C1_CCR 	(*(volatile unsigned int *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE  (*(volatile unsigned int *)(I2C1_BASE + 0x20))
#define I2C1_SR1 	(*(volatile unsigned int *)(I2C1_BASE + 0x14))
#define I2C1_SR2 	(*(volatile unsigned int *)(I2C1_BASE + 0x18))
#define I2C1_DR 	(*(volatile unsigned int *)(I2C1_BASE + 0x10))
#define I2C1_OAR1 	(*(volatile unsigned int *)(I2C1_BASE + 0x08))

#define TIM2_PSC (*(volatile unsigned int *)(TIM2_BASE + 0x28))
#define TIM2_ARR (*(volatile unsigned int *)(TIM2_BASE + 0x2C))
#define TIM2_CNT (*(volatile unsigned int *)(TIM2_BASE + 0x24))
#define TIM2_CR1 (*(volatile unsigned int *)(TIM2_BASE + 0x00))
#define TIM2_SR  (*(volatile unsigned int *)(TIM2_BASE + 0x10))

// List of Function Declarations
void I2C1_Init(void);
void I2C1_Read(int n, char *str);
void Delay(unsigned int ms);

int main(void) {
	char str[6];
	I2C1_Init();
	while(1) {
		I2C1_Read(6, str);
	}
}

/*************************************************************************************************
*  FUNCTION-1: I2C1_Init
*
*  Author: Mehedi Hasan Shakil
*
*  DESCRIPTION:
*
* This C function initializes the I2C1 peripheral on the STM32F446RE micro-controller in **slave mode**.
* It configures the GPIO pins, enables the necessary clocks, and sets up the I2C communication.
*
* The function follows these steps:
*
* 1. **Select GPIO Pins for I2C1**:
*    - PB8 is set as SCL, and PB9 is set as SDA.
*    - Alternative options (PB6 for SCL, PB7 for SDA) are not used.
*
* 2. **Enable GPIOB Clock**:
*    - The clock for GPIOB is enabled through the AHB1ENR register.
*
* 3. **Configure GPIOB Pins (PB8 & PB9) for I2C Communication**:
*    - **Mode Selection**: PB8 and PB9 are set to Alternate Function mode using GPIOB_MODER.
*    - **Output Type**: Both pins are configured as open-drain using GPIOB_OTYPER.
*    - **Pull-up Configuration**: Internal pull-up resistors are enabled for PB8 and PB9 using GPIOB_PUPDR.
*    - **Alternate Function Selection**: PB8 and PB9 are assigned to I2C1 (AF4) using GPIOB_AFRH.
*
* 4. **Enable I2C1 Peripheral Clock**:
*    - The clock for I2C1 is enabled using RCC_APB1ENR.
*
* 5. **Reset and Configure I2C1 Peripheral**:
*    - **Reset I2C1**: The I2C1_CR1 register's 15th bit is set to put I2C1 in reset mode.
*    - **Exit Reset Mode**: The 15th bit is cleared to bring I2C1 out of reset mode.
*
* 6. **Set I2C1 Clock Frequency**:
*    - The default frequency is set to 16 MHz using I2C1_CR2.
*
* 7. **Configure I2C1 Addressing**:
*    - The slave address is set to 0x12 (7-bit address, left-shifted by 1).
*    - The address is enabled by setting the 14th bit of I2C1_OAR1.
*
* 8. **Enable I2C1 Peripheral**:
*    - The I2C1 peripheral is enabled (PE) by setting the 0th bit of I2C1_CR1.
*
* Once this function executes, I2C1 is fully configured and ready for communication.
*
* CALLED BY: This function should be called during system initialization before using I2C1 for communication.
*
* PARAMETERS  :  None
* RETURN VALUE:  None (void function)
*
***************************************************************************************************/
void I2C1_Init(void) {

	/*Enable clock of GPIOB through AHB1ENR*/
	RCC_AHB1ENR |= (1<<1);

	/*Set PB8 & PB9 mode to alternate function thru GPIOB_MODER*/
	GPIOB_MODER |= (1<<19);
	GPIOB_MODER &= ~(1<<18);
	GPIOB_MODER |= (1<<17);
	GPIOB_MODER &= ~(1<<16);

	/*Set PB8 & PB9 output type to open-drain*/
	GPIOB_OTYPER |= (1<<8);
	GPIOB_OTYPER |= (1<<9);

	/*Set PB8 & PB9 to output pull-up*/
	GPIOB_PUPDR &= ~(1<<19);
	GPIOB_PUPDR |= (1<<18);
	GPIOB_PUPDR &= ~(1<<17);
	GPIOB_PUPDR |= (1<<16);

	/*Set PB8 & PB9 alternate function type to I2C (AF4)*/
	GPIOB_AFRH &= ~(0xFF<<0);
	GPIOB_AFRH |= (1<<2);
	GPIOB_AFRH |= (1<<6);

	/*Enable clock of I2C1*/
	RCC_APB1ENR |= (1<<21);

	/*Enter I2C peripheral reset mode*/
	I2C1_CR1 |= (1<<15);

	/*Come out of reset mode*/
	I2C1_CR1 &= ~(1<<15);

	/*Set I2C clock frequency (default 16 MHz)*/
	I2C1_CR2 = 16;

	/*Set slave address*/
	I2C1_OAR1 = (0x12<<1);

	/*Enable the address*/
	I2C1_OAR1 |= (1<<14);

	/*Now finally enable the I2C1*/
	I2C1_CR1 |= (1<<0);
}

/*************************************************************************************************
*  FUNCTION-2: I2C1_Read
*
*  Author: Mehedi Hasan Shakil
*
*  DESCRIPTION:
*
*  This C function reads a specified number of bytes from the I2C1 peripheral on the STM32F446RE
*  micro-controller. It waits for data reception and stores the received bytes into a provided
*  character array.
*
*  The function follows these steps:
*
*  1. **Enable Acknowledgment for Incoming Data**:
*     - The ACK bit (bit 10) in the I2C1_CR1 register is set to acknowledge received data.
*
*  2. **Wait for Address Match**:
*     - The function waits until the ADDR flag (bit 1 in I2C1_SR1) is set,
*       indicating that the slave address has been matched.
*
*  3. **Clear the ADDR Flag**:
*     - Reading I2C1_SR2 clears the ADDR flag and allows the reception to continue.
*
*  4. **Receive Data Bytes in a Loop**:
*     - A loop runs for `n` iterations to read `n` bytes.
*     - In each iteration:
*       - The function waits for the RxNE (Receive Data Register Not Empty) flag (bit 6 in I2C1_SR1)
*         to be set, indicating that data is available in I2C1_DR.
*       - The received byte is read from the I2C1_DR register and stored in the provided buffer.
*
*  5. **Disable Acknowledgment**:
*     - After all bytes have been read, the ACK bit (bit 10 in I2C1_CR1) is cleared to disable
*       acknowledgment, preventing further data reception.
*
*  Once this function executes, the received data is stored in the provided buffer, and
*  I2C1 is ready for further communication.
*
*  CALLED BY: This function should be called when data needs to be received via I2C1.
*
*  PARAMETERS:
*      - `n`   : Number of bytes to read (integer).
*      - `str` : Pointer to a character array where received data will be stored.
*
*  RETURN VALUE: None (void function).
*
***************************************************************************************************/
void I2C1_Read(int n, char *str) {

	/*Enable acknowledge for slave*/
	I2C1_CR1 |= (1<<10);

    /* Wait for address match */
    while (!(I2C1_SR1 & (1 << 1))) {}

    /* Clear ADDR flag */
    (void)I2C1_SR2;

    for (int i = 0; i < n; i++) {

        /* Wait until data is received */
        while (!(I2C1_SR1 & (1 << 6))) {}

        /* Read received data */
        str[i] = I2C1_DR;
    }

	/*Disable acknowledge for slave*/
	I2C1_CR1 &= ~(1<<10);
}
