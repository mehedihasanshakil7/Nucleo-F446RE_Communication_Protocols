/***********************************************************************
*  RAJSHAHI UNIVERSITY, BANGLADESH
*  DEPARTMENT OF COMPUTER SCIENCE & ENGINEERING (CSE)
*
*  File Name           : I2C_Master.c
*  Applicable Processor: STM32F446RE
*  Last Update  	   : 30 March 2025
*  Author's Name       : Mehedi Hasan Shakil
*
*  Processsor Reference: https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
*  Datasheet           : https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html
*
*  Table of Contents   :
*  -----------------
*
*  1. Function-1: I2C1_Init
*  2. Function-2: I2C1_Send
*  3. Function-3: Delay
*
************************************************************************/

// List of Register Declarations
#define RCC_BASE   (0x40023800)
#define GPIOA_BASE (0x40020000)
#define GPIOB_BASE (0x40020400)
#define I2C1_BASE  (0x40005400)
#define TIM2_BASE  (0x40000000)

#define RCC_AHB1ENR (*(volatile unsigned int *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile unsigned int *)(RCC_BASE + 0x40))

#define GPIOA_MODER (*(volatile unsigned int *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER (*(volatile unsigned int *)(GPIOA_BASE + 0x04))
#define GPIOA_ODR   (*(volatile unsigned int *)(GPIOA_BASE + 0x14))

#define GPIOB_MODER (*(volatile unsigned int *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER (*(volatile unsigned int *)(GPIOB_BASE + 0x04))
#define GPIOB_PUPDR (*(volatile unsigned int *)(GPIOB_BASE + 0x0C))
#define GPIOB_AFRH  (*(volatile unsigned int *)(GPIOB_BASE + 0x24))

#define I2C1_CR1 (*(volatile unsigned int *)(I2C1_BASE + 0x00))
#define I2C1_CR2 (*(volatile unsigned int *)(I2C1_BASE + 0x04))
#define I2C1_CCR (*(volatile unsigned int *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE (*(volatile unsigned int *)(I2C1_BASE + 0x20))
#define I2C1_SR1 (*(volatile unsigned int *)(I2C1_BASE + 0x14))
#define I2C1_SR2 (*(volatile unsigned int *)(I2C1_BASE + 0x18))
#define I2C1_DR  (*(volatile unsigned int *)(I2C1_BASE + 0x10))

#define TIM2_PSC (*(volatile unsigned int *)(TIM2_BASE + 0x28))
#define TIM2_ARR (*(volatile unsigned int *)(TIM2_BASE + 0x2C))
#define TIM2_CR1 (*(volatile unsigned int *)(TIM2_BASE + 0x00))
#define TIM2_SR  (*(volatile unsigned int *)(TIM2_BASE + 0x10))

// List of Function Declarations
void I2C1_Init(void);
void I2C1_Send(char saddr, int n, char* str);
void Delay(unsigned int ms);

int main(void) {
	I2C1_Init();
	while(1) {
		I2C1_Send(0x12, 6, "Shakil");
		Delay(50);
	}
}

/*************************************************************************************************
*  FUNCTION-1: I2C1_Init
*
*  Author: Mehedi Hasan Shakil
*
*  DESCRIPTION:
*
* This C function initializes the I2C1 peripheral on the STM32F446RE micro-controller in **master mode**.
* It configures the GPIO pins, enables the necessary clocks, and sets up the I2C communication parameters.
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
*    - The APB1 clock frequency is set to 16 MHz using I2C1_CR2 (bit 4 is set to 1).
*
* 7. **Configure I2C1 in Standard Mode (100 KHz)**:
* 		   APB1 clock frequency, f_PCLK1 = 16 MHz
* 		   Standard mode I2C clock frequency, f_SCL = 100 KHz
*	       CCR = f_PCLK1 / (2 * f_SCL)
*	    	   = 16,000,000 / (2 * 100,000)
*	    	   = 80
*	 - The clock control register (I2C1_CCR) is set to 80, configuring a standard mode with a 100 KHz clock speed.
*
* 8. **Set Rise Time for I2C Signal Stability**:
* 		   Maximum allowed SCL rise time, t_r = 1000 ns (as per reference manual RM0390.pdf)
* 		   APB1 clock frequency, f_PCLK1 = 16 MHz
* 		   TRISE = (t_r * f_PCLK1) + 1
* 		   		 = (1x10^-6 * 16x10^6) + 1
* 		   		 = 17
*    - The maximum rise time is set to 17 using the I2C1_TRISE register.
*
* 9. **Enable I2C1 Peripheral**:
*    - The I2C1 peripheral is enabled (PE) by setting the 0th bit of I2C1_CR1.
*
* Once this function executes, I2C1 is fully configured in **master mode** and ready for communication.
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
	I2C1_CR2 |= (1<<4);

	/*Set I2C in standard mode, 100 KHz*/
	I2C1_CCR = 80;

	/*Set rise time*/
	I2C1_TRISE = 17;

	/*Now finally enable the I2C1*/
	I2C1_CR1 |= (1<<0);
}

/*************************************************************************************************
*  FUNCTION-2: I2C1_Send
*
*  Author: Mehedi Hasan Shakil
*
*  DESCRIPTION:
*
*  This C function sends a specified number of bytes to a slave device using the I2C1 peripheral
*  on the STM32F446RE micro-controller. It ensures proper communication by handling the start
*  condition, slave addressing, data transmission, and stop condition.
*
*  The function follows these steps:
*
*  1. **Wait Until the Bus is Free**:
*     - The function checks the BUSY flag (bit 1 in I2C1_SR2) to ensure the I2C bus is not in use.
*
*  2. **Generate a Start Condition**:
*     - The START bit (bit 8 in I2C1_CR1) is set to initiate communication.
*     - The function waits until the SB (Start Bit) flag (bit 0 in I2C1_SR1) is set, indicating the start condition has been generated.
*
*  3. **Send the Slave Address in Write Mode**:
*     - The slave address is written to I2C1_DR with the least significant bit (LSB) as `0` (write mode).
*     - A timeout mechanism ensures the slave responds in time.
*     - If the timeout expires, a STOP condition is generated to terminate the communication.
*
*  4. **Clear the ADDR Flag**:
*     - Reading I2C1_SR2 clears the ADDR flag, allowing further communication.
*
*  5. **Transmit Data Bytes in a Loop**:
*     - The function loops `n` times to send `n` bytes from the provided data buffer.
*     - In each iteration:
*       - The function waits for the TxE (Transmit Data Register Empty) flag (bit 7 in I2C1_SR1) to be set, indicating readiness for the next byte.
*       - The next byte is placed into I2C1_DR for transmission.
*
*  6. **Wait for the Transmission to Complete**:
*     - The function waits for the BTF (Byte Transfer Finished) flag (bit 2 in I2C1_SR1), ensuring that all bytes have been sent.
*
*  7. **Generate a Stop Condition**:
*     - The STOP bit (bit 9 in I2C1_CR1) is set to release the bus and terminate communication.
*
*  Once this function executes, the data is successfully transmitted to the slave, and I2C1 is ready for the next operation.
*
*  CALLED BY: This function should be called when data needs to be sent via I2C1 as a master.
*
*  PARAMETERS:
*      - `saddr`: 7-bit slave address (left-shifted by 1 internally for write operation).
*      - `n`    : Number of bytes to send (integer).
*      - `str`  : Pointer to a character array containing the data to be transmitted.
*
*  RETURN VALUE: None (void function).
*
***************************************************************************************************/
void I2C1_Send(char saddr, int n, char* str) {

	/*Wait when bus is busy*/
	while(I2C1_SR2 & (1<<1)) {}

	/*Generate start condition and wait*/
	I2C1_CR1 |= (1<<8);
	while(!(I2C1_SR1 & (1<<0))) {}

	/*Send slave address + write (0) bit and wait for 500 milliseconds*/
	I2C1_DR = (saddr<<1);
	unsigned int timeout = 500;
	while(!(I2C1_SR1 & (1<<1))) {
		if(--timeout == 0) {
			/*Generate stop*/
			I2C1_CR1 |= (1<<9);
			return;
		}
		Delay(1);
	}

	/*Clear ADDR flag*/
	(void)I2C1_SR2;

	for(int i = 0; i < n; i++) {

		/*Wait until transmitter is empty*/
		while(!(I2C1_SR1 & (1<<7))) {}

		/*Place the data to the data register one byte at time*/
		I2C1_DR = *str++;
	}

	/*Wait until transfer finished*/
	while(!(I2C1_SR1 & (1<<2))) {}

	/*Generate stop*/
	I2C1_CR1 |= (1<<9);
}

/*************************************************************************************************
*  FUNCTION-3: Delay
*
*  Author: Mehedi Hasan Shakil
*
*  DESCRIPTION:
*
*  This C function generates a delay in milliseconds using Timer 2 (TIM2) on the STM32F446RE
*  micro-controller. The timer is configured to count in 1 ms steps, and the function waits
*  until the desired delay duration has elapsed.
*
*  The function follows these steps:
*
*  1. **Enable TIM2 Clock**:
*     - The TIM2 clock is enabled via the RCC_APB1ENR register (bit 0).
*
*  2. **Set Timer Prescaler**:
*     - The prescaler (TIM2_PSC) is set to `16000-1`, ensuring the timer increments every 1 ms
*       (assuming the system clock is 16 MHz).
*
*  3. **Set Auto-Reload Value**:
*     - The auto-reload register (TIM2_ARR) is loaded with the desired delay duration in
*       milliseconds (`ms`), determining how long the timer should run.
*
*  4. **Reset Update Flag**:
*     - The update interrupt flag (UIF, bit 0 of TIM2_SR) is cleared before starting the timer
*       to ensure a fresh count.
*
*  5. **Enable Timer**:
*     - The counter is started by setting the CEN (bit 0 of TIM2_CR1).
*
*  6. **Wait for Timer to Complete**:
*     - The function continuously checks the UIF flag in TIM2_SR.
*     - When UIF is set, it indicates that the specified delay has elapsed.
*
*  7. **Disable Timer**:
*     - The timer is stopped by clearing the CEN bit in TIM2_CR1.
*
*  Once this function executes, a delay of `ms` milliseconds has been completed.
*
*  CALLED BY: Any function requiring a time delay.
*
*  PARAMETERS:
*      - `ms` : Number of milliseconds to delay (unsigned integer).
*
*  RETURN VALUE: None (void function).
*
***************************************************************************************************/
void Delay(unsigned int ms) {

	//Enable TIM2 clock (APB1 peripheral bus)
	RCC_APB1ENR |= (1<<0);

	//Set prescaler to 16000-1 (so timer counts in 1 ms steps)
	TIM2_PSC = 16000 - 1;

	//Set auto-reload value
	TIM2_ARR = ms;

	//Reset update flag (UIF)
	TIM2_SR &= ~(1<<0);

	//Enable timer
	TIM2_CR1 |= (1<<0);

	//Wait for update flag (UIF) to be set
	while(!(TIM2_SR & (1<<0))) {}

	//Disable timer
	TIM2_CR1 &= ~(1<<0);
}
