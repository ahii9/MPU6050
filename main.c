#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>"
#include <stdint.h>
#include "lcd_i2c.h"
// ??a ch? I2C c?a MPU6050
#define MPU6050_I2C 0xD0

// C?c thanh ghi c?a MPU6050
#define	PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACC_CONFIG 0x1C
#define INT_ENABLE 0x38
#define	INT_PIN_CFG 0x37
#define INT_STATUS 0x3A
#define ACC_XOUT_H 0x3B
#define ACC_XOUT_L 0x3C
#define ACC_YOUT_H 0x3D
#define ACC_YOUT_L 0x3E
#define ACC_ZOUT_H 0x3F
#define ACC_ZOUT_L 0x40

volatile float ax, ay, az, a_all;
volatile	float max = 0;
int count =0;
//*******//

volatile float Threshold = 7.6; // NGUONG PHAT HIEN NGA

//*******//
volatile uint8_t sys_on = 1;
volatile uint8_t is_falled = 0;
volatile uint8_t led_high = 0;
volatile uint8_t time_led = 5;
void sleep_mode(void) {
	//Turn on PWR_EN (bit28 on RCC_APB1ENR)
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    // CLEAR PDDS ( bit1) to enter stop mode
    PWR->CR &= ~1<<1;
    // SET LPDS (bit0) to 1 
    PWR->CR |= 1;
    // Set the SleepDeep mode in the System Control Register (SCR)
    SCB->SCR |= 1<<2;
	
    //Synchronize memory and device instructions before entering sleep mode
    __DSB();
    __ISB();
	
    // Enter sleep mode by executing the Wait For Interrupt (WFI) instruction
    __WFI();
}
void RCC_Configuration(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait until HSE is ready
    // Configure PLL
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9); // HSE * 9 = 72MHz

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Select PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void I2C_Init(void) {
    // cap xung cho I2C va GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6 SCL, PB7 SDA
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7);
    GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1;
    GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;

    // reset software
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // I2C1 config
    I2C1->CR2 |= 36; // freq is 36MHz
    I2C1->CCR = 180; // 100kHz
    I2C1->TRISE = 37; // Maximum rise time

    // Enable I2C1
    I2C1->CR1 |= I2C_CR1_PE;
}

void sendData(uint8_t address, uint8_t value) {
    I2C1->CR1 |= I2C_CR1_START; // Generate START condition
    while (!(I2C1->SR1 & (uint16_t)0x0001)); // Wait for START condition generated
    I2C1->DR = MPU6050_I2C; // Send slave address
    while (!(I2C1->SR1 & (uint16_t)0x0002)); // Wait for address sent
    I2C1->SR2;
    I2C1->DR = address; // Send register address
    while (!(I2C1->SR1 & (uint16_t)0x0080)); // Wait for data register to be empty
    I2C1->DR = value; // Send value
    while (!(I2C1->SR1 & (uint16_t)0x0004)); // Wait for byte transfer finished
    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
}

void ReceiveOneByte(uint8_t address, uint8_t *data) {
    while (I2C1->SR2 & I2C_SR2_BUSY); // Wait for bus to be idle
    I2C1->CR1 |= (uint16_t)0x0100; // Generate START condition
    while (!(I2C1->SR1 & (uint16_t)0x0001)); // Wait for START condition generated
    I2C1->DR = MPU6050_I2C; // Send slave address
    while (!(I2C1->SR1 & (uint16_t)0x0002)); // Wait for address sent
    I2C1->SR2;
    I2C1->DR = address; // Send register address
    while (!(I2C1->SR1 & I2C_SR1_TXE)); // Wait for data register to be empty
    // Receive
    I2C1->CR1 |= (uint16_t)0x0100; // Generate repeated START condition
    while (!(I2C1->SR1 & (uint16_t)0x0001)); // Wait for START condition generated
    I2C1->DR = MPU6050_I2C | 0x01; // Send slave address with read bit
    while (!(I2C1->SR1 & (uint16_t)0x0002)); // Wait for address sent
    I2C1->SR2;
    I2C1->CR1 &= ~I2C_CR1_ACK; // Disable acknowledge
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait for data register to not be empty
    *data = I2C1->DR; // Read data
    while (I2C1->SR1 & I2C_SR1_BTF); // Wait for data register to be empty
    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
}

void MPU6050_Init(void) {
    sendData(PWR_MGMT_1, 0x00);  // Wake up MPU6050 by writing 0x00 to the power management register
    sendData(SMPLRT_DIV, 0x07);  // Set the sample rate divider to 7 (1 kHz / (1 + 7) = 125 Hz)
    sendData(CONFIG, 0x00);      // Disable FSYNC and set the Digital Low Pass Filter (DLPF) to 260 Hz
    sendData(GYRO_CONFIG, 0x00); // Set the gyro full-scale range to ±250 degrees/sec
    sendData(ACC_CONFIG, 0x00);  // Set the accelerometer full-scale range to ±2g
    sendData(INT_ENABLE, 0x01);  // Enable data ready interrupt
    sendData(INT_PIN_CFG, 0x10); // Configure the interrupt pin as push-pull, active low
}

uint16_t MeasureAcc(uint8_t address) {
    uint8_t value0;
	  uint8_t value1;
    uint16_t raw;
    ReceiveOneByte(address, &value0);
    ReceiveOneByte(address + 1, &value1);
    raw = ((uint16_t)value0) << 8 |(uint16_t)value1;
    return raw;
}
void MPU6050_ReadAccel(float* ax, float* ay, float* az) {
		//doi sang so float
    *ax = (float)MeasureAcc(ACC_XOUT_H) / 16384.0f *9.8;
    *ay = (float)MeasureAcc(ACC_XOUT_H) / 16384.0f *9.8;
    *az = (float)MeasureAcc(ACC_XOUT_H) / 16384.0f *9.8;
}
void Calculator (float ax, float ay, float az, float* a_all) {
		*a_all = sqrt(pow(ax-9.8,2)+pow(ay-9.8,2)+pow(az-9.8,2));
} 
void Max(float *max,float a_all) {
	if(*max < a_all) {
		*max = a_all;
	}
}
void EXTI_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;  // Enable clock for AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable clock for GPIOA
    
    // Configure PA0 as pull-up input
    GPIOA->CRL &= ~(0x0F << 0); // Clear bits 0-3 for PA0
    GPIOA->CRL |= (0x02<< 2);  // Set bits 2-3 for CNF0 to 10 (pull-up input mode), leaving MODE0 as 00 (input)
    GPIOA->ODR |= (0x01 << 0);  // Set PA0 to 1 to enable the pull-up resistor

    // Configure AFIO for EXTI on PA0
    AFIO->EXTICR[0] &= ~(AFIO_EXTICR1_EXTI0); // Clear previous EXTI0 configuration
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA; // Configure EXTI line 0 to be mapped to PA0

    // Clear pending interrupt flag for EXTI line 0
    EXTI->PR |= EXTI_PR_PR0;

    // Configure EXTI line 0 to trigger on the falling edge
    EXTI->FTSR |= EXTI_FTSR_TR0;

    // Disable rising edge trigger for EXTI line 0
    EXTI->RTSR &= ~(EXTI_RTSR_TR0);

    // Enable interrupt mask for EXTI line 0 (PA0)
    EXTI->IMR |= EXTI_IMR_MR0;

    // Disable event mask for EXTI line 0
    EXTI->EMR &= ~(EXTI_EMR_MR0);

    // Set priority and enable the NVIC interrupt for EXTI line 0
    NVIC_SetPriority(EXTI0_IRQn, 1);  // Set priority to 1 (lower number = higher priority)
    NVIC_EnableIRQ(EXTI0_IRQn);       // Enable EXTI line 0 interrupt in NVIC
}
void TIM1_Config(void) {
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 7200 - 1; 
    TIM1->ARR = 1000 - 1;
    TIM1->DIER |= TIM_DIER_UIE; 
    NVIC_EnableIRQ(TIM1_UP_IRQn); 
    TIM1->CR1 |= TIM_CR1_CEN;
}

void checkfall (int a_all, uint8_t *is_falled) {
	if (a_all >= Threshold) {
			*is_falled = 1;
	}
	else {
		*is_falled = 0;
	}
}
// X? lý ng?t cho TIM1
void TIM1_UP_IRQHandler(void) {
	char tmp[6];
    if (TIM1->SR & TIM_SR_UIF) {  // Ki?m tra c? ng?t c?p nh?t (Update Interrupt Flag)
        TIM1->SR &= ~TIM_SR_UIF;  // Xóa c? ng?t
        MPU6050_ReadAccel(&ax,&ay,&az);
				Calculator(ax,ay,az,&a_all);
				Max(&max,a_all);
				checkfall(max, &is_falled);
			if(is_falled ) {
					time_led = 2;
				} else {
        // Th?c hi?n các hành d?ng c?n thi?t khi ng?t x?y ra
						time_led = 5;
				}
				if (count > time_led) {
						if(led_high == 0) {
							GPIOC->BSRR = GPIO_BSRR_BR13;
							count =0;
							led_high = 1;
						} else {
							GPIOC->BSRR = GPIO_BSRR_BS13;
							count =0;
							led_high = 0;
						}
				}
				count++;
				lcd_put_cur(0, 0);
				lcd_send_string("TRANGTHAI:");
				lcd_put_cur(0, 10);
				if(is_falled == 1) {
						lcd_send_string("FALLED");
				} else {
						lcd_send_string("NORMAL");
				}
				lcd_put_cur(1, 0);
				lcd_send_string("MAX:");
				lcd_put_cur(1,4);
				sprintf(tmp,"%.2f",max);
				lcd_send_string(tmp);
    }
}
void EXTI0_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR0) {
      sys_on = !sys_on; 
			EXTI->PR |= EXTI_PR_PR0;
    }
}
void LED_INIT(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;
}
void TIM2_config(void){
	//moi lan dem ton 1 micro sec
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//bat timer2
	TIM2->ARR |= 0xffff;//chon gia tri arr
	TIM2->PSC |= 72 - 1;//apb1 clock = sysclock = 72Mhz
	TIM2->CR1 |= 1;//bat counter
	while(!(TIM2->SR & (1<<0)));//cho update
}
void delayUs(uint32_t us){
	TIM2->CNT |= 0;
	while(TIM2->CNT);
}

void delayMs(uint32_t ms){
	for(uint32_t i = 0;i<ms;i++){
		delayUs(1000);//delay 1ms
	}
}
int main(void) {
	RCC_Configuration();
    LED_INIT();
    I2C_Init();
    MPU6050_Init();
    EXTI_Config();
	TIM1_Config();
	TIM2_config();
	lcd_init();
    while (1) {
		if (!sys_on) {
		TIM1->CR1 &= ~TIM_CR1_CEN; // Turn off TIM1
		sleep_mode();
		// config RCC after sleep_mode
		RCC_Configuration();
		TIM1->CR1 |= TIM_CR1_CEN; //turn on tim1
		}
	}
}
