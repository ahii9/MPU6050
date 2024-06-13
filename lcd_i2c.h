#include"stm32f10x.h"
// Constants
#define SLAVE_HMC5883L 0x0C
#define SLAVE_ADDRESS_LCD 0x27
#define I2C_WRITE 0
#define I2C_READ 1
void I2C_Start(void);
void I2C_Write(uint8_t data);
void I2C_Address(uint8_t address, uint8_t direction);
void I2C_Stop(void);
void I2C_WriteMul(uint8_t *data, uint8_t size);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_init(void);
void lcd_send_string(char *str);
void Delay_us(uint32_t us);
void Delay_Config(void);
void Delay_ms(uint32_t ms);
void I2C_Read(uint8_t address, uint8_t *data, uint8_t size);
uint8_t I2C_Read_Ack(void);
uint8_t I2C_Read_Nack(void);