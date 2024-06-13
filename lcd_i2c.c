#include"lcd_i2c.h"
void I2C_Start(void)
{
    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C_Write(uint8_t data)
{
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2C_Address(uint8_t address, uint8_t direction)
{
    uint8_t addr = (address << 1) | direction;
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void) I2C1->SR1;
    (void) I2C1->SR2;
}

void I2C_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_WriteMul(uint8_t *data, uint8_t size)
{
    while (size--)
    {
        I2C_Write(*data++);
    }
}

uint8_t I2C_Read_Ack(void)
{
    I2C1->CR1 |= I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

uint8_t I2C_Read_Nack(void)
{
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C_Stop();
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

void I2C_Read(uint8_t address, uint8_t *data, uint8_t size)
{
    I2C_Start();
    I2C_Address(address, I2C_READ);
    while (size--)
    {
        if (size == 0)
        {
            *data = I2C_Read_Nack();
        }
        else
        {
            *data = I2C_Read_Ack();
        }
        data++;
    }
}

/* LCD Functions */

void LCD_Write(uint8_t address, uint8_t *data, int size)
{
    I2C_Start();
    I2C_Address(address, I2C_WRITE);
    I2C_WriteMul(data, size);
    I2C_Stop();
}

void lcd_send_cmd(char cmd)
{
    char data_u = cmd & 0xF0;
    char data_l = (cmd << 4) & 0xF0;
    uint8_t data_t[4] = {
        data_u | 0x0C, data_u | 0x08,
        data_l | 0x0C, data_l | 0x08
    };
    LCD_Write(SLAVE_ADDRESS_LCD, data_t, 4);
}

void lcd_send_data(char data)
{
    char data_u = data & 0xF0;
    char data_l = (data << 4) & 0xF0;
    uint8_t data_t[4] = {
        data_u | 0x0D, data_u | 0x09,
        data_l | 0x0D, data_l | 0x09
    };
    LCD_Write(SLAVE_ADDRESS_LCD, data_t, 4);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x80);
    for (int i = 0; i < 70; i++)
    {
        lcd_send_data(' ');
    }
}

void lcd_put_cur(int row, int col)
{
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_send_cmd(addr);
}

void lcd_init(void)
{
    Delay_ms(50);
    lcd_send_cmd(0x30);
    Delay_ms(5);
    lcd_send_cmd(0x30);
    Delay_us(150);
    lcd_send_cmd(0x30);
    Delay_ms(10);
    lcd_send_cmd(0x20);

    lcd_send_cmd(0x28);
    Delay_ms(1);
    lcd_send_cmd(0x08);
    Delay_ms(1);
    lcd_send_cmd(0x01);
    Delay_ms(1);
    lcd_send_cmd(0x06);
    Delay_ms(1);
    lcd_send_cmd(0x0C);
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}

/* Delay Functions */

void Delay_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->ARR = 0xFFFF;
    TIM2->PSC = 71;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM2->EGR |= TIM_EGR_UG;
}

void Delay_us(uint32_t us)
{
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

void Delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        Delay_us(1000);
    }
}