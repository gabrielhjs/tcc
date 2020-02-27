/*
 * lcd.c
 *
 *  Created on: 15 de dez de 2017
 *      Author: Guilherme
 */

#include "daelttiva.h"

/*
 *   LCD interface example
 *   Uses routines from delay.c
 *   This code will interface to a standard LCD controller
 *   like the Hitachi HD44780. It uses it in 4 bit mode, with
 *   the hardware connected as follows (the standard 14 pin
 *   LCD connector is used):
 *
 *   PORTB bits 0-3 are connected to the LCD data bits 4-7 (high nibble)
 *   PORTA bit 2 is connected to the LCD RS input (register select)
 *   PORTA bit 3 is connected to the LCD EN bit (enable)
 *
 *   To use these routines, set up the port I/O (TRISA, TRISB) then
 *   call lcd_init(), then other routines as required.
 *
 */

#define LCD_DATA_SYSCTL SYSCTL_PERIPH_GPIOA
#define LCD_DATA_PORT GPIO_PORTA_BASE
#define LCD_DATA_SHIFT (4) //os dados do lcd são 4 pinos consecutivos, considerando que a porta tem 8 bits, qual é a distância do shiftleft na porta para os dados ficarem alinhados com o pinos conectados

#define LCD_EN_SYSCTL SYSCTL_PERIPH_GPIOA
#define LCD_EN_PORT GPIO_PORTA_BASE
#define LCD_EN_PIN GPIO_PIN_3

#define LCD_RS_SYSCTL SYSCTL_PERIPH_GPIOA
#define LCD_RS_PORT GPIO_PORTA_BASE
#define LCD_RS_PIN GPIO_PIN_2


//comanda o RS do display
// Register select
inline void LCD_RS(uint8_t value)
{
    GPIOPinWrite(LCD_RS_PORT, LCD_RS_PIN, value?0xFF:0x0);
}

//comanda o RS do display
// Enable
inline void LCD_EN(uint8_t value)
{
    GPIOPinWrite(LCD_EN_PORT, LCD_EN_PIN, value?0xFF:0x0);
}

//escreve dados no display
inline void LCD_DATA(uint8_t value)
{
    GPIOPinWrite(LCD_DATA_PORT, 0b1111<<LCD_DATA_SHIFT, value<<LCD_DATA_SHIFT);
}

inline void LCD_STROBE()
{
    LCD_EN(1);
    SysCtlDelay(3000);
    LCD_EN(0);
}


void
lcd_write(unsigned char c)
{
    LCD_DATA(c >> 4);
    LCD_STROBE();
    LCD_DATA(c & 0x0F);
    LCD_STROBE();
    SysCtlDelay(1500);
}

/* initialise the LCD - put into 4 bit mode */
void
lcd_init(void)
{
    //habilita o periféricos do pino do LCD
    SysCtlPeripheralEnable(LCD_DATA_SYSCTL);
    SysCtlPeripheralEnable(LCD_EN_SYSCTL);
    SysCtlPeripheralEnable(LCD_RS_SYSCTL);

    //configura todos os pinos do LCD como saídas
    GPIOPinTypeGPIOOutput(LCD_EN_PORT, LCD_EN_PIN);
    GPIOPinTypeGPIOOutput(LCD_RS_PORT, LCD_RS_PIN);
    GPIOPinTypeGPIOOutput(LCD_DATA_PORT, 0b1111<<LCD_DATA_SHIFT);

    LCD_RS(0); // write control bytes
    Delay_ms(15);    // power on delay
    LCD_DATA(0x3);    // attention!
    LCD_STROBE();
    Delay_ms(5);
    LCD_STROBE();
    SysCtlDelay(3000);
    LCD_STROBE();
    Delay_ms(5);
    LCD_DATA(0x2);    // set 4 bit mode
    LCD_STROBE();
    SysCtlDelay(1500);
    lcd_write(0x28);    // 4 bit mode, 1/16 duty, 5x8 font
    lcd_write(0x08);    // display off
    lcd_write(0x0F);    // display on, blink curson on
    lcd_write(0x06);    // entry mode
}

void lcd_cmd(uint8_t cmd)
{
       LCD_RS(0);
       lcd_write(cmd);
}

/*
 * Clear and home the LCD
 */

void
lcd_clear(void)
{
    LCD_RS(0);
    lcd_write(0x1);
    Delay_ms(2);
}

/* write a string of chars to the LCD */

void
lcd_puts(const char * s)
{
    uint32_t i=0;
    LCD_RS(1); // write characters
    while(*s)
    {
        lcd_write(*s++);
        i++;
        if(i>100)
        {
            break;
        }
    }
}

/* write one character to the LCD */

void
lcd_putch(char c)
{
    LCD_RS(1); // write characters
    lcd_write(c);
}


/*
 * Go to the specified position
 */

void
lcd_goto(unsigned char pos)
{
    LCD_RS(0);
    lcd_write(0x80+pos);
}




