#ifndef LCD_H_
#define LCD_H_



#define _LCD_FIRST_ROW  0xAA//Move cursor to the 1st row
#define _LCD_SECOND_ROW 0xBB//Move cursor to the 2nd row
#define _LCD_THIRD_ROW  0xCC//Move cursor to the 3rd row
#define _LCD_FOURTH_ROW 0xDD//Move cursor to the 4th row
#define _LCD_CLEAR 0x1 //Clear display
#define _LCD_RETURN_HOME 0x2 //Return cursor to home position, returns a shifted display to its original position. Display data RAM is unaffected.
#define _LCD_CURSOR_OFF 0x0C //Turn off cursor
#define _LCD_UNDERLINE_ON 0x0E  //Underline cursor on
#define _LCD_BLINK_CURSOR_ON  0x0F //  Blink cursor on
#define _LCD_MOVE_CURSOR_LEFT  0x10 //Move cursor left without changing display data RAM
#define _LCD_MOVE_CURSOR_RIGHT 0x14 //Move cursor right without changing display data RAM
#define _LCD_TURN_ON    0x0C //Turn Lcd display on
#define _LCD_TURN_OFF  0x08 //Turn Lcd display off
#define _LCD_SHIFT_LEFT 0x18 //Shift display left without changing display data RAM
#define _LCD_SHIFT_RIGHT 0x1E //Shift display right without changing display data RAM

void lcd_write(unsigned char c);
void lcd_init(void);
void lcd_clear(void);
void lcd_puts(const char * s);
void lcd_putch(char c);
void lcd_goto(unsigned char pos);
void lcd_cmd(uint8_t cmd);

#endif /* DAELTIVA_H_ */
