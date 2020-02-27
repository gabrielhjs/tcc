/*
 * daeltiva.h
 *
 *  Created on: 8 de dez de 2017
 *      Author: Guilherme
 */

#ifndef DAELTIVA_H_
#define DAELTIVA_H_


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "lcd.h"
#include "driverlib/uart.h"
#include "uartstdio.h"

#define GPIO_PIN_ALL (0xFF)

//*****************************************************************************
//
//! \addtogroup KeyPad_api
//! @author Guilherme Luiz Moritz
//! @date 08 May 2018
//! @brief KeyPad Library for TivaDAELT
//!
//! @{
//
//*****************************************************************************

/**
 * @brief Keypad Pin Definition
 *
 * This typedef holds a pin definition for KeyPad Usage, see #Keypad_Init
 */
typedef struct pindefinition
{
    uint32_t ui32Port;  /**< Base location of the GPIO of the selected pin, using the macro from Texas Instruments (GPIO_PORTX_BASE) */
    uint8_t ui8Pin;  /**< Pin number on designated port from the gpio.h pin definitions (GPIO\_PIN\_X) */

} PinDefinition;

//! @}

//Retorna a versão da biblioteca
void TivaDaeltVersion();


//inicializa o gpio do kit para acionar os 8 leds do kit de desenvolvimento e o display de 7 segmentos
//certifique-se que os jumpers P18 e P19 estão inseridos
void init_leds(void);


void Delay_ms(uint32_t time_in_ms);
void Delay_us(uint32_t time_in_us);


//ADC
uint16_t ADC_Get_Sample(uint32_t channel);

//PWM
//O PWM configurado: PF2 (M1PWM6)
void PWM1_Init(uint32_t freq);
void PWM1_Set_Duty(uint32_t duty_ratio);
void PWM1_Start(void);
void PWM1_Stop(void);

//O PWM configurado: PF3 (M1PWM7)
void PWM2_Init(uint32_t freq);
void PWM2_Set_Duty(uint32_t duty_ratio);
void PWM2_Start(void);
void PWM2_Stop(void);

//Timer configurado: PC4 (WT0CCP0)
void PWM3_PWM4_Init(uint32_t freq3, uint32_t freq4);
void PWM3_Set_Duty(uint32_t duty_ratio);
void PWM3_Start(void);
void PWM3_Stop(void);

//Timer configurado: PC5 (WT0CCP1)
void PWM4_Set_Duty(uint32_t duty_ratio);
void PWM4_Start(void);
void PWM4_Stop(void);

//Botões
uint16_t Button(uint32_t ui32Port, uint8_t ui8Pin, uint16_t time, uint16_t active_state);

//LCD
void Lcd_Init();
void Lcd_Out(uint8_t row, uint8_t column, uint8_t *text);
void Lcd_Out_Cp(uint8_t *text);
void Lcd_Chr(uint8_t row, uint8_t column, uint8_t out_char);
void Lcd_Chr_Cp(uint8_t out_char);
void Lcd_Cmd(uint8_t out_char);

//UART
int UART_Init(uint32_t ui32Base, uint32_t baudRate, uint32_t dataLen, uint32_t stopBits, uint32_t parity);
void UART_Out_Cp(uint32_t ui32Base, uint8_t *text, uint32_t ui32Len);
void UART_Chr_Cp(uint32_t ui32Base, uint8_t out_char);
uint8_t UART_Chr_Get(uint32_t ui32Base);
void UART_Str_Get(uint32_t ui32Base, uint8_t *buf, uint8_t bufSize);

//Porta Serial
void ConfigureUART(void);
float Get_Float(void);
int32_t Get_Int(void);
void Print_Float(float numero);


//Keypad
void Keypad_Init(PinDefinition* ColumnPinDefinitions, PinDefinition* RowPinDefinitions);
uint8_t Keypad_Key_Press(void);
uint8_t Keypad_Key_Click(void);


#endif /* DAELTIVA_H_ */
