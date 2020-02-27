//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "daelttiva/daelttiva.h"

//*****************************************************************************
// Definições das interfaces da placa Tiva
//*****************************************************************************
#define RED_LED   GPIO_PIN_1    //PF1
#define BLUE_LED  GPIO_PIN_2    //PF2
#define GREEN_LED GPIO_PIN_3    //PF3
#define SW1       GPIO_PIN_4    //PF4
#define SW2       GPIO_PIN_0    //PF0
float x = 0;
//*****************************************************************************
// Parametros para o módulo PWM

// Frequencia do MCU = 50 MHz
// Divisor para o PWM = 16
//*****************************************************************************
uint32_t periodo = (50e6/16)/50;    // Frequência de 50 Hz
uint32_t teste;

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif



//*****************************************************************************
//
// Função para inicilizar o modulo PWM
//
//*****************************************************************************

void Init_PWM (void){
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16); //50/16 = 3,125MHz

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_2,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, periodo);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 0);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodo);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);

    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT|PWM_OUT_6_BIT|PWM_OUT_5_BIT, true);
    //PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
    //PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT|PWM_OUT_5_BIT, false);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
}

//*****************************************************************************
//
// Rotina de atraso
//
//*****************************************************************************

void delay_ms (uint16_t atraso){

    if (atraso != 0)
        SysCtlDelay(atraso*(SysCtlClockGet()/(3*1000))); // Atraso de 1s
}

//*****************************************************************************
//
// Função para teste do módulo PWM.
//
//*****************************************************************************

void teste_pwm (void){
   uint32_t current_duty = 0; // initial value for current_duty

   Init_PWM();

   while(1){
       delay_ms(10);
       /*if (current_duty < periodo)
           current_duty+=500;
       else
           current_duty = 0;
       */
       x += 0.02*(2*M_PI);
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, (periodo/2)*((1.1+sin(x))/2));
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (periodo/2)*((1.1+sin(x+(2*M_PI/3)))/2));
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, (periodo/2)*((1.1+sin(x+(4*M_PI/3)))/2));
       }
 }



//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int main(void)
{
    volatile uint32_t ui32Loop;

    // Habilita clock geral do sistema para rodar em 50 MHz a partir do PLL com cristal
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Habilita e espera o acesso ao PORTF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    // Configura o GPIOF para operação com LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);

    // Configura os pinos da placa MICROTIVA_DAELT
    init_leds();             // habilita LEDs da placa MICROTIVA_DAELT

    // Configura os dois pinos para leitura do estado das chaves SW1 e SW2
    // ***Deixar a função comentada para aplicações com os displays de 7 segmentos
    // ButtonsInit();        // habilita o botão SW2 da placa TIVA.

    ConfigureUART();         // habilita a UART
    UARTprintf("Disciplina de Sistemas Microcontrolados\n");

    teste_pwm();
}
