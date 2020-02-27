/****************************************************************************
 * Copyright (C) 2018 by Guilherme Luiz Moritz e Ohara Kerusauskas Rayel    *
 *                                                                          *
 * Bibliotecas do DAELTTiva                                                 *
 *                                                                          *
 *   AQUI DEPOIS SERÁ ADICIONADO ALGUM TEXTO DE COPYRIGHT                   *
 ****************************************************************************/

 
//*****************************************************************************
//
//! \addtogroup daelttiva_api
//! @author Guilherme Luiz Moritz
//! @date 16 Mar 2018
//! @brief Implementação da biblioteca do DaeltTiva.
//!
//! Aqui explicações mais detalhadas sobre o arquivo vão ser adicionadas
//! Por ora apenas fica o site da disciplina
//! @see http://paginapessoal.utfpr.edu.br/moritz
//! @{
//
//*****************************************************************************

#include "daelttiva.h"

/**
 * @brief Versão da biblioteca DAELTTiva
 *
 * Retorna, pela UART0, a versão e a data da biblioteca DAELTTiva utilizada
 * Uma boa prática é chamá-la no início de todo código realizado
 */
void TivaDaeltVersion()
{
    //uint32_t* USR_REG = (int32_t*)0x400FE1E0;
    UARTprintf("TivaDAELT: v0.4 2018-05-08\n");
    //UARTprintf("User Regs: 0x%x 0x%x 0x%x 0x%x\n",USR_REG[0],USR_REG[1],USR_REG[2],USR_REG[3]);
}



void init_leds(void)
{

    //os leds e os displays de 7 segmentos estão ligados na porta b

    //habilita o periférico GPIOB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //tanto os displays de 7 segmentos quanto os LCDs possuem um transistor de enable
    //Display unidade - PD6
    //Display dezena - PF4
    //Leds - PE0
    //habilita os periféricos correspondentes GPIOD, GPIOE, GPIOF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //configura todos os pinos dos leds e enables como saídas
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_ALL);

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);


    //desliga todos os pinos
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_ALL, 0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);

}

//! @}
//*****************************************************************************
//
//! \addtogroup ADC_api
//! @author Guilherme Luiz Moritz
//! @date 16 Mar 2018
//! @brief ADC Library for TivaDAELT
//!
//! @{
//
//*****************************************************************************

/**
 * @brief Aquire analog value from the specified channel
 *
 * The function aquires analog value from the specified channel.
 * Usage:
 * @code
 * uint16_t adc_value;
 * float adc_voltage;
 *
 * void main() {
 *    //Enable System PLL @ 80MHz - ADC only works if PLL is on (unless some extra steps are taken)
 *    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); *
 *
 *    // Enable the GPIO port that is used for the button
 *    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
 *
 *    // Check if the peripheral access is enabled.
 *    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
 *
 *    //Configure UART
 *    ConfigureUART();
 *
 *  do {
 *    if(Button(GPIO_PORTF_BASE, GPIO_PIN_4, 10, 0)) { // Detect logical one
 *        adc_value = ADC_Get_Sample(ADC_CTL_CH0); //gets analog value from PE3
 *        adc_voltage = ((float)adc_value*3.3)/4095.0;
 *        UARTprintf("Converted value from CH0 is ");
 *        Print_Float(adc_voltage);
 *        UARTprintf(" volts (%d binary)", adc_value);
 *        Delay_ms(300);
 *    }
 *    if(Button(GPIO_PORTF_BASE, GPIO_PIN_0, 10, 0)) { // Detect logical one
 *        adc_value = ADC_Get_Sample(ADC_CTL_CH2); //gets analog value from PE1
 *        adc_voltage = ((float)adc_value*3.3)/4095.0;
 *        UARTprintf("Converted value from CH2 is ");
 *        Print_Float(adc_voltage);
 *        UARTprintf(" volts (%d binary)", adc_value);
 *        Delay_ms(300);
 *    }
 *  } while(1);                                    // Endless loop
 * }
 * @endcode
 * @param channel Represents the channel from which the analog value is to be acquired. Refer to the adc.h (from TivaWare) input definitions (ADC_CTL_CHX) macros.
 * You can use the macro ADC_CTL_TS to read TM4C123 core temperature as well. To convert the raw value to degrees celcius you can use coretemp = (147.5 - ((247.5 * Return_value)) / 4096.0)
 * @return 12-bit unsigned value read from the specified channel
 * @note This function uses ADC0 from TM4C123, so it only works with ADC_CTL_CH0 to ADC_CTL_CH11 macros from TivaWare
 * @note This function only works with PLL on. To operate the ADC without PLL, additional steps not taken by this driver must be performed. Please refer to section 13.3.2.6 of TM4C123GH6PM datasheet for more information.
 */
uint16_t ADC_Get_Sample(uint32_t channel)
{
    //
    // Habilita o ADC0
    //
    SysCtlPeripheralEnable (SYSCTL_PERIPH_ADC0);
    uint32_t ui32Value[8];

    //
    // Wait for the ADC0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }

    //configura o AD para capturar o valor do canal especificado quando ocorrer um trigger de processador
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | channel);
    ADCSequenceEnable(ADC0_BASE, 0);

    //configura a interrupção do ADC
    ADCIntClear(ADC0_BASE, 0);
    //ADCIntRegister(ADC0_BASE, 0, &ADCConversionDone);

    //ADCIntEnable(ADC0_BASE, 0);
    ADCSequenceEnable(ADC0_BASE, 0);

    //
    // Trigger the sample sequence.
    //
    ADCProcessorTrigger(ADC0_BASE, 0);
    //
    // Wait until the sample sequence has completed.
    //
    while(!ADCIntStatus(ADC0_BASE, 0, false))
    {
    }
    //
    // Read the value from the ADC.
    //
    ADCSequenceDataGet(ADC0_BASE, 0, ui32Value);
    return ui32Value[0];
}


//! @}
//*****************************************************************************
//
//! \addtogroup Delay_api
//! @author Guilherme Luiz Moritz
//! @date 16 Mar 2018
//! @brief Timer Delays for TivaDAELT
//!
//! @{
//
//*****************************************************************************



volatile int32_t timerwait = 1;

/**
 * @brief Timer Delay Interrupt Service Routine
 *
 * This interrupt is called to change timerwait variable (used by timer delays) and is intended to internal use only
 */
void DelayInterrupt()
{
    TimerIntClear(WTIMER5_BASE, TIMER_TIMA_TIMEOUT);
    timerwait = 0;
}

/**
 * @brief Timer Delay
 *
 * Creates a timer delay in duration of time_in_us microsseconds.
 * This function uses WTIMER5 - code cannot use this timer if the function is used
 * @param time_in_us Time for the Delay in microsseconds
 * @warning This function cannot be used on interrupt service routines and is not thread safe
 * @warning This function uses WTIMER5 - code cannot use this timer if the function is used
 */
void Delay_us(uint32_t time_in_us)
{
    uint64_t ticks = (time_in_us / (1.0/SysCtlClockGet()))/1000000;

    // Enable the WTimer5 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);

    // Wait for the WTimer5 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER5));

    TimerConfigure(WTIMER5_BASE, TIMER_CFG_ONE_SHOT);

    //Set the count time for the one shot timer
    TimerLoadSet64(WTIMER5_BASE, ticks);

    //Register the interrupt
    TimerIntClear(WTIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(WTIMER5_BASE, TIMER_A, DelayInterrupt);
    TimerIntEnable(WTIMER5_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(WTIMER5_BASE, TIMER_A);
    timerwait = 1;
    while(timerwait);
}


/**
 * @brief Timer Delay
 *
 * Creates a timer delay in duration of time_in_ms milliseconds.
 * @param time_in_ms Time for the Delay in milliseconds
 * @warning This function cannot be used on interrupt service routines and is not thread safe
 * @warning This function uses WTIMER5 - code cannot use this timer if the function is used
 */
void Delay_ms(uint32_t time_in_ms)
{
    uint64_t ticks = (time_in_ms / (1.0/SysCtlClockGet()))/1000;

    // Enable the WTimer0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);

    // Wait for the WTimer0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER5));

    // Configure TimerA as a half width periodic timer
    TimerConfigure(WTIMER5_BASE, TIMER_CFG_ONE_SHOT);

    //Set the count time for the the periodic timer (TimerA).
    TimerLoadSet64(WTIMER5_BASE, ticks);

    //Register the interrupt
    TimerIntClear(WTIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(WTIMER5_BASE, TIMER_A, DelayInterrupt);
    TimerIntEnable(WTIMER5_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(WTIMER5_BASE, TIMER_A);
    timerwait = 1;
    while(timerwait);
}

//! @}
//*****************************************************************************
//
//! \addtogroup PWM_api
//! @author Guilherme Luiz Moritz
//! @date 16 Mar 2018
//! @briefPWM Library for TivaDAELT
//!
//! @{
//
//*****************************************************************************



/**
 * @brief Initializes PWM1
 *
 * Initializes the PWM1, Generator 3, output A. With Duty Ratio 0.
 * This Output (PF2 - M1PWM6) is tied to EN1 from the half bridge from TivaDaelt as well on Header P2
 * Usage:
 * @code
 *  PWM1_Init(15000);                    // Initialize PWM1 module at 15KHz
 *  PWM2_Init(15000);                    // Initialize PWM2 module at 15KHz
 *
 *  uint32_t current_duty[2]  = {0,0};                // initial value for current_duty
 *
 *
 *  PWM1_Start(); // start PWM1
 *  PWM2_Start(); // start PWM2
 *
 *  PWM1_Set_Duty(current_duty[0]); // Set current duty for PWM1
 *  PWM2_Set_Duty(current_duty[1]); // Set current duty for PWM2
 *  while (1) {
 *  if (Button(GPIO_PORTF_BASE, GPIO_PIN_0, 5, 0)) //button on SW1 pressed
 *  {
 *      Delay_ms(100);
 *      current_duty[0] = (current_duty[0] + 1000) % 100000; // increment current_duty
 *      PWM1_Set_Duty(current_duty[0]);
 *  }
 *  if (Button(GPIO_PORTF_BASE, GPIO_PIN_4, 5, 0)) // button on SW2 pressed
 *  {
 *      Delay_ms(100);
 *      current_duty[1] = (current_duty[1] + 1000) % 100000; // increment current_duty
 *      PWM2_Set_Duty(current_duty[1]);
 *  }
 *
 *  Delay_ms(100);                      // slow down change pace a little
 * @endcode
 * @param freq Frequency in Hertz configured to the generator. The minimum frequency is limited to SysCtlClockGet()/65536. Note that due hardware limitations this value is always same used in PWM2. If the function PWM2_Init is called, the frequency of PWM1 will be configured as well
 */
void PWM1_Init(uint32_t freq)
{
    //Calculate the number of ticks for the desired frequency
    int32_t Cycle=(SysCtlClockGet()/freq);
    //
    // Enable the GPIO Peripheral used by PWM1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure GPIO Pins for PWM6
    //
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //Habilita o PWM1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));
    //
    // Configure the PWM generator for count down mode with immediate updates
    // to the parameters.
    //
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);
    //
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
    // microseconds. For a 50 MHz clock, this translates to 1000 clock ticks.
    // Use this value to set the period.
    //
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, Cycle);

    //
    // Set the pulse width of PWM0 for a 0% duty cycle.
    //
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1);
    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    //
    // Disable the outputs.
    //
    PWMOutputInvert(PWM1_BASE, (PWM_OUT_6_BIT), false);
    PWMOutputState(PWM1_BASE, (PWM_OUT_6_BIT), false);
}

/**
 * @brief Initializes PWM2
 *
 * Initializes the PWM2, Generator 3, output B. With Duty Ratio 0.
 * This Output (PF3 - M1PWM7) is tied to EN2 from the half bridge from TivaDaelt as well on Header P2
 * Usage:
 * @code
 *  PWM1_Init(15000);                    // Initialize PWM1 module at 15KHz
 *  PWM2_Init(15000);                    // Initialize PWM2 module at 15KHz
 *
 *  uint8_t current_duty[2]  = {0,0};                // initial value for current_duty
 *
 *
 *  PWM1_Start(); // start PWM1
 *  PWM2_Start(); // start PWM2
 *
 *  PWM1_Set_Duty(current_duty[0]); // Set current duty for PWM1
 *  PWM2_Set_Duty(current_duty[1]); // Set current duty for PWM2
 *  while (1) {
 *  if (Button(GPIO_PORTF_BASE, GPIO_PIN_0, 5, 0)) //button on SW1 pressed
 *  {
 *      Delay_ms(100);
 *      current_duty[0] = (current_duty[0] + 1000) % 100000; // increment current_duty
 *      PWM1_Set_Duty(current_duty[0]);
 *  }
 *  if (Button(GPIO_PORTF_BASE, GPIO_PIN_4, 5, 0)) // button on SW2 pressed
 *  {
 *      Delay_ms(100);
 *      current_duty[1] = (current_duty[1] + 1000) % 100000; // increment current_duty
 *      PWM2_Set_Duty(current_duty[1]);
 *  }
 *
 *  Delay_ms(100);                      // slow down change pace a little
 * @endcode
 * @param freq Frequency in Hertz configured to the generator. The minimum frequency is limited to SysCtlClockGet()/65536. Note that due hardware limitations this value is always same used in PWM1. If the function PWM1_Init is called, the frequency of PWM2 will be configured as well
 */
void PWM2_Init(uint32_t freq)
{
    //Calculate the number of ticks for the desired frequency
    int32_t Cycle=(SysCtlClockGet()/freq);
    //
    // Enable the GPIO Peripheral used by PWM1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure GPIO Pins for PWM6
    //
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    //Habilita o PWM1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));
    //
    // Configure the PWM generator for count down mode with immediate updates
    // to the parameters.
    //
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);
    //
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
    // microseconds. For a 50 MHz clock, this translates to 1000 clock ticks.
    // Use this value to set the period.
    //
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, Cycle);

    //
    // Set the pulse width of PWM0 for a 0% duty cycle.
    //
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 1);
    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    //
    // Disable the outputs.
    //
    PWMOutputState(PWM1_BASE, (PWM_OUT_7_BIT), false);
}

/**
 * @brief Initializes PWM3 and PWM4
 *
 * Initializes the WTIMER0 to work as PWM, starting at Duty Ratio 0.
 * The WTIMER0 has two outputs that will be configured as PWM, (PC4 - WT0CCP0) is tied to servo motor connector P11, while PC5 (WT0CCP1) is tied to servo motor connector P12
 * Usage:
 * @code
 *  PWM3_PWM4_Init(50, 80);                    // Initialize PWM3 to 50Hz (servo) and PWM4 to 80Hz (test)
 *
 *  uint32_t current_duty[2]  = {0,0};                // initial value for current_duty
 *
 *
 *  PWM3_Start(); // start PWM3
 *  PWM4_Start(); // start PWM4
 *
 *  PWM2_Set_Duty(current_duty[0]); // Set current duty for PWM1
 *  PWM3_Set_Duty(current_duty[1]); // Set current duty for PWM2
 *  while (1) {
 *  if (Button(GPIO_PORTF_BASE, GPIO_PIN_0, 5, 0)) //button on SW1 pressed
 *  {
 *      Delay_ms(100);
 *      current_duty[0] = (current_duty[0] + 1000) % 100000; // increment current_duty
 *      PWM3_Set_Duty(current_duty[0]);
 *  }
 *  if (Button(GPIO_PORTF_BASE, GPIO_PIN_4, 5, 0)) // button on SW2 pressed
 *  {
 *      Delay_ms(100);
 *      current_duty[1] = (current_duty[1] + 1000) % 100000; // increment current_duty
 *      PWM4_Set_Duty(current_duty[1]);
 *  }
 *
 *  Delay_ms(100);                      // slow down change pace a little
 * @endcode
 * @param freq3 Frequency in Hertz configured to PWM3
 * @param freq4 Frequency in Hertz configured to PWM4
 */
void PWM3_PWM4_Init(uint32_t freq3, uint32_t freq4)
{
    //Calculate the number of ticks for the desired frequency
    int32_t Cycle3=(SysCtlClockGet()/freq3);
    int32_t Cycle4=(SysCtlClockGet()/freq4);

    //
    // Enable the GPIO Peripheral used by WTIMER0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    // Configure GPIO Pins for WTIMER
    //
    GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);

    GPIOPinConfigure(GPIO_PC5_WT0CCP1);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5);

    //Habilita o WTIMER0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0));

    //Configura o timer para modo PWM
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    TimerLoadSet(WTIMER0_BASE, TIMER_A, Cycle3);
    TimerLoadSet(WTIMER0_BASE, TIMER_B, Cycle4);
    //
    // Set the pulse width of WTIMER0 - for a 0% duty cycle.
    //
    TimerMatchSet(WTIMER0_BASE, TIMER_A, Cycle3-1);
    TimerMatchSet(WTIMER0_BASE, TIMER_B, Cycle4-1);

    PWM3_Stop();
    PWM4_Stop();
}

/**
 * @brief Sets PWM1 (PF2 - M1PWM6) duty ratio
 *
 * Sets PWM duty ratio.
 * @param duty_ratio This parameter takes values from 0 to 100000, where 0 is 0%, 50000 is 50%, and 100000 is 100% duty ratio. Other specific values for duty ratio can be calculated as (Percent*100000)/100.
 * @warning Before using this function, function PWM1_Init must be called
 */
void PWM1_Set_Duty(uint32_t duty_ratio)
{
    //lê o valor de Load do PWM1 generator 3, que determina o valor máximo que pode ser colocado em ticks
    int32_t Cycle = HWREG(PWM1_BASE + PWM_O_3_LOAD);

    //calcula o número de ticks de clock que o PWM deve ficar em alto para que se obtenha a duty_ratio configurada
    int32_t ticks = duty_ratio*Cycle / 100000;

    //verifica se o número calculado não é inválido e corrige de acordo
    if (ticks < 1)
    {
        ticks = 1;
    }
    else if(ticks > Cycle - 1)
    {
        ticks = Cycle - 1;
    }
    //configura o valor do PWM
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ticks);
}

/**
 * @brief Sets PWM2 (PF2 - M1PWM7) duty ratio
 *
 * Sets PWM duty ratio.
 * @param duty_ratio This parameter takes values from 0 to 100000, where 0 is 0%, 50000 is 50%, and 100000 is 100% duty ratio. Other specific values for duty ratio can be calculated as (Percent*100000)/100.
 * @warning Before using this function, function PWM2_Init must be called
 */
void PWM2_Set_Duty(uint32_t duty_ratio)
{
    //lê o valor de Load do PWM1 generator 3, que determina o valor máximo que pode ser colocado em ticks
    int32_t Cycle = HWREG(PWM1_BASE + PWM_O_3_LOAD);

    //calcula o número de ticks de clock que o PWM deve ficar em alto para que se obtenha a duty_ratio configurada
    int32_t ticks = duty_ratio*Cycle / 100000;

    //verifica se o número calculado não é inválido e corrige de acordo
    if (ticks < 1)
    {
        ticks = 1;
    }
    else if(ticks > Cycle - 1)
    {
        ticks = Cycle - 1;
    }
    //configura o valor do PWM
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ticks);
}

/**
 * @brief Sets PWM3 (PC4 - WT0CCP0) duty ratio
 *
 * Sets PWM duty ratio.
 * @param duty_ratio This parameter takes values from 0 to 100000, where 0 is 0%, 50000 is 50%, and 100000 is 100% duty ratio. Other specific values for duty ratio can be calculated as (Percent*100000)/100.
 * @warning Before using this function, function PWM3_PWM4_Init must be called
 */
void PWM3_Set_Duty(uint32_t duty_ratio)
{
    //lê o valor de Load WTIMER0-A, que determina o valor máximo que pode ser colocado em ticks
    int32_t Cycle = TimerLoadGet(WTIMER0_BASE, TIMER_A);

    //calcula o número de ticks de clock que o PWM deve ficar em alto para que se obtenha a duty_ratio configurada
    int32_t ticks = duty_ratio*Cycle / 100000;

    //O contador do pwm é decrescente, então precisamos inverter o número
    ticks = Cycle - ticks;

    //verifica se o número calculado não é inválido e corrige de acordo
    if (ticks < 1)
    {
        ticks = 1;
    }
    else if(ticks > Cycle - 1)
    {
        ticks = Cycle - 1;
    }
    //configura o valor do PWM
    TimerMatchSet(WTIMER0_BASE, TIMER_A, ticks);
}

/**
 * @brief Sets PWM4 (PC5 - WT0CCP1) duty ratio
 *
 * Sets PWM duty ratio.
 * @param duty_ratio This parameter takes values from 0 to 100000, where 0 is 0%, 50000 is 50%, and 100000 is 100% duty ratio. Other specific values for duty ratio can be calculated as (Percent*100000)/100.
 * @warning Before using this function, function PWM3_PWM4_Init must be called
 */
void PWM4_Set_Duty(uint32_t duty_ratio)
{
    //lê o valor de Load WTIMER0-B, que determina o valor máximo que pode ser colocado em ticks
    int32_t Cycle = TimerLoadGet(WTIMER0_BASE, TIMER_B);

    //calcula o número de ticks de clock que o PWM deve ficar em alto para que se obtenha a duty_ratio configurada
    int32_t ticks = duty_ratio*Cycle / 100000;

    //O contador do pwm é decrescente, então precisamos inverter o número
    ticks = Cycle - ticks;

    //verifica se o número calculado não é inválido e corrige de acordo
    if (ticks < 1)
    {
        ticks = 1;
    }
    else if(ticks > Cycle - 1)
    {
        ticks = Cycle - 1;
    }
    //configura o valor do PWM
    TimerMatchSet(WTIMER0_BASE, TIMER_B, ticks);
}


/**
 * @brief Starts PWM1 (PF2 - M1PWM6)
 *
 * Starts PWM1 (PF2 - M1PWM6)
 * @warning Before using this function, function PWM1_Init must be called
 */
void PWM1_Start(void)
{
    //
    // Enable the outputs.
    //
    PWMOutputState(PWM1_BASE, (PWM_OUT_6_BIT), true);
}

/**
 * @brief Stops PWM1 (PF2 - M1PWM6)
 *
 * Starts PWM1 (PF2 - M1PWM6)
 * @warning Before using this function, function PWM1_Init must be called
 */
void PWM1_Stop(void)
{
    //
    // Disable the outputs.
    //
    PWMOutputState(PWM1_BASE, (PWM_OUT_6_BIT), false);
}

/**
 * @brief Starts PWM2 (PF3 - M1PWM7)
 *
 * Starts PWM2 (PF3 - M1PWM7)
 * @warning Before using this function, function PWM2_Init must be called
 */
void PWM2_Start(void)
{
    //
    // Enable the outputs.
    //
    PWMOutputState(PWM1_BASE, (PWM_OUT_7_BIT), true);
}

/**
 * @brief Stops PWM2 (PF3 - M1PWM7)
 *
 * Stops PWM2 (PF3 - M1PWM7)
 * @warning Before using this function, function PWM2_Init must be called
 */
void PWM2_Stop(void)
{
    //
    // Disable the outputs.
    //
    PWMOutputState(PWM1_BASE, (PWM_OUT_7_BIT), false);
}


/**
 * @brief Starts PWM3 (PC4 - WT0CCP0)
 *
 * Starts PWM3 (PC4 - WT0CCP0)
 * @warning Before using this function, function PWM3_PWM4_Init must be called
 */
void PWM3_Start(void)
{
    //
        // Start the timer
        //
        TimerEnable(WTIMER0_BASE, TIMER_A);
}

/**
 * @brief Stops PWM3 (PC4 - WT0CCP0)
 *
 * Stops PWM3 (PC4 - WT0CCP0)
 * @warning Before using this function, function PWM3_PWM4_Init must be called
 */
void PWM3_Stop(void)
{
    //
    // Stop the timer
    //
    TimerDisable(WTIMER0_BASE, TIMER_A);
}

/**
 * @brief Starts PWM4 (PC4 - WT0CCP0)
 *
 * Stops PWM4 (PC4 - WT0CCP0)
 * @warning Before using this function, function PWM3_PWM4_Init must be called
 */
void PWM4_Start(void)
{
    //
        // Start the timer
        //
        TimerEnable(WTIMER0_BASE, TIMER_B);
}

/**
 * @brief Stops PWM4 (PC5 - WT0CCP1)
 *
 * Stops PWM4 (PC5 - WT0CCP1)
 * @warning Before using this function, function PWM3_PWM4_Init must be called
 */
void PWM4_Stop(void)
{
    //
    // Stop the timer
    //
    TimerEnable(WTIMER0_BASE, TIMER_B);
}

//! @}
//*****************************************************************************
//
//! \addtogroup Button_api
//! @author Guilherme Luiz Moritz
//! @date 16 Mar 2018
//! @brief Button Library for TivaDAELT
//!
//! @{
//
//*****************************************************************************




/**
 * @brief Returns pin number from pin mask.
 *
 * TivaWare Peripheral Driver Library has a bug in GPIODirModeGet() and GPIOPadConfigGet()
 * Instead of receiving the bit packed pin mask it must receive the pin number
 * This function returns the pin number when it receives the bit packed mask
 * @param ui8Pin  Pin number on designated port from the gpio.h pin definitions (GPIO\_PIN\_X)
 * @returns the pin number such as 1<<(pin number) equals to the bit packed pin mask
 */
static uint8_t
GetPinNumber(uint8_t ui8Pin)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        if(ui8Pin & 1<<i)
        {
            return i;
        }
    }
    return 0;
}


/**
 * @brief Button debounce.
 *
 * Function eliminates the influence of contact flickering upon pressing a button (debouncing).
 * @warning This function uses WTIMER5. This timer should not be used by the software (unpredictable behavior can occur)
 * Usage:
 * @code
 * uint8_t oldstate;  // Old state flag
 * void main() {
 *
 *    //
 *    // Enable the GPIO port that is used for the on-board LED.
 *    //
 *    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
 *
 *    //
 *    // Check if the peripheral access is enabled.
 *    //
 *    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
 *
 *  do {
 *    if(Button(GPIO_PORTF_BASE, GPIO_PIN_4, 1, 0)) { // Detect logical one
 *      oldstate = 1;                              // Update flag
 *    }
 *    if (oldstate && Button(GPIO_PORTF_BASE, GPIO_PIN_4, 1, 0)) {   // Detect one-to-zero transition
 *      UARTprintf("PF4 was pressed");
 *      oldstate = 0;                              // Update flag
 *    }
 *  } while(1);                                    // Endless loop
 * }
 * @endcode
 * @param ui32Port Base location of the GPIO where button is connected, using the macro from Texas Instruments (GPIO_PORTX_BASE)
 * @param ui8Pin  Pin number on designated port from the gpio.h pin definitions (GPIO\_PIN\_X)
 * @param time Debounce period in milliseconds
 * @param active_state can be either 0 or 1, and it determines if the button is active upon logical zero or logical one
 * @return 255 if the pin was in the active state for given period, 0 otherwise
 */
uint16_t Button(uint32_t ui32Port, uint8_t ui8Pin, uint16_t time, uint16_t active_state)
{
    //lê o botão 'tries' vezes durante 'time' milissegundos
    //o botão deve passar pelo menos a metade final do período em active_state
    uint32_t i,tries = 10;
    uint8_t pressed = 0, unpressed = 0;
    uint16_t ret = 0;
    uint32_t delay_us = time*100;

    if(ui32Port==GPIO_PORTF_BASE&&(ui8Pin&GPIO_PIN_0))
    {
        // Unlock PF0 so we can change it to a GPIO input
        // Once we have enabled (unlocked) the commit register then re-lock it
        // to prevent further changes.  PF0 is muxed with NMI thus a special case.
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    }

    //salva o contexto do pino
    //A função GetPinNumber foi necessária porque há um bug no tivaware
    uint32_t old_dirmode = GPIODirModeGet(ui32Port, GetPinNumber(ui8Pin));
    uint32_t old_padtype, old_pinstrength, old_value;

    if(old_dirmode == GPIO_DIR_MODE_OUT)
    {
        old_value = GPIOPinRead(ui32Port, ui8Pin);
    }

    GPIOPadConfigGet(ui32Port, GetPinNumber(ui8Pin), &old_pinstrength, &old_padtype);

    for(i=0;i<tries;i++)
    {
    //configura o pino como entrada
    GPIODirModeSet(ui32Port, ui8Pin, GPIO_DIR_MODE_IN);
    //se o estado ativo for 0, configura pulldown, senão, pullup
    if(active_state)
    {
        GPIOPadConfigSet(ui32Port, ui8Pin, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);
    }
    else
    {
        GPIOPadConfigSet(ui32Port, ui8Pin, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
    }

        if(GPIOPinRead(ui32Port, ui8Pin))
        {
            pressed++;
            unpressed=0;
        }
        else
        {
            unpressed++;
            pressed = 0;
        }

        //restaura o contexto
        GPIODirModeSet(ui32Port, ui8Pin, old_dirmode);
        GPIOPadConfigSet(ui32Port, ui8Pin, old_pinstrength, old_padtype);
        if(old_dirmode == GPIO_DIR_MODE_OUT)
        {
            GPIOPinWrite(ui32Port, ui8Pin, old_value);
        }
        Delay_us(delay_us);
    }
    if(active_state)
    {
        if (pressed>(tries/2))
        {
            ret = 0xFF;
        }

    }
    else
    {
        if (unpressed>(tries/2))
        {
            ret = 0xFF;
        }
    }
    //restaura o contexto
    //GPIODirModeSet(ui32Port, ui8Pin, old_dirmode);
    //GPIOPadConfigSet(ui32Port, ui8Pin, old_pinstrength, old_padtype);
    //if(old_dirmode == GPIO_DIR_MODE_OUT)
    //{
    //    GPIOPinWrite(ui32Port, ui8Pin, old_value);
    //}
    return ret;

}

//! @}
//*****************************************************************************
//
//! \addtogroup LCD_api
//! @author Guilherme Luiz Moritz
//! @date 16 Mar 2018
//! @brief LCD Library for TivaDAELT
//!
//! @{
//
//*****************************************************************************

/**
 * @brief Initializes Lcd module.
 *
 * Initializes Lcd module..
 * @warning This function uses WTIMER5. This timer should not be used by the software (unpredictable behavior can occur)
 * Usage:
 * @code
 * void main() {
 * 	Lcd_Init();
 * 	Lcd_Cmd(_LCD_CLEAR);
 * 	Lcd_Cmd(_LCD_CURSOR_OFF);
 * 	Lcd_Out(1, 1, "Hello DAELT TIVA");
 * }
 * @endcode
 */
void Lcd_Init()
{
    lcd_init();
}

/**
 * @brief Prints text on Lcd starting from specified position. Both string variables and literals can be passed as a text.
 *
 * Prints text on Lcd starting from specified position. Both string variables and literals can be passed as a text.
 * @warning This function uses WTIMER5. This timer should not be used by the software (unpredictable behavior can occur)
 * @param row starting position row number
 * @param column starting position column number
 * @param text text to be written
 */
void Lcd_Out(uint8_t row, uint8_t column, uint8_t *text)
{
    uint8_t string[17];
    memcpy(string,text,16);
    string[16]=0;
    lcd_goto(64*(row-1)+(column-1));
    lcd_puts((char *)string);
}

/**
 * @brief Prints text on Lcd at current cursor position. Both string variables and literals can be passed as a text.
 *
 * Prints text on Lcd at current cursor position. Both string variables and literals can be passed as a text.
 * @warning This function uses WTIMER5. This timer should not be used by the software (unpredictable behavior can occur)
 * @param text text to be written
 */
void Lcd_Out_Cp(uint8_t *text)
{
    lcd_puts((char *)text);
}

/**
 * @brief Prints character on Lcd at specified position. Both variables and literals can be passed as a character.
 *
 * Prints character on Lcd at specified position. Both variables and literals can be passed as a character.
 * @warning This function uses WTIMER5. This timer should not be used by the software (unpredictable behavior can occur)
 * @param row writing position row number
 * @param column writing position column number
 * @param out_char character to be written
 */
void Lcd_Chr(uint8_t row, uint8_t column, uint8_t out_char)
{
    lcd_goto(64*(row-1)+(column-1));
    lcd_putch(out_char);
}

/**
 * @brief Prints character on Lcd at current cursor position. Both variables and literals can be passed as a character.
 *
 * Prints character on Lcd at current cursor position. Both variables and literals can be passed as a character.
 * @warning This function uses WTIMER5. This timer should not be used by the software (unpredictable behavior can occur)
 * @param out_char character to be written
 */
void Lcd_Chr_Cp(uint8_t out_char)
{
    lcd_putch(out_char);
}

/**
 * @brief Sends command to Lcd.
 *
 * Sends command to Lcd.
 * Available Lcd Commands:
 * Lcd Command | Purpose
 * ----------- | -------
 * _LCD_FIRST_ROW | Move cursor to the 1st row
 * _LCD_SECOND_ROW | Move cursor to the 2nd row
 * _LCD_THIRD_ROW | Move cursor to the 3rd row
 * _LCD_FOURTH_ROW | Move cursor to the 4th row
 * _LCD_CLEAR | Clear display
 * _LCD_RETURN_HOME | Return cursor to home position, returns a shifted display to its original position. Display data RAM is unaffected.
 * _LCD_CURSOR_OFF | Turn off cursor
 * _LCD_UNDERLINE_ON | Underline cursor on
 * _LCD_BLINK_CURSOR_ON | Blink cursor on
 * _LCD_MOVE_CURSOR_LEFT | Move cursor left without changing display data RAM
 * _LCD_MOVE_CURSOR_RIGHT | Move cursor right without changing display data RAM
 * _LCD_TURN_ON | Turn Lcd display on
 * _LCD_TURN_OFF | Turn Lcd display off
 * _LCD_SHIFT_LEFT | Shift display left without changing display data RAM
 * _LCD_SHIFT_RIGHT | Shift display right without changing display data RAM
 * @warning This function uses WTIMER5. This timer should not be used by the software (unpredictable behavior can occur)
 * @param out_char command to be sent
 */
void Lcd_Cmd(uint8_t out_char)
{
    if((out_char==_LCD_FIRST_ROW)||(out_char==_LCD_RETURN_HOME))
    {
        lcd_goto(0);
    }
    else if(out_char==_LCD_SECOND_ROW)
    {
        lcd_goto(64);
    }
    else if(out_char==_LCD_THIRD_ROW)
    {
        lcd_goto(128);
    }
    else if(out_char==_LCD_FOURTH_ROW)
    {
        lcd_goto(192);
    }else if(out_char==_LCD_CLEAR)
    {
        lcd_cmd(out_char);
        Delay_ms(2);
    }
    else
    {
        lcd_cmd(out_char);
    }
}

//! @}
//*****************************************************************************
//
//! \addtogroup UART_Helper_api
//! @author Guilherme Luiz Moritz
//! @date 16 Mar 2018
//! @brief UART Library for TivaDAELT
//!
//! @{
//
//*****************************************************************************




/**
 * @brief State Machine enum for Get_Int and Get_Float functions
 *
 * Specifies which state of number receiving that is taking place.
 */
typedef enum getstate {SIGN,INTEGERS,DECIMALS} GetState;


/**
 * @brief Configure the UART and its pins.
 *
 * Configure the UART and its pins.  This must be called before UARTprintf()
 */
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}



/**
 * @brief Print Float
 *
 * Since UARTPrintf doesn't support float, this function was created to print them over UART
 * @param numero The number to be printed
 */
void Print_Float(float numero)
{
    if(numero<0)
    {
        numero *= -1;
        UARTprintf("-%d.%d",(int32_t)numero,(int32_t)((numero-(int32_t)numero)*100));
    }
    else
    {
        UARTprintf("%d.%d",(int32_t)numero,(int32_t)((numero-(int32_t)numero)*100));
    }
}


/**
 * @brief Get Float.
 *
 * This function receives a floating point number over UART. Once called it waits for numbers until Enter is pressed.
 * This function ignores any invalid character that is pressed
 * Usage:
 * @code
 * void main() {
     * float number;
     * number = Get_Float();
     * UARTPrintf("\nThe number ");
     * Print_Float();
     * UARTPrintf("was received.");
     * while(1);
 * }
 * @endcode
 * @return The number received from the UART. 0.0 if an empty number was typed or NaN if ESC key was pressed
 */
float Get_Float(void)
{
    GetState state = SIGN;
    unsigned char NextChar;
    float number=0;
    float negative = 1;
    float div = 10;
    uint8_t accept=0;
    do
    {
        NextChar = UARTgetc();
        if(NextChar==13)//enter
        {
            break;
        }
        if(NextChar==27)
        {
            negative = 1;
            number = 0.0/0.0;
            break;
        }
        switch(state)
        {
        case SIGN:
        {
            if(NextChar=='-')
            {
                negative = -1;
                state = INTEGERS;
                accept = 1;
            }
            //break ommited
        }
        case INTEGERS:
        {
            if(NextChar==',')
            {
                NextChar='.';
            }
            if(NextChar=='.')
            {
                accept = 1;
                state = DECIMALS;
            }
            if(NextChar>=48&&NextChar<=57) //valid number
            {
                accept = 1;
                number = number*10+(NextChar-48);
            }
            break;
        }
        case DECIMALS:
        {
            if(NextChar>=48&&NextChar<=57) //valid number
            {
                accept = 1;
                number = number + (NextChar-48)/div;
                div = div*10;
            }
            break;
        }
        }
        if(accept)
        {
            UARTprintf("%c",NextChar);
            accept = 0;
        }
    }while(1);
    return negative*number;
}

/**
 * @brief Get Int
 *
 * This function receives a 32 bit integer number over UART. Once called it waits for numbers until Enter is pressed.
 * This function ignores any invalid character that is pressed
 * Usage:
 * @code
 * void main() {
 * int32_t number;
 * number = Get_Int();
 * UARTPrintf("\nThe number %d was received.", number);
 * while(1);
 * }
 * @endcode
 * @return The number received from the UART. 0 if an empty number was typed or 0xFFFFFFFF if ESC key was pressed
 */
int32_t Get_Int(void)
{
    GetState state = SIGN;
    unsigned char NextChar;
    int32_t number=0;
    float negative = 1;
    uint8_t accept=0;
    do
    {
        NextChar = UARTgetc();
        if(NextChar==13) //enter
        {
            break;
        }
        if(NextChar==27)
        {
            negative = 1;
            number = 0xFFFFFFFF;
            break;
        }
        switch(state)
        {
        case SIGN:
        {
            if(NextChar=='-')
            {
                negative = -1;
                state = INTEGERS;
                accept = 1;
            }
            //break ommited
        }
        case INTEGERS:
        {
            if(NextChar>=48&&NextChar<=57) //valid number
            {
                accept = 1;
                number = number*10+(NextChar-48);
            }
            break;
        }
        }
        if(accept)
        {
            UARTprintf("%c",NextChar);
            accept = 0;
        }
    }while(1);
    return negative*number;
}

/**
 * @brief Initializes UART module.
 * Usage:
 * @code
 * void main() {
 *  UART_Init(UART3_BASE,9600,UART_CONFIG_WLEN_8,UART_CONFIG_STOP_ONE,UART_CONFIG_PAR_NONE);
 * }
 * @endcode
 * @param ui32Base Base address of the UARTx
 * @param baudRate Required baud rate
 * @param dataLen Data length, 8 or 9 bits
 * @param stopBits Number of stop bits
 * @param parity Parity check on or off
 */
int32_t UART_Init(uint32_t ui32Base, uint32_t baudRate, uint32_t dataLen, uint32_t stopBits, uint32_t parity)
{
    uint32_t ui32Peripheral;
    switch(ui32Base)
    {
        case UART1_BASE:
        {
            ui32Peripheral = SYSCTL_PERIPH_UART1;
            //
            // Enable the GPIO Peripheral used by the UART.
            //
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
            //
            // Check if the peripheral access is enabled.
            //
            while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
            {
            }
            //
            // Configure GPIO Pins for UART mode.
            //
            GPIOPinConfigure(GPIO_PC4_U1RX);
            GPIOPinConfigure(GPIO_PC5_U1TX);
            GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
            break;
        }
        case UART2_BASE:
        {
            ui32Peripheral = SYSCTL_PERIPH_UART2;
            //
            // Enable the GPIO Peripheral used by the UART.
            //
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
            //
            // Check if the peripheral access is enabled.
            //
            while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
            {
            }
            //
            // Configure GPIO Pins for UART mode.
            //
            GPIOPinConfigure(GPIO_PD6_U2RX);
            GPIOPinConfigure(GPIO_PD7_U2TX);
            GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
            break;
        }
        case UART3_BASE:
        {
            ui32Peripheral = SYSCTL_PERIPH_UART3;
            //
            // Enable the GPIO Peripheral used by the UART.
            //
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
            //
            // Check if the peripheral access is enabled.
            //
            while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
            {
            }
            //
            // Configure GPIO Pins for UART mode.
            //
            GPIOPinConfigure(GPIO_PC6_U3RX);
            GPIOPinConfigure(GPIO_PC7_U3TX);
            GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
            break;
        }
        case UART4_BASE:
        {
            ui32Peripheral = SYSCTL_PERIPH_UART4;
            //
            // Enable the GPIO Peripheral used by the UART.
            //
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
            //
            // Check if the peripheral access is enabled.
            //
            while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
            {
            }
            //
            // Configure GPIO Pins for UART mode.
            //
            GPIOPinConfigure(GPIO_PC4_U4RX);
            GPIOPinConfigure(GPIO_PC5_U4TX);
            GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
            break;
        }
        case UART5_BASE:
        {
            ui32Peripheral = SYSCTL_PERIPH_UART5;
            //
            // Enable the GPIO Peripheral used by the UART.
            //
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
            //
            // Check if the peripheral access is enabled.
            //
            while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
            {
            }
            //
            // Configure GPIO Pins for UART mode.
            //
            GPIOPinConfigure(GPIO_PE4_U5RX);
            GPIOPinConfigure(GPIO_PE5_U5TX);
            GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
            break;
        }
        case UART6_BASE:
        {
            ui32Peripheral = SYSCTL_PERIPH_UART6;
            //
            // Enable the GPIO Peripheral used by the UART.
            //
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
            //
            // Check if the peripheral access is enabled.
            //
            while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
            {
            }
            //
            // Configure GPIO Pins for UART mode.
            //
            GPIOPinConfigure(GPIO_PD4_U6RX);
            GPIOPinConfigure(GPIO_PD5_U6TX);
            GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
            break;
        }
        case UART7_BASE:
        {
            ui32Peripheral = SYSCTL_PERIPH_UART7;
            //
            // Enable the GPIO Peripheral used by the UART.
            //
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
            //
            // Check if the peripheral access is enabled.
            //
            while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
            {
            }
            //
            // Configure GPIO Pins for UART mode.
            //
            GPIOPinConfigure(GPIO_PE0_U7RX);
            GPIOPinConfigure(GPIO_PE1_U7TX);
            GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
            break;
        }
        default:
        {
            return -1;
        }
    }
    //
    // Enable the UARTx module.
    //
    SysCtlPeripheralEnable(ui32Peripheral);
    //
    // Wait for the UARTx module to be ready.
    //
    while(!SysCtlPeripheralReady(ui32Peripheral))
    {
    }
    // Initialize the UART. Set the baud rate, number of data bits, turn off
    // parity, number of stop bits, and stick mode. The UART is enabled by the
    //July 25, 2016 579
    //UART
    // function call.
    //
    UARTConfigSetExpClk(ui32Base, SysCtlClockGet(), baudRate, (dataLen | stopBits | parity));
    return 1;
}

/**
 * @brief Prints text on UARTx. Both string variables and literals can be passed as a text.
 *
 * Prints text on UARTx. Both string variables and literals can be passed as a text.
 * @param ui32Base Base address of the UARTx
 * @param *text Pointer to the text to be written
 * @param ui32Len Length of the string to be written
 */
void UART_Out_Cp(uint32_t ui32Base, uint8_t *text, uint32_t ui32Len)
{
    unsigned int uIdx;

    //
    // Check for valid UART base address, and valid arguments.
    //
    //ASSERT(ui32Base != 0);
    //ASSERT(text != 0);

    //
    // Send the characters
    //
    for(uIdx = 0; uIdx < ui32Len; uIdx++)
    {
        //
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output.
        //
        if(text[uIdx] == '\n')
        {
            UARTCharPut(ui32Base, '\r');
        }

        //
        // Send the character to the UART output.
        //
        UARTCharPut(ui32Base, text[uIdx]);
    }
}

/**
 * @brief Prints character on UARTx. Both variables and literals can be passed as a character.
 *
 * Prints character on UARTx. Both variables and literals can be passed as a character.
 * @param ui32Base Base address of the UARTx
 * @param out_char character to be written
 */
void UART_Chr_Cp(uint32_t ui32Base, uint8_t out_char)
{
    UARTCharPut(ui32Base, out_char);
}

/**
 * @brief Gets character on UARTx.
 *
 * Returns character received on UARTx.
 */
uint8_t UART_Chr_Get(uint32_t ui32Base)
{
    //
    // Check for characters. Spin here until a character is placed
    // into the receive FIFO.
    //
    while(!UARTCharsAvail(ui32Base))
    {
    }
    //
    // Get the character(s) in the receive FIFO.
    //
    return (uint8_t) UARTCharGetNonBlocking(ui32Base);

}

/**
 * @brief Gets string on UARTx.
 * @param ui32Base Base address of the UARTx
 * @param *buf Pointer to the buffer that will receive the string read on UARTx
 * @param bufSize Length of the given buffer
 */
void UART_Str_Get(uint32_t ui32Base, uint8_t *buf, uint8_t bufSize)
{
    uint8_t i=0;
    //
    // Check for characters. Spin here until a character is placed
    // into the receive FIFO.
    //
    while(!UARTCharsAvail(ui32Base))
    {
    }

    while(UARTCharsAvail(ui32Base) && i<bufSize-1)
    {
        buf[i] =  UARTCharGetNonBlocking(ui32Base);
        i++;
    }
    buf[i] = 0x00;
    return;

}

//! @}

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
 * @brief Keypad Column Pin Definitions
 *
 * This pointer holds an 4 position array of @PinDefinition structs, ColumnPinDefinitions[0] must hold Column0 pin, while ColumnPinDefinitions[3] must hold Column3 pin.
 */
static PinDefinition* gColumnPinDefinitions=0;

/**
 * @brief Keypad Row Pin Definitions
 *
 * This pointer holds an 4 position array of @PinDefinition structs, RowPinDefinitions[0] must hold Row0 pin, while ColumnPinDefinitions[3] must hold Row3 pin.
 */
static PinDefinition* gRowPinDefinitions=0;

#define KEYPAD_SIZE (4)

/**
 * @brief Keypad Init
 *
 * Initializes KeyPad ports.
 * Rows will be configured as inputs with internal pull ups and columns as outputs.
 * The keypad must be connected as indicated in the following picture (Row Pull ups are internal to the TM4C123. There is no need to include external pull ups):
 * @image latex keypad.png "KeyPad Schematics" width=7.5cm
 * @image html keypad.png

 * API Usage:
 * @code
 * void main()
 * {
 *     uint8_t ret=0;
 *
 *     //KeyPad Definition Macros
 *     //Must be declared on main or as global
 *     PinDefinition KeypadColumns[4],KeypadRows[4];
 *
 *     //UART
 *     ConfigureUART();
 *     TivaDaeltVersion();
 *
 *     //port B will be used for KeyPad
 *     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
 *
 *     //port E will be used to disable DaeltTiva Leds on port B
 *     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
 *
 *     // Check if the peripheral access is enabled.
 *     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
 *
 *     // Check if the peripheral access is enabled.
 *     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
 *
 *     // Disbale leds
 *     GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0); *
 *     GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0x0);
 *
 *     //KeyPad Pin Definition
 *     KeypadColumns[0].ui32Port = GPIO_PORTB_BASE;
 *     KeypadColumns[0].ui8Pin = GPIO_PIN_4;
 *
 *     KeypadColumns[1].ui32Port = GPIO_PORTB_BASE;
 *     KeypadColumns[1].ui8Pin = GPIO_PIN_5;
 *
 *     KeypadColumns[2].ui32Port = GPIO_PORTB_BASE;
 *     KeypadColumns[2].ui8Pin = GPIO_PIN_6;
 *
 *     KeypadColumns[3].ui32Port = GPIO_PORTB_BASE;
 *     KeypadColumns[3].ui8Pin = GPIO_PIN_7;
 *
 *     KeypadRows[0].ui32Port = GPIO_PORTB_BASE;
 *     KeypadRows[0].ui8Pin = GPIO_PIN_3;
 *
 *     KeypadRows[1].ui32Port = GPIO_PORTB_BASE;
 *     KeypadRows[1].ui8Pin = GPIO_PIN_2;
 *
 *     KeypadRows[2].ui32Port = GPIO_PORTB_BASE;
 *     KeypadRows[2].ui8Pin = GPIO_PIN_1;
 *
 *     KeypadRows[3].ui32Port = GPIO_PORTB_BASE;
 *     KeypadRows[3].ui8Pin = GPIO_PIN_0;
 *
 *     Keypad_Init(KeypadColumns, KeypadRows);
 *
 *     while(1)
 *     {
 *        ret = Keypad_Key_Click();
 *        sprintf(str,"Key %d was pressed\n",ret);
 *        UARTPrintf(str);
 *     }
 * }
 * @endcode
 * @param ColumnPinDefinitions An 4 position array of PinDefinition structs, ColumnPinDefinitions[0] must hold Column0 pin, while ColumnPinDefinitions[3] must hold Column3 pin. Its caller responsability to mantain the struct in memory since only the reference pointer is saved by init.
 * @param RowPinDefinitions An 4 position array of PinDefinition structs, RowPinDefinitions[0] must hold Line0 pin, while Line[3] must hold Column3 pin. Its caller responsability to mantain the struct in memory since only the reference pointer is saved by init.
 * @note This library is not implemented to detect multiple keypresses
 * @note This function assumes that the clock of all involved GPIOs are already enabled.
 * @note KeyPad is incompatible with the TM4C123 locked pins (NMI - PF0 and JTAG pins)
 * @warning This function uses WTIMER5 - code cannot use this timer if the function is used
 */
void Keypad_Init(PinDefinition* ColumnPinDefinitions, PinDefinition* RowPinDefinitions)
{
    int32_t i=0;

    gColumnPinDefinitions = ColumnPinDefinitions;
    gRowPinDefinitions = RowPinDefinitions;

    //configura todos os pinos como entrada
    for(i=0;i<KEYPAD_SIZE;i++)
    {
        GPIODirModeSet(gColumnPinDefinitions[i].ui32Port, gColumnPinDefinitions[i].ui8Pin, GPIO_DIR_MODE_IN);
        GPIOPadConfigSet(gColumnPinDefinitions[i].ui32Port, gColumnPinDefinitions[i].ui8Pin, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

        GPIODirModeSet(gRowPinDefinitions[i].ui32Port, gRowPinDefinitions[i].ui8Pin, GPIO_DIR_MODE_IN);
        GPIOPadConfigSet(gRowPinDefinitions[i].ui32Port, gRowPinDefinitions[i].ui8Pin, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    }
}

/**
 * @brief Reads the key from keypad when key gets pressed.
 *
 * Reads the key from keypad when key gets pressed.
 * @return The code of a pressed key (1..16). 1 means 1st column 1st row, 2 means 1st column 2nd row ... 16 means 4th column 4th row. If no key is pressed or the Keypad was not initialized, returns 0
 * @note Port needs to be initialized for working with the Keypad library, see #Keypad_Init.
 * @note This library is not implemented to detect multiple keypresses
 * @note KeyPad is incompatible with the TM4C123 locked pins (NMI - PF0 and JTAG pins)
 * @warning This function uses WTIMER5 - code cannot use this timer if the function is used
 */
uint8_t Keypad_Key_Press(void)
{
    int32_t i=0,j=0;

    if(gColumnPinDefinitions == 0)
    {
        return 0;
    }

    if(gRowPinDefinitions ==  0)
    {
        return 0;
    }

    //configura todos os pinos como entrada
    for(i=0;i<KEYPAD_SIZE;i++)
    {
        //configura o pino como saída
        GPIODirModeSet(gColumnPinDefinitions[i].ui32Port, gColumnPinDefinitions[i].ui8Pin, GPIO_DIR_MODE_OUT);

        //seta o pino
        GPIOPinWrite(gColumnPinDefinitions[i].ui32Port, gColumnPinDefinitions[i].ui8Pin, 0x0);
        Delay_us(500);

        for(j=0;j<KEYPAD_SIZE;j++)
        {
            if(Button(gRowPinDefinitions[j].ui32Port, gRowPinDefinitions[j].ui8Pin, 1, 0))
            {
                return (j*KEYPAD_SIZE)+(i+1);
            }
        }
        //configura o pino como entrada
        GPIODirModeSet(gColumnPinDefinitions[i].ui32Port, gColumnPinDefinitions[i].ui8Pin, GPIO_DIR_MODE_IN);
        Delay_us(500);
    }

    return 0;
}


/**
 * @brief Wait for a KeyPad KeyPress and return its value.
 *
 * Call to Keypad_Key_Click is a blocking call: the function waits until some key is pressed then returns 1 to 16, depending on the key.
 * @return The code of a pressed key (1..16). 1 means 1st column 1st row, 2 means 1st column 2nd row ... 16 means 4th column 4th row. The function returns 0 if the KeyPad was not initialized (see #Keypad_Init)
 * @note Port needs to be initialized for working with the Keypad library, see #Keypad_Init.
 * @note This library is not implemented to detect multiple keypresses
 * @note KeyPad is incompatible with the TM4C123 locked pins (NMI - PF0 and JTAG pins)
 * @warning This function uses WTIMER5 - code cannot use this timer if the function is used
 */
uint8_t Keypad_Key_Click(void)
{
    uint8_t return_value;

    if(gColumnPinDefinitions == 0)
    {
        return 0;
    }

    if(gRowPinDefinitions ==  0)
    {
        return 0;
    }

    do
    {
        return_value = Keypad_Key_Press();
        Delay_ms(1);
    }while(return_value==0);

    while(Keypad_Key_Press());

    return return_value;
}
//! @}
