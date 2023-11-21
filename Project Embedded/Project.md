# TEMPERATURE SENSING AND CONTROLLING


## Group 11: 
            EE23MT017 Sahil Bagade;   EE23MT017@gmail.com            
            EE23Mt008 JAYASURIYA T J  EE23MT008@gmail.com

## Aim: 
Create a temperature controller using a DAC, ADC and LM35 as temperature sensor:

## Components Used: 
            1 : Tiva™ TM4C123GH6PM Microcontroller
            2 : LM35 (Temperature sensor module)
            3 : Power Transistor 
            4 : 1kohm resistor
            5 : 2N2222 transistor
            6 : DC power supply
            7 : Breadboard
            8 : Jumpers

## Description 

.

The Microcontroller generates the PWM signal which has a duty cycle of 50% and is given to the base of the transistor through the resistor. The collector end of the BJT is connected to the power supply and the emitter is grounded. The output voltage from the BJT is given to the input of the Temperature Sensor Module, due to which the resistor will heat up. This temperature rise will be sensed by the LM35 temperature sensor. The LM35 sensor has 3 pins for Vin, Vout, and Ground. The range of the LM35 sensor for temperature is  −55°C to 150°C. The LM35 sensor has a working range of 4v to 30v. The change in temperature in the resistor is proportional to the Vout signal. The signal is sent from the output pin of LM35 to the microcontroller. For every 1-degree change, it will send 0.1mv to the microcontroller.  

Temperature sensor module


<img src="images/Temperature Sensor Module.jpeg" alt="Temperature Sensor Module" width="550"/>



 
The Temperature sensor module consists of one resistor and one  LM35 as a temperature sensor.
<img src="images/LM35.png" alt="Tiva" height="350" width="550"/>

    LM35 has three pins:

    PIN 1 : Vcc, it is the input pin (5v)

    PIN 2 : Vout, we get output (it should be connected to the analog pin microcontroller)

    PIN 3 : GND, it is used for ground

    Features: 
    • Calibrated Directly in Celsius (Centigrade)
    • Linear + 10-mV/°C Scale Factor
    • 0.5°C Ensured Accuracy (at 25°C)
    • Rated for Full −55°C to 150°C Range
    • Suitable for Remote Applications
    • Low-Cost Due to Wafer-Level Trimming
    • Operates From 4 V to 30 V
    • Less Than 60-μA Current Drain
    • Low Self-Heating, 0.08°C in Still Air
    • Non-Linearity Only ±¼°C Typical
    • Low-Impedance Output, 0.1 Ω for 1-mA Load
 
TIVA Microcontroller


<img src="images/Tiva.jpeg" alt="Tiva" width="550"/>


BJT


<img src="images/BJT 2N3055.jpeg" alt="Tiva" width="550"/>

<img src="images/BJT Pins.jpeg" alt="Tiva" width="550"/>
    
    
    
## Circuit Diagram
<img src="images/Ckt Diag.jpeg" alt="Tiva" width="750"/>

## Circuit Implementation
<img src="images/Ckt implementation.jpeg" alt="Tiva" width="750"/>

## PWM Signal
<img src="images/PWM.jpeg" alt="Tiva" width="750"/>

## PWM Max Duty Cycle
<img src="images/PWM Max.jpeg" alt="Tiva" width="750"/>

## PWM Min Duty Cycle
<img src="images/PWM Min.jpeg" alt="Tiva" width="750"/>


 



## Code

    /*PWM Waveform 100KHz with variable duty cycle on pin PF2 (Blue LED)
    Initial duty cycle=50%, SW1 decreases duty cycle by 5%, SW2 increases duty cycle by 5% */

    /*Analog input pin is PE1(ADC0)*/

    /*UART0*/


    #include <stdint.h>
    #include <stdbool.h>
    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include "tm4c123gh6pm.h"

    #define PWM_FREQ 100000                      // 100 KHz PWM Frequency
    #define PWM_PERIOD (16000000 / PWM_FREQ)    // PWM Period = 160
    #define PWM_START_DUTY 50                  // Start duty cycle

    volatile uint32_t Duty = PWM_START_DUTY;  // Initialize duty cycle to 50
    volatile uint32_t adc_value;
    volatile uint32_t analog_voltage;
    char temperature;

    void GPIO_Init(void);                      //GPIO Init function declaration
    void PWM_Init(void);                      //PWM Init function declaration
    void ADC_Init(void);                     //ADC Init function declaration
    void PortA_Config(void);                // PORTA GPIO configuration (UART0 Send Data)
    void UART_Config(void);                // UART0 (Tx Data) Configuration
    void UART0_SendString(char *str);     // UART0 transmit string
    char str[1];                         // UART0 send string char argument


    int main(void)
    {

     GPIO_Init();         //Initialize GPIO for pin PF0, PF4
        PWM_Init();          //Initialize PWM for pin PF0, PF4
        ADC_Init();         //Initialize ADC for pin PE1
        PortA_Config();     // GPIO PORTA Configuration for UART
        UART_Config();     //UART0 Configuration


        while (1)
        {
            Data_Send();    // Transmit data to UART0
        }
    }


    void GPIO_Init(void)
    {
            SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;            // Enable GPIO Port F (for SW1 and SW2)


            // Configure SW1 (PF4) and SW2 (PF0) as inputs
            GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // Unlock GPIO PortF
            GPIO_PORTF_CR_R = 0x01;           // Uncommit and Allow changes to PF0 (SW2)
            GPIO_PORTF_DIR_R &= ~(0x11);     // Direction: bits 0 and 4 set as inputs
            GPIO_PORTF_DEN_R |= 0x11;       // Digital Enable: bits 0 and 4
            GPIO_PORTF_PUR_R |= 0x11;      // Pull-up resistors enabled on PF0 AND PF4

            // Configure GPIO interrupt for Port F (Switches)
            GPIO_PORTF_IS_R &= ~0x11;     // Interrupt Sense: Edge-sensitive
            GPIO_PORTF_IBE_R &= ~0x11;   // Interrupt Both Edges: Not both edges
            GPIO_PORTF_IEV_R &= ~0x11;  // Interrupt Event: Falling edge event
            GPIO_PORTF_ICR_R |= 0x11;  // Interrupt Clear: Clear the interrupt flags for PF0 , PF4
            GPIO_PORTF_IM_R |= 0x11;  // Interrupt Mask: Unmask (Enable) interrupt on PF0 , PF4

            //NVIC configuration
            NVIC_EN0_R |= 1 << 30;   // Interrupt Enable: Enable interrupt for GPIO PF0 (bit 30)
            NVIC_PRI7_R |= 0xFF2FFFFF;//Interrupt Priority 1 to Port F
    }


    void PWM_Init(void)
    {
            SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;            // Enable the PWM1 module



            //Configure PF2 (M1PWM6) as PWM Output
            GPIO_PORTF_AFSEL_R |= 0x04;          // Enable alternate function on PF2
            GPIO_PORTF_PCTL_R &= ~0x00000F00;   // Port Control: Clear PF2
            GPIO_PORTF_PCTL_R |= 0x00000500;   // Port Control: Configure PF2 as M1PWM6
            GPIO_PORTF_DIR_R |= 0x04;         // Direction: Make PF2 as output
            GPIO_PORTF_DEN_R |= 0x04;        // Digital enable PF2

            //Configure Module 1 PWM Generator 3 which controls Module 1 PWM6 (M1PWM6) on pin PF2
            PWM1_3_CTL_R = 0;             // Disable PWM while configuring
            PWM1_3_GENA_R = 0x0000008C;  /* Down Count: M1PWM6 output is high at the start of the period
                                   and low when the counter matches comparator A */
            PWM1_3_LOAD_R = PWM_PERIOD - 1;             // Set PWM period
            PWM1_3_CMPA_R = (PWM_PERIOD * Duty) / 100; // Set Compare A value, consider initial duty
            PWM1_3_CTL_R |= 0x00000001;               // Enable PWM1 Generator 3
            PWM1_ENABLE_R |= 0x00000040;             // Enable Module1 PWM6 (M1PWM6) output
    }


    void ADC_Init(void)
    {
            //ADC MODULE CONFIGURATION
                SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;    //enabling clock for ADC module0
                SYSCTL_RCGCGPIO_R |=SYSCTL_RCGCGPIO_R4;  //enabling the clock for GPIO PORT E as ANI0 pin is connected to PORT E (PE3)
                GPIO_PORTE_DIR_R &= ~(0x08);            //PE3 is configured as an input pin
                GPIO_PORTE_AFSEL_R |= 0x08;            //AIN0 is connected to PE3,analog function is selected for PF3
                GPIO_PORTE_DEN_R &= ~(0x08);          //digital functions of PE3 is disabled
                GPIO_PORTE_AMSEL_R |= 0x08;          //analog function of PE3 pin is enabled


        //ADC SAMPLE SEQUENCER CONFIGURATION
                ADC0_ACTSS_R &= ~(1<<3);           //sample sequencer 3 of ADC module 0 is disabled
                ADC0_EMUX_R &= ~(0xF000);         //software trigger sampling
                ADC0_SSMUX3_R |= 0x00;           //first sample input is AIN0
                ADC0_SSCTL3_R |= (1<<1)|(1<<2); //take one sample at a time and set flag at first sample


        //ADC INTERRUPT CONFIGURATION
                ADC0_IM_R |= (1<<3);           //unmask ADC0 sequence 3 interrupt
                NVIC_EN0_R |= 1<<17;          //enabled interrupt 17(ADC0 SS3)
                NVIC_PRI4_R |= 0xFFFF4FFF;   // NVIC priority for ADC0 SS3 is 2
                ADC0_ACTSS_R |= (1<<3);     //sample sequencer 3 of ADC module 0 is enabled
                ADC0_PSSI_R |= (1<<3);     //start of sampling
    }
    void PortA_Config(void)
    {
            GPIO_PORTA_LOCK_R = 0x4C4F434B; // Unlock PortA register
            GPIO_PORTA_CR_R = 0xFF;         // Enable Commit function
            GPIO_PORTA_DEN_R = 0xFF;        // Enable all pins on port A
            GPIO_PORTA_DIR_R |= (1 << 1);    // Define PA1 as output
            GPIO_PORTA_AFSEL_R |= 0x03;      // Enable Alternate function for PA0 and PA1
            GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) | 0x00000011; // Selecting UART function for PA0 and PA1
    }

    void UART_Config(void)
    {
        /*
            *BRDI = integer part of the BRD; BRDF = fractional part
            *BRD = BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)
            *UARTSysClk = 16MHz, ClkDiv = 16, Baud Rate = 9600
            *BRD = 104.167; BRDI = 104; BRDF = 167;
            *UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5) = 11
        */


        //PortA (UART0) configuration
            UART0_CTL_R &= ~UART_CTL_UARTEN; // Disable UART0 during configuration
            UART0_IBRD_R = 104;              // Integer part of the baud rate
            UART0_FBRD_R = 11;               // Fractional part of the baud rate
            UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN); // 8-bit word length, enable FIFO
            UART0_CTL_R |= (UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE); // Enable UART0, RX, and TX
    }

    void GPIOF_Handler(void)               // GPIO PortF Interrupt handler
    {
            if ( GPIO_PORTF_RIS_R & 0x10)     // Raw Interrupt Status: Check for PF4 (SW1) interrupt
        {
            if (Duty < 95)               // Check if current duty cycle is less than max value 95
            {
                    Duty += 5;              // Increase duty by 5, this increases CMPA value and decreases duty cycle
            }
            GPIO_PORTF_ICR_R = 0x10;   // Clear GPIO interrupt for PF4 (SW1)
        }
        if ( GPIO_PORTF_RIS_R & 0x01)  // Raw Interrupt Status: Check for PF0 (SW2) interrupt
        {
            if (Duty > 5)      // Check if current duty cycle is greater than min value 5%
            {
                    Duty -= 5;    // Decrease duty by 5, this decreases CMPA value and increases duty cycle
            }
            GPIO_PORTF_ICR_R = 0x01; // Clear GPIO interrupt for PF0 (SW2)
        }

            PWM1_3_CMPA_R = (PWM_PERIOD * Duty) / 100; // Update PWM1 Generator 3 Compare A value
    }




    void ADC0SS3_Handler(void)
    {
            adc_value = ADC0_SSFIFO3_R;        //sampled value is stored in the adc_value variable
            ADC0_ISC_R |= (1<<3);             //clear flag which is set for sampling
            ADC0_PSSI_R |= (1<<3);           //start of sampling
            adc_value=ADC0_SSFIFO3_R;       //storing the adc output in a variable
            analog_voltage=(0.8*adc_value);  //analog voltage corresponding to the digital output
            temperature=0.1*analog_voltage; //temperature value is stored here

    }

    void Data_Send(void)
    {
            //send variable value to terminal
            UART0_SendString("Temperature = ");    // Sending String
            UART0_SendString(temperature);          // Latitude value is parseValue[1] in ddmm.mmmm format
    }
    void UART0_SendString(char *str)             // UART0 String Send Function
    {
        while (*str)
        {
            // Send each character of the string
            while ((UART0_FR_R & 0x20) != 0); // Wait as long as Transmit FIFO (TXFF) is full
            UART0_DR_R = *str; // Send the character to UART0 Data Register
            str++;             // Go to next character in the string

    }
    }




## Results:

We have to sense the temperature of a resistor using an LM35 temperature sensor. And we have to control the temperature of a resistor using a switch. 
The  temperature sensing is done by controlling the current in the Temperature sensing module using a switch of the microcontroller, where we have used the LM35 sensor to detect the rise in temperature and sent the data to the microcontroller

=======

