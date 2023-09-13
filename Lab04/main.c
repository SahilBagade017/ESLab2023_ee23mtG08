#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"


#define MASK_BITS 0x11                            /* pins for which interrupts should be enabled*/
#define SW1_MASK 0x10
#define SW2_MASK 0x01

#define STCTRL *((volatile long *) 0xE000E010)    /* control status register */
#define STRELOAD *((volatile long *) 0xE000E014)  /* reload value register*/
#define STCURRENT *((volatile long *) 0xE000E018) /* current value register*/



void default_cfg()
{
   SYSCTL_RCGC2_R |= 0x00000020;
   GPIO_PORTF_LOCK_R = 0x4C4F434B;
   GPIO_PORTF_CR_R = 0x01;
   GPIO_PORTF_DIR_R = 0x0E;
   GPIO_PORTF_DEN_R = 0x1F;
   GPIO_PORTF_PUR_R = 0x11;
}



void GPIOF_Handler(void)

{

      if (GPIO_PORTF_MIS_R & SW1_MASK)
         {
           GPIO_PORTF_DATA_R = 0x08;
           GPIO_PORTF_ICR_R = SW1_MASK;
           STCTRL |=0x07;
         }

      if (GPIO_PORTF_MIS_R & SW2_MASK)
         {
           GPIO_PORTF_DATA_R = 0x02;
           GPIO_PORTF_ICR_R = SW2_MASK;
           STCTRL |=0x07;
         }
      GPIO_PORTF_ICR_R = MASK_BITS;
}



void systick_Handler(void)
{
      GPIO_PORTF_DATA_R=0x00;
      STCTRL =0x00;
}


int main(void)
{


    default_cfg();

    GPIO_PORTF_IM_R &= ~MASK_BITS;               /* mask interrupt by clearing bits*/
    GPIO_PORTF_IS_R &= MASK_BITS;                /* edge sensitive interrupts*/
    GPIO_PORTF_IBE_R &= ~MASK_BITS;              /* interrupt NOT on both edges*/
    GPIO_PORTF_IEV_R &= MASK_BITS;               /* falling edge trigger*/
    /* Prioritize and enable interrupts in NVIC */
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xF1FFFFFF) | (3 << 21);
                                                 /*interrupt priority register 7*/
                                                 /* bits 21-23 for interrupt 30 (port F)*/
    NVIC_EN0_R |= 1 << 30;                       /* enable interrupt 30 (port F)*/
                                                 /* Enable interrupt generation in GPIO */
    GPIO_PORTF_ICR_R = MASK_BITS;                /* clear any prior interrupt*/
    GPIO_PORTF_IM_R |= MASK_BITS;                /* unmask interrupt by setting bits*/

    STCTRL=0;
    STRELOAD=16000*1000;

    while(1);


}


