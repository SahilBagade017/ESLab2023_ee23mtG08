#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define Red_Led   1
#define Blue_Led  2
#define Green_Led 3

// Define a debouncing delay (adjust as needed)
#define DEBOUNCE_DELAY 100000

int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;       /* enable clock to GPIOF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    GPIO_PORTF_CR_R = 0x0F;             /* make PORTF0+PORTF1+PORTF2+PORTF3 configurable */
    GPIO_PORTF_DIR_R = 0x0E;            /* set PORTF3+PORTF2+PORTF1 pin as output (LED) pin */
    GPIO_PORTF_DEN_R = 0x0F;            /* set PORTF pins 3-2-1-0 as digital pins */
    GPIO_PORTF_PUR_R = 0x01;            /* enable pull up for pin 0 */

    int a, count = 0;
    int debounce_counter = 0;

    while (1)
    {
        a = (GPIO_PORTF_DATA_R & 0x01); // Read the state of the push-button switch

        // Implement debounce
        if (a == 0)
        {
            debounce_counter++;
            if (debounce_counter >= DEBOUNCE_DELAY)
            {
                debounce_counter = 0; // Reset debounce counter
                count++;

                if (count == 1)
                {
                    GPIO_PORTF_DATA_R = (1 << Red_Led); // Only Red Led is on
                }
                else if (count == 2)
                {
                    GPIO_PORTF_DATA_R = (1 << Blue_Led); // Only Blue Led is on
                }
                else if (count == 3)
                {
                    GPIO_PORTF_DATA_R = (1 << Green_Led); // Only Green Led is on
                }
                else
                {
                    count = 0; // Reset count to start the sequence again
                }
            }
        }
        else
        {
            debounce_counter = 0; // Reset debounce counter when the button is released
        }
    }
}
