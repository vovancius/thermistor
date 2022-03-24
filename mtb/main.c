/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 6 MCU: Hello World Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_thermistor_ntc_gpio.h"
#include <string.h>


/*******************************************************************************
* Macros
*******************************************************************************/

/* LED blink timer clock value in Hz */
#define LED_BLINK_TIMER_CLOCK_HZ          (10000)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD            (10000)
#define DEFAULT_TIMER_PERIOD			  (1)

/*
 * THERMISTOR PIN CONFIG MACROS - CY8CPROTO-062-WiFi-BT kit
 */
#define THERM_GND	P10_3
#define THERM_VDD	P10_0
#define THERM_OUT	P10_2
/*ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss*/
#define RX_LENGTH   5
/*ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss*/
#define START_CMD	0xA0
#define STOP_CMD	0xB0
#define SET_CMD		0xC0


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(uint32_t sec);
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);
float temperature(void);


/*******************************************************************************
* Global Variables
*******************************************************************************/
bool timer_interrupt_flag = false;
bool timer_active_flag = true;

/* Variable for storing character read from terminal */
uint8_t uart_read_value[RX_LENGTH];

/* Timer object used for blinking the LED */
cyhal_timer_t led_blink_timer;

/* Thermistor globals */
mtb_thermistor_ntc_gpio_t mtb_thermistor_obj;

cyhal_adc_t adc_obj;

/* Refer thermistor datasheet for the values */
mtb_thermistor_ntc_gpio_cfg_t therm_gpio_cfg=
    {
    		.b_const = (float)(3380),         //refers to beta constant

			/* Resistance of the thermistor is 10K at 25 degrees C (from datasheet)
			 * Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
			 * R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855
			 */

			.r_infinity = (float)(0.1192855), //refers to resistance at infinity, ideally 0
			.r_ref = (float)(10000)           //refres to reference resistance
    };

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It sets up a timer to trigger a 
* periodic interrupt. The main while loop checks for the status of a flag set 
* by the interrupt and toggles an LED at 1Hz to create an LED blinky. The 
* while loop also checks whether the 'Enter' key was pressed and 
* stops/restarts LED blinking.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, 
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
///    printf("\x1b[2J\x1b[;H");

    printf("----------------|Temperature Measurement|----------------\r\n\n");
    
    /* Initialize timer to toggle the LED */
    uint32_t sec = DEFAULT_TIMER_PERIOD;
    timer_init(sec);

    result = cyhal_adc_init(&adc_obj, THERM_OUT, NULL);
    result = mtb_thermistor_ntc_gpio_init(&mtb_thermistor_obj, &adc_obj, THERM_GND,
    									  THERM_VDD, THERM_OUT, &therm_gpio_cfg,
										  MTB_THERMISTOR_NTC_WIRING_VIN_R_NTC_GND);


    for (;;)
    {

        /* Check if 'stop' was written */
        if (cyhal_uart_read_async(&cy_retarget_io_uart_obj, uart_read_value, RX_LENGTH)
             == CY_RSLT_SUCCESS)
        {

            /* Check if uart is 'stop'  */
        	if ((timer_active_flag) & (uart_read_value[0] == STOP_CMD))
        	{

        		cyhal_timer_stop(&led_blink_timer);

                timer_active_flag ^=1;

                printf("Measurement paused\r\n");

                /* Move cursor to previous line */
                ///printf("\x1b[1F");
             }

        	/* Check if uart is 'start'  */
        	else if ((!timer_active_flag) & (uart_read_value[0] == START_CMD))
			{
        		cyhal_timer_start(&led_blink_timer);

        		timer_active_flag ^=1;

        		printf("Measurement resumed\r\n");
        		///printf("\x1b[1F");
			}

        	/* Check if uart is 'set period'  */
        	else if (uart_read_value[0] == SET_CMD)
        	{
        		sec = (uart_read_value[1] << 24) + (uart_read_value[2] << 16) +
        			  (uart_read_value[3] << 8) + uart_read_value[4];

        		printf("Period is %us\r\n",sec);

        		if (timer_active_flag)
        		{
            		cyhal_timer_stop(&led_blink_timer);
        		}

        		timer_init(sec);
        	}


        }

        /* Check if timer elapsed (interrupt fired) and toggle the LED */
        if (timer_interrupt_flag)
        {
            /* Clear the flag */
            timer_interrupt_flag = false;

            /* Invert the USER LED state */
            cyhal_gpio_toggle(CYBSP_USER_LED);/*вимір температури, запис, вивід на екран */
            temperature();
        }
    }
}


/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks 
* continuously and produces a periodic interrupt on every terminal count 
* event. The period is defined by the 'period' and 'compare_value' of the 
* timer configuration structure 'led_blink_timer_cfg'. Without any changes, 
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
 void timer_init(uint32_t sec)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg = 
    {
        .period = sec * LED_BLINK_TIMER_PERIOD - 1,  	/* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    			/* Timer counts up */
        .is_compare = false,                			/* Don't use compare mode */
        .is_continuous = true,              			/* Run timer indefinitely */
        .value = 0                          			/* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction, 
       duration */
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                              7, true);

    /* Start the timer with the configured settings */
    if (timer_active_flag)
    {
    	cyhal_timer_start(&led_blink_timer);
    }
 }


/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;
}


/*******************************************************************************
 * Function Name: temperature
 *******************************************************************************
 * Summary:
 *  Function to handle the temperature readings from the ADC.
 *
 * Parameters:
 *  void: unused
 *
 * Return:
 *  float result: Result of the readings
 *
 *******************************************************************************/

float temperature(void)
{
	float value;
	float sum=0;
	uint8_t loop;
	for (loop = 0; loop < 10; loop++)
	{
		value = mtb_thermistor_ntc_gpio_get_temp(&mtb_thermistor_obj);
		sum=sum+value;
	}
	sum = sum/10;
	printf("Temperature is %2.2f C\r\n",sum);
	return sum;

}

/* [] END OF FILE */
