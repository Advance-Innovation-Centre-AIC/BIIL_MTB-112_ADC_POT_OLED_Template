/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "stdio.h"
#include "cy8ckit_028_sense.h"
#include "GUI.h"

// Define function
void lcd_print_top(const char * s);
void lcd_print_bot(const char * s);
void lcd_print_line_n(const char * s,int linenum);

cyhal_i2c_t i2c;
cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
};




int main(void)
{
    cy_rslt_t result;
	cyhal_adc_t adc_obj;
	cyhal_adc_channel_t adc_chan_0_obj;
    char str[80];

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    __enable_irq();

    /* Initialize i2c */
    result = cyhal_i2c_init(&i2c, CY8CKIT_028_SENSE_PIN_I2C_SDA, CY8CKIT_028_SENSE_PIN_I2C_SCL, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    result = cyhal_i2c_configure(&i2c, &i2c_cfg);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize the OLED display */
    result = mtb_ssd1306_init_i2c(&i2c);
    CY_ASSERT(result == CY_RSLT_SUCCESS);


	/* ADC conversion result. */
	int adc_out;

	/* Initialize ADC. The ADC block which can connect to pin 10[6] is selected */
	result = cyhal_adc_init(&adc_obj, P10_6, NULL);

	// ADC configuration structure
	const cyhal_adc_config_t ADCconfig ={
		.continuous_scanning = false,
		.resolution = 12,
		.average_count = 1,
		.average_mode_flags = 0,
		.ext_vref_mv = 0,
		.vneg = CYHAL_ADC_VNEG_VREF,
		.vref = CYHAL_ADC_REF_VDDA,
		.ext_vref = NC,
		.is_bypassed = false,
		.bypass_pin = NC
	};

	// Configure to use VDD as Vref
	result = cyhal_adc_configure(&adc_obj, &ADCconfig);

	/* Initialize ADC channel, allocate channel number 0 to pin 10[6] as this is the first channel initialized */
	const cyhal_adc_channel_config_t channel_config = { .enable_averaging = false, .min_acquisition_ns = 220, .enabled = true };
	result = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, P10_6, CYHAL_ADC_VNEG, &channel_config);


    GUI_Init();

    for (;;)
    {
    	/* Read the ADC conversion result for corresponding ADC channel. Repeat as necessary. */
		adc_out = cyhal_adc_read(&adc_chan_0_obj);
		sprintf(str,"ADC = %d\r\n", adc_out);
		lcd_print_top(str);
		cyhal_system_delay_ms(100);


        sprintf(str,"print line 2");
        lcd_print_line_n(str,2);

        sprintf(str,"print line 4");
        lcd_print_line_n(str,4);

        lcd_print_bot("print bottom");
        cyhal_system_delay_ms(1000);
    }

}


 // Create function for OLED

void lcd_print_top(const char * s){
	GUI_GotoXY(0,0);
	GUI_DispString(s);
}

void lcd_print_bot(const char * s){
	GUI_GotoXY(0,50);
	GUI_DispString(s);
}

void lcd_print_line_n(const char * s,int linenum){ //linenum is 1-6
	int line = (linenum-1)*10;
	GUI_GotoXY(0,line);
	GUI_DispString(s);
}
