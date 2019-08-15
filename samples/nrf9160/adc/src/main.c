/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <gpio.h>
#include <net/socket.h>
#include <nrf9160.h>
#include <stdio.h>
#include <string.h>
#include <uart.h>
#include <adc.h>
#include <zephyr.h>

struct device *adc_dev;
struct device *gpio_dev;

#include <hal/nrf_saadc.h>
#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0
#define ADC_2ND_CHANNEL_ID 2
#define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2
#define ADC_3RD_CHANNEL_ID 3
#define ADC_3RD_CHANNEL_INPUT NRF_SAADC_INPUT_AIN3

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_1ST_CHANNEL_INPUT,
#endif
};

static const struct adc_channel_cfg m_2nd_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_2ND_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_2ND_CHANNEL_INPUT,
#endif
};

static const struct adc_channel_cfg m_3rd_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_3RD_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_3RD_CHANNEL_INPUT,
#endif
};


#define BUFFER_SIZE 3
static s16_t m_sample_buffer[BUFFER_SIZE];

static int adc_sample(void)
{
	int ret;

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID) | BIT(ADC_2ND_CHANNEL_ID) | BIT(ADC_3RD_CHANNEL_ID),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return -1;
	}
        gpio_pin_write(gpio_dev, 10, 1);    
	ret = adc_read(adc_dev, &sequence);
        gpio_pin_write(gpio_dev, 10, 0);

	/*printk("ADC read err: %d\n", ret);

	/* Print the AIN0 values */
	/*for (int i = 0; i < BUFFER_SIZE; i++) {
		float adc_voltage = 0;
		adc_voltage = (float)(((float)m_sample_buffer[i] / 1023.0f) *
				      3600.0f);
		printk("ADC raw value: %d\n", m_sample_buffer[i]);
		printf("Measured voltage: %f mV\n", adc_voltage);
	}
*/
	return ret;
}



static inline void nrf_delay_us(u32_t microsec)
{
	NRF_TIMER1_NS->TASKS_CLEAR = 1;
	if (microsec < 2 )
		return;
	NRF_TIMER1_NS->CC[0] = (microsec << 4) - 26;
	NRF_TIMER1_NS->PRESCALER = 0;
	NRF_TIMER1_NS->TASKS_START = 1;
	while (NRF_TIMER1_NS->EVENTS_COMPARE[0] == 0)
		;
	NRF_TIMER1_NS->EVENTS_COMPARE[0] = 0;
	NRF_TIMER1_NS->TASKS_STOP = 1;	
}

int main(void)
{
	int err;
        int timer_count = 0;

	printk("nrf91 saadc sampling AIN0 (P0.13)\n");
	printk("Example requires secure_boot to have ");
	printk("SAADC set to non-secure!\n");
	printk("If not; BusFault/UsageFault will be triggered\n");

	adc_dev = device_get_binding("ADC_0");
	if (!adc_dev) {
		printk("device_get_binding ADC_0 failed\n");
	}
	/*err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (err) {
		printk("Error in adc setup: %d\n", err);
	}

        err = adc_channel_setup(adc_dev, &m_2nd_channel_cfg);
	if (err) {
		printk("Error in adc setup: %d\n", err);
	}

        err = adc_channel_setup(adc_dev, &m_3rd_channel_cfg);
	if (err) {
		printk("Error in adc setup: %d\n", err);
	}
*/
        gpio_dev = device_get_binding(DT_GPIO_P0_DEV_NAME);

        if (!gpio_dev) {
		printk("Cannot bind gpio device");
		return -ENODEV;
	}

        err = gpio_pin_configure(gpio_dev, 10, GPIO_DIR_OUT);//Set pin 0 to output


        //Startup the high frequency clock
        NRF_CLOCK_NS->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK_NS->EVENTS_HFCLKSTARTED == 0);

        //Configure the timer
        NRF_TIMER1_NS->TASKS_CLEAR = 1;
	NRF_TIMER1_NS->PRESCALER = 0;//Run at 16MHz


	/* Trigger offset calibration
	 * As this generates a _DONE and _RESULT event
	 * the first result will be incorrect.
	 */
	NRF_SAADC_NS->TASKS_CALIBRATEOFFSET = 1;
	while (1) {

             //   timer_count = 0;
              //  NRF_TIMER1_NS->TASKS_CLEAR = 1;//Clear the timer
              //  NRF_TIMER1_NS->TASKS_START = 1;//Start the timer


           //     gpio_pin_write(gpio_dev, 10, 1);
		err = adc_sample();
//                gpio_pin_write(gpio_dev, 10, 0);
              //  NRF_TIMER1_NS->TASKS_CAPTURE[0] = 1;
                
            //    NRF_TIMER1_NS->TASKS_STOP = 1;	
            //    timer_count = NRF_TIMER1_NS->CC[0];
           //     printk("Timer Count: %d\n", err);

		if (err) {
			printk("Error in adc sampling: %d\n", err);
		}
		k_sleep(500);

	}
}
