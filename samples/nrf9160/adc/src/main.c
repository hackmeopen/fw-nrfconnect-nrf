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
#include <nrfx.h>
#include <nrfx_saadc.h>
//#include <hal/nrf_saadc.h>
//#include <nrfx_dppi.h> 


struct device *gpio_dev;

#define SAADC_DEFAULT_CONFIG                                               \
{                                                                               \
    .resolution         = (nrf_saadc_resolution_t)NRFX_SAADC_CONFIG_RESOLUTION, \
    .oversample         = (nrf_saadc_oversample_t)NRFX_SAADC_CONFIG_OVERSAMPLE, \
    .interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,                       \
    .low_power_mode     = NRFX_SAADC_CONFIG_LP_MODE                             \
}

#define SAADC_DEFAULT_CHANNEL_CONFIG_SE(PIN_P) \
{                                                   \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
    .gain       = NRF_SAADC_GAIN1_6,                \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
    .acq_time   = NRF_SAADC_ACQTIME_3US,           \
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
    .burst      = NRF_SAADC_BURST_DISABLED,         \
    .pin_p      = (nrf_saadc_input_t)(PIN_P),       \
    .pin_n      = NRF_SAADC_INPUT_DISABLED          \
}

nrf_saadc_channel_config_t saadc_channel_config = SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
static bool adcCalibrateDone;


#define BUFFER_SIZE 3
static nrf_saadc_value_t m_sample_buffer[BUFFER_SIZE];

static void saadc_callback(nrfx_saadc_evt_t const * p_event){

    if ( p_event->type == NRFX_SAADC_EVT_DONE )
    {
       printk("ADC raw value: %d\n", m_sample_buffer[0]);
    }
    else if( p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE )
    {
      adcCalibrateDone = true;
    }
}

void InitAdcModule(void)
{
    nrfx_err_t err_code;
    nrfx_saadc_config_t drv_saadc_cfg = SAADC_DEFAULT_CONFIG;
		
    err_code = nrfx_saadc_init( &drv_saadc_cfg, saadc_callback );
    if(err_code != NRFX_SUCCESS){
        //Blah error
    }
	
    adcCalibrateDone = false;
    err_code = nrfx_saadc_calibrate_offset( );
    if(err_code != NRFX_SUCCESS){
        //Blah error
    }
	
    if( err_code == NRFX_SUCCESS )
    {
	err_code = 0;
	while( !adcCalibrateDone );  
    }
	
}

//void timer_handler(nrf_timer_event_t event_type, void * p_context)
//{
//
//}


//void saadc_sampling_event_init(void)
//{
//    ret_code_t err_code;
//
//    err_code = nrf_drv_ppi_init();
//    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
//    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
//    APP_ERROR_CHECK(err_code);
//
//    /* setup m_timer for compare event every 400ms */
//    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
//    nrf_drv_timer_extended_compare(&m_timer,
//                                   NRF_TIMER_CC_CHANNEL0,
//                                   ticks,
//                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
//                                   false);
//    nrf_drv_timer_enable(&m_timer);
//
//    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
//                                                                                NRF_TIMER_CC_CHANNEL0);
//    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();
//
//    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
//    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
//                                          timer_compare_event_addr,
//                                          saadc_sample_task_addr);
//    APP_ERROR_CHECK(err_code);
//}

static int adc_sample(void)
{
	int ret;


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
	nrfx_err_t err;
        nrf_saadc_value_t  adc_value;

	printk("nrf91 saadc sampling AIN0 (P0.13)\n");
	printk("Example requires secure_boot to have ");
	printk("SAADC set to non-secure!\n");
	printk("If not; BusFault/UsageFault will be triggered\n");

//        gpio_dev = device_get_binding(DT_GPIO_P0_DEV_NAME);
//
//        if (!gpio_dev) {
//		printk("Cannot bind gpio device");
//		return -ENODEV;
//	}
//
//        err = gpio_pin_configure(gpio_dev, 10, GPIO_DIR_OUT);//Set pin 0 to output


        //Startup the high frequency clock
        NRF_CLOCK_NS->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK_NS->EVENTS_HFCLKSTARTED == 0);

        //Configure the timer
//        NRF_TIMER1_NS->TASKS_CLEAR = 1;
//	NRF_TIMER1_NS->PRESCALER = 0;//Run at 16MHz

        InitAdcModule();

	while (1) {

		err = nrfx_saadc_sample();

		if (err) {
			printk("Error in adc sampling: %d\n", err);
		}
		k_sleep(500);

	}
}
