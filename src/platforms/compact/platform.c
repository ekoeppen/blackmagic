/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"
#include "morse.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/f1/adc.h>

static void adc_init(void);
static void setup_vbus_irq(void);

int platform_hwversion(void)
{
	return 0;
}

void platform_init(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_CRC);

	/* Setup GPIO ports */
	gpio_clear(USB_PU_PORT, USB_PU_PIN);
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
			USB_PU_PIN);

	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			TMS_PIN | TCK_PIN | TDI_PIN);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			LED_UART | LED_IDLE_RUN | LED_ERROR);

	platform_srst_set_val(false);
	gpio_set_mode(SRST_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, SRST_PIN);

	if (platform_hwversion() > 0) {
		adc_init();
	} else {
		gpio_clear(GPIOB, GPIO0);
		gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
	}

	platform_timing_init();
	cdcacm_init();
	usbuart_init();
	setup_vbus_irq();
}

void platform_srst_set_val(bool assert)
{
	gpio_set_val(SRST_PORT, SRST_PIN, !assert);
}

bool platform_target_get_power(void)
{
	return 0;
}

void platform_target_set_power(bool power)
{
	(void)power;
}

static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_ANALOG, GPIO0);

	adc_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (int i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);
}

const char *platform_target_voltage(void)
{
	if (platform_hwversion() == 0)
		return gpio_get(GPIOB, GPIO0) ? "OK" : "ABSENT!";

	static char ret[] = "0.0V";
	const uint8_t channel = 8;
	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&channel);

	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1));

	uint32_t val = adc_read_regular(ADC1) * 99; /* 0-4095 */
	ret[0] = '0' + val / 81910;
	ret[2] = '0' + (val / 8191) % 10;

	return ret;
}

void platform_request_boot(void)
{
	/* Disconnect USB cable */
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, 0, USB_PU_PIN);

	/* Drive boot request pin */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOB, GPIO12);
}

void exti15_10_isr(void)
{
	if (gpio_get(USB_VBUS_PORT, USB_VBUS_PIN)) {
		/* Drive pull-up high if VBUS connected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	} else {
		/* Allow pull-up to float if VBUS disconnected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);
	}

	exti_reset_request(USB_VBUS_PIN);
}

static void setup_vbus_irq(void)
{
	nvic_set_priority(USB_VBUS_IRQ, IRQ_PRI_USB_VBUS);
	nvic_enable_irq(USB_VBUS_IRQ);

	gpio_set(USB_VBUS_PORT, USB_VBUS_PIN);
	gpio_set(USB_PU_PORT, USB_PU_PIN);

	gpio_set_mode(USB_VBUS_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, USB_VBUS_PIN);

	/* Configure EXTI for USB VBUS monitor */
	exti_select_source(USB_VBUS_PIN, USB_VBUS_PORT);
	exti_set_trigger(USB_VBUS_PIN, EXTI_TRIGGER_BOTH);
	exti_enable_request(USB_VBUS_PIN);

	exti15_10_isr();
}

