/*******************************************************************************
Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensor.
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "adbms_main.h"
#include "common.h"

/* SPI Configuration */
#define SPI_NODE DT_NODELABEL(spi1)
#define CS_GPIO_NODE DT_ALIAS(cs)

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
static const struct gpio_dt_spec chip_select = GPIO_DT_SPEC_GET_OR(CS_GPIO_NODE, gpios, {0});

void spi_init(void);

int main(void) {
    k_sleep(K_MSEC(1000));
    printk("Initialization Check.\n");
    k_sleep(K_MSEC(1000));

    spi_init();
    adbms_main();
    return 0;
}

void spi_init() {
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not found!\n");
        return;
    }

    if (chip_select.port && device_is_ready(chip_select.port)) {
        gpio_pin_configure_dt(&chip_select, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&chip_select, 1);
    } else {
        printk("Chip select GPIO not found or not ready!\n");
    }

    printk("SPI initialized.\n");
}

