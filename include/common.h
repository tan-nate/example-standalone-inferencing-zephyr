/*******************************************************************************
Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensor.
******************************************************************************
* @file:    common.h
* @brief:   Standard common header files
* @version: $Revision$
* @date:    $Date$
* Developed by: ADIBMS Software team, Bangalore, India
*****************************************************************************/
#ifndef __COMMON_H_
#define __COMMON_H_

// Zephyr 3.1.x and newer uses different include scheme
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>      // For debug printing

// Edge Impulse SDK
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

#ifdef EI_NORDIC
#include <nrfx_clock.h>
#endif

#endif /* __COMMON_H_ */
