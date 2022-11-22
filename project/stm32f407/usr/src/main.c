/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      main.c
 * @brief     main source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-06-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_mpu6050_register_test.h"
#include "driver_mpu6050_read_test.h"
#include "driver_mpu6050_fifo_test.h"
#include "driver_mpu6050_dmp_read_test.h"
#include "driver_mpu6050_dmp_tap_orient_motion_test.h"
#include "driver_mpu6050_dmp_pedometer_test.h"
#include "driver_mpu6050_basic.h"
#include "driver_mpu6050_fifo.h"
#include "driver_mpu6050_dmp.h"
#include "shell.h"
#include "clock.h"
#include "delay.h"
#include "gpio.h"
#include "uart.h"
#include <stdlib.h>

/**
 * @brief global var definition
 */
uint8_t res;
uint32_t i;
uint32_t times = UINT32_MAX;
uint32_t cnt;
uint16_t len;
uint8_t (*g_gpio_irq)(void) = NULL;
static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];      
static float gs_gyro_dps[128][3];        
static int32_t gs_quat[128][4];          
static float gs_pitch[128];              
static float gs_roll[128];                
static float gs_yaw[128];                  
static int32_t gs_quat[128][4];         
static float gs_pitch[128];             
static float gs_roll[128];             
static float gs_yaw[128];     
mpu6050_address_t addr = MPU6050_ADDRESS_AD0_LOW;

/**
 * @brief exti 0 irq
 * @note  none
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief     gpio exti callback
 * @param[in] pin is the gpio pin
 * @note      none
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == GPIO_PIN_0)
    {
        if (g_gpio_irq)
        {
            g_gpio_irq();
        }
    }
}

static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
        {
            mpu6050_interface_debug_print("mpu6050: irq motion.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_I2C_MAST :
        {
            mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DMP :
        {
            mpu6050_interface_debug_print("mpu6050: irq dmp\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DATA_READY :
        {
            mpu6050_interface_debug_print("mpu6050: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
            
            break;
        }
    }
}

static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6050_DMP_TAP_X_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_X_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");
            
            break;
        }
    }
}

static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6050_DMP_ORIENT_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_LANDSCAPE :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");
            
            break;
        }
    }
}

int main(void)
{    
    /* stm32f407 clock init and hal init */
    clock_init();
    
    /* delay init */
    delay_init();
    
    /* uart1 init */
    uart1_init(460800);

		/* init */
		if (gpio_interrupt_init() != 0)
		{
				return 1;
		}
		g_gpio_irq = mpu6050_dmp_irq_handler;

		/* run dmp function */
		if (mpu6050_dmp_init(addr, a_receive_callback, 
												 a_dmp_tap_callback, a_dmp_orient_callback) != 0)
		{
				g_gpio_irq = NULL;
				(void)gpio_interrupt_deinit();

				return 1;
		}

		/* delay 500 ms */
		mpu6050_interface_delay_ms(500);
				
		for (i = 0; i < times; i++)
		{
				len = 128;

				/* read */
				if (mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
																 gs_gyro_raw, gs_gyro_dps, 
																 gs_quat,
																 gs_pitch, gs_roll, gs_yaw,
																 &len) != 0)
				{
						(void)mpu6050_dmp_deinit();
						g_gpio_irq = NULL;
						(void)gpio_interrupt_deinit();

						return 1;
				}

				/* output */
				mpu6050_interface_debug_print("\033[1;1Hmpu6050: %d/%lld. \n", i + 1, times);
				mpu6050_interface_debug_print("mpu6050: fifo %d.\n", len);
				mpu6050_interface_debug_print("mpu6050: pitch[0] is %0.2fdps. \n", gs_pitch[0]);
				mpu6050_interface_debug_print("mpu6050: roll[0] is %0.2fdps. \n", gs_roll[0]);
				mpu6050_interface_debug_print("mpu6050: yaw[0] is %0.2fdps. \n", gs_yaw[0]);
				mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg. \n", gs_accel_g[0][0]);
				mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg. \n", gs_accel_g[0][1]);
				mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg. \n", gs_accel_g[0][2]);
				mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps. \n", gs_gyro_dps[0][0]);
				mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps. \n", gs_gyro_dps[0][1]);
				mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps. \n\033[m", gs_gyro_dps[0][2]);

				mpu6050_interface_delay_ms(10);
								
				/* get the pedometer step count */
				res = mpu6050_dmp_get_pedometer_counter(&cnt);
				if (res != 0)
				{
						(void)mpu6050_dmp_deinit();
						g_gpio_irq = NULL;
						(void)gpio_interrupt_deinit();

						return 1;
				}
		}

		/* deinit */
		(void)mpu6050_dmp_deinit();
		g_gpio_irq = NULL;
		(void)gpio_interrupt_deinit();

		return 0;
			
}
