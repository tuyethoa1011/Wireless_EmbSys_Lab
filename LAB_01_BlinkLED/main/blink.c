/**
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************
  File:	      blink.c
  Modifier:   Team 5
  Updated:    20th MAY 2023
  ***************************************************************************************************************
  Copyright (C) 2023 https://github.com/tuyethoa1011
  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.
  ***************************************************************************************************************
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO //PIN mac dinh la: GPIO_NUM_5

static uint32_t ledState = 0;

//---- function prototype ----
static void config_led(void);
static void blink_led(void);
//----------------------------

//----- FUNCTION ------
static void config_led(void)
{   
    gpio_reset_pin(BLINK_GPIO); //Reset pin to its initial state 
    gpio_pad_select_gpio(BLINK_GPIO); 
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void blink_led(void)
{   
    if(ledState == 1)
    {
        printf("ON\n");
    } else if(ledState == 0)
    {
        printf("OFF\n");
    }
    gpio_set_level(BLINK_GPIO, ledState);
}
//----------------------

void app_main(void)
{   
    /* Configure the peripheral according to the LED type */
    config_led();

    while(1) {
        blink_led();
        //Toggle led state
        ledState = !ledState;
        //blink 500ms len xuong de tao thanh 1 chu ky 1s
        vTaskDelay(500 / portTICK_PERIOD_MS);

    }
}

/*************************************
 Reference Material:
 To understand VtaskDelay & portTICK_PERIOD_MS: https://esp32.com/viewtopic.php?t=2377
 Optimizing source code better: https://controllerstech.com/getting-started-with-esp32-and-esp-idf-led-blinking/
**************************************/