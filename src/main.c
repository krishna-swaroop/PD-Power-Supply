#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "TPS55289.h"

#define LED_PIN 25
#define RED_LED 23

#define GPIO_ON     1
#define GPIO_OFF    0

void GreenLEDTask(void *param)
{
    for (;;)
    {
        /* code */
        gpio_put(LED_PIN, GPIO_ON);
        vTaskDelay(1000);
        gpio_put(LED_PIN, GPIO_OFF);
        vTaskDelay(1000);
    }
}

void RedLEDTask(void *param)
{
    for(;;){
        gpio_put(RED_LED, GPIO_ON);
        vTaskDelay(100);
        gpio_put(RED_LED, GPIO_OFF);
        vTaskDelay(100);
    }
}

int main() 
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(RED_LED);
    gpio_set_dir(RED_LED, GPIO_OUT);

    TaskHandle_t gLEDtask = NULL;
    TaskHandle_t rLEDtask = NULL;

    // TPS55289 device;

    // TPS55289Init(&device);

    // uint8_t registe = device.TPS55289_IOUT_LIMIT.regValue;



    uint32_t status = xTaskCreate(
                    GreenLEDTask,
                    "Green LED",
                    1024,
                    NULL,
                    tskIDLE_PRIORITY,
                    &gLEDtask);

    status = xTaskCreate(
                    RedLEDTask,
                    "Red LED",
                    1024,
                    NULL,
                    tskIDLE_PRIORITY,
                    &rLEDtask);

    vTaskStartScheduler();

    for( ;; )
    {
        //should never get here
    }

}