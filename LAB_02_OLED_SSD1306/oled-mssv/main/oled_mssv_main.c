#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "ssd1306.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)


#define CONFIG_SSD1306_ADDR 0x3C
#define CONFIG_SSD1306_OPMODE 0x3C

#define I2C_MASTER_SCL_IO 4            /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5              /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define SSD1306_OLED_ADDR   CONFIG_SSD1306_ADDR  /*!< slave address for OLED SSD1306 */
#define SSD1306_CMD_START CONFIG_SSD1306_OPMODE   /*!< Operation mode */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */


//khởi tạo biến semaphore
SemaphoreHandle_t sem1 = NULL;

static void task_id_display_oled(void *arg)
{   
    ssd1306_init(I2C_MASTER_NUM);
    uint32_t task_idx = (uint32_t)arg;
    task_ssd1306_display_clear(I2C_MASTER_NUM);
    while(1) //forever loop để chạy task
    {   
        xSemaphoreTake(sem1, portMAX_DELAY);
        //task_ssd1306_display_text("Lab2_OLED", I2C_MASTER_NUM); 
        task_ssd1306_display_text("20521333\n20521739\n20521816\n20521886", I2C_MASTER_NUM); 
        xSemaphoreGive(sem1);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
    }
    vSemaphoreDelete(sem1);
    vTaskDelete(NULL);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void app_main(void)
{   
    sem1 = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(task_id_display_oled,"i2c_oled_task_0",1024*2,(void *)0, 10, NULL);
}