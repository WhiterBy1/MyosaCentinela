/* gesture_test_main.c
 *
 * Prueba MINIMA y AISLADA del APDS9960, análoga al Gesture Test.ino de
 * ejemplo (apds.begin() / apds.enableProximity() / apds.enableGesture() /
 * apds.readGesture()). Sin OLED, sin MPU6050, sin BMP180: solo el sensor
 * de gestos y el puerto serie.
 *
 * Usa el mismo bus I2C compartido (SDA=GPIO21, SCL=GPIO22) y la misma
 * librería oficial espressif/apds9960 que ya tenemos en el proyecto, para
 * no depender de mover el sensor de pines.
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "apds9960.h"

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_FREQ_HZ  100000

void app_main(void)
{
    printf("\n");

    /* ---- setup() ---- */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus_handle_t bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (bus == NULL) {
        printf("failed to initialize device! Please check your wiring.\n");
        return;
    }

    apds9960_handle_t apds = apds9960_create(bus, APDS9960_I2C_ADDRESS);
    if (apds == NULL) {
        printf("failed to initialize device! Please check your wiring.\n");
        return;
    }

    uint8_t id = 0;
    if (apds9960_get_deviceid(apds, &id) != ESP_OK) {
        printf("failed to initialize device! Please check your wiring.\n");
        return;
    }
    printf("Device initialized! (ID: 0x%02X)\n", id);

    /* apds9960_gesture_init() ya deja el sensor con proximidad + gesto
     * habilitados (equivalente a apds.enableProximity(true) +
     * apds.enableGesture(true) del sketch de Arduino). */
    if (apds9960_gesture_init(apds) != ESP_OK) {
        printf("failed to enable proximity/gesture engine!\n");
        return;
    }

    /* ---- loop() ---- */
    int64_t last_raw_us = 0;
    while (1) {
        /* Lectura CRUDA de proximidad, independiente del chequeo interno
         * de GVALID que hace apds9960_read_gesture(). Esto es para saber,
         * sin ninguna duda, si PDATA se mueve con la mano ahora que subimos
         * PGAIN a 8x y el LED boost a 300%. Si esto se queda en 0 siempre,
         * el problema ya no es de configuración sino físico (LED IR del
         * módulo, o los 4 fotodiodos de gesto tapados/dañados). */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_raw_us > 300000) {
            uint8_t prox = apds9960_read_proximity(apds);
            bool gvalid = apds9960_gesture_valid(apds);
            printf("[raw] PROX=%3u GVALID=%d\n", (unsigned)prox, gvalid ? 1 : 0);
            last_raw_us = now_us;
        }

        uint8_t gesture = apds9960_read_gesture(apds);
        if (gesture == APDS9960_DOWN) {
            printf("down\n");
        } else if (gesture == APDS9960_UP) {
            printf("up\n");
        } else if (gesture == APDS9960_LEFT) {
            printf("left\n");
        } else if (gesture == APDS9960_RIGHT) {
            printf("right\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
