/*
 * MyosaCentinela - firmware de CAMPO combinado (sensores I2C + LoRa)
 *
 * Junta lo ya validado por separado:
 *   - Sensores I2C (i2c_basic_example_main.c): OLED, MPU6050, BMP180,
 *     APDS9960, con el mismo protocolo TLM:/RATE:/EVT: por UART0 para
 *     seguir usando tools/dashboard.html en desarrollo.
 *   - Enlace LoRa (lora_wioe5_test_main.c): Wio-E5 por UART2, 903.9MHz
 *     (canal 8 de la rejilla US915 que vigila el gateway HT-M7603),
 *     SF7/BW125, sync word publico (NET=ON) - confirmado recibido por
 *     el gateway con RSSI/SNR reales antes de este merge.
 *
 * Lo nuevo es lora_task (CORE 1, junto a monitor_task): cada
 * LORA_TX_PERIOD_MS arma un payload compacto (sin JSON, sin comillas,
 * para no romper el parseo de AT+TEST=TXLRSTR) con los mismos datos que
 * ya se cachean para el TLM: por UART, y lo transmite por radio.
 *
 * Pines (sin conflicto entre si):
 *   I2C:   SDA=GPIO21 SCL=GPIO22   (OLED 0x3C, MPU6050 0x68/0x69, BMP180
 *          0x77, APDS9960 0x39)
 *   LoRa:  UART2 TX=GPIO17 RX=GPIO16 (Wio-E5, NUNCA en GPIO1/3: eso es
 *          la consola/USB y compite con el flasheo si se comparte)
 *   USB:   UART0 GPIO1/3 (consola, monitor, flasheo)
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "i2c_bus.h"
#include "apds9960.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "sensores";
static const char *LORA_TAG = "lora";

#define ENABLE_MPU6050  1
#define ENABLE_BMP180   1

/* ---------- Config I2C ---------- */
#define I2C_MASTER_SCL_IO      GPIO_NUM_22
#define I2C_MASTER_SDA_IO      GPIO_NUM_21
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_TIMEOUT_MS         1000

/* ---------- Config OLED ---------- */
#define OLED_ADDR               0x3C
#define OLED_WIDTH               128
#define OLED_HEIGHT                64
#define OLED_PAGES         (OLED_HEIGHT / 8)
#define OLED_CTRL_CMD             0x00
#define OLED_CTRL_DATA             0x40

/* ---------- Config MPU6050 ---------- */
#define MPU6050_REG_PWR_MGMT_1                0x6B
#define MPU6050_REG_ACCEL_XOUT_H               0x3B
#define ACCEL_SCALE_2G                    16384.0f
#define GYRO_SCALE_250DPS                   131.0f

/* ---------- Config BMP180 ---------- */
#define BMP180_ADDR              0x77
#define BMP180_REG_CALIB          0xAA
#define BMP180_REG_CONTROL        0xF4
#define BMP180_REG_RESULT         0xF6
#define BMP180_CMD_TEMP           0x2E
#define BMP180_CMD_PRESSURE       0x34

/* ---------- Config de telemetría / eventos ---------- */
#define TELEMETRY_PERIOD_MS       50   /* ~20Hz por UART0 (dashboard local) */
#define RATE_REPORT_PERIOD_US 5000000
#define BMP_CACHE_PERIOD_MS     1000
#define APDS_CACHE_PERIOD_MS     300

static uint32_t gesture_cooldown_ms = 1;

static i2c_master_dev_handle_t oled_handle;
static i2c_master_dev_handle_t mpu_handle;
static i2c_master_dev_handle_t bmp_handle;
static i2c_bus_handle_t shared_i2c_bus = NULL;
static apds9960_handle_t apds_sensor = NULL;

static bool mpu_ok = false;
static bool bmp_ok = false;
static bool apds_ok = false;

/* =======================================================================
 *  Fuente 5x7 (subconjunto necesario + flechas para el juego)
 * ======================================================================= */
typedef struct {
    char c;
    uint8_t bits[5];
} font_glyph_t;

static const font_glyph_t font5x7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x41, 0x3E}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x40, 0x40, 0x3F}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x03, 0x04, 0x78, 0x04, 0x03}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
    {'^', {0x04, 0x06, 0x7F, 0x06, 0x04}},   /* arriba */
    {'v', {0x30, 0x70, 0x7F, 0x70, 0x30}},   /* abajo */
    {'<', {0x00, 0x08, 0x1C, 0x3E, 0x7F}},   /* izquierda */
    {'>', {0x7F, 0x3E, 0x1C, 0x08, 0x00}},   /* derecha */
    {'?', {0x02, 0x01, 0x51, 0x09, 0x06}},
};

static const uint8_t *get_glyph(char c)
{
    for (size_t i = 0; i < sizeof(font5x7) / sizeof(font5x7[0]); i++) {
        if (font5x7[i].c == c) {
            return font5x7[i].bits;
        }
    }
    return font5x7[0].bits;
}

/* =======================================================================
 *  OLED - funciones de bajo nivel
 * ======================================================================= */
static esp_err_t oled_write(uint8_t ctrl, const uint8_t *data, size_t len)
{
    uint8_t buf[len + 1];
    buf[0] = ctrl;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(oled_handle, buf, len + 1, I2C_TIMEOUT_MS);
}

static esp_err_t oled_cmd(uint8_t cmd)
{
    return oled_write(OLED_CTRL_CMD, &cmd, 1);
}

static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_cmd(0xB0 + page);
    oled_cmd(0x00 + (col & 0x0F));
    oled_cmd(0x10 + (col >> 4));
}

static void oled_init_sequence(void)
{
    const uint8_t init_cmds[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4,
        0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12,
        0xDB, 0x40, 0x8D, 0x14, 0xAF
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        ESP_ERROR_CHECK(oled_cmd(init_cmds[i]));
    }
}

static void oled_clear(void)
{
    uint8_t blank[OLED_WIDTH] = {0};
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_cursor(page, 0);
        oled_write(OLED_CTRL_DATA, blank, sizeof(blank));
    }
}

static void oled_draw_string(uint8_t page, uint8_t col, const char *str)
{
    uint8_t data[6];
    oled_set_cursor(page, col);
    while (*str) {
        const uint8_t *glyph = get_glyph(*str);
        memcpy(data, glyph, 5);
        data[5] = 0x00;
        oled_write(OLED_CTRL_DATA, data, sizeof(data));
        str++;
    }
}

static void oled_clear_line(uint8_t page)
{
    uint8_t blank[OLED_WIDTH] = {0};
    oled_set_cursor(page, 0);
    oled_write(OLED_CTRL_DATA, blank, sizeof(blank));
}

/* =======================================================================
 *  MPU6050
 * ======================================================================= */
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float temp_c;
} mpu6050_data_t;

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(mpu_handle, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t mpu6050_init(void)
{
    return mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
}

static esp_err_t mpu6050_read(mpu6050_data_t *out)
{
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t raw[14];

    esp_err_t err = i2c_master_transmit_receive(mpu_handle, &reg, 1, raw, sizeof(raw), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax_raw = (raw[0] << 8) | raw[1];
    int16_t ay_raw = (raw[2] << 8) | raw[3];
    int16_t az_raw = (raw[4] << 8) | raw[5];
    int16_t temp_raw = (raw[6] << 8) | raw[7];
    int16_t gx_raw = (raw[8] << 8) | raw[9];
    int16_t gy_raw = (raw[10] << 8) | raw[11];
    int16_t gz_raw = (raw[12] << 8) | raw[13];

    out->ax = ax_raw / ACCEL_SCALE_2G;
    out->ay = ay_raw / ACCEL_SCALE_2G;
    out->az = az_raw / ACCEL_SCALE_2G;
    out->gx = gx_raw / GYRO_SCALE_250DPS;
    out->gy = gy_raw / GYRO_SCALE_250DPS;
    out->gz = gz_raw / GYRO_SCALE_250DPS;
    out->temp_c = (temp_raw / 340.0f) + 36.53f;

    return ESP_OK;
}

#define COMP_FILTER_ALPHA   0.98f

typedef struct {
    float roll, pitch, yaw;
} orientation_t;

static orientation_t g_orient = {0};
static bool g_orient_initialized = false;
static mpu6050_data_t g_mpu_data = {0};

static void orientation_update(const mpu6050_data_t *m, float dt)
{
    float accel_roll  = atan2f(m->ay, m->az) * 180.0f / (float)M_PI;
    float accel_pitch = atan2f(-m->ax, sqrtf(m->ay * m->ay + m->az * m->az)) * 180.0f / (float)M_PI;

    if (!g_orient_initialized) {
        g_orient.roll = accel_roll;
        g_orient.pitch = accel_pitch;
        g_orient.yaw = 0.0f;
        g_orient_initialized = true;
        return;
    }

    g_orient.roll  = COMP_FILTER_ALPHA * (g_orient.roll  + m->gx * dt) + (1.0f - COMP_FILTER_ALPHA) * accel_roll;
    g_orient.pitch = COMP_FILTER_ALPHA * (g_orient.pitch + m->gy * dt) + (1.0f - COMP_FILTER_ALPHA) * accel_pitch;
    g_orient.yaw  += m->gz * dt;
}

/* Implementadas mas abajo, en el modulo de deteccion de caidas
 * embebida - mpu_task (justo debajo) las llama en cada muestra nueva,
 * asi que necesitan forward-declare aqui (mismo patron que tcp_send_line
 * mas abajo en el archivo). */
static void fall_buffer_push(int64_t t_us, float mag, float gyro_mag, float roll, float pitch);
static void fall_check_trigger(float mag, int64_t now_us);

/* El detector de caidas se entreno con datos de telemetria.csv, que se
 * graba a 20Hz (TELEMETRY_PERIOD_MS) - asi que el buffer/features del
 * detector tienen que alimentarse tambien a 20Hz, NO a los 100Hz de
 * mpu_task. Con 100Hz se cuela mucho mas ruido/microvibracion normal
 * (caminar, pulso) que a 20Hz queda promediado - eso infla mag_std/
 * gyro_std/late_std frente a lo que el modelo aprendio, y dispara con
 * casi cualquier movimiento. mpu_task sigue leyendo el sensor a 100Hz
 * para orientacion/gestos (eso no cambia), pero solo empuja al detector
 * cada FALL_SAMPLE_PERIOD_US. */
#define FALL_SAMPLE_PERIOD_US   (TELEMETRY_PERIOD_MS * 1000)

/* CORE 0: solo el MPU6050, a ~100Hz, sin depender de nada más. */
static void mpu_task(void *arg)
{
    (void)arg;
    int64_t last_us = esp_timer_get_time();
    int64_t last_fall_sample_us = 0;
    while (1) {
        if (mpu_ok) {
            int64_t now_us = esp_timer_get_time();
            float dt = (now_us - last_us) / 1000000.0f;
            last_us = now_us;
            if (mpu6050_read(&g_mpu_data) == ESP_OK) {
                orientation_update(&g_mpu_data, dt);
                if (now_us - last_fall_sample_us >= FALL_SAMPLE_PERIOD_US) {
                    last_fall_sample_us = now_us;
                    float mag = sqrtf(g_mpu_data.ax * g_mpu_data.ax +
                                       g_mpu_data.ay * g_mpu_data.ay +
                                       g_mpu_data.az * g_mpu_data.az);
                    float gyro_mag = sqrtf(g_mpu_data.gx * g_mpu_data.gx +
                                            g_mpu_data.gy * g_mpu_data.gy +
                                            g_mpu_data.gz * g_mpu_data.gz);
                    fall_buffer_push(now_us, mag, gyro_mag, g_orient.roll, g_orient.pitch);
                    fall_check_trigger(mag, now_us);
                }
            }
        } else {
            last_us = esp_timer_get_time();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =======================================================================
 *  BMP180 (temperatura / presión / altitud)
 * ======================================================================= */
typedef struct {
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
} bmp180_calib_t;

static bmp180_calib_t bmp_cal;

static esp_err_t bmp180_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(bmp_handle, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t bmp180_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_master_transmit(bmp_handle, b, sizeof(b), I2C_TIMEOUT_MS);
}

static esp_err_t bmp180_read_calibration(void)
{
    uint8_t buf[22];
    esp_err_t err = bmp180_read_bytes(BMP180_REG_CALIB, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }
    bmp_cal.ac1 = (buf[0] << 8) | buf[1];
    bmp_cal.ac2 = (buf[2] << 8) | buf[3];
    bmp_cal.ac3 = (buf[4] << 8) | buf[5];
    bmp_cal.ac4 = (buf[6] << 8) | buf[7];
    bmp_cal.ac5 = (buf[8] << 8) | buf[9];
    bmp_cal.ac6 = (buf[10] << 8) | buf[11];
    bmp_cal.b1  = (buf[12] << 8) | buf[13];
    bmp_cal.b2  = (buf[14] << 8) | buf[15];
    bmp_cal.mb  = (buf[16] << 8) | buf[17];
    bmp_cal.mc  = (buf[18] << 8) | buf[19];
    bmp_cal.md  = (buf[20] << 8) | buf[21];
    return ESP_OK;
}

static esp_err_t bmp180_read_raw_temp(int32_t *ut)
{
    esp_err_t err = bmp180_write_reg(BMP180_REG_CONTROL, BMP180_CMD_TEMP);
    if (err != ESP_OK) return err;
    /* Datasheet: 4.5ms max a oss=0. 10ms da margen real (este modulo
     * necesitaba mas de 5ms en la practica: con 5ms el registro de
     * resultado a veces todavia traia la conversion anterior). */
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t buf[2];
    err = bmp180_read_bytes(BMP180_REG_RESULT, buf, sizeof(buf));
    if (err != ESP_OK) return err;
    *ut = (buf[0] << 8) | buf[1];
    return ESP_OK;
}

static esp_err_t bmp180_read_raw_pressure(int32_t *up)
{
    esp_err_t err = bmp180_write_reg(BMP180_REG_CONTROL, BMP180_CMD_PRESSURE);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t buf[3];
    err = bmp180_read_bytes(BMP180_REG_RESULT, buf, sizeof(buf));
    if (err != ESP_OK) return err;
    *up = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 8;
    return ESP_OK;
}

static esp_err_t bmp180_read(float *temp_c, float *pressure_pa, float *altitude_m)
{
    int32_t ut, up;
    esp_err_t err = bmp180_read_raw_temp(&ut);
    if (err != ESP_OK) return err;
    err = bmp180_read_raw_pressure(&up);
    if (err != ESP_OK) return err;

    int32_t x1 = ((ut - (int32_t)bmp_cal.ac6) * (int32_t)bmp_cal.ac5) >> 15;
    int32_t x2 = ((int32_t)bmp_cal.mc << 11) / (x1 + bmp_cal.md);
    int32_t b5 = x1 + x2;
    *temp_c = ((b5 + 8) >> 4) / 10.0f;

    int32_t b6 = b5 - 4000;
    x1 = ((int32_t)bmp_cal.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = ((int32_t)bmp_cal.ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((int32_t)bmp_cal.ac1 * 4 + x3) + 2) >> 2;

    x1 = ((int32_t)bmp_cal.ac3 * b6) >> 13;
    x2 = ((int32_t)bmp_cal.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)bmp_cal.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * 50000UL;

    int32_t p;
    if (b7 < 0x80000000UL) {
        p = (int32_t)((b7 * 2) / b4);
    } else {
        p = (int32_t)((b7 / b4) * 2);
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    *pressure_pa = (float)p;
    *altitude_m = 44330.0f * (1.0f - powf(p / 101325.0f, 1.0f / 5.255f));

    return ESP_OK;
}

/* =======================================================================
 *  APDS9960 (gestos)
 * ======================================================================= */
static esp_err_t apds9960_setup(bool *out_ok)
{
    *out_ok = false;

    if (shared_i2c_bus == NULL) {
        ESP_LOGE(TAG, "Bus I2C compartido no inicializado");
        return ESP_FAIL;
    }

    apds_sensor = apds9960_create(shared_i2c_bus, APDS9960_I2C_ADDRESS);
    if (apds_sensor == NULL) {
        ESP_LOGE(TAG, "No se pudo crear el objeto APDS9960 (revisa cableado/dirección 0x39)");
        return ESP_FAIL;
    }

    uint8_t id = 0;
    esp_err_t err = apds9960_get_deviceid(apds_sensor, &id);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "APDS9960 ID: 0x%02X", id);

    err = apds9960_gesture_init(apds_sensor);
    if (err != ESP_OK) {
        return err;
    }

    err = apds9960_enable_color_engine(apds_sensor, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo habilitar el motor de color/luz ambiente (err=0x%x)", err);
    }

    *out_ok = true;
    return ESP_OK;
}

typedef struct {
    uint8_t id;
    uint8_t prox;
    bool gvalid;
    uint16_t r, g, b, c;
} apds9960_debug_t;

static esp_err_t apds9960_read_debug(apds9960_debug_t *d)
{
    esp_err_t err = apds9960_get_deviceid(apds_sensor, &d->id);
    if (err != ESP_OK) return err;
    d->prox = apds9960_read_proximity(apds_sensor);
    d->gvalid = apds9960_gesture_valid(apds_sensor);
    return apds9960_get_color_data(apds_sensor, &d->r, &d->g, &d->b, &d->c);
}

/* =======================================================================
 *  Caché de fondo: BMP180 y proximidad/luz del APDS9960
 * ======================================================================= */
static float g_last_temp_c = 0.0f;
static float g_last_pres_hpa = 0.0f;
static float g_last_alt_m = 0.0f;
static uint8_t g_last_prox = 0;
static uint16_t g_last_light = 0;

/* Forward-declare (definida junto al modulo WiFi/UDP mas abajo, mismo
 * patron ya usado en el resto del archivo): la necesita
 * battery_check_low() para mandar el EVT de bateria baja. */
static void tcp_send_line(const char *line);

/* =======================================================================
 *  Bateria: divisor resistivo 62k/62k desde la salida del boost 18650->5V
 *  (con diodo de proteccion, llega a ~4.70V max) hasta GND, punto medio
 *  en BATT_ADC_GPIO. R1=R2 -> Vrail = Vadc_medido * 2.
 *
 *  OJO: esto mide el RAIL DE SALIDA DEL BOOST, no la celda 18650
 *  directamente. Un boost mantiene esa salida casi constante mientras
 *  puede regular, asi que el voltaje se va a quedar cerca de su maximo
 *  la mayor parte de la descarga y recien cae cuando el boost entra en
 *  dropout (celda ya bastante baja). BATT_LOW_THRESHOLD_V es un punto de
 *  partida conservador - hay que descargar la bateria una vez viendo la
 *  lectura en el OLED/dashboard y ajustar este numero al punto real
 *  donde empieza a caer en este modulo especifico.
 * ======================================================================= */
#define BATT_ADC_GPIO            GPIO_NUM_34
#define BATT_ADC_UNIT            ADC_UNIT_1
#define BATT_ADC_ATTEN           ADC_ATTEN_DB_12
#define BATT_DIVIDER_RATIO       2.0f   /* R1=R2=62k -> Vrail = Vadc * 2 */
#define BATT_CACHE_PERIOD_MS     1000
#define BATT_LOW_THRESHOLD_V     4.40f  /* AJUSTAR tras probar una descarga real */
#define BATT_LOW_HYSTERESIS_V    0.10f  /* recupera por encima de threshold+hyst */
#define BATT_LOW_DEBOUNCE_N      5      /* muestras seguidas antes de declarar bajo */

static adc_oneshot_unit_handle_t g_batt_adc_handle = NULL;
static adc_cali_handle_t g_batt_cali_handle = NULL;
static adc_channel_t g_batt_adc_channel;
static bool g_batt_adc_ok = false;

static float g_last_batt_v = 0.0f;
static bool g_batt_low = false;

static void battery_adc_init(void)
{
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel(BATT_ADC_GPIO, &unit, &g_batt_adc_channel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d no es un pin ADC valido (err=0x%x)", BATT_ADC_GPIO, err);
        return;
    }

    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = BATT_ADC_UNIT };
    if (adc_oneshot_new_unit(&init_cfg, &g_batt_adc_handle) != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo crear la unidad ADC para bateria");
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(g_batt_adc_handle, g_batt_adc_channel, &chan_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo configurar el canal ADC de bateria");
        return;
    }

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = BATT_ADC_UNIT,
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &g_batt_cali_handle) != ESP_OK) {
        ESP_LOGW(TAG, "Calibracion ADC no disponible, se usara el crudo sin calibrar");
        g_batt_cali_handle = NULL;
    }

    g_batt_adc_ok = true;
    ESP_LOGI(TAG, "ADC bateria OK en GPIO%d", BATT_ADC_GPIO);
}

/* EVT edge-triggered con debounce+histeresis, para no spamear un EVT por
 * segundo mientras la bateria esta baja ni disparar con un solo dato
 * ruidoso. */
static void battery_check_low(void)
{
    static int low_count = 0;

    bool low_now = g_last_batt_v < BATT_LOW_THRESHOLD_V;
    bool recovered_now = g_last_batt_v > (BATT_LOW_THRESHOLD_V + BATT_LOW_HYSTERESIS_V);

    if (low_now) {
        if (low_count < BATT_LOW_DEBOUNCE_N) low_count++;
    } else if (recovered_now) {
        low_count = 0;
    }

    char line[64];
    if (!g_batt_low && low_count >= BATT_LOW_DEBOUNCE_N) {
        g_batt_low = true;
        snprintf(line, sizeof(line),
               "EVT:{\"t\":%" PRId64 ",\"type\":\"BATERIA_BAJA\",\"dir\":\"\"}\n",
               esp_timer_get_time() / 1000);
        fputs(line, stdout);
        tcp_send_line(line);
        ESP_LOGW(TAG, "*** BATERIA BAJA: %.2fV ***", g_last_batt_v);
    } else if (g_batt_low && recovered_now) {
        g_batt_low = false;
        snprintf(line, sizeof(line),
               "EVT:{\"t\":%" PRId64 ",\"type\":\"BATERIA_OK\",\"dir\":\"\"}\n",
               esp_timer_get_time() / 1000);
        fputs(line, stdout);
        tcp_send_line(line);
    }
}

static void update_background_cache(void)
{
    static int64_t last_bmp_us = 0;
    static int64_t last_apds_us = 0;
    static int64_t last_batt_us = 0;
    int64_t now_us = esp_timer_get_time();

    if (bmp_ok && (now_us - last_bmp_us > (int64_t)BMP_CACHE_PERIOD_MS * 1000)) {
        float t, p, alt;
        if (bmp180_read(&t, &p, &alt) == ESP_OK) {
            float p_hpa = p / 100.0f;
            /* A veces (aun con el delay de conversion en 10ms) el
             * registro de resultado trae la lectura anterior a medio
             * actualizar y sale un numero fisicamente imposible (ej.
             * 615 hPa = ~4000m de altitud estando en el suelo). En vez
             * de perseguir mas margen de timing, se descarta la lectura
             * mala y se mantiene el ultimo valor bueno - la presion
             * real no cambia de golpe entre un segundo y el siguiente,
             * asi que quedarse con el valor previo no distorsiona nada. */
            if (p_hpa < 300.0f || p_hpa > 1100.0f || t < -40.0f || t > 85.0f) {
                ESP_LOGW(TAG, "BMP180 lectura descartada (fuera de rango): t=%.1f p=%.1fhPa", t, p_hpa);
            } else {
                g_last_temp_c = t;
                g_last_pres_hpa = p_hpa;
                g_last_alt_m = alt;
            }
        }
        last_bmp_us = now_us;
    }

    if (apds_ok && (now_us - last_apds_us > (int64_t)APDS_CACHE_PERIOD_MS * 1000)) {
        apds9960_debug_t d;
        if (apds9960_read_debug(&d) == ESP_OK) {
            g_last_prox = d.prox;
            g_last_light = d.c;
        }
        last_apds_us = now_us;
    }

    if (g_batt_adc_ok && (now_us - last_batt_us > (int64_t)BATT_CACHE_PERIOD_MS * 1000)) {
        int mv = 0;
        esp_err_t err;
        if (g_batt_cali_handle) {
            err = adc_oneshot_get_calibrated_result(g_batt_adc_handle, g_batt_cali_handle, g_batt_adc_channel, &mv);
        } else {
            int raw = 0;
            err = adc_oneshot_read(g_batt_adc_handle, g_batt_adc_channel, &raw);
            /* Sin calibracion: aproximacion cruda para DB_12 (fondo de escala
             * ~2500mV en ~4095 cuentas a 12 bits) - suficiente como respaldo. */
            mv = (raw * 2500) / 4095;
        }
        if (err == ESP_OK) {
            g_last_batt_v = (mv / 1000.0f) * BATT_DIVIDER_RATIO;
            /* DEBUG temporal: sacar despues de confirmar que la lectura
             * tiene sentido - ayuda a ver el mV crudo del pin (antes del
             * *2 del divisor) para diagnosticar. */
            ESP_LOGI(TAG, "BATT_DEBUG: adc_mv=%d -> Vrail=%.2fV", mv, g_last_batt_v);
            battery_check_low();
        } else {
            ESP_LOGW(TAG, "BATT_DEBUG: lectura ADC fallo (err=0x%x)", err);
        }
        last_batt_us = now_us;
    }
}

/* Contador de eventos de gesto, compartido con el payload LoRa. */
static volatile unsigned int g_event_count = 0;

/* Estado de WiFi, escrito por el modulo WiFi (mas abajo) y leido aqui
 * para incluirlo en el payload LoRa - asi se puede ver a que red esta
 * conectado el ESP32 desde el dashboard LoRa, sin necesitar el propio
 * WiFi como canal de datos. */
static volatile bool g_wifi_connected = false;
static char g_wifi_ssid[33] = "OFF";

/* =======================================================================
 *  Telemetría de alta frecuencia por UART0 (para tools/dashboard.html en
 *  desarrollo). Independiente del enlace LoRa.
 * ======================================================================= */
/* Definida mas abajo (modulo WiFi+TCP); forward-declare para poder
 * usarla aqui. Con WIFI_RECORDING_ENABLED=0 es un no-op. */
static void tcp_send_line(const char *line);

static esp_timer_handle_t g_tlm_timer;
static uint32_t g_tlm_seq = 0;
static int64_t g_last_tlm_send_us = 0;
static uint32_t g_tlm_window_count = 0;
static int64_t g_tlm_window_start_us = 0;

static void tlm_timer_cb(void *arg)
{
    (void)arg;
    int64_t now_us = esp_timer_get_time();
    int64_t dt_us = (g_last_tlm_send_us == 0) ? 0 : (now_us - g_last_tlm_send_us);
    g_last_tlm_send_us = now_us;
    g_tlm_seq++;
    g_tlm_window_count++;

    char line[256];
    /* -Wformat-truncation asume el rango completo de los tipos (ej. un
     * uint32_t podria imprimir 10 digitos), no lo que realmente puede
     * pasar aqui (t/seq/dt_us son pequeños en la practica). Silenciado
     * solo para este snprintf. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
    snprintf(line, sizeof(line),
           "TLM:{\"t\":%" PRId64 ",\"seq\":%" PRIu32 ",\"dt_us\":%" PRId64 ","
           "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
           "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
           "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
           "\"temp\":%.1f,\"pres\":%.1f,\"alt\":%.1f,"
           "\"prox\":%u,\"light\":%u,\"batt\":%.2f}\n",
           now_us / 1000, g_tlm_seq, dt_us,
           g_orient.roll, g_orient.pitch, g_orient.yaw,
           g_mpu_data.ax, g_mpu_data.ay, g_mpu_data.az,
           g_mpu_data.gx, g_mpu_data.gy, g_mpu_data.gz,
           g_last_temp_c, g_last_pres_hpa, g_last_alt_m,
           (unsigned)g_last_prox, (unsigned)g_last_light, g_last_batt_v);
#pragma GCC diagnostic pop
    fputs(line, stdout);
    tcp_send_line(line);

    if (g_tlm_window_start_us == 0) {
        g_tlm_window_start_us = now_us;
    } else if (now_us - g_tlm_window_start_us > RATE_REPORT_PERIOD_US) {
        double secs = (now_us - g_tlm_window_start_us) / 1000000.0;
        double hz = g_tlm_window_count / secs;
        printf("RATE:{\"t\":%" PRId64 ",\"tlm_hz\":%.2f,\"target_hz\":%.2f,\"count\":%" PRIu32 "}\n",
               now_us / 1000, hz, 1000.0 / TELEMETRY_PERIOD_MS, g_tlm_window_count);
        g_tlm_window_count = 0;
        g_tlm_window_start_us = now_us;
    }
}

/* =======================================================================
 *  Calibración de gestos (CORE 1)
 * ======================================================================= */
typedef enum { DIR_NONE = 0, DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT } dir_t;

static char dir_to_char(dir_t d)
{
    switch (d) {
    case DIR_UP:    return '^';
    case DIR_DOWN:  return 'v';
    case DIR_LEFT:  return '<';
    case DIR_RIGHT: return '>';
    default:        return '?';
    }
}

static dir_t apds_gesture_to_dir(uint8_t g)
{
    switch (g) {
    case APDS9960_UP:    return DIR_UP;
    case APDS9960_DOWN:  return DIR_DOWN;
    case APDS9960_LEFT:  return DIR_LEFT;
    case APDS9960_RIGHT: return DIR_RIGHT;
    default:             return DIR_NONE;
    }
}

static const char *dir_name(dir_t d)
{
    switch (d) {
    case DIR_UP:    return "ARRIBA";
    case DIR_DOWN:  return "ABAJO";
    case DIR_LEFT:  return "IZQUIERDA";
    case DIR_RIGHT: return "DERECHA";
    default:        return "?";
    }
}

/* Ultima direccion de gesto detectada, compartida entre gesture_task
 * (quien la escribe) y monitor_task (quien solo la lee para el OLED). */
static volatile char g_last_evt_dir = '-';

/* =======================================================================
 *  Buzzer + deteccion de caidas EMBEBIDA (corre 100% en el ESP32, sin
 *  depender de WiFi/LoRa/PC).
 *
 *  Modelo: RandomForest entrenado con tools/train_fall_model.py y
 *  exportado a C++ con tools/export_model_to_c.py -> main/fall_model.h
 *  (fall_model_predict(), puente en fall_model_wrap.cpp). Misma
 *  filosofia validada con datos reales en tools/fall_features.py: no se
 *  juzga el golpe aislado. Un candidato (umbral de magnitud) dispara una
 *  ventana de FALL_POST_WINDOW_MS; al cumplirse, se extraen las mismas
 *  15 features del entrenamiento y se corre el modelo. Si dice CAIDA,
 *  suena el buzzer.
 *
 *  Cancelacion: mientras suena la alarma, cualquiera de los 4 pares de
 *  gestos opuestos (izq-der, der-izq, arriba-abajo, abajo-arriba)
 *  seguidos dentro de GESTURE_CANCEL_WINDOW_MS la apaga y la marca como
 *  "HUMANO_DESCARTA". Reusa la deteccion de direccion del APDS9960 que
 *  ya corre en gesture_task, mas abajo.
 * ======================================================================= */
#define BUZZER_GPIO              GPIO_NUM_4

#define FALL_ACCEL_IMPACT_G      1.8f
#define FALL_ACCEL_FREEFALL_G    0.3f
#define FALL_TRIGGER_COOLDOWN_MS 1000
#define FALL_POST_WINDOW_MS      3000   /* = POST_WINDOW_S en tools/fall_features.py */
#define FALL_POLL_PERIOD_MS      100
#define FALL_N_FEATURES          15
#define FALL_BUFFER_LEN          400    /* ~4s a 100Hz, con margen sobre FALL_POST_WINDOW_MS */

#define ALARM_DURATION_MS        10000  /* 10s, o hasta cancelar con el gesto */
#define ALARM_BEEP_ON_MS         120
#define ALARM_GAP_START_MS       450    /* pitido lento al empezar... */
#define ALARM_GAP_END_MS         60     /* ...cada vez mas rapido */
#define GESTURE_CANCEL_WINDOW_MS 2500   /* tiempo maximo entre los dos gestos de la secuencia */

/* main/fall_model.h (generado por micromlgen) es C++; fall_model_wrap.cpp
 * expone esta unica funcion con enlace C para que este archivo la use
 * como cualquier otra funcion C. */
int fall_model_predict(const float *x);

static volatile bool g_alarm_active = false;
static int64_t g_alarm_start_us = 0;

typedef struct {
    int64_t t_us;
    float mag;
    float gyro_mag;
    float roll;
    float pitch;
} fall_sample_t;

static fall_sample_t g_fall_buf[FALL_BUFFER_LEN];
static int g_fall_buf_head = 0;   /* proximo indice a escribir */
static int g_fall_buf_count = 0;
static portMUX_TYPE g_fall_buf_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool g_fall_pending = false;
static int64_t g_fall_trigger_us = 0;
static int64_t g_fall_last_trigger_us = 0;

static void fall_buffer_push(int64_t t_us, float mag, float gyro_mag, float roll, float pitch)
{
    portENTER_CRITICAL(&g_fall_buf_mux);
    g_fall_buf[g_fall_buf_head] = (fall_sample_t){ t_us, mag, gyro_mag, roll, pitch };
    g_fall_buf_head = (g_fall_buf_head + 1) % FALL_BUFFER_LEN;
    if (g_fall_buf_count < FALL_BUFFER_LEN) {
        g_fall_buf_count++;
    }
    portEXIT_CRITICAL(&g_fall_buf_mux);
}

/* Copia (bajo seccion critica, rapido) las muestras con t_us en
 * [from_us, to_us) al arreglo de trabajo del llamador. */
static int fall_buffer_copy_window(int64_t from_us, int64_t to_us, fall_sample_t *out, int out_cap)
{
    int n = 0;
    portENTER_CRITICAL(&g_fall_buf_mux);
    int idx = (g_fall_buf_head - g_fall_buf_count + FALL_BUFFER_LEN) % FALL_BUFFER_LEN;
    for (int i = 0; i < g_fall_buf_count; i++) {
        const fall_sample_t *s = &g_fall_buf[idx];
        if (s->t_us >= from_us && s->t_us < to_us && n < out_cap) {
            out[n++] = *s;
        }
        idx = (idx + 1) % FALL_BUFFER_LEN;
    }
    portEXIT_CRITICAL(&g_fall_buf_mux);
    return n;
}

/* Llamado desde mpu_task en cada muestra nueva (~100Hz). */
static void fall_check_trigger(float mag, int64_t now_us)
{
    if (g_fall_pending || g_alarm_active) {
        return; /* ya hay una ventana corriendo o una alarma sonando */
    }
    if ((mag > FALL_ACCEL_IMPACT_G || mag < FALL_ACCEL_FREEFALL_G) &&
        (now_us - g_fall_last_trigger_us > (int64_t)FALL_TRIGGER_COOLDOWN_MS * 1000)) {
        g_fall_last_trigger_us = now_us;
        g_fall_trigger_us = now_us;
        g_fall_pending = true;
    }
}

/* Replica EXACTA (mismo orden, mismas formulas) de
 * tools/fall_features.py:extract_window_features - si se desalinean, el
 * modelo entrenado no significa nada aplicado aqui. */
static bool fall_extract_features(const fall_sample_t *w, int n, float out[FALL_N_FEATURES])
{
    if (n < 5) {
        return false;
    }

    float mag_sum = 0, mag_min = w[0].mag, mag_max = w[0].mag;
    float gyro_sum = 0, gyro_max = w[0].gyro_mag;
    float roll_min = w[0].roll, roll_max = w[0].roll;
    float pitch_min = w[0].pitch, pitch_max = w[0].pitch;
    int n_freefall = 0, n_impact = 0;

    for (int i = 0; i < n; i++) {
        float m = w[i].mag, g = w[i].gyro_mag;
        mag_sum += m;
        if (m < mag_min) mag_min = m;
        if (m > mag_max) mag_max = m;
        gyro_sum += g;
        if (g > gyro_max) gyro_max = g;
        if (m < FALL_ACCEL_FREEFALL_G) n_freefall++;
        if (m > FALL_ACCEL_IMPACT_G) n_impact++;
        if (w[i].roll < roll_min) roll_min = w[i].roll;
        if (w[i].roll > roll_max) roll_max = w[i].roll;
        if (w[i].pitch < pitch_min) pitch_min = w[i].pitch;
        if (w[i].pitch > pitch_max) pitch_max = w[i].pitch;
    }
    float mag_mean = mag_sum / n;
    float gyro_mean = gyro_sum / n;

    float mag_var = 0, gyro_var = 0;
    for (int i = 0; i < n; i++) {
        float dm = w[i].mag - mag_mean;
        mag_var += dm * dm;
        float dg = w[i].gyro_mag - gyro_mean;
        gyro_var += dg * dg;
    }
    mag_var /= n;
    gyro_var /= n;
    float mag_std = sqrtf(mag_var);
    float gyro_std = sqrtf(gyro_var);

    int half = n / 2;
    const fall_sample_t *early = w;
    int n_early = half;
    const fall_sample_t *late = w + half;
    int n_late = n - half;

    float early_mean = 0, late_mean = 0;
    for (int i = 0; i < n_early; i++) early_mean += early[i].mag;
    if (n_early > 0) early_mean /= n_early;
    for (int i = 0; i < n_late; i++) late_mean += late[i].mag;
    if (n_late > 0) late_mean /= n_late;

    float early_var = 0, late_var = 0;
    for (int i = 0; i < n_early; i++) { float d = early[i].mag - early_mean; early_var += d * d; }
    if (n_early > 0) early_var /= n_early;
    for (int i = 0; i < n_late; i++) { float d = late[i].mag - late_mean; late_var += d * d; }
    if (n_late > 0) late_var /= n_late;
    float early_std = sqrtf(early_var);
    float late_std = sqrtf(late_var);

    /* Mismo orden que FEATURE_NAMES en tools/fall_features.py */
    out[0]  = mag_mean;
    out[1]  = mag_std;
    out[2]  = mag_min;
    out[3]  = mag_max;
    out[4]  = mag_max - mag_min;
    out[5]  = gyro_mean;
    out[6]  = gyro_std;
    out[7]  = gyro_max;
    out[8]  = (float)n_freefall / n;
    out[9]  = (float)n_impact / n;
    out[10] = late_std;
    out[11] = fabsf(late_mean - 1.0f);
    out[12] = (early_std + 1e-6f) / (late_std + 1e-6f);
    out[13] = roll_max - roll_min;
    out[14] = pitch_max - pitch_min;
    return true;
}

static void buzzer_set(bool on)
{
    gpio_set_level(BUZZER_GPIO, on ? 1 : 0);
}

static void alarm_start(void)
{
    g_alarm_start_us = esp_timer_get_time();
    g_alarm_active = true;
    ESP_LOGW(TAG, "*** ALERTA_CAIDA: buzzer activo (cancelable con gesto izq-der/der-izq/arriba-abajo/abajo-arriba) ***");
}

/* CORE 1: revisa si ya se cumplio la ventana de espera de un candidato
 * pendiente; si es asi, corre el modelo embebido y decide. */
static void fall_task(void *arg)
{
    (void)arg;
    static fall_sample_t window[FALL_BUFFER_LEN];

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(FALL_POLL_PERIOD_MS));

        if (!g_fall_pending) {
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        if (now_us - g_fall_trigger_us < (int64_t)FALL_POST_WINDOW_MS * 1000) {
            continue; /* todavia no se cumple la ventana de espera */
        }

        int64_t from_us = g_fall_trigger_us;
        int64_t to_us = g_fall_trigger_us + (int64_t)FALL_POST_WINDOW_MS * 1000;
        int n = fall_buffer_copy_window(from_us, to_us, window, FALL_BUFFER_LEN);
        g_fall_pending = false; /* se libera el "slot" de trigger, sin importar el resultado */

        float feats[FALL_N_FEATURES];
        char line[64];
        if (!fall_extract_features(window, n, feats)) {
            continue; /* se perdieron muestras, muy pocas para confiar */
        }

        int pred = fall_model_predict(feats);
        if (pred == 1) {
            snprintf(line, sizeof(line),
                   "EVT:{\"t\":%" PRId64 ",\"type\":\"ALERTA_CAIDA\",\"dir\":\"\"}\n",
                   from_us / 1000);
            fputs(line, stdout);
            tcp_send_line(line);
            alarm_start();
        } else {
            snprintf(line, sizeof(line),
                   "EVT:{\"t\":%" PRId64 ",\"type\":\"MODELO_DESCARTA\",\"dir\":\"\"}\n",
                   from_us / 1000);
            fputs(line, stdout);
            tcp_send_line(line);
        }
    }
}

/* CORE 1: patron de pitidos - encendido/apagado con pausas que se
 * acortan a lo largo de ALARM_DURATION_MS. En tarea propia porque usa
 * vTaskDelay bloqueante y no puede compartir tiempos con gestos/LoRa. */
static void alarm_task(void *arg)
{
    (void)arg;
    buzzer_set(false);
    while (1) {
        if (!g_alarm_active) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int64_t elapsed_ms = (esp_timer_get_time() - g_alarm_start_us) / 1000;
        if (elapsed_ms >= ALARM_DURATION_MS) {
            g_alarm_active = false;
            buzzer_set(false);
            char line[80];
            snprintf(line, sizeof(line),
                   "EVT:{\"t\":%" PRId64 ",\"type\":\"ALARMA_TIMEOUT\",\"dir\":\"\"}\n",
                   esp_timer_get_time() / 1000);
            fputs(line, stdout);
            tcp_send_line(line);
            continue;
        }

        float progress = (float)elapsed_ms / (float)ALARM_DURATION_MS;
        int gap_ms = (int)(ALARM_GAP_START_MS - progress * (ALARM_GAP_START_MS - ALARM_GAP_END_MS));

        buzzer_set(true);
        vTaskDelay(pdMS_TO_TICKS(ALARM_BEEP_ON_MS));
        buzzer_set(false);
        if (!g_alarm_active) {
            continue; /* se pudo haber cancelado durante el beep */
        }
        vTaskDelay(pdMS_TO_TICKS(gap_ms));
    }
}

/* =======================================================================
 *  Gestos (CORE 1) - timer dedicado, igual patron que lora_task: el
 *  esp_timer dispara con precision cada GESTURE_POLL_PERIOD_MS y solo
 *  entrega un semaforo; la tarea hace la lectura I2C real. Antes, la
 *  lectura de gestos vivia en el mismo loop que el dibujo del OLED
 *  (I2C tambien) y el refresco de BMP180/luz - cualquiera de esos podia
 *  atrasar el siguiente poll lo suficiente para que el FIFO de gestos
 *  del APDS9960 se llenara a mitad de un swipe y se perdiera el evento.
 *  Separado asi, nada compite por el tiempo del poll de gestos. */
#define GESTURE_POLL_PERIOD_MS   15

static esp_timer_handle_t g_gesture_timer;
static SemaphoreHandle_t g_gesture_sem;

static void gesture_timer_cb(void *arg)
{
    (void)arg;
    xSemaphoreGive(g_gesture_sem); /* si ya estaba en 1 (task ocupada), no hace nada: se salta ese tick en vez de acumular */
}

/* true si (prev, actual) es uno de los 4 pares de gestos opuestos que
 * cancelan la alarma: izq-der, der-izq, arriba-abajo, abajo-arriba. */
static bool is_cancel_pair(dir_t prev, dir_t cur)
{
    return (prev == DIR_LEFT  && cur == DIR_RIGHT) ||
           (prev == DIR_RIGHT && cur == DIR_LEFT)  ||
           (prev == DIR_UP    && cur == DIR_DOWN)  ||
           (prev == DIR_DOWN  && cur == DIR_UP);
}

static void gesture_task(void *arg)
{
    (void)arg;
    int64_t cooldown_until_us = 0;
    dir_t prev_dir = DIR_NONE;
    int64_t prev_dir_us = 0;

    while (1) {
        xSemaphoreTake(g_gesture_sem, portMAX_DELAY);

        if (!apds_ok) {
            continue;
        }

        uint8_t g = apds9960_read_gesture(apds_sensor);
        int64_t gnow_us = esp_timer_get_time();
        if (g != 0 && gnow_us >= cooldown_until_us) {
            dir_t d = apds_gesture_to_dir(g);
            cooldown_until_us = gnow_us + (int64_t)gesture_cooldown_ms * 1000;
            g_event_count++;
            g_last_evt_dir = dir_to_char(d);
            char evt_line[96];
            snprintf(evt_line, sizeof(evt_line),
                   "EVT:{\"t\":%" PRId64 ",\"type\":\"GESTURE\",\"dir\":\"%s\"}\n",
                   gnow_us / 1000, dir_name(d));
            fputs(evt_line, stdout);
            tcp_send_line(evt_line);

            /* Cancelacion de alarma: solo importa mientras hay
             * ALERTA_CAIDA sonando - fuera de eso, los gestos siguen
             * siendo solo telemetria normal. */
            if (g_alarm_active && prev_dir != DIR_NONE &&
                (gnow_us - prev_dir_us) <= (int64_t)GESTURE_CANCEL_WINDOW_MS * 1000 &&
                is_cancel_pair(prev_dir, d)) {
                g_alarm_active = false;
                buzzer_set(false);
                char cancel_line[80];
                snprintf(cancel_line, sizeof(cancel_line),
                       "EVT:{\"t\":%" PRId64 ",\"type\":\"HUMANO_DESCARTA\",\"dir\":\"%s\"}\n",
                       gnow_us / 1000, dir_name(d));
                fputs(cancel_line, stdout);
                tcp_send_line(cancel_line);
                prev_dir = DIR_NONE; /* evita re-disparar con el mismo gesto repetido */
            } else {
                prev_dir = d;
                prev_dir_us = gnow_us;
            }
        }
    }
}

/* CORE 1: OLED + refresco de BMP180/luz en cache, sin espera de
 * calibracion inicial (arranca directo). Ya no lee gestos aqui - eso lo
 * hace gesture_task, para que el dibujo del OLED (I2C, mas lento) nunca
 * le robe tiempo al poll de gestos.
 *
 * OLED (pantalla montada en vertical): solo temperatura y presión fijas
 * arriba, y la última dirección de gesto detectada abajo. */
static void monitor_task(void *arg)
{
    (void)arg;
    char line[24];

    oled_clear();

    while (1) {
        update_background_cache();

        oled_clear_line(1);
        snprintf(line, sizeof(line), "TEMP: %4.1fC", g_last_temp_c);
        oled_draw_string(1, 0, line);

        oled_clear_line(3);
        snprintf(line, sizeof(line), "PRES: %6.1f", g_last_pres_hpa);
        oled_draw_string(3, 0, line);

        oled_clear_line(4);
        snprintf(line, sizeof(line), "BAT: %4.2fV%s", g_last_batt_v, g_batt_low ? " !" : "");
        oled_draw_string(4, 0, line);

        oled_clear_line(6);
        snprintf(line, sizeof(line), "GESTO: %c", g_last_evt_dir);
        oled_draw_string(6, 0, line);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* =======================================================================
 *  LoRa (Wio-E5 por UART2) - config validada contra el gateway HT-M7603:
 *  903.9MHz (canal 8, rejilla US915) / SF7 / BW125 / sync word publico.
 * ======================================================================= */
#define LORA_UART_NUM          UART_NUM_2
#define LORA_UART_TX_GPIO      GPIO_NUM_17
#define LORA_UART_RX_GPIO      GPIO_NUM_16
#define LORA_UART_BUF_SIZE     1024

#define LORA_BAUD_DEFAULT      9600
#define LORA_BAUD_FALLBACK     115200

/* 903.9 MHz = canal 8 de la rejilla US915 (902.3 + 0.2*8), dentro del
 * rango que vigila el gateway ("Channels 8-15,65"). NET=ON = sync word
 * publico (0x34, el que usan gateways LoRaWAN) en vez del privado P2P
 * (0x12) - sin esto el gateway nunca engancha la trama. */
#define LORA_RFCFG             "AT+TEST=RFCFG,903.9,SF7,125,8,8,14,ON,OFF,ON"
/* El airtime real (SF7/BW125, ~70 bytes) es de apenas 50-70ms. Con el
 * UART2 corriendo a 115200 (lora_try_speed_up), el roundtrip del AT baja
 * de ~300ms a ~100-150ms, así que 250ms deja margen de sobra para que
 * "TX DONE" siempre llegue antes del siguiente ciclo. US915 no exige
 * duty cycle como EU868 - ajustable libremente, subir para ahorrar
 * batería en el wearable. */
#define LORA_TX_PERIOD_MS      250

static char s_lora_resp[512];
static bool lora_ok = false;

static void lora_uart_init(int baud)
{
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, LORA_UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_UART_TX_GPIO, LORA_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush(LORA_UART_NUM);
    ESP_LOGI(LORA_TAG, "UART2 listo: TX=GPIO%d RX=GPIO%d baud=%d",
             (int)LORA_UART_TX_GPIO, (int)LORA_UART_RX_GPIO, baud);
}

static void lora_uart_reinit(int baud)
{
    uart_driver_delete(LORA_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(50));
    lora_uart_init(baud);
}

static bool lora_at_cmd(const char *cmd, const char *expect, int timeout_ms)
{
    uart_flush(LORA_UART_NUM);

    char line[260];
    int n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    if (n < 0) {
        return false;
    }
    if (n >= (int)sizeof(line)) {
        n = (int)sizeof(line) - 1; /* snprintf trunco: no leer mas alla de lo que realmente escribio */
    }
    uart_write_bytes(LORA_UART_NUM, line, n);
    ESP_LOGI(LORA_TAG, ">> %s", cmd);

    memset(s_lora_resp, 0, sizeof(s_lora_resp));
    int used = 0;
    int64_t deadline = (int64_t)xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while ((int64_t)xTaskGetTickCount() < deadline && used < (int)sizeof(s_lora_resp) - 1) {
        int got = uart_read_bytes(LORA_UART_NUM, (uint8_t *)s_lora_resp + used,
                                  sizeof(s_lora_resp) - 1 - used, pdMS_TO_TICKS(50));
        if (got > 0) {
            used += got;
            s_lora_resp[used] = '\0';
            if (expect && strstr(s_lora_resp, expect) != NULL) {
                break;
            }
        }
    }

    for (int i = 0; s_lora_resp[i]; i++) {
        if (s_lora_resp[i] == '\r') {
            s_lora_resp[i] = ' ';
        }
    }

    if (used == 0) {
        ESP_LOGW(LORA_TAG, "<< (silencio)");
        return false;
    }
    ESP_LOGI(LORA_TAG, "<< %s", s_lora_resp);
    return (expect == NULL) || (strstr(s_lora_resp, expect) != NULL);
}

static bool lora_ping_at(void)
{
    return lora_at_cmd("AT", "+AT: OK", 800);
}

/* Intenta subir el modulo a 115200 para acortar el roundtrip de cada AT
 * (a 9600, ~93ms solo en transferir el comando por UART; a 115200, ~8ms).
 * AT+UART=BR,115200 segun la doc del Wio-E5. No es critico si falla:
 * nos quedamos en el baud que ya funcionaba, todo sigue operando igual
 * de bien, solo mas lento. */
static void lora_try_speed_up(int current_baud)
{
    if (current_baud == 115200) {
        return;
    }

    if (!lora_at_cmd("AT+UART=BR,115200", NULL, 500)) {
        ESP_LOGW(LORA_TAG, "AT+UART=BR no respondio, sigo a %d baud.", current_baud);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    lora_uart_reinit(115200);
    vTaskDelay(pdMS_TO_TICKS(200));

    if (lora_ping_at()) {
        ESP_LOGI(LORA_TAG, "UART2 a 115200 (roundtrip de AT mas rapido).");
    } else {
        ESP_LOGW(LORA_TAG, "No respondio a 115200 tras el cambio, vuelvo a %d.", current_baud);
        lora_uart_reinit(current_baud);
        vTaskDelay(pdMS_TO_TICKS(100));
        lora_ping_at();
    }
}

/* Deja el Wio-E5 listo para transmitir P2P. No es fatal si falla: el
 * resto del sistema (sensores, OLED, dashboard USB) sigue funcionando
 * sin LoRa, solo se salta el envio por radio. */
static bool lora_setup(void)
{
    int connected_baud = LORA_BAUD_DEFAULT;
    lora_uart_init(LORA_BAUD_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(2000)); /* boot del Wio-E5 */
    uart_flush(LORA_UART_NUM);

    if (!lora_ping_at()) {
        ESP_LOGW(LORA_TAG, "Sin respuesta a 9600. Probando 115200...");
        lora_uart_reinit(LORA_BAUD_FALLBACK);
        connected_baud = LORA_BAUD_FALLBACK;
        vTaskDelay(pdMS_TO_TICKS(200));
        if (!lora_ping_at()) {
            ESP_LOGE(LORA_TAG, "Wio-E5 no responde (revisa TX=GPIO%d/RX=GPIO%d, GND comun, 5V).",
                     (int)LORA_UART_TX_GPIO, (int)LORA_UART_RX_GPIO);
            return false;
        }
    }

    lora_try_speed_up(connected_baud);

    if (!lora_at_cmd("AT+MODE=TEST", "+MODE: TEST", 1500)) {
        ESP_LOGE(LORA_TAG, "No entro a MODE=TEST, sin eso no hay P2P.");
        return false;
    }

    if (!lora_at_cmd(LORA_RFCFG, "+TEST:", 1500)) {
        ESP_LOGW(LORA_TAG, "RFCFG no confirmo; se intenta TX de todos modos.");
    }

    ESP_LOGI(LORA_TAG, "LoRa listo: 903.9MHz SF7 BW125 sync publico, cada %dms.", LORA_TX_PERIOD_MS);
    return true;
}

/* Payload compacto SIN comillas (rompen el parseo de AT+TEST=TXLRSTR)
 * y sin JSON: clave=valor separado por comas.
 *
 * Incluye ax/ay/az/gx/gy/gz crudos (no solo roll/pitch/yaw derivados):
 * para grabar caidas por LoRa hace falta la señal cruda, igual que por
 * UART0 - una caida se distingue por la forma de la aceleracion/giro en
 * el tiempo, no solo por la orientacion ya filtrada.
 *
 * "net" trae la red WiFi actual (o "OFF") solo como dato de diagnostico
 * visible en el dashboard - el WiFi no es el canal de grabacion, LoRa
 * lo es. */
static void lora_build_payload(char *out, size_t out_len, uint32_t seq)
{
    snprintf(out, out_len,
             "seq=%" PRIu32 ",r=%.1f,p=%.1f,y=%.1f,"
             "ax=%.2f,ay=%.2f,az=%.2f,gx=%.1f,gy=%.1f,gz=%.1f,"
             "t=%.1f,pr=%.0f,px=%u,lt=%u,ev=%u,net=%.10s",
             seq, g_orient.roll, g_orient.pitch, g_orient.yaw,
             g_mpu_data.ax, g_mpu_data.ay, g_mpu_data.az,
             g_mpu_data.gx, g_mpu_data.gy, g_mpu_data.gz,
             g_last_temp_c, g_last_pres_hpa,
             (unsigned)g_last_prox, (unsigned)g_last_light, g_event_count,
             g_wifi_connected ? g_wifi_ssid : "OFF");
}

/* CORE 1, tarea aparte de monitor_task: arma y transmite el payload de
 * telemetria cada LORA_TX_PERIOD_MS. Si el Wio-E5 no respondio en el
 * setup, no intenta transmitir (lora_ok queda en false). */
static esp_timer_handle_t g_lora_tx_timer;
static SemaphoreHandle_t g_lora_tx_sem;

static void lora_tx_timer_cb(void *arg)
{
    (void)arg;
    xSemaphoreGive(g_lora_tx_sem); /* si la tarea sigue en el roundtrip anterior, se salta ese tick */
}

static void lora_task(void *arg)
{
    (void)arg;
    uint32_t seq = 0;
    char payload[200];
    char cmd[240];

    while (1) {
        xSemaphoreTake(g_lora_tx_sem, portMAX_DELAY);

        if (lora_ok) {
            lora_build_payload(payload, sizeof(payload), seq);
            snprintf(cmd, sizeof(cmd), "AT+TEST=TXLRSTR,\"%s\"", payload);
            bool ok = lora_at_cmd(cmd, "TX DONE", 3000);
            if (ok) {
                ESP_LOGI(LORA_TAG, "TX #%" PRIu32 " OK: %s", seq, payload);
            } else {
                ESP_LOGW(LORA_TAG, "TX #%" PRIu32 " sin TX DONE", seq);
            }
            seq++;
        }
    }
}

/* =======================================================================
 *  WiFi + TCP: SOLO para grabar datos de prueba (caidas, sesiones) sin
 *  el cable USB - el ESP32 se une a una red WiFi normal y abre una
 *  conexion TCP hacia la PC (tools/data_recorder.py en modo --tcp-listen),
 *  igual patron que gateway<-dispositivo en LoRa: el ESP32 busca al
 *  servidor, no al reves, asi no hace falta saber la IP del ESP32.
 *
 *  Esto es SOLO para pruebas de captura de datos en casa/oficina - el
 *  despliegue de campo real sigue siendo LoRa (por eso todo esto vive
 *  detras de un define, facil de apagar). No reemplaza LoRa para nada.
 *
 *  Manda los datos por UDP BROADCAST en la subred donde el ESP32 obtuvo
 *  IP, en vez de conectarse a una IP fija de PC: asi funciona en
 *  cualquier red (casa, hotspot del celular, la del amigo) sin volver a
 *  tocar el codigo ni reflashear - antes, si cambiabas de red, la IP de
 *  la PC ya no coincidia con la que tenia grabada el firmware y todo
 *  fallaba en silencio. */
#define WIFI_RECORDING_ENABLED 1
#define PC_SERVER_PORT         5005

#if WIFI_RECORDING_ENABLED

/* Intenta conectarse a estas redes en orden; si una falla o se cae,
 * prueba la siguiente (y vuelve a la primera al llegar al final) - asi
 * sirve tanto en casa (WHITERBY) como en campo con el hotspot del
 * celular (Sensorix) sin tener que reflashear para cambiar de red. */
typedef struct { const char *ssid; const char *pass; } wifi_ap_t;
static const wifi_ap_t WIFI_CANDIDATES[] = {
    { "WHITERBY", "Liomatis2605" },
    { "Sensorix", "Jose1234" },
};
#define WIFI_CANDIDATES_COUNT (sizeof(WIFI_CANDIDATES) / sizeof(WIFI_CANDIDATES[0]))

static const char *WIFI_TAG = "wifi_rec";
static int g_udp_sock = -1;
static struct sockaddr_in g_broadcast_dest;
static bool g_broadcast_ready = false;
static bool g_wifi_got_ip = false;
static int g_wifi_candidate_idx = 0;

static void wifi_try_candidate(int idx)
{
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_CANDIDATES[idx].ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_CANDIDATES[idx].pass, sizeof(wifi_cfg.sta.password) - 1);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    ESP_LOGI(WIFI_TAG, "Probando red '%s' (%d/%d)...",
             WIFI_CANDIDATES[idx].ssid, idx + 1, (int)WIFI_CANDIDATES_COUNT);
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifi_try_candidate(g_wifi_candidate_idx);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        g_wifi_got_ip = false;
        g_wifi_connected = false;
        strncpy(g_wifi_ssid, "OFF", sizeof(g_wifi_ssid) - 1);
        ESP_LOGW(WIFI_TAG, "Desconectado de '%s' (reason=%d), probando siguiente red...",
                 WIFI_CANDIDATES[g_wifi_candidate_idx].ssid, disc->reason);
        g_wifi_candidate_idx = (g_wifi_candidate_idx + 1) % WIFI_CANDIDATES_COUNT;
        vTaskDelay(pdMS_TO_TICKS(500));
        wifi_try_candidate(g_wifi_candidate_idx);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "IP obtenida: " IPSTR " (red: %s)",
                 IP2STR(&ev->ip_info.ip), WIFI_CANDIDATES[g_wifi_candidate_idx].ssid);
        g_wifi_got_ip = true;
        g_wifi_connected = true;
        strncpy(g_wifi_ssid, WIFI_CANDIDATES[g_wifi_candidate_idx].ssid, sizeof(g_wifi_ssid) - 1);

        /* Broadcast = (ip AND netmask) OR (NOT netmask) - se recalcula
         * cada vez que se obtiene IP porque puede cambiar de red (otra
         * subred = otro broadcast). Con esto ya no hace falta saber de
         * antemano la IP de la PC: se manda a "todos" en la subred y
         * quien este escuchando (tools/data_recorder.py --udp-listen)
         * lo recibe, sea cual sea su IP. */
        uint32_t bcast = (ev->ip_info.ip.addr & ev->ip_info.netmask.addr) | (~ev->ip_info.netmask.addr);
        g_broadcast_dest.sin_family = AF_INET;
        g_broadcast_dest.sin_port = htons(PC_SERVER_PORT);
        g_broadcast_dest.sin_addr.s_addr = bcast;
        g_broadcast_ready = true;

        if (g_udp_sock < 0) {
            g_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (g_udp_sock >= 0) {
                int yes = 1;
                setsockopt(g_udp_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
            }
        }
        ESP_LOGI(WIFI_TAG, "Broadcast de grabacion a " IPSTR ":%d",
                 IP2STR((esp_ip4_addr_t *)&bcast), PC_SERVER_PORT);
    }
}

static void wifi_recording_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* Manda una linea (con su \n) por broadcast UDP en la subred actual. No
 * bloquea el resto del sistema si no hay WiFi/IP aun - simplemente no
 * manda nada. UDP no tiene "conexion" que mantener (a diferencia del
 * TCP anterior): en cuanto hay IP y broadcast calculado, ya se puede
 * mandar, sin reintentos ni estado de socket que reparar. */
static void tcp_send_line(const char *line)
{
    if (g_udp_sock < 0 || !g_broadcast_ready) {
        return;
    }
    sendto(g_udp_sock, line, strlen(line), 0,
           (struct sockaddr *)&g_broadcast_dest, sizeof(g_broadcast_dest));
}

static void net_task(void *arg)
{
    (void)arg;
    wifi_recording_init();
    vTaskDelete(NULL); /* todo lo demas pasa por eventos (wifi_event_handler) */
}

#else
static void tcp_send_line(const char *line) { (void)line; }
#endif /* WIFI_RECORDING_ENABLED */

/* =======================================================================
 *  app_main: inicializa I2C + sensores + LoRa, lanza las tareas.
 * ======================================================================= */
void app_main(void)
{
    /* ---- Buzzer (deteccion de caidas embebida) ---- */
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0);

    /* ---- ADC de bateria (divisor 62k/62k en GPIO34) ---- */
    battery_adc_init();

    i2c_config_t shared_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    shared_i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &shared_conf);
    if (shared_i2c_bus == NULL) {
        ESP_LOGE(TAG, "No se pudo crear el bus I2C compartido");
        return;
    }
    i2c_master_bus_handle_t bus_handle = i2c_bus_get_internal_bus_handle(shared_i2c_bus);
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "No se pudo obtener el handle interno del bus I2C");
        return;
    }

    /* ---- OLED ---- */
    i2c_device_config_t oled_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &oled_cfg, &oled_handle));
    ESP_LOGI(TAG, "Inicializando OLED...");
    oled_init_sequence();
    oled_clear();

    vTaskDelay(pdMS_TO_TICKS(100));

    /* ---- MPU6050 (0x68 o 0x69) ---- */
#if ENABLE_MPU6050
    uint8_t mpu_addr = 0;
    if (i2c_master_probe(bus_handle, 0x68, 100) == ESP_OK) {
        mpu_addr = 0x68;
    } else if (i2c_master_probe(bus_handle, 0x69, 100) == ESP_OK) {
        mpu_addr = 0x69;
    }
    if (mpu_addr != 0) {
        i2c_device_config_t mpu_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = mpu_addr,
            .scl_speed_hz = 400000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu_cfg, &mpu_handle));
        mpu_ok = (mpu6050_init() == ESP_OK);
        ESP_LOGI(TAG, "MPU6050 %s en 0x%02X", mpu_ok ? "OK" : "ERROR INIT", mpu_addr);
    } else {
        ESP_LOGW(TAG, "MPU6050 no detectado");
    }
    if (mpu_ok) {
        xTaskCreatePinnedToCore(mpu_task, "mpu_task", 3072, NULL, 5, NULL, 0);
    }
#else
    ESP_LOGI(TAG, "MPU6050 deshabilitado (ENABLE_MPU6050=0)");
#endif

    /* ---- BMP180 (0x77) ---- */
#if ENABLE_BMP180
    if (i2c_master_probe(bus_handle, BMP180_ADDR, 100) == ESP_OK) {
        i2c_device_config_t bmp_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BMP180_ADDR,
            .scl_speed_hz = 400000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp_cfg, &bmp_handle));
        bmp_ok = (bmp180_read_calibration() == ESP_OK);
        ESP_LOGI(TAG, "BMP180 %s", bmp_ok ? "OK" : "ERROR CALIB");
    } else {
        ESP_LOGW(TAG, "BMP180 no detectado en 0x77");
    }
#else
    ESP_LOGI(TAG, "BMP180 deshabilitado (ENABLE_BMP180=0)");
#endif

    /* ---- APDS9960 (0x39, mismo bus compartido) ---- */
    esp_err_t apds_err = apds9960_setup(&apds_ok);
    if (apds_ok) {
        ESP_LOGI(TAG, "APDS9960 OK");
    } else {
        ESP_LOGW(TAG, "APDS9960 no detectado / error de init (err=0x%x)", apds_err);
    }

    /* ---- Timer dedicado de gestos (precision, no compite con el OLED) ---- */
    g_gesture_sem = xSemaphoreCreateBinary();
    const esp_timer_create_args_t gesture_timer_args = {
        .callback = &gesture_timer_cb,
        .name = "gesture_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&gesture_timer_args, &g_gesture_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_gesture_timer, (uint64_t)GESTURE_POLL_PERIOD_MS * 1000));

    /* ---- Telemetría UART0 (dashboard local) ---- */
    const esp_timer_create_args_t tlm_timer_args = {
        .callback = &tlm_timer_cb,
        .name = "tlm_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&tlm_timer_args, &g_tlm_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_tlm_timer, (uint64_t)TELEMETRY_PERIOD_MS * 1000));
    ESP_LOGI(TAG, "Telemetria UART0: %d ms (%.1fHz)", TELEMETRY_PERIOD_MS, 1000.0 / TELEMETRY_PERIOD_MS);

    /* ---- LoRa (Wio-E5, UART2) ---- */
    lora_ok = lora_setup();
    if (!lora_ok) {
        ESP_LOGW(LORA_TAG, "LoRa deshabilitado para esta corrida (sensores siguen activos).");
    }

    g_lora_tx_sem = xSemaphoreCreateBinary();
    const esp_timer_create_args_t lora_timer_args = {
        .callback = &lora_tx_timer_cb,
        .name = "lora_tx_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&lora_timer_args, &g_lora_tx_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_lora_tx_timer, (uint64_t)LORA_TX_PERIOD_MS * 1000));

    /* Prioridades: gestos (6) > OLED/cache (5) > LoRa (4) - los gestos son
     * los mas sensibles al timing (FIFO del APDS9960 se pierde si se
     * atrasa el poll), LoRa es el que mas tolera jitter ocasional. */
    xTaskCreatePinnedToCore(gesture_task, "gesture_task", 3072, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(monitor_task, "monitor_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(lora_task, "lora_task", 4096, NULL, 4, NULL, 1);
    /* fall_task solo hace aritmetica + una pasada por 200 arbolitos poco
     * profundos cada FALL_POLL_PERIOD_MS - prioridad baja, nunca es
     * urgente. alarm_task si necesita despertar puntual para que el
     * patron de pitidos no se sienta irregular. */
    xTaskCreatePinnedToCore(fall_task, "fall_task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(alarm_task, "alarm_task", 2048, NULL, 5, NULL, 1);

#if WIFI_RECORDING_ENABLED
    /* Solo para grabar datos de prueba (caidas/sesiones) sin cable USB.
     * Prioridad baja: es "mejor esfuerzo", nunca debe robarle tiempo a
     * gestos/OLED/LoRa. */
    xTaskCreatePinnedToCore(net_task, "net_task", 4096, NULL, 3, NULL, 0);
#endif
}
