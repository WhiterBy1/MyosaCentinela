/*
 * MyosaCentinela - firmware de MONITOREO CONTINUO (ESP-IDF, driver
 * i2c_master, IDF v5.x/v6.x)
 *
 * Esta es la versión "de producción" para capturar datos reales y
 * estudiarlos (con miras a detección de eventos anómalos más adelante).
 * El UART hace ahora el papel del enlace LoRa que todavía falta: el ESP32
 * transmite telemetría continua en JSON por consola, y el dashboard en PC
 * (tools/dashboard.html) la lee y la muestra en vivo, como si ya estuviera
 * viendo a un trabajador remoto con el dispositivo puesto.
 *
 * Arquitectura de tareas (ESP32-D0WD-V3, dos núcleos reales; el "tercero"
 * que trae el chip es el coprocesador ULP, pensado para tareas muy simples
 * en sueño profundo -sin soporte práctico de I2C multibyte-, así que NO
 * sirve para leer estos sensores. Repartimos el trabajo real entre los dos
 * núcleos de verdad):
 *
 *   - Core 0: mpu_task -> lee MPU6050 y actualiza el filtro complementario
 *     (roll/pitch/yaw) a ~100Hz, sin depender de nada más.
 *   - Core 1: monitor_task -> calibración inicial, luego monitoreo continuo:
 *     lee APDS9960 (gestos = eventos, con cooldown), refresca BMP180/luz
 *     en caché, dibuja un estado mínimo en el OLED, y transmite telemetría
 *     por UART.
 *
 * PROTOCOLO POR UART (una línea por mensaje, JSON, fácil de parsear)
 * --------------------------------------------------------------------
 *   TLM:{"t":<ms>,"seq":<contador>,"dt_us":<intervalo real desde el TLM
 *        anterior, en microsegundos>,"roll":..,"pitch":..,"yaw":..,
 *        "ax":..,"ay":..,"az":..,"temp":..,"pres":..,"alt":..,
 *        "prox":..,"light":..}
 *        Se manda sola desde un esp_timer periódico (g_tlm_timer), cada
 *        TELEMETRY_PERIOD_MS (20Hz por defecto) - no depende del loop de
 *        monitor_task ni de que el APDS9960 tarde en resolver un gesto,
 *        así que la tasa real es estable. "seq"/"dt_us" sirven para
 *        diagnosticar jitter o líneas de UART perdidas (saltos en seq).
 *
 *   RATE:{"t":<ms>,"tlm_hz":<frecuencia real medida>,"target_hz":..,
 *        "count":<muestras en la ventana>}   (cada ~5s, para comparar la
 *        tasa real contra el objetivo directamente en el log)
 *
 *   EVT:{"t":<ms>,"type":"GESTURE","dir":"ARRIBA|ABAJO|IZQUIERDA|DERECHA"}
 *        (cuando se detecta un gesto real, ya filtrado por el cooldown)
 *
 *   CALIB_WAIT,<direccion> / CALIB_EVT,<t_ms>,<esperado>,<detectado> /
 *   CALIB_DONE   (solo durante la calibración inicial, una vez al arrancar)
 *
 * El cooldown (gesture_cooldown_ms, más abajo) evita contar dos veces el
 * mismo gesto físico (el "eco" de cuando retiras el dedo). Se calibró con
 * tools/gesture_report.html a partir de datos reales capturados con el
 * juego de prueba de la versión anterior.
 *
 * Bus I2C: UNO SOLO, I2C_NUM_0 (SDA=GPIO21, SCL=GPIO22), compartido por los
 * 4 sensores (OLED 0x3C, MPU6050 0x68/0x69 autodetectado, BMP180 0x77,
 * APDS9960 0x39 con la librería oficial "espressif/apds9960", vendorizada
 * en components/espressif__apds9960 con parches propios de FIFO/timeout/
 * ganancia/tope de tiempo).
 *
 * El bus se crea una sola vez con el componente "i2c_bus" (i2c_bus_create),
 * y de ahí se saca con i2c_bus_get_internal_bus_handle() el
 * i2c_master_bus_handle_t interno que se sigue usando "a mano" con
 * driver/i2c_master.h para OLED/MPU6050/BMP180.
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "apds9960.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "sensores";

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

/* ---------- Config de telemetría / eventos ----------
 * TELEMETRY_PERIOD_MS ahora la maneja un esp_timer periódico (hardware,
 * no un "if paso el tiempo" dentro de un loop que también hace otras
 * cosas), así el intervalo real es preciso y no depende de que el resto
 * de monitor_task (lectura de gestos, que puede bloquear) vaya rápido. */
#define TELEMETRY_PERIOD_MS       50   /* ~20Hz */
#define RATE_REPORT_PERIOD_US 5000000  /* reporta la frecuencia real cada 5s */
#define BMP_CACHE_PERIOD_MS     1000
#define APDS_CACHE_PERIOD_MS     300

/* Cooldown tras cualquier gesto real detectado: mientras esté activo,
 * cualquier gesto adicional se ignora (evita contar dos veces el "eco" de
 * cuando retiras el dedo). Calibrado con tools/gesture_report.html a
 * partir de datos reales; 350ms funcionó bien en las pruebas. */
static uint32_t gesture_cooldown_ms = 350;

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
    /* Flechas para el juego (también sirven de "cursor" de progreso) */
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
    return font5x7[0].bits; /* espacio si no se encuentra */
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

/* Escribe una cadena en la página (0-7) y columna (0-127) indicadas */
static void oled_draw_string(uint8_t page, uint8_t col, const char *str)
{
    uint8_t data[6];
    oled_set_cursor(page, col);
    while (*str) {
        const uint8_t *glyph = get_glyph(*str);
        memcpy(data, glyph, 5);
        data[5] = 0x00; /* espacio entre caracteres */
        oled_write(OLED_CTRL_DATA, data, sizeof(data));
        str++;
    }
}

/* Borra una línea completa (útil para "refrescar" texto de largo variable
 * sin dejar caracteres viejos pegados al final) */
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
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* deg/s */
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

/* Filtro complementario (pitch/roll) + integración de gyro (yaw, deriva) */
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

/* CORE 0: solo el MPU6050, a ~100Hz, sin depender de nada más. */
static void mpu_task(void *arg)
{
    (void)arg;
    int64_t last_us = esp_timer_get_time();
    while (1) {
        if (mpu_ok) {
            int64_t now_us = esp_timer_get_time();
            float dt = (now_us - last_us) / 1000000.0f;
            last_us = now_us;
            if (mpu6050_read(&g_mpu_data) == ESP_OK) {
                orientation_update(&g_mpu_data, dt);
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
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t buf[2];
    err = bmp180_read_bytes(BMP180_REG_RESULT, buf, sizeof(buf));
    if (err != ESP_OK) return err;
    *ut = (buf[0] << 8) | buf[1];
    return ESP_OK;
}

static esp_err_t bmp180_read_raw_pressure(int32_t *up)
{
    /* oss = 0 (sin sobremuestreo, conversión más rápida) */
    esp_err_t err = bmp180_write_reg(BMP180_REG_CONTROL, BMP180_CMD_PRESSURE);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));
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
 *  APDS9960 (gestos) - librería oficial "espressif/apds9960", vendorizada
 *  en components/espressif__apds9960 con parches propios.
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
 *  Caché de fondo: BMP180 y proximidad/luz del APDS9960 se refrescan cada
 *  cierto intervalo (no hace falta más rápido, cambian lento) y se guardan
 *  en variables globales. La telemetría de alta frecuencia (TLM) solo lee
 *  esta caché, así nunca se bloquea esperando una conversión I2C lenta.
 * ======================================================================= */
static float g_last_temp_c = 0.0f;
static float g_last_pres_hpa = 0.0f;
static float g_last_alt_m = 0.0f;
static uint8_t g_last_prox = 0;
static uint16_t g_last_light = 0;

static void update_background_cache(void)
{
    static int64_t last_bmp_us = 0;
    static int64_t last_apds_us = 0;
    int64_t now_us = esp_timer_get_time();

    if (bmp_ok && (now_us - last_bmp_us > (int64_t)BMP_CACHE_PERIOD_MS * 1000)) {
        float t, p, alt;
        if (bmp180_read(&t, &p, &alt) == ESP_OK) {
            g_last_temp_c = t;
            g_last_pres_hpa = p / 100.0f;
            g_last_alt_m = alt;
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
}

/* =======================================================================
 *  Telemetría de alta frecuencia (esp_timer, precisa e independiente de
 *  monitor_task). Antes el TLM: se mandaba dentro del while(1) de
 *  monitor_task, así que su ritmo real dependía de que apds9960_read_gesture()
 *  no se tardara (podía bloquear hasta 1.5s) - de ahí la tasa efectiva tan
 *  baja e irregular. Con un esp_timer periódico, el envío no depende de
 *  nada más: dispara solo, cada TELEMETRY_PERIOD_MS, y solo lee variables
 *  ya cacheadas (nunca hace I2C directo), así que es rápido y consistente.
 *
 *  Cada TLM trae "seq" (contador) y "dt_us" (intervalo real medido desde
 *  el envío anterior, en microsegundos) para poder diagnosticar jitter o
 *  saltos de secuencia (líneas de UART perdidas) desde el propio log o el
 *  dashboard. Cada RATE_REPORT_PERIOD_US se manda además un RATE: con la
 *  frecuencia real lograda en la ventana, para comparar contra el objetivo.
 * ======================================================================= */
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

    printf("TLM:{\"t\":%" PRId64 ",\"seq\":%" PRIu32 ",\"dt_us\":%" PRId64 ","
           "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
           "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
           "\"temp\":%.1f,\"pres\":%.1f,\"alt\":%.1f,"
           "\"prox\":%u,\"light\":%u}\n",
           now_us / 1000, g_tlm_seq, dt_us,
           g_orient.roll, g_orient.pitch, g_orient.yaw,
           g_mpu_data.ax, g_mpu_data.ay, g_mpu_data.az,
           g_last_temp_c, g_last_pres_hpa, g_last_alt_m,
           (unsigned)g_last_prox, (unsigned)g_last_light);

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

/* Etapa previa obligatoria: antes de arrancar el juego con secuencias al
 * azar, se pide cada dirección UNA VEZ, en orden fijo (arriba, abajo,
 * izquierda, derecha), y no avanza hasta que el gesto detectado coincida
 * con la pedida. Así el usuario aprende/confirma qué movimiento físico de
 * la mano corresponde a cada dirección que reporta el sensor, antes de que
 * empiece a exigir velocidad con secuencias aleatorias. */
static void calibration_stage(void)
{
    static const dir_t calib_order[4] = { DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT };
    char line[24];

    oled_clear();
    oled_draw_string(0, 0, "CALIBRACION");
    oled_draw_string(2, 0, "MUEVE EL DEDO");
    oled_draw_string(3, 0, "COMO SE INDICA");
    vTaskDelay(pdMS_TO_TICKS(1500));

    for (int i = 0; i < 4; i++) {
        dir_t expected = calib_order[i];

        oled_clear();
        snprintf(line, sizeof(line), "PASO %d DE 4", i + 1);
        oled_draw_string(0, 0, line);
        oled_draw_string(2, 0, "MUEVE:");
        oled_draw_string(3, 0, dir_name(expected));
        snprintf(line, sizeof(line), "  %c   %c   %c", dir_to_char(expected), dir_to_char(expected), dir_to_char(expected));
        oled_draw_string(5, 0, line);

        ESP_LOGI(TAG, "CALIB_WAIT,%s", dir_name(expected));

        bool matched = false;
        while (!matched) {
            update_background_cache();

            if (!apds_ok) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            uint8_t g = apds9960_read_gesture(apds_sensor);
            if (g != 0) {
                dir_t raw = apds_gesture_to_dir(g);
                int64_t t_ms = esp_timer_get_time() / 1000;
                ESP_LOGI(TAG, "CALIB_EVT,%" PRId64 ",%s,%s", t_ms, dir_name(expected), dir_name(raw));

                oled_clear_line(6);
                oled_clear_line(7);
                if (raw == expected) {
                    matched = true;
                    oled_draw_string(6, 0, "BIEN");
                } else {
                    snprintf(line, sizeof(line), "ESO FUE: %s", dir_name(raw));
                    oled_draw_string(6, 0, line);
                    oled_draw_string(7, 0, "INTENTA DE NUEVO");
                }
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "CALIB_DONE");
    oled_clear();
    oled_draw_string(2, 0, "CALIBRACION LISTA");
    oled_draw_string(4, 0, "INICIA MONITOREO");
    vTaskDelay(pdMS_TO_TICKS(1500));
}

/* CORE 1: calibración inicial, luego monitoreo continuo. Lee el APDS9960
 * (gestos -> eventos, con cooldown), refresca BMP180/luz en caché, dibuja
 * un estado mínimo en el OLED, y transmite telemetría en JSON por UART a
 * ~5Hz (TLM:) más un mensaje por cada evento de gesto (EVT:). Esto hace
 * las veces del enlace LoRa que todavía falta: tools/dashboard.html se
 * conecta al puerto serie y muestra todo esto en vivo. */
static void monitor_task(void *arg)
{
    (void)arg;
    char line[24];

    if (apds_ok) {
        calibration_stage();
    }

    oled_clear();
    oled_draw_string(0, 0, "MYOSA CENTINELA");
    oled_draw_string(1, 0, "MONITOREANDO");

    int64_t cooldown_until_us = 0;
    unsigned int event_count = 0;
    char last_evt_dir = '-';

    while (1) {
        update_background_cache();

        /* La telemetría (TLM:) ya no se manda aquí: la dispara sola
         * g_tlm_timer (esp_timer periódico) cada TELEMETRY_PERIOD_MS,
         * desacoplada de este loop y de lo que tarde apds9960_read_gesture(). */

        /* --- Gestos -> eventos, con cooldown (evita contar el mismo swipe
         * dos veces por el "eco" de cuando retiras el dedo) --- */
        if (apds_ok) {
            uint8_t g = apds9960_read_gesture(apds_sensor);
            int64_t gnow_us = esp_timer_get_time();
            if (g != 0 && gnow_us >= cooldown_until_us) {
                dir_t d = apds_gesture_to_dir(g);
                cooldown_until_us = gnow_us + (int64_t)gesture_cooldown_ms * 1000;
                event_count++;
                last_evt_dir = dir_to_char(d);
                printf("EVT:{\"t\":%" PRId64 ",\"type\":\"GESTURE\",\"dir\":\"%s\"}\n",
                       gnow_us / 1000, dir_name(d));
            }
        }

        /* --- Estado mínimo en pantalla (el detalle vive en el dashboard) --- */
        oled_clear_line(3);
        snprintf(line, sizeof(line), "R:%5.1f P:%5.1f", g_orient.roll, g_orient.pitch);
        oled_draw_string(3, 0, line);

        oled_clear_line(4);
        snprintf(line, sizeof(line), "T:%4.1fC EVT:%u", g_last_temp_c, event_count);
        oled_draw_string(4, 0, line);

        oled_clear_line(5);
        snprintf(line, sizeof(line), "ULT:%c CD:%uMS", last_evt_dir, (unsigned)gesture_cooldown_ms);
        oled_draw_string(5, 0, line);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* =======================================================================
 *  app_main: solo inicializa hardware y lanza las dos tareas (una por
 *  núcleo), luego regresa (ESP-IDF borra la tarea principal sola).
 * ======================================================================= */
void app_main(void)
{
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

    vTaskDelay(pdMS_TO_TICKS(100)); /* estabilizar alimentación de sensores */

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

    /* Telemetría de alta frecuencia: esp_timer periódico, preciso, corre
     * solo (no depende del loop de monitor_task ni de que el APDS9960
     * tarde en resolver un gesto). */
    const esp_timer_create_args_t tlm_timer_args = {
        .callback = &tlm_timer_cb,
        .name = "tlm_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&tlm_timer_args, &g_tlm_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_tlm_timer, (uint64_t)TELEMETRY_PERIOD_MS * 1000));
    ESP_LOGI(TAG, "Telemetria: %d ms (%.1fHz) via esp_timer", TELEMETRY_PERIOD_MS, 1000.0 / TELEMETRY_PERIOD_MS);

    xTaskCreatePinnedToCore(monitor_task, "monitor_task", 4096, NULL, 5, NULL, 1);
}
