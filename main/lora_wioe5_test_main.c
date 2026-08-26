/*
 * lora_wioe5_test_main.c
 *
 * Prueba AISLADA del Seeed Wio-E5-LE-HF por UART. No toca I2C ni
 * sensores: solo UART2 <-> modulo LoRa, para confirmar cableado y que
 * el radio realmente transmite.
 *
 * Cableado (CRUZADO, GND comun):
 *   ESP32 TX  (LORA_UART_TX_GPIO) -> Wio-E5 RX
 *   ESP32 RX  (LORA_UART_RX_GPIO) -> Wio-E5 TX
 *   GND ESP32                     -> GND Wio-E5
 *   5V  ESP32 (USB)               -> 5V  Wio-E5
 *
 * IMPORTANTE: pon la antena SMA ANTES de la parte de transmision.
 * El comando AT no usa RF; AT+TEST=TXLRSTR si. Transmitir sin antena
 * puede danar el PA del modulo.
 *
 * Consola USB sigue en UART0 @ 115200 (idf.py monitor). El Wio-E5 va
 * por UART2 @ 9600 (default de fabrica del firmware AT).
 *
 * Exito esperado en el monitor:
 *   1) AT        -> +AT: OK          (UART vivo)
 *   2) TXLRSTR   -> +TEST: TX DONE   (el radio emitio un paquete)
 *
 * Con UN solo modulo, TX DONE es la prueba de envio. Para ver el
 * paquete en el aire hace falta un segundo Wio-E5 en RX, o un
 * gateway LoRa.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "lora_wioe5";

/* ---- Cambia estos dos GPIOs si no usaste 16/17 ----
 * GPIO16/17 son UART2 clasico en ESP32-WROOM (D0WD-V3).
 * NO uses GPIO1/3 (USB/consola) ni GPIO21/22 (I2C de los sensores). */
#define LORA_UART_NUM          UART_NUM_2
#define LORA_UART_TX_GPIO      GPIO_NUM_17  /* ESP32 TX -> Wio-E5 RX */
#define LORA_UART_RX_GPIO      GPIO_NUM_16  /* ESP32 RX <- Wio-E5 TX */
#define LORA_UART_BUF_SIZE     1024

#define LORA_BAUD_DEFAULT      9600
#define LORA_BAUD_FALLBACK     115200

/* Mexico / US915. El Wio-E5-LE-HF es la version de alta frecuencia.
 * 903.9 MHz = canal 8 de la rejilla US915 (902.3 + 0.2*8), para caer
 * dentro del rango que vigila el gateway Heltec HT-M7603 configurado
 * con "Channels 8-15,65" (region US915). Con 915 MHz exactos el gateway
 * nunca detecta el paquete: sus demoduladores son por canal fijo, no
 * escuchan toda la banda.
 * Ultimo parametro NET=ON: sync word publico (0x34, el que usan los
 * gateways LoRaWAN) en vez del privado (0x12, P2P). Con NET=OFF el
 * gateway ni siquiera engancha la trama aunque freq/SF/BW coincidan. */
#define LORA_RFCFG             "AT+TEST=RFCFG,903.9,SF7,125,8,8,14,ON,OFF,ON"

static char s_resp[512];

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
    ESP_LOGI(TAG, "UART2 listo: TX=GPIO%d RX=GPIO%d baud=%d",
             (int)LORA_UART_TX_GPIO, (int)LORA_UART_RX_GPIO, baud);
}

static void lora_uart_reinit(int baud)
{
    uart_driver_delete(LORA_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(50));
    lora_uart_init(baud);
}

/* Envia un comando AT (sin \r\n; se agregan aqui) y junta la respuesta
 * hasta timeout_ms. Devuelve true si 'expect' aparece en la respuesta
 * (pasa NULL para solo loguear). */
static bool at_cmd(const char *cmd, const char *expect, int timeout_ms)
{
    uart_flush(LORA_UART_NUM);

    char line[128];
    int n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    uart_write_bytes(LORA_UART_NUM, line, n);
    ESP_LOGI(TAG, ">> %s", cmd);

    memset(s_resp, 0, sizeof(s_resp));
    int used = 0;
    int64_t deadline = (int64_t)xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while ((int64_t)xTaskGetTickCount() < deadline && used < (int)sizeof(s_resp) - 1) {
        int got = uart_read_bytes(LORA_UART_NUM, (uint8_t *)s_resp + used,
                                  sizeof(s_resp) - 1 - used, pdMS_TO_TICKS(50));
        if (got > 0) {
            used += got;
            s_resp[used] = '\0';
            if (expect && strstr(s_resp, expect) != NULL) {
                break;
            }
        }
    }

    /* Limpia CR para que el log se lea bien. */
    for (int i = 0; s_resp[i]; i++) {
        if (s_resp[i] == '\r') {
            s_resp[i] = ' ';
        }
    }

    if (used == 0) {
        ESP_LOGW(TAG, "<< (silencio)");
        return false;
    }
    ESP_LOGI(TAG, "<< %s", s_resp);
    return (expect == NULL) || (strstr(s_resp, expect) != NULL);
}

static bool ping_at(void)
{
    return at_cmd("AT", "+AT: OK", 800);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Prueba Wio-E5 (UART AT) ===");
    ESP_LOGI(TAG, "Si no responde: cruza TX/RX, comparte GND, espera 2s al boot.");
    ESP_LOGI(TAG, "Si vas a transmitir: ANTENA SMA puesta.");

    lora_uart_init(LORA_BAUD_DEFAULT);

    /* El Wio-E5 arranca el firmware AT ~1-2s despues de alimentar. */
    ESP_LOGI(TAG, "Esperando boot del Wio-E5...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    uart_flush(LORA_UART_NUM);

    if (!ping_at()) {
        ESP_LOGW(TAG, "Sin respuesta a 9600. Probando 115200...");
        lora_uart_reinit(LORA_BAUD_FALLBACK);
        vTaskDelay(pdMS_TO_TICKS(200));
        if (!ping_at()) {
            ESP_LOGE(TAG, "El Wio-E5 no responde. Revisa:");
            ESP_LOGE(TAG, "  1) ESP32 TX (GPIO%d) va a RX del modulo", (int)LORA_UART_TX_GPIO);
            ESP_LOGE(TAG, "  2) ESP32 RX (GPIO%d) va a TX del modulo", (int)LORA_UART_RX_GPIO);
            ESP_LOGE(TAG, "  3) GND comun, 5V alimentando el modulo");
            ESP_LOGE(TAG, "  4) Si usaste otros GPIOs, cambia LORA_UART_TX/RX_GPIO");
            return;
        }
    }

    at_cmd("AT+VER", "+VER:", 800);
    at_cmd("AT+ID", "+ID:", 800);

    if (!at_cmd("AT+MODE=TEST", "+MODE: TEST", 1500)) {
        ESP_LOGE(TAG, "No entro a MODE=TEST. Sin eso no se puede hacer P2P.");
        return;
    }

    if (!at_cmd(LORA_RFCFG, "+TEST:", 1500)) {
        ESP_LOGW(TAG, "RFCFG no confirmo; se intenta TX de todos modos.");
    }

    ESP_LOGI(TAG, "UART OK. Enviando paquetes P2P cada 5s (915 MHz, SF7).");
    ESP_LOGI(TAG, "Busca '+TEST: TX DONE' = el radio transmitio.");

    unsigned seq = 0;
    while (1) {
        char payload[64];
        snprintf(payload, sizeof(payload), "AT+TEST=TXLRSTR,\"MYOSA-%u\"", seq);
        bool ok = at_cmd(payload, "TX DONE", 3000);
        if (ok) {
            ESP_LOGI(TAG, "TX #%u OK", seq);
        } else {
            ESP_LOGW(TAG, "TX #%u sin TX DONE", seq);
        }
        seq++;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
