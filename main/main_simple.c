#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"

#define TAG "OSCILOSCOPIO"

/* ESP32-C6:
 * CH1 = GPIO0
 * CH2 = GPIO1
 */
#define ADC_CH1                 ADC_CHANNEL_0
#define ADC_CH2                 ADC_CHANNEL_1

/* Frecuencia objetivo por canal */
#define FRECUENCIA_POR_CANAL    40000
#define FRECUENCIA_TOTAL_ADC    (FRECUENCIA_POR_CANAL * 2)

/* Memoria del ADC continuo */
#define ADC_BUFFER_SIZE         1024
#define ADC_FRAME_SIZE          256

/* Impresión del monitor cada 100 ms */
#define INTERVALO_MONITOR_US    100000

static adc_continuous_handle_t adc_handle;

/* Inicialización del ADC continuo */
static void inicializar_adc(void)
{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = ADC_BUFFER_SIZE,
        .conv_frame_size = ADC_FRAME_SIZE,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    static adc_digi_pattern_config_t pattern[2] = {
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = ADC_CH1,
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        },
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = ADC_CH2,
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        },
    };

    adc_continuous_config_t adc_config = {
        .sample_freq_hz = FRECUENCIA_TOTAL_ADC,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 2,
        .adc_pattern = pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

/* Conversión aproximada de RAW a voltios */
static float raw_a_voltios(uint32_t raw)
{
    return ((float)raw / 4095.0f) * 3.3f;
}

/* Función principal */
void app_main(void)
{
    uint8_t buffer[ADC_FRAME_SIZE];
    uint32_t bytes_leidos = 0;

    uint32_t raw_ch1 = 0;
    uint32_t raw_ch2 = 0;

    bool hay_ch1 = false;
    bool hay_ch2 = false;

    int64_t ultima_impresion = esp_timer_get_time();

    ESP_LOGI(TAG, "Osciloscopio digital - ESP32-C6");
    ESP_LOGI(TAG, "CH1: GPIO0 | CH2: GPIO1");
    ESP_LOGI(TAG, "Frecuencia objetivo: %d Sa/s por canal",
             FRECUENCIA_POR_CANAL);

    inicializar_adc();

    while (1) {

        esp_err_t err = adc_continuous_read(
            adc_handle,
            buffer,
            sizeof(buffer),
            &bytes_leidos,
            100
        );

        if (err == ESP_OK) {

            for (uint32_t i = 0; i < bytes_leidos;
                 i += SOC_ADC_DIGI_RESULT_BYTES) {

                adc_digi_output_data_t *dato =
                    (adc_digi_output_data_t *)&buffer[i];

                uint32_t canal = dato->type2.channel;
                uint32_t raw = dato->type2.data;

                if (canal == ADC_CH1) {
                    raw_ch1 = raw;
                    hay_ch1 = true;
                }
                else if (canal == ADC_CH2) {
                    raw_ch2 = raw;
                    hay_ch2 = true;
                }
            }
        }
        else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Error de lectura ADC: %s",
                     esp_err_to_name(err));
        }

        /* Impresión periódica sin detener la adquisición */
        int64_t ahora = esp_timer_get_time();

        if ((ahora - ultima_impresion) >= INTERVALO_MONITOR_US) {

            if (hay_ch1 && hay_ch2) {

                ESP_LOGI(TAG,
                    "Objetivo: %d Sa/s/canal | CH1 (GPIO0): %.3f V | CH2 (GPIO1): %.3f V",
                    FRECUENCIA_POR_CANAL,
                    raw_a_voltios(raw_ch1),
                    raw_a_voltios(raw_ch2));
            }

            ultima_impresion = ahora;
        }
    }
}
