nano main_simple.c{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = TAM_BUFFER_ADC,
        .conv_frame_size = TAM_FRAME_ADC,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern[2] = {
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

    adc_continuous_config_t adc_cfg = {
        .sample_freq_hz = FRECUENCIA_ADC_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 2,
        .adc_pattern = pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

// Conversión orientativa del código RAW a voltios.
// Para medidas precisas se debe calibrar el ADC con tensiones conocidas.
static float raw_a_voltios(uint32_t raw)
{
    return ((float)raw / 4095.0f) * 3.3f;
}

void app_main(void)
{
    uint8_t buffer[TAM_FRAME_ADC];
    uint32_t bytes_leidos = 0;

    uint32_t raw_ch1 = 0;
    uint32_t raw_ch2 = 0;
    bool hay_ch1 = false;
    bool hay_ch2 = false;

    int64_t ultima_impresion = esp_timer_get_time();

    ESP_LOGI(TAG, "Osciloscopio digital ESP32-C6");
    ESP_LOGI(TAG, "CH1: GPIO0 | CH2: GPIO1");
    ESP_LOGI(TAG, "Frecuencia objetivo por canal: %d Sa/s",
             FRECUENCIA_CANAL_HZ);

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
                } else if (canal == ADC_CH2) {
                    raw_ch2 = raw;
                    hay_ch2 = true;
                }
            }
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Error leyendo ADC: %s", esp_err_to_name(err));
        }

        // La adquisición sigue funcionando continuamente por DMA.
        // Se imprime cada 100 ms para no saturar el puerto serie.
        int64_t ahora = esp_timer_get_time();
        if ((ahora - ultima_impresion) >= 100000) {
            if (hay_ch1 && hay_ch2) {
                ESP_LOGI(TAG,
                         "Frecuencia objetivo: %d Sa/s/canal | CH1: %.3f V | CH2: %.3f V",
                         FRECUENCIA_CANAL_HZ,
                         raw_a_voltios(raw_ch1),
                         raw_a_voltios(raw_ch2));
            }
            ultima_impresion = ahora;
        }
    }
}

