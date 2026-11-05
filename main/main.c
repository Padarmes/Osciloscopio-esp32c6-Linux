
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"

#define TAG "OSCILOSCOPIO"

// =====================================================
// 1. CONFIGURACION GENERAL
// =====================================================

#define OSC_MUESTRAS       64
#define OSC_PAUSA_MS       500

#define ADC_CH1            ADC_CHANNEL_0  // GPIO0
#define ADC_CH2            ADC_CHANNEL_1  // GPIO1

#define ADC_FRAME_SIZE     256
#define ADC_BUFFER_SIZE    4096
#define ADC_READ_TIMEOUT   1000

#define COMANDO_COLA       8
#define COMANDO_STACK     3072

#define SPI_PIN_MOSI       7
#define SPI_PIN_MISO       2
#define SPI_PIN_SCLK       6
#define SPI_PIN_CS        16

// =====================================================
// 2. FRECUENCIAS Y DATOS
// =====================================================

typedef struct {
    uint32_t frecuencia;
    uint32_t periodo_us;
    uint32_t frecuencia_adc;
    uint32_t decimacion;
} osc_config_t;

typedef struct {
    uint32_t id;
    uint16_t ch1[OSC_MUESTRAS];
    uint16_t ch2[OSC_MUESTRAS];
    uint32_t n1;
    uint32_t n2;
    uint64_t tiempo_us;
    uint32_t errores;
} osc_ventana_t;

typedef struct {
    double frecuencia_real;
    double error_porcentual;
    bool cumple;
} osc_diagnostico_t;

static const uint32_t frecuencias[] = {
    100, 1000, 10000, 40000
};

static osc_config_t config;
static osc_ventana_t ventana;
static osc_diagnostico_t diagnostico;

static adc_continuous_handle_t adc_handle;
static QueueHandle_t cola_comandos;

static uint32_t contador_ventanas = 0;
static uint32_t ultima_frecuencia_impresa = 0;
static bool adc_activo = false;

// =====================================================
// 3. CONFIGURACION SPI PARA RASPBERRY PI
// =====================================================

/*
   RESERVA PARA IMPLEMENTACION FUTURA.

   Raspberry Pi: MAESTRO SPI
   ESP32-C6:     ESCLAVO SPI

   MOSI: GPIO7
   MISO: GPIO2
   SCLK: GPIO6
   CS:   GPIO16

   Datos previstos:
   - ID de ventana.
   - Frecuencia de adquisicion.
   - 64 muestras CH1.
   - 64 muestras CH2.
   - Diagnostico de adquisicion.

   Funciones futuras:
   osc_spi_inicializar()
   osc_spi_recibir_comando()
   osc_spi_enviar_ventana()

   No se inicializa SPI en esta version.
*/

// =====================================================
// 4. CONFIGURACION DE FRECUENCIA
// =====================================================

static void osc_configurar(uint32_t indice)
{
    config.frecuencia = frecuencias[indice];
    config.periodo_us = 1000000 / config.frecuencia;

    // Frecuencia agregada de los dos canales.
    // El ADC continuo tiene un limite inferior.
    config.frecuencia_adc =
        (config.frecuencia == 100) ? 1000 :
        config.frecuencia * 2;

    // A 100 Sa/s se selecciona 1 de cada 5
    // conversiones de cada canal.
    config.decimacion = config.frecuencia_adc /
                         (2 * config.frecuencia);

    ESP_LOGI(TAG, "Frecuencia seleccionada: %" PRIu32 " Sa/s",
             config.frecuencia);

    ESP_LOGI(TAG, "Frecuencia ADC agregada: %" PRIu32 " Sa/s",
             config.frecuencia_adc);

    ESP_LOGI(TAG, "Periodo objetivo: %" PRIu32 " us",
             config.periodo_us);
}

// =====================================================
// 5. INICIALIZACION ADC CONTINUO CON DMA
// =====================================================

static void osc_adc_inicializar(void)
{
    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = ADC_BUFFER_SIZE,
        .conv_frame_size = ADC_FRAME_SIZE
    };

    ESP_ERROR_CHECK(
        adc_continuous_new_handle(&handle_config, &adc_handle)
    );

    ESP_LOGI(TAG, "ADC continuo DMA inicializado");
    ESP_LOGI(TAG, "CH1: GPIO0");
    ESP_LOGI(TAG, "CH2: GPIO1");
}

// =====================================================
// 6. CONFIGURACION DEL CONTROLADOR ADC
// =====================================================

static void osc_adc_configurar(void)
{
    if (adc_activo) {
        ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
        adc_activo = false;
    }

    ESP_ERROR_CHECK(adc_continuous_flush_pool(adc_handle));

    adc_digi_pattern_config_t patron[2] = {
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = ADC_CH1,
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12
        },
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = ADC_CH2,
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12
        }
    };

    adc_continuous_config_t adc_config = {
        .pattern_num = 2,
        .adc_pattern = patron,
        .sample_freq_hz = config.frecuencia_adc,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2
    };

    ESP_ERROR_CHECK(
        adc_continuous_config(adc_handle, &adc_config)
    );
}

// =====================================================
// 7. ADQUISICION DE UNA VENTANA
// =====================================================

static void osc_capturar_ventana(void)
{
    adc_continuous_data_t datos[64];

    uint32_t contador_ch1 = 0;
    uint32_t contador_ch2 = 0;

    ventana.n1 = 0;
    ventana.n2 = 0;
    ventana.errores = 0;
    ventana.id = ++contador_ventanas;

    osc_adc_configurar();

    int64_t inicio = esp_timer_get_time();

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    adc_activo = true;

    while (ventana.n1 < OSC_MUESTRAS ||
           ventana.n2 < OSC_MUESTRAS) {

        uint32_t cantidad = 0;

        esp_err_t resultado = adc_continuous_read_parse(
            adc_handle,
            datos,
            64,
            &cantidad,
            ADC_READ_TIMEOUT
        );

        if (resultado != ESP_OK) {
            ventana.errores++;
            ESP_LOGW(TAG, "Error de lectura ADC: %s",
                     esp_err_to_name(resultado));

            if (ventana.errores >= 5) {
                break;
            }
            continue;
        }

        for (uint32_t i = 0; i < cantidad; i++) {

            if (!datos[i].valid) {
                ventana.errores++;
                continue;
            }

            if (datos[i].channel == ADC_CH1) {

                if (contador_ch1++ % config.decimacion == 0 &&
                    ventana.n1 < OSC_MUESTRAS) {

                    ventana.ch1[ventana.n1++] =
                        (uint16_t)datos[i].raw_data;
                }
            }

            else if (datos[i].channel == ADC_CH2) {

                if (contador_ch2++ % config.decimacion == 0 &&
                    ventana.n2 < OSC_MUESTRAS) {

                    ventana.ch2[ventana.n2++] =
                        (uint16_t)datos[i].raw_data;
                }
            }
        }
    }

    ventana.tiempo_us =
        (uint64_t)(esp_timer_get_time() - inicio);

    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    adc_activo = false;
}

// =====================================================
// 8. DIAGNOSTICO DE ADQUISICION
// =====================================================

static void osc_diagnostico_calcular(void)
{
    if (ventana.tiempo_us == 0) {
        diagnostico.frecuencia_real = 0;
    } else {
        diagnostico.frecuencia_real =
            (double)ventana.n1 * 1000000.0 /
            (double)ventana.tiempo_us;
    }

    diagnostico.error_porcentual =
        fabs(diagnostico.frecuencia_real -
             config.frecuencia) * 100.0 /
        config.frecuencia;

    diagnostico.cumple =
        (ventana.n1 == OSC_MUESTRAS) &&
        (ventana.n2 == OSC_MUESTRAS) &&
        (ventana.errores == 0) &&
        (diagnostico.error_porcentual <= 5.0);
}

// =====================================================
// 9. CONVERSION A VOLTIOS
// =====================================================

static uint32_t osc_a_milivoltios(uint16_t raw)
{
    int32_t mv = ((int32_t)raw - 32) * 1000 / 1024;

    if (mv < 0) mv = 0;

    return (uint32_t)mv;
}

// =====================================================
// 10. IMPRESION DE RESULTADOS
// =====================================================

static void osc_imprimir(void)
{
    uint32_t mv1 = osc_a_milivoltios(
        ventana.ch1[OSC_MUESTRAS - 1]
    );

    uint32_t mv2 = osc_a_milivoltios(
        ventana.ch2[OSC_MUESTRAS - 1]
    );

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "ID ventana: %" PRIu32, ventana.id);

    ESP_LOGI(TAG, "Frecuencia configurada: %" PRIu32 " Sa/s",
             config.frecuencia);

    ESP_LOGI(TAG, "Captura CH1: %" PRIu32 "/64", ventana.n1);
    ESP_LOGI(TAG, "Captura CH2: %" PRIu32 "/64", ventana.n2);

    ESP_LOGI(TAG, "Periodo objetivo: %" PRIu32 " us",
             config.periodo_us);

    ESP_LOGI(TAG, "Tiempo total de ventana: %" PRIu64 " us",
             ventana.tiempo_us);

    ESP_LOGI(TAG, "Frecuencia efectiva estimada: %.2f Sa/s",
             diagnostico.frecuencia_real);

    ESP_LOGI(TAG, "Error estimado: %.2f %%",
             diagnostico.error_porcentual);

    ESP_LOGI(TAG, "Errores de lectura: %" PRIu32,
             ventana.errores);

    ESP_LOGI(TAG, "Ultima muestra CH1: %lu.%03lu V",
             (unsigned long)(mv1 / 1000),
             (unsigned long)(mv1 % 1000));

    ESP_LOGI(TAG, "Ultima muestra CH2: %lu.%03lu V",
             (unsigned long)(mv2 / 1000),
             (unsigned long)(mv2 % 1000));

    if (diagnostico.cumple) {
        ESP_LOGI(TAG, "DIAGNOSTICO: CUMPLE");
    } else {
        ESP_LOGW(TAG, "DIAGNOSTICO: NO CUMPLE");
    }
}

// =====================================================
// 11. IMPRESION DE DATOS ADC ORIGINALES
// =====================================================

static void osc_imprimir_datos(void)
{
    ESP_LOGI(TAG, "INICIO VENTANA ADC");

    for (int i = 0; i < OSC_MUESTRAS; i++) {
        ESP_LOGI(TAG,
                 "Muestra %02d | CH1: %u | CH2: %u",
                 i + 1,
                 ventana.ch1[i],
                 ventana.ch2[i]);
    }

    ESP_LOGI(TAG, "FIN VENTANA ADC");
}

// =====================================================
// 12. RECEPCION DE COMANDOS USB
// =====================================================

static void osc_tarea_comandos(void *param)
{
    ESP_LOGI(TAG, "Seleccion manual de frecuencia:");
    ESP_LOGI(TAG, "1 = 100 Sa/s");
    ESP_LOGI(TAG, "2 = 1000 Sa/s");
    ESP_LOGI(TAG, "3 = 10000 Sa/s");
    ESP_LOGI(TAG, "4 = 40000 Sa/s (BONUS)");

    while (1) {
        int tecla = getchar();

        if (tecla == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint8_t comando = (uint8_t)tecla;

        if (comando >= '1' && comando <= '4') {
            xQueueSend(cola_comandos, &comando, 0);
        }
    }
}

// =====================================================
// 13. PROCESAMIENTO DE COMANDOS
// =====================================================

static void osc_procesar_comandos(void)
{
    uint8_t comando;

    while (xQueueReceive(cola_comandos, &comando, 0) == pdTRUE) {

        uint32_t indice = comando - '1';

        if (config.frecuencia != frecuencias[indice]) {
            osc_configurar(indice);
        }
    }
}

// =====================================================
// 14. FUNCION PRINCIPAL
// =====================================================

void app_main(void)
{
    ESP_LOGI(TAG, "OSCILOSCOPIO DIGITAL ESP32-C6");

    osc_adc_inicializar();

    cola_comandos = xQueueCreate(
        COMANDO_COLA,
        sizeof(uint8_t)
    );

    if (cola_comandos == NULL) {
        ESP_LOGE(TAG, "Error creando cola");
        return;
    }

    osc_configurar(0);

    if (xTaskCreate(
        osc_tarea_comandos,
        "comandos",
        COMANDO_STACK,
        NULL,
        5,
        NULL
    ) != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea de comandos");
        return;
    }

    while (1) {

        osc_procesar_comandos();

        osc_capturar_ventana();

        osc_diagnostico_calcular();

        osc_imprimir();

        if (ultima_frecuencia_impresa != config.frecuencia) {
            osc_imprimir_datos();
            ultima_frecuencia_impresa = config.frecuencia;
        }

        // FUTURO:
        // osc_spi_enviar_ventana(&ventana);

        vTaskDelay(pdMS_TO_TICKS(OSC_PAUSA_MS));
    }
}
