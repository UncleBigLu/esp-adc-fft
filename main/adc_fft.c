#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

#define FFT_POINTS 2048
/* Size of ADC conversion frame
 * Number of ADC result per frame equals to
 * ADC_CONV_FRAME_SZ/SOC_ADC_DIGI_RESULT_BYTES
 */
#define ADC_CONV_FRAME_SZ 256
#define ADC_SAMPLE_RATE 20e3
#define ADC_MAX_STORE_BUF_SZ 1024

#define ADC_READ_CHECK(x)                                   \
    do {                                                    \
        esp_err_t err_rc_ = (x);                            \
        if (err_rc_ == ESP_ERR_INVALID_STATE) {             \
            ESP_LOGW(TAG, "ADC internal buf full.");        \
        } else if (err_rc_ == ESP_ERR_TIMEOUT) {            \
            ESP_LOGW(TAG, "ADC reports NO data available"); \
        }                                                   \
    } while (0)

static const char *TAG = "main";

// As we're using ESP32S3...
// Only one channel is required
// Should map to GPIO 3
static adc_channel_t channel[1] = {ADC_CHANNEL_2};

static TaskHandle_t s_task_handle;

// Hann window for FFT
static float wind[FFT_POINTS] __attribute__((aligned(16)));
static uint8_t adc_buf[FFT_POINTS * SOC_ADC_DIGI_RESULT_BYTES] = {0};
// fft_buf[2 * i + 0] contains real, fft_buf[2 * i + 1] contains imaginary
static float fft_buf[2 * FFT_POINTS] __attribute__((aligned(16)));
static float spectrum[FFT_POINTS / 2] __attribute__((aligned(16)));

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data) {
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of
    // conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle) {
    esp_err_t ret;

    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_MAX_STORE_BUF_SZ,
        .conv_frame_size = ADC_CONV_FRAME_SZ,
    };
    ret = adc_continuous_new_handle(&adc_config, &handle);
    switch (ret) {
        case ESP_ERR_NO_MEM:
            ESP_LOGE(TAG, "out of memory");
            break;
        case ESP_ERR_NOT_FOUND:
            ESP_LOGE(TAG, "not found");
            break;
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(TAG, "invalid arg");
            break;
    }

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_RATE,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i,
                 adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i,
                 adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i,
                 adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static void fft_init(void) {
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE));
    ESP_ERROR_CHECK(dsps_fft4r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE));
    // Generate hann window
    dsps_wind_hann_f32(wind, FFT_POINTS);
}

void app_main(void) {
    size_t i;
    uint32_t ret_num = 0, tot_read_num = 0;
    memset(adc_buf, 0xcc, FFT_POINTS * SOC_ADC_DIGI_RESULT_BYTES);

    s_task_handle = xTaskGetCurrentTaskHandle();

    fft_init();

    /* ADC initial */
    adc_continuous_handle_t adc_handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t),
                        &adc_handle);

    /* Register conversion done callback */
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(
        adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    /* Start ADC */
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    while (1) {
        // Read ADC and fill fft buffer
        do {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            ADC_READ_CHECK(adc_continuous_read(
                adc_handle, adc_buf + tot_read_num,
                FFT_POINTS * SOC_ADC_DIGI_RESULT_BYTES - tot_read_num, &ret_num,
                0));
            tot_read_num += ret_num;
        } while (tot_read_num < FFT_POINTS * SOC_ADC_DIGI_RESULT_BYTES);
        tot_read_num = 0;

        for (i = 0; i < FFT_POINTS * SOC_ADC_DIGI_RESULT_BYTES;
             i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&adc_buf[i];
            uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
            if (chan_num >= SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                ESP_LOGW(TAG, "invalid data");
                continue;
            }
            // Real part of signal
            fft_buf[2 * (i / SOC_ADC_DIGI_RESULT_BYTES) + 0] =
                (float)EXAMPLE_ADC_GET_DATA(p) *
                wind[i / SOC_ADC_DIGI_RESULT_BYTES];
            // Only real part
            fft_buf[2 * (i / SOC_ADC_DIGI_RESULT_BYTES) + 1] = 0;
            // ESP_LOGI(TAG, "fft_buf[%u]: %f", 2 * (i /
            // SOC_ADC_DIGI_RESULT_BYTES), fft_buf[2 * (i /
            // SOC_ADC_DIGI_RESULT_BYTES)]);
        }
        // FFT Radix-2
        dsps_fft2r_fc32(fft_buf, FFT_POINTS);
        dsps_bit_rev2r_fc32(fft_buf, FFT_POINTS);
        dsps_cplx2real_fc32(fft_buf, FFT_POINTS);

        for (i = 0; i < FFT_POINTS / 2; ++i) {
            spectrum[i] = sqrtf(fft_buf[i * 2] * fft_buf[i * 2] +
                                fft_buf[i * 2 + 1] * fft_buf[i * 2 + 1]) /
                          FFT_POINTS;
            // ESP_LOGI(TAG, "spectrum[%u]: %f", i, spectrum[i]);
        }
        // I don't want DC components so remove it
        for (i = 0; i < 10; ++i) {
            spectrum[i] = 0;
        }
        // data, len, width, height, min, max, char
        dsps_view(spectrum, FFT_POINTS / 2, 128, 10, 0, 100, '|');

        // Do not starve watch dog as print is slow
        vTaskDelay(1);
    }
}
