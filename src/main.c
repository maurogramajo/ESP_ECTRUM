/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

#include "driver/timer.h"
#include "driver/adc.h"
#include <math.h>
#include "esp_dsp.h"


#define CORE0   0
#define CORE1   1

#define LED_BUILTIN 2

#define FREQ_MUESTREO   40000
#define NUM_MUESTRAS   1024
uint16_t muestras[NUM_MUESTRAS];

static esp_adc_cal_characteristics_t *adc_chars;
#define V_REF   1100
#define ADC_CANAL   ADC_CHANNEL_6   //GPIO34 if ADC1, GPIO14 if ADC2
#define ADC_ATT     ADC_ATTEN_DB_11
#define ADC_WIDTH   ADC_WIDTH_BIT_12
#define ADC_UNIT    ADC_UNIT_1

//EL DIVIDER POR 200 OBTENGO 80MHZ/20=4MHZ
#define TIMER_DIVIDER   20
#define TIMER_GROUP     TIMER_GROUP_0
#define TIMER_NUM       TIMER_0

timer_config_t config_timer0;

//SEMAFORO BINARIO DEL TIMER0
xSemaphoreHandle semBin_timer0;

//SEMAFORO BINARIO DE FFT
xSemaphoreHandle semBin_fft;

void IRAM_ATTR isr_timer0(void *b) {
    //int task_woken = 0;
    int timer_idx = (int)b;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0)
    {
        TIMERG0.hw_timer[timer_idx].update = 1;
        TIMERG0.int_clr_timers.t0 = 1;
        TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

        xSemaphoreGiveFromISR(semBin_timer0, NULL);
        /*if (task_woken)
        {
            portYIELD_FROM_ISR();
        }*/
        
    }
    
}

//ESTA TAREA UTILIZA EL TIMER0 Y EL ADC_1 CANAL 6
void tMuestreo(void * a) {
    
    /*--------TIMER0 INIT-------*/
    config_timer0.alarm_en = 1;
    config_timer0.auto_reload = 1;
    config_timer0.counter_dir = TIMER_COUNT_UP;
    config_timer0.divider = TIMER_DIVIDER;
    config_timer0.intr_type = TIMER_INTR_LEVEL;
    config_timer0.counter_en = TIMER_PAUSE;

    timer_init(TIMER_GROUP, TIMER_NUM, &config_timer0);
    timer_pause(TIMER_GROUP, TIMER_NUM);
    timer_set_counter_value(TIMER_GROUP, TIMER_NUM, 0x00000000ULL); //QUE SER REINICIE SIEMPRE A 0
    timer_set_alarm_value(TIMER_GROUP, TIMER_NUM, 100);
    timer_enable_intr(TIMER_GROUP, TIMER_NUM);
    timer_isr_register(TIMER_GROUP, TIMER_NUM, isr_timer0, (void *)TIMER_NUM, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP, TIMER_NUM);
    
    /*--------ADC1 CH6 INIT-------*/
    #if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
    #elif CONFIG_IDF_TARGET_ESP32S2
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
            printf("eFuse Two Point: Supported\n");
        } else {
            printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
        }
    #else
    #error "This example is configured for ESP32/ESP32S2."
    #endif

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CANAL, ADC_ATT);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT, ADC_ATT, ADC_WIDTH, V_REF, adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }

    int pos = 0;

    while (1)
    {
        if (xSemaphoreTake(semBin_timer0, portMAX_DELAY) == pdTRUE)
        {
            if (pos < NUM_MUESTRAS)
            {
                muestras[pos] = adc1_get_raw(ADC_CANAL);
                pos++;
            } else
            {
                pos = 0;
                xSemaphoreGive(semBin_fft);
                vTaskDelay(30/portTICK_PERIOD_MS);
            }
        }
    }
    
    
}

void tFFT(void *a){
    // This example shows how to use FFT from esp-dsp library
    float senial[NUM_MUESTRAS];
    // Window coefficients
    float wind[NUM_MUESTRAS];
    // working complex array
    float y_cf[NUM_MUESTRAS*2];
    // Pointers to result arrays
    float* y1_cf = &y_cf[0];
    //float* y2_cf = &y_cf[NUM_MUESTRAS];

    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    // Generate hann window
    dsps_wind_hann_f32(wind, NUM_MUESTRAS);

    while (1)
    {
        if(xSemaphoreTake(semBin_fft, portMAX_DELAY) == pdTRUE){
            //TRANSFORMO LAS MUESTRAS EN UNA senial
            for (int i = 0; i < NUM_MUESTRAS; i++)
            {
                senial[i] = muestras[i] - 2048;
            }
            
            
            // Convert two input vectors to one complex vector
            for (int i=0 ; i< NUM_MUESTRAS ; i++)
            {
                y_cf[i*2 + 0] = senial[i] * wind[i]; // Real part is your signal multiply with window
                y_cf[i*2 + 1] = 0; // Imag part is 0
            }
            // FFT
            dsps_fft2r_fc32(y_cf, NUM_MUESTRAS);
            // Bit reverse 
            dsps_bit_rev_fc32(y_cf, NUM_MUESTRAS);
            // Convert one complex vector to two complex vectors
            dsps_cplx2reC_fc32(y_cf, NUM_MUESTRAS);

            // y1_cf - is your result in log scale
            // y2_cf - magnitude of your signal in linear scale

            for (int i = 0 ; i < NUM_MUESTRAS/2 ; i++) {
                y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1])/NUM_MUESTRAS);
                // y2_cf[i] = ((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1])/NUM_MUESTRAS);
            }
            printf("FFT\n");
            // for (int i = 0; i < NUM_MUESTRAS/2; i++)
            // {
                // printf("%d ",y1_cf[i]);
            // }
            
            //Show power spectrum in 64x10 window from -60 to 0 dB from 0..N/2 samples
            //dsps_view(y1_cf, NUM_MUESTRAS/2, 64, 10,  -60, 40, '|');
            //dsps_view(y2_cf, N/2, 64, 10,  0, 2, '|');
            printf("\n\n\n");
            // y1_cf = &y_cf[0];
        }
    }
    
}

void tBlinky(void * a){

    gpio_pad_select_gpio(LED_BUILTIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);

    uint8_t estado_led = 0;
    while(1) {
        gpio_set_level(LED_BUILTIN, estado_led%2);
        estado_led++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    vSemaphoreCreateBinary(semBin_timer0);
    vSemaphoreCreateBinary(semBin_fft);
    xTaskCreatePinnedToCore(tBlinky, "Blinky", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, 0, CORE1);
    
    xTaskCreatePinnedToCore(tMuestreo, "Muestreo", 4096, NULL, tskIDLE_PRIORITY+1, NULL, CORE1);
    xTaskCreatePinnedToCore(tFFT, "FFT", configMINIMAL_STACK_SIZE*100, NULL, tskIDLE_PRIORITY+2, NULL, CORE1);
}
