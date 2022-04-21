#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

///////////////////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "DHT.h"

//pump input
#define pump_gpio (21)
static uint8_t pump_in_state = 0;

//ADC Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_6
#define ADC1_EXAMPLE_CHAN1          ADC1_CHANNEL_0
#define ADC1_EXAMPLE_CHAN2          ADC1_CHANNEL_3
#define ADC1_EXAMPLE_CHAN3          ADC1_CHANNEL_4
#define ADC2_EXAMPLE_CHAN0          ADC2_CHANNEL_0
static const char *TAG_CH[5][10] = {{"ADC1_CH6"},{"ADC1_CH0"},{"ADC1_CH3"},{"ADC1_CH4"},{"ADC2_CH0"}};
#else
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_2
#define ADC1_EXAMPLE_CHAN1          ADC1_CHANNEL_0
#define ADC1_EXAMPLE_CHAN2          ADC1_CHANNEL_3
#define ADC1_EXAMPLE_CHAN3          ADC1_CHANNEL_4
#define ADC2_EXAMPLE_CHAN0          ADC2_CHANNEL_0
static const char *TAG_CH[2][10] = {{"ADC1_CH2"}, {"ADC1_CH0"},{"ADC1_CH3"},{"ADC1_CH4"},{"ADC2_CH0"}};
#endif

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static int adc_raw[5][10];
static const char *TAG = "ADC SINGLE";
esp_mqtt_client_handle_t client;

static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc1_1_chars;
static esp_adc_cal_characteristics_t adc1_2_chars;
static esp_adc_cal_characteristics_t adc1_3_chars;
static esp_adc_cal_characteristics_t adc2_chars;

//////////////////////////////////////////////////////////////////////////////////////////////

static const char *TAGE = "MQTT_TCP";


static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation 					s1.1
    esp_event_loop_create_default();     // event loop 			                s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station 	                    s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); // 					                    s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "telenet-5274953",
            .password = "ezP8dcmvMvev"}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    char str[50];
    client = event->client;
    switch (event->event_id)
    
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAGE, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "ESP32/LEDOUTPUT", 0);
        esp_mqtt_client_publish(client, "ESP32/TEXT", "This is data from the ESP32", 0, 1, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAGE, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAGE, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAGE, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAGE, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAGE, "MQTT_EVENT_DATA");
        printf("\nTOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        sprintf(str,"DATA=%.*s", event->data_len, event->data);
        
        
        

        if(strcmp(str,"DATA=1")==0){
            gpio_set_level(GPIO_NUM_21, 1);
        } else if(strcmp(str,"DATA=0")==0){
            gpio_set_level(GPIO_NUM_21, 0);
        }
        
        
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAGE, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAGE, "Other event id:%d", event->event_id);
        break;
    }

    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAGE, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://192.168.0.105",//here you have
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_1_chars);
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_2_chars);
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_3_chars);
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc2_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

void adc_init(void){
    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN1, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN2, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN3, ADC_EXAMPLE_ATTEN));
    //ADC2 config
    ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

}

void DHT_task(void *pvParameter)
{
    setDHTgpio( 21 );
    printf( "Starting DHT Task\n\n");

    while(1) {
    
        printf("=== Reading DHT ===\n" );
        int ret = readDHT();
        
        errorHandler(ret);

        printf( "Hum %.1f\n", getHumidity() );
        printf( "Tmp %.1f\n", getTemperature() );
        esp_mqtt_client_publish(client,"ESP32/TEXT", "iets van HUM of TEMP ", 0, 1, 0);
        
        // -- wait at least 2 sec before reading again ------------
        // The interval of whole process must be beyond 2 seconds !! 
        vTaskDelay( 3000 / portTICK_RATE_MS );
    }
}



void app_main(void)
{   
    adc_init();
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
    nvs_flash_init();


    wifi_connection();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("WIFI was initiated ...........\n");

    mqtt_app_start();

    //vTaskDelay( 1000 / portTICK_RATE_MS );
	//xTaskCreate( &DHT_task, "DHT_task", 2048, NULL, 5, NULL );

    esp_err_t ret = ESP_OK;
    uint32_t voltage = 0;
    bool cali_enable = adc_calibration_init();
    char str0[50];
    char str1[50];
    char str2[50];
    char str3[50];

    uint8_t chipid[6];
    esp_efuse_read_mac(chipid);


    while (1) {
        adc_raw[0][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN0); //moisture sensor to control the pump
        ESP_LOGI(TAG_CH[0][0], "raw  data: %d", adc_raw[0][0]);
        //if(adc_raw[0][0]>2500)
        //{pump_in_state=1;gpio_set_level(GPIO_NUM_21, 1); printf("pump on");}
        //else
        //{pump_in_state=0;gpio_set_level(pump_gpio, pump_in_state);printf("pump off");}

        if (cali_enable) {
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
            ESP_LOGI(TAG_CH[0][0], "cali data: %d mV", voltage);
            sprintf(str0,"ADC0 value: %d",voltage);
            esp_mqtt_client_publish(client,"ESP32/value", str0, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/espID", chipid, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/sensorID", 0, 0, 1, 0);

        }
        vTaskDelay(pdMS_TO_TICKS(5000));


        adc_raw[1][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN1);
        ESP_LOGI(TAG_CH[1][0], "raw  data: %d", adc_raw[1][0]);
        if (cali_enable) {
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[1][0], &adc1_1_chars);
            ESP_LOGI(TAG_CH[1][0], "cali data: %d mV", voltage);
            sprintf(str1,"ADC1 value: %d",voltage);
            esp_mqtt_client_publish(client,"ESP32/value", str1, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/espID", chipid, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/sensorID", 1, 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));

        adc_raw[2][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN2);
        ESP_LOGI(TAG_CH[2][0], "raw  data: %d", adc_raw[2][0]);
        if (cali_enable) {
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[2][0], &adc1_2_chars);
            ESP_LOGI(TAG_CH[2][0], "cali data: %d mV", voltage);
            sprintf(str2,"ADC2 value: %d",voltage);
            esp_mqtt_client_publish(client,"ESP32/value",str2, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/espID", chipid, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/sensorID", 2, 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));

        adc_raw[3][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN3);
        ESP_LOGI(TAG_CH[3][0], "raw  data: %d", adc_raw[3][0]);
        if (cali_enable) {
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[3][0], &adc1_3_chars);
            ESP_LOGI(TAG_CH[3][0], "cali data: %d mV", voltage);
            sprintf(str3,"ADC3 value: %d",voltage);
            esp_mqtt_client_publish(client, "ESP32/value",str3, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/espID", chipid, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/sensorID", 3, 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}