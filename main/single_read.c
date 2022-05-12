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
#include "DHT22.h"
#include "driver/mcpwm.h"
#include <time.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "lwip/sys.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11
#define pump_gpio (21)
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

#define BUTTON_GPIO_NUM_DEFAULT     0
#define BUTTON_WAKEUP_LEVEL_DEFAULT     0
#define SERVO_MIN_PULSEWIDTH_US (575) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2460) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        (200)   // Maximum angle in degree upto which servo can rotate
#define SERVO_PULSE_GPIO        (12)   // GPIO connects to the PWM signal line   
#define EXAMPLE_ESP_WIFI_SSID      "iPhone van Matthia"
#define EXAMPLE_ESP_WIFI_PASS      "manzini4"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static int adc_raw[5][10];
static uint8_t pump_in_state = 0;
static const char *TAG = "ADC SINGLE";
static const char *TAG_servo = "example";
static const char *TAGE = "MQTT_TCP";
static uint32_t counter=0;
esp_mqtt_client_handle_t client;
static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc1_1_chars;
static esp_adc_cal_characteristics_t adc1_2_chars;
static esp_adc_cal_characteristics_t adc1_3_chars;
static esp_adc_cal_characteristics_t adc2_chars;
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static const char *TAGG = "wifi station";
static int s_retry_num = 0;
//variables
esp_err_t ret = ESP_OK;
uint32_t voltage0 = 0;
uint32_t voltage1=0;
uint32_t voltage2=0;
uint32_t voltage3=0;
char str0[50];
char str1[50];
char str2[50];
char str3[50];   
bool pos=0;
int angle;

//////////////////////////////////////////////////////////////////////////////////////////////

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAGG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAGG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAGG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAGG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAGG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAGG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAGG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    char str[50];
    char str2[50];
    client = event->client;
    switch (event->event_id)
    
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAGE, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "ESP32/LEDOUTPUT", 0);
        uint8_t mac[6];
    char macStr[18] = { 0 };
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    sprintf(macStr, "%X:%X:%X:%X:%X:%X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf(macStr, "%X:%X:%X:%X:%X:%X \n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        esp_mqtt_client_publish(client, "ESP32/espID", macStr, 0, 1, 0);
        
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
        sprintf(str2,"\nTOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        sprintf(str,"DATA=%.*s", event->data_len, event->data);
        
        


        if(strcmp(str,"DATA=1")==0){
            gpio_set_level(GPIO_NUM_23, 1);
        }  if(strcmp(str,"DATA=0")==0){
            gpio_set_level(GPIO_NUM_23, 0);
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
        .uri = "mqtt://test.mosquitto.org",//here you have
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
}

static inline uint32_t example_convert_servo_angle_to_duty_us(int angle)
{
    return (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}


void DHT_task(void *pvParameter)
{
    setDHTgpio( 21 );
    printf( "Starting DHT Task\n\n");
    while(1) {
    
        printf("=== Reading DHT ===\n" );
        int ret = readDHT();
        
        errorHandler(ret);

        char temp[50];
        char hum[50];
        sprintf( temp,"%.1f\n", getHumidity() );
        sprintf(hum, "%.1f\n", getTemperature() );
        esp_mqtt_client_publish(client,"ESP32/sensorID", "4", 0, 1, 0);
        esp_mqtt_client_publish(client,"ESP32/value", temp, 0, 1, 0);
        esp_mqtt_client_publish(client,"ESP32/sensorID", "5", 0, 1, 0);
        esp_mqtt_client_publish(client,"ESP32/value", hum, 0, 1, 0);
        
        // -- wait at least 2 sec before reading again ------------
        // The interval of whole process must be beyond 2 seconds !! 
        vTaskDelay( 200000 / portTICK_RATE_MS );
    }
}

void Task_moisture1(void *pvParameters)  // This is a task.
{
  printf( "Starting moisture_1 Task\n\n");
  bool cali_enable = adc_calibration_init();
  for (;;) // A Task shall never return or exit.
  {
    adc_raw[0][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN0); //moisture sensor 1
    ESP_LOGI(TAG_CH[0][0], "raw  data: %d", adc_raw[0][0]);
    if (cali_enable) 
    {
        voltage0 = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
        ESP_LOGI(TAG_CH[0][0], "cali data: %d mV", voltage0);
        sprintf(str0,"%d",voltage0);
        esp_mqtt_client_publish(client,"ESP32/value", str0, 0, 1, 0);
        esp_mqtt_client_publish(client,"ESP32/sensorID", "0", 0, 1, 0);
    }
    //vTaskDelay( 200000 / portTICK_PERIOD_MS ); // wait for one second
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void Task_moisture2(void *pvParameters)  // This is a task.
{ bool cali_enable = adc_calibration_init();
  printf( "Starting moisture_2 Task\n\n");
  for (;;) // A Task shall never return or exit.
  {
    adc_raw[1][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN1); //moisture sensor 2
    ESP_LOGI(TAG_CH[1][0], "raw  data: %d", adc_raw[1][0]);
    if (cali_enable) {
            voltage1 = esp_adc_cal_raw_to_voltage(adc_raw[1][0], &adc1_1_chars);
            ESP_LOGI(TAG_CH[1][0], "cali data: %d mV", voltage1);
            sprintf(str1,"%d",voltage1);
            esp_mqtt_client_publish(client,"ESP32/value", str1, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/sensorID", "1", 0, 1, 0);
        }
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void Task_light1(void *pvParameters)  // This is a task.
{ bool cali_enable = adc_calibration_init();
  printf( "Starting light_1 Task\n\n");
  for (;;) // A Task shall never return or exit.
  {
    adc_raw[2][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN2);//light sensor1
        ESP_LOGI(TAG_CH[2][0], "raw  data: %d", adc_raw[2][0]);
        if (cali_enable) {
            voltage2 = esp_adc_cal_raw_to_voltage(adc_raw[2][0], &adc1_2_chars);
            ESP_LOGI(TAG_CH[2][0], "cali data: %d mV", voltage2);
            sprintf(str2,"%d",voltage2);
            esp_mqtt_client_publish(client,"ESP32/value",str2, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/sensorID", "2", 0, 1, 0);
        }
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void Task_light2(void *pvParameters)  // This is a task.
{ bool cali_enable = adc_calibration_init();
  printf( "Starting light_2 Task\n\n");
  for (;;) // A Task shall never return or exit.
  {
    adc_raw[3][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN3);//light sensor2
        ESP_LOGI(TAG_CH[3][0], "raw  data: %d", adc_raw[3][0]);
        if (cali_enable) {
            voltage3 = esp_adc_cal_raw_to_voltage(adc_raw[3][0], &adc1_3_chars);
            ESP_LOGI(TAG_CH[3][0], "cali data: %d mV", voltage3);
            sprintf(str3,"%d",voltage3);
            esp_mqtt_client_publish(client, "ESP32/value",str3, 0, 1, 0);
            esp_mqtt_client_publish(client,"ESP32/sensorID", "3", 0, 1, 0);
        }
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void Task_servo(void *pvParameters)  // This is a task.
{
  printf( "Starting servo Task\n\n");
  for (;;) // A Task shall never return or exit.
  {
    if(pos==0)
    {
        counter=counter+voltage2+voltage3;
        if(counter>10000){
            printf("threshold light reached 500V in total \n");
            counter=0;
            pos=1;
            angle=0;
            ESP_LOGI(TAG_servo, "Angle of rotation: %d", angle);
            ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(angle)));
            vTaskDelay(pdMS_TO_TICKS(600)); 
        }
    }
    else if(pos==1)
    {
        counter=counter+voltage2+voltage3;
        if(counter>10000){
            printf("threshold light reached 500V in total \n");
            counter=0;
            pos=0;
            angle=200;
            ESP_LOGI(TAG_servo, "Angle of rotation: %d", angle);
            ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(angle)));
            vTaskDelay(pdMS_TO_TICKS(600)); 
            }
    }
    printf("counter value %d \n",counter);
  }
}

void Task_waterLevel(void *pvParameters)  // This is a task.
{
  printf( "Starting check_water_level Task\n\n");
  for (;;) // A Task shall never return or exit.
  {
    if(voltage0<1000){//soil moisture
            gpio_set_level(GPIO_NUM_21, 1); printf("pump on \n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(pump_gpio, pump_in_state);printf("pump off \n");
        }

    if(voltage1<1000){
            esp_mqtt_client_publish(client, "ESP32/tank", "water lvl low! \n", 0, 1, 0);
            printf("water lvl low in tank low! \n");
        }
    vTaskDelay(pdMS_TO_TICKS(600)); 
  }
}

void Task_LED(void *pvParameters)  // This is a task.
{
  printf( "Starting update_LED_status Task\n\n");
  for (;;) // A Task shall never return or exit.
  {
    if(voltage0<1000 && voltage1<1000 ){
            gpio_set_level(GPIO_NUM_33, 1);
            gpio_set_level(GPIO_NUM_15, 1);
            gpio_set_level(GPIO_NUM_32, 1);
        } else if(voltage0<1000 && voltage1<1000 ){
            gpio_set_level(GPIO_NUM_33, 1);
            gpio_set_level(GPIO_NUM_15, 1);
            gpio_set_level(GPIO_NUM_32, 0);
        } else {
            gpio_set_level(GPIO_NUM_33, 1);
            gpio_set_level(GPIO_NUM_15, 0);
            gpio_set_level(GPIO_NUM_32, 0);
        }
    vTaskDelay(pdMS_TO_TICKS(600)); 
  }
}

void Task_lightSleep(void *pvParameters)  // This is a task.
{
  printf( "Starting light_sleep_mode Task\n\n");
  for (;;) // A Task shall never return or exit.
  {
        esp_mqtt_client_disconnect(client);
        esp_mqtt_client_stop(client);
        vTaskDelay(1000);
        esp_wifi_disconnect();
        esp_wifi_stop();
        vTaskDelay(1000);

        //light_sleep mode
        /* Wake up in 2 seconds, or when button is pressed */
        esp_sleep_enable_timer_wakeup(6000000);
        esp_sleep_enable_wifi_wakeup();


        

        printf("Entering light sleep\n");
        /* To make sure the complete line is printed before entering sleep mode,
         * need to wait until UART TX FIFO is empty:
         */
        uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

        

        /* Get timestamp before entering sleep */
        int64_t t_before_us = esp_timer_get_time();

        /* Enter sleep mode */
        esp_light_sleep_start();
        /* Execution continues here after wakeup */

        /* Get timestamp after waking up from sleep */
        int64_t t_after_us = esp_timer_get_time();

        /* Determine wake up reason */
        const char* wakeup_reason;
        switch (esp_sleep_get_wakeup_cause()) {
            case ESP_SLEEP_WAKEUP_TIMER:
                wakeup_reason = "timer";
                break;
            case ESP_SLEEP_WAKEUP_GPIO:
                wakeup_reason = "pin";
                break;
            case ESP_SLEEP_WAKEUP_WIFI:
                wakeup_reason = "wifi";
                break;    
            default:
                wakeup_reason = "other";
                break;
        }
        printf("Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms\n",
                wakeup_reason, t_after_us / 1000, (t_after_us - t_before_us) / 1000);

       
        esp_wifi_start();
        esp_wifi_connect();
        vTaskDelay(1000);
        esp_mqtt_client_start(client);
        esp_mqtt_client_reconnect(client);
        vTaskDelay(pdMS_TO_TICKS(600)); 
  }
}

void app_main(void)
{   
    //Initialize NVS
    esp_err_t retu = nvs_flash_init();
    if (retu == ESP_ERR_NVS_NO_FREE_PAGES || retu == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      retu = nvs_flash_init();
    }
    ESP_ERROR_CHECK(retu);

    ESP_LOGI(TAGG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    

    //sleep configuration
    const int button_gpio_num = BUTTON_GPIO_NUM_DEFAULT;
    const int wakeup_level = BUTTON_WAKEUP_LEVEL_DEFAULT;
    gpio_config_t config = {
            .pin_bit_mask = BIT64(button_gpio_num),
            .mode = GPIO_MODE_INPUT
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_wakeup_enable(button_gpio_num,
            wakeup_level == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);


    //start ADC
    adc_init();
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
    nvs_flash_init();

    

    //start MQTT
    mqtt_app_start();

    //configure PWM settings
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    vTaskDelay( 1000 / portTICK_RATE_MS );
	xTaskCreate( &DHT_task, "DHT_task", 2048, NULL, 5, NULL );
    xTaskCreate(&Task_moisture1, "Task_moisture1",2048,NULL,5,NULL);
    xTaskCreate(&Task_moisture2, "Task_moisture2",2048,NULL,5,NULL);
    xTaskCreate(&Task_light1, "Task_light1",2048,NULL,5,NULL);
    xTaskCreate(&Task_light2, "Task_light2",2048,NULL,5,NULL);
    xTaskCreate(&Task_servo,"Task_servo",2048,NULL,5,NULL);
    xTaskCreate(&Task_waterLevel,"Task_waterLevel",2048,NULL,5,NULL);
    xTaskCreate(&Task_LED, "Task_LED",2048,NULL,5,NULL);
    xTaskCreate(&Task_lightSleep,"Task_lightSleep",2048,NULL,5,NULL);
        
}
