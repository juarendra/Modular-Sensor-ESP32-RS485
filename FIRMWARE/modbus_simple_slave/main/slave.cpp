/* 

Created By : Juarendra Ramadhani
Maintenance By : Juarendra Ramadhani
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "mbcontroller.h"   
#include "nvs_flash.h"    
#include "esp_log.h"           
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include <bme680.h>
#include <string.h>
#include <esp_idf_lib_helpers.h>
#include <button.h>
#include <dht.h>
#include <si7021.h>
#include <inttypes.h>
#include "ModbusSlave.h"
#include <mpu6050.h>
#include <tsl2561.h>
#include "driver/gpio.h"
#include <inttypes.h>
#include "protocol_examples_common.h"

#include <esp_wifi.h>
#include <esp_netif.h>
#include "wifi_manager.h"
#include "esp_wifi_types.h"
#include "esp_event.h" 
#include "esp_netif.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_mqtt.h"
#include "cJSON.h"

#define PUBLISH_INTERVAL 1000
#define RESTART_INTERVAL 20000


#ifdef __cplusplus
extern "C" {
#endif

const char* MQTT_HOST = "";
const char* MQTT_USER = "";
const char* MQTT_PASS = "";

const char* MQTT_PORT = "";
const char* MQTTS_PORT = "8883";

bool enabledMQTT = true;

wifi_config_t* config_modbus = NULL;
wifi_config_t* config_wifi = NULL;
mqtt_config_t* config_mqtt = NULL;


#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define MB_RETRIES      (100)
#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_SLAVE_ADDR   (CONFIG_MB_SLAVE_ADDR)      // The address of device in Modbus network
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART


#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  (mb_event_group_t)(MB_READ_MASK | MB_WRITE_MASK)

static const char *SLAVE_TAG = "SLAVE_TEST";

uint16_t dataModbus[11] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

#define pinConfigParameter gpio_num_t(32)

#define BME680_I2C_ADDR 0x77
#define PORT I2C_NUM_0
#define CONFIG_EXAMPLE_I2C_MASTER_SDA gpio_num_t(14)
#define CONFIG_EXAMPLE_I2C_MASTER_SCL gpio_num_t(32)

#define pinDrycontact gpio_num_t(32)
#define pinWaterleak gpio_num_t(32)
#define pinMotionSensor gpio_num_t(32)

struct listSensor{
    int BME680 = 1;
    int SHT21 = 2;
    int Drycontact = 3;
    int DHT22 = 4;
    int Waterleak = 5;
    int MotionSensor = 6;
    int accelGyro = 7;
    int luxSensor = 8; 
}listSensors; 

int selectSensor = listSensors.Drycontact;

static button_t sensorGPIO;
static const char *states[] = {
    [BUTTON_PRESSED]      = "pressed",
    [BUTTON_RELEASED]     = "released",
    [BUTTON_CLICKED]      = "clicked",
    [BUTTON_PRESSED_LONG] = "pressed long",
};

#define SENSOR_TYPE DHT_TYPE_AM2301

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif


void modbusControl(void*);
void readingSensorBME680(void*);
void readingSensorSHT21(void*);
void readingSensorDHT22(void*);
void readingSensorMPU6050(void*);
void readingSensorTSL2561(void*);

void readingSensorTSL2561(void *parameter){
    ESP_LOGI(TAG,"Reading Sensor LUX TSL2561 running from core %d!\n", xPortGetCoreID() );
    uint16_t *dataModbus = (uint16_t*)parameter;

    tsl2561_t dev;

    ESP_ERROR_CHECK(tsl2561_init_desc(&dev, TSL2561_I2C_ADDR_FLOAT, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(tsl2561_init(&dev));

    ESP_LOGI(TAG, "Found TSL2561 in package %s", dev.package_type == TSL2561_PACKAGE_CS ? "CS" : "T/FN/CL");

    uint32_t lux;
    esp_err_t res;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        if ((res = tsl2561_read_lux(&dev, &lux)) != ESP_OK)
            ESP_LOGI(TAG, "Could not read illuminance value: %d (%s)", res, esp_err_to_name(res));
        else{
            ESP_LOGI(TAG, "Illuminance: %" PRIu32 " Lux", lux);
            dataModbus[0] = uint16_t(lux*10);

            if(enabledMQTT){
                uint16_t value = uint16_t(lux*10);
                char Illuminance[2];
                Illuminance[0] = value & 0xFF;
                Illuminance[1] = value >> 8;

                const char* tag_1 =  "RADING TSL2561: Illuminance: ";

                char *buffer = new char[strlen(tag_1)+strlen(Illuminance)+1];
                strcpy(buffer, tag_1);
                strcat(buffer, Illuminance);
                esp_mqtt_publish("/hello", (uint8_t *)buffer, strlen(buffer), 2, false);  
            }   
        }
            
    }
}

void readingSensorMPU6050(void *parameter){
    ESP_LOGI(TAG,"Reading Sensor MPU6050 running from core %d!\n", xPortGetCoreID() );
    mpu6050_dev_t dev;
    uint16_t *dataModbus = (uint16_t*)parameter;

    

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel;
        mpu6050_rotation_t rotation;

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        ESP_LOGI(TAG, "Temperature:  %.1f", temp);
        dataModbus[0] = uint16_t(accel.x);
        dataModbus[1] = uint16_t(accel.y);
        dataModbus[2] = uint16_t(accel.z);
        dataModbus[3] = uint16_t(rotation.x);
        dataModbus[4] = uint16_t(rotation.y);
        dataModbus[5] = uint16_t(rotation.z);
        dataModbus[6] = uint16_t(temp * 10);
        
        if(enabledMQTT){
            uint16_t value = uint16_t(accel.x);
            char accelx[2];
            accelx[0] = value & 0xFF;
            accelx[1] = value >> 8;

            value = uint16_t(accel.y);
            char accely[2];
            accely[0] = value & 0xFF;
            accely[1] = value >> 8;

            value = uint16_t(accel.z);
            char accelz[2];
            accelz[0] = value & 0xFF;
            accelz[1] = value >> 8;

            value = uint16_t(rotation.x);
            char rotationx[2];
            rotationx[0] = value & 0xFF;
            rotationx[1] = value >> 8;

            value = uint16_t(rotation.y);
            char rotationy[2];
            rotationy[0] = value & 0xFF;
            rotationy[1] = value >> 8;

            value = uint16_t(rotation.z);
            char rotationz[2];
            rotationz[0] = value & 0xFF;
            rotationz[1] = value >> 8;


            const char* tag_1 = "Accel x, y, z: ";
            const char* tag_2 = ", rot x, y, x: ";
            const char* tag_3 = ", ";

            char *buffer = new char[strlen(tag_1)+strlen(accelx)+strlen(tag_3)+strlen(accely)+strlen(tag_3)+strlen(accelz)+strlen(tag_2)+strlen(rotationx)+strlen(tag_3)+strlen(rotationy)+strlen(tag_3)+strlen(rotationz)+1];
            strcpy(buffer, tag_1);
            strcat(buffer, accelx);
            strcat(buffer, tag_3);
            strcat(buffer, accely);
            strcat(buffer, tag_3);
            strcat(buffer, accelz);
            strcat(buffer, tag_2);
            strcat(buffer, rotationx);
            strcat(buffer, tag_3);
            strcat(buffer, rotationy);
            strcat(buffer, tag_3);
            strcat(buffer, rotationz);

            esp_mqtt_publish("/hello", (uint8_t *)buffer, strlen(buffer), 2, false);  
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void readingSensorDHT22(void *parameter){
    ESP_LOGI(TAG,"Reading Sensor DHT22 running from core %d!\n", xPortGetCoreID() );
    float temperature, humidity;
    uint16_t *dataModbus = (uint16_t*)parameter;

#ifdef CONFIG_EXAMPLE_INTERNAL_PULLUP
    gpio_set_pull_mode(CONFIG_EXAMPLE_I2C_MASTER_SCL, GPIO_PULLUP_ONLY);
#endif

    while (1)
    {
        if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_I2C_MASTER_SCL, &humidity, &temperature) == ESP_OK){
            ESP_LOGI(TAG,"Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
            dataModbus[0] = uint16_t(temperature*100);
            dataModbus[1] = uint16_t(humidity*100);

            if(enabledMQTT){
                uint16_t value = uint16_t(temperature*100);
                char temp[2];
                temp[0] = value & 0xFF;
                temp[1] = value >> 8;

                value = uint16_t(humidity*100);
                char hum[2];
                hum[0] = value & 0xFF;
                hum[1] = value >> 8;

                
                const char* tag_1 =  "RADING DHT22: Temp: ";
                const char* tag_2 =  ", Hum";

                char *buffer = new char[strlen(tag_1)+strlen(temp)+strlen(tag_2)+strlen(hum)+1];
                strcpy(buffer, tag_1);
                strcat(buffer, temp);
                strcat(buffer, tag_2);
                strcat(buffer, hum);
                
                esp_mqtt_publish("/hello", (uint8_t *)buffer, strlen(buffer), 2, false);  
            }          
        }
        else{
            ESP_LOGI(TAG,"Could not read data from sensor\n");
        }
            
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void on_GPIO(button_t *btn, button_state_t state)
{
    ESP_LOGI(TAG, "GPIO %s", states[state]);
    dataModbus[1] = state;

    const char* tag_ =  "GPIO READING: ";
    char *buffer = new char[strlen(tag_)+strlen(states[state])+1];
    strcpy(buffer, tag_);
    strcat(buffer, states[state]);

    if(enabledMQTT){
        esp_mqtt_publish("/hello", (uint8_t *)buffer, strlen(buffer), 2, false);  
    }
}

void readingGPIO(gpio_num_t pinGPIO){
    ESP_LOGI(TAG,"Reading GPIO running from core %d!\n", xPortGetCoreID() );
    sensorGPIO.gpio = pinGPIO;
    sensorGPIO.pressed_level = 0;
    sensorGPIO.internal_pull = true;
    sensorGPIO.autorepeat = false;
    sensorGPIO.callback = on_GPIO;

    ESP_ERROR_CHECK(button_init(&sensorGPIO));
}

void readingSensorSHT21(void *parameter){
    ESP_LOGI(TAG,"Reading Sensor SHT21 running from core %d!\n", xPortGetCoreID() );
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));
    uint16_t *dataModbus = (uint16_t*)parameter;


    ESP_ERROR_CHECK(si7021_init_desc(&dev, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    float val;
    esp_err_t res;

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        res = si7021_measure_temperature(&dev, &val);
        if (res != ESP_OK)
            ESP_LOGE(TAG,"Could not measure temperature: %d (%s)\n", res, esp_err_to_name(res));
        else{
            ESP_LOGI(TAG,"Temperature: %.2f", val);
            dataModbus[0] = uint16_t(val*100);
        }
        
        uint16_t value = uint16_t(val*100);
        char temp[2];
        temp[0] = value & 0xFF;
        temp[1] = value >> 8;



        res = si7021_measure_humidity(&dev, &val);
        if (res != ESP_OK)
            ESP_LOGE(TAG,"Could not measure humidity: %d (%s)\n", res, esp_err_to_name(res));
        else{
            ESP_LOGI(TAG,"Humidity: %.2f", val);
            dataModbus[1] = uint16_t(val*100);
        }

        value = uint16_t(val*100);
        char hum[2];
        hum[0] = value & 0xFF;
        hum[1] = value >> 8;


        const char* tag_1 =  "RADING SHT21: Temp: ";
        const char* tag_2 =  ", Hum";

        char *buffer = new char[strlen(tag_1)+strlen(temp)+strlen(tag_2)+strlen(hum)+1];
        strcpy(buffer, tag_1);
        strcat(buffer, temp);
        strcat(buffer, tag_2);
        strcat(buffer, hum);

        if(enabledMQTT){
            esp_mqtt_publish("/hello", (uint8_t *)buffer, strlen(buffer), 2, false);  
        }  

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void readingSensorBME680(void *parameter){
    ESP_LOGI(TAG,"Reading Sensor BME680 running from core %d!\n", xPortGetCoreID() );
    uint16_t *dataModbus = (uint16_t*)parameter;
    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_2X, BME680_OSR_2X);
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);
    bme680_set_ambient_temperature(&sensor, 10);
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    TickType_t last_wakeup = xTaskGetTickCount();

    bme680_values_float_t values;
    while (1)
    {

        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            vTaskDelay(duration);
            if (bme680_get_results_float(&sensor, &values) == ESP_OK){
                ESP_LOGI(TAG,"BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                        values.temperature, values.humidity, values.pressure, values.gas_resistance);
                dataModbus[0] = uint16_t(values.temperature*100);
                dataModbus[1] = uint16_t(values.humidity*100);
                dataModbus[2] = uint16_t(values.pressure);
                dataModbus[3] = uint16_t(values.gas_resistance);

                if(enabledMQTT){
                    uint16_t value = uint16_t(values.temperature*100);
                    char temp[2];
                    temp[0] = value & 0xFF;
                    temp[1] = value >> 8;

                    value = uint16_t(values.humidity*100);
                    char hum[2];
                    hum[0] = value & 0xFF;
                    hum[1] = value >> 8;

                    value = uint16_t(values.pressure*100);
                    char pressure[2];
                    pressure[0] = value & 0xFF;
                    pressure[1] = value >> 8;

                    value = uint16_t(values.gas_resistance*100);
                    char gas_resistance[2];
                    gas_resistance[0] = value & 0xFF;
                    gas_resistance[1] = value >> 8;


                    
                    const char* tag_1 =  "RADING BME680: Temp: ";
                    const char* tag_2 =  ", Hum";
                    const char* tag_3 =  ", pressure";
                    const char* tag_4 =  ", gas";

                    char *buffer = new char[strlen(tag_1)+strlen(temp)+strlen(tag_2)+strlen(hum)+strlen(tag_3)+strlen(pressure)+strlen(tag_4)+strlen(gas_resistance)+1];
                    strcpy(buffer, tag_1);
                    strcat(buffer, temp);
                    strcat(buffer, tag_2);
                    strcat(buffer, hum);
                    strcat(buffer, tag_3);
                    strcat(buffer, pressure);
                    strcat(buffer, tag_4);
                    strcat(buffer, gas_resistance);

                    esp_mqtt_publish("/hello", (uint8_t *)buffer, strlen(buffer), 2, false);  
                }   

            }
                
        }
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
    }
    
}

void modbusControl(void *parameter){
    ESP_LOGI(TAG,"Modbus Control running from core %d!", xPortGetCoreID() );
    uint16_t *dataModbus = (uint16_t*)parameter;
    ModbusSlave* pModbusSlave = new ModbusSlave();
    mb_communication_info_t comm;

    // Slave address get from nvs
    char* slaveAddr = (char *) config_modbus->sta.ssid;
    // Baudrate get from nvs
    char* baudrate = (char *) config_modbus->sta.bssid;

    #if CONFIG_MB_COMM_MODE_ASCII
        comm.mode = MB_MODE_ASCII,
    #elif CONFIG_MB_COMM_MODE_RTU
        comm.mode = MB_MODE_RTU,
    #endif
    
    
    printf("Slave Address  %d \n", atoi(slaveAddr));
    printf("Baudrate  %d \n", atoi(baudrate));


    comm.slave_addr = atoi(slaveAddr);
    comm.port = MB_PORT_NUM;
    comm.baudrate = atoi(baudrate);
    comm.parity = MB_PARITY_NONE;

    ESP_ERROR_CHECK(pModbusSlave->begin(&comm));

    uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                                    CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);

    esp_err_t err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    SLAVE_CHECK((err == ESP_OK), ; ,
                    "mb serial set mode failure, uart_set_mode() returned (0x%" PRIu32 ").", (uint32_t)err);
    
    
    pModbusSlave->addRegisterBank(40001, &dataModbus[0], 11);
    for (int i = 0; i < MB_RETRIES; i++) {
        ESP_LOG_BUFFER_HEX_LEVEL("Holding Register bank", dataModbus, 22, ESP_LOG_INFO);
        mb_event_group_t event = pModbusSlave->run(MB_READ_WRITE_MASK);

        if (!(event & MB_READ_WRITE_MASK)) {
            ESP_LOGI(SLAVE_TAG, "Incorrect modbus access type: %d.", event);
        }
        vTaskDelay(1);
    }
    ESP_LOGI("MODBUS", "Modbus Test completed.");
    delete pModbusSlave;
}

void monitoring_task(void *pvParameter)
{
	for(;;){
		//ESP_LOGI(TAG, "free heap: %d",esp_get_free_heap_size());
		vTaskDelay( pdMS_TO_TICKS(10000) );
	}
}
void cb_connection_ok(void *pvParameter){
	ip_event_got_ip_t* param = (ip_event_got_ip_t*)pvParameter;

	/* transform IP to human readable string */
	char str_ip[16];
	esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

	ESP_LOGI(TAG, "I have a connection and my IP is %s!", str_ip);
}

static void connect_() {
  static bool use_tls = false;
  ESP_LOGI("test", "starting mqtt (tls=%d)", use_tls);
  esp_mqtt_start(MQTT_HOST, use_tls ? MQTTS_PORT : MQTT_PORT, "esp-mqtt", MQTT_USER, MQTT_PASS);
}

static void status_callback(esp_mqtt_status_t status) {
  switch (status) {
    case ESP_MQTT_STATUS_CONNECTED:
      break;

    case ESP_MQTT_STATUS_DISCONNECTED:
    default:
      break;
  }
}
static void message_callback(const char *topic, const uint8_t *payload, size_t len, int qos, bool retained) {
  ESP_LOGI("test", "incoming: %s => %s (len=%d qos=%d ret=%d)", topic, payload, (int)len, qos, retained);
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "WiFi connecting");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "WiFi connected");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "WiFi lost connection");
        break;
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "WiFi got IP");
        break;
    default:
        break;
    }
}

void wifi_connection()
{

    esp_netif_init();                   
    esp_event_loop_create_default();    
    esp_netif_create_default_wifi_sta(); 
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); 
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    esp_wifi_set_config(WIFI_IF_STA, config_wifi);
    esp_wifi_start();
    esp_wifi_connect();
}

void app_main(void)
{
    gpio_set_direction(pinConfigParameter, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pinConfigParameter, GPIO_PULLUP_ONLY);

    bool Configdisbale =  gpio_get_level(pinConfigParameter);
    if (Configdisbale ){
        ESP_LOGI(TAG,"Try to Connect Wifi with saved ssid and password" );
        wifi_manager_start();
        if(wifi_manager_fetch_wifi_sta_config()){
            ESP_LOGI(TAG, "Saved WIFI found on startup");
            config_wifi = wifi_manager_get_wifi_sta_config();
		    ESP_LOGI(TAG, "SSID : %s, PASS: %s", config_wifi->sta.ssid, config_wifi->sta.password);
            wifi_connection();
        }
        else{
            ESP_LOGI(TAG, "No saved WIFI found on startup");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "WIFI was initiated ...........\n");

        if(mqtt_manager_fetch_config()){
            ESP_LOGI(TAG, "Saved MQTT found on startup");
            config_mqtt = mqtt_manager_get_config();
		    MQTT_HOST = (char *) config_mqtt->st.mqttBroker;
            MQTT_PASS = (char *) config_mqtt->st.mqttPassword;
            MQTT_PORT = (char *) config_mqtt->st.mqttPort;
            MQTT_USER = (char *) config_mqtt->st.mqttUsername;
            ESP_LOGI(TAG, "mqttBroker: %s, mqttUsername: %s,  mqttPort: %s, mqttPassword: %s ", MQTT_HOST, MQTT_USER, MQTT_PORT, MQTT_PASS);
        }
        else{
            ESP_LOGI(TAG, "No saved MQTT found on startup");
        }
        if(enabledMQTT){
            esp_mqtt_init(status_callback, message_callback, 256, 2000, 1);
            connect_();
        }
        

        if(modbus_manager_fetch_config()){
            ESP_LOGI(TAG, "Saved modbus found on startup");
            config_modbus = modbus_manager_get_config();
		    ESP_LOGI(TAG, "slaveAddr: %s, sensorType: %s,  baudrate: %s", config_modbus->sta.ssid, config_modbus->sta.password, config_modbus->sta.bssid);
        }
        else{
            ESP_LOGI(TAG, "No saved modbus found on startup");
        }
        // Baudrate get from nvs
        char* sensorType = (char *) config_modbus->sta.password;
        selectSensor = atoi(sensorType);
        ESP_LOGI(TAG,"Running Reading Sensor" );
        xTaskCreatePinnedToCore(&modbusControl, "MODBUSCONTROL", 4096, (void*)&dataModbus, 1, NULL, 0);
        if(selectSensor == listSensors.BME680){
            ESP_ERROR_CHECK(i2cdev_init());
            xTaskCreatePinnedToCore(&readingSensorBME680, "READINGSENSORBME680", configMINIMAL_STACK_SIZE *8, (void*)&dataModbus, 1, NULL, 1);    
        } else if(selectSensor == listSensors.SHT21){
            ESP_ERROR_CHECK(i2cdev_init());
            xTaskCreatePinnedToCore(&readingSensorSHT21, "READINGSENSORSHT21", configMINIMAL_STACK_SIZE *8, (void*)&dataModbus, 1, NULL, 1);    
        } else if(selectSensor == listSensors.accelGyro){
            ESP_ERROR_CHECK(i2cdev_init());
            xTaskCreatePinnedToCore(&readingSensorMPU6050, "READINGSENSORMPU6050", configMINIMAL_STACK_SIZE *8, (void*)&dataModbus, 1, NULL, 1);    
        } else if(selectSensor == listSensors.luxSensor){
            ESP_ERROR_CHECK(i2cdev_init());
            xTaskCreatePinnedToCore(&readingSensorTSL2561, "READINGSENSORTSL2561", configMINIMAL_STACK_SIZE *8, (void*)&dataModbus, 1, NULL, 1);    
        } else if(selectSensor == listSensors.Drycontact){
            readingGPIO(pinDrycontact);
        } else if(selectSensor == listSensors.DHT22){
            xTaskCreatePinnedToCore(&readingSensorDHT22, "READINGSENSORDHT22", configMINIMAL_STACK_SIZE *8, (void*)&dataModbus, 1, NULL, 1);
        } else if(selectSensor == listSensors.Waterleak){
            readingGPIO(pinWaterleak);
        }
        else if(selectSensor == listSensors.MotionSensor){
            readingGPIO(pinMotionSensor);
        }
    } else{
        ESP_LOGI(TAG,"Configure parameter on system");
        wifi_manager_start_config();
        wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);
        xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);

    }


}

#ifdef __cplusplus
}
#endif
