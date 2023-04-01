/***********************************************************************
 * Project      :     Solar_GreenHouse_IoT
 * Description  :     Example project for test to connect MQTT server
 * Hardware     :     tiny32_v3
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     30/11/2022
 * Revision     :     1.0
 * Rev0.1       :     Origital
 * Rev0.4       :     - Add control mqtt command (solid, auto, upper humi, lower humi, google time, mqtt update time
 *                      reset, reset engergy)
 *                    - temp & humi average for control solid staete
 *                    - Add update mqtt command
 * Rev0.5       :     - Add Google spreadsheet
 * Rev0.6       :     - AP timeout 5 minute and reset ESP
 * Rev0.7       :     - Add solid state status in json
 * Rev0.8       :     - Add Auto flag to json and mysql
 * Rev0.9       :     - Add command for set time to send database
 * Rev0.10      :     - Add update config_db
 * Rev1.0       :     - Release firmware (22-02-2023)
 *                    - Line Notification
 *                    - Add backup mqtt server
 * Rev1.1       :     - Fix reconnect backup server
 *                    - Add control Solid state
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     admin@innovation.co.th
 * TEL          :     +66 62-308-3299
 ***********************************************************************/
#include <Arduino.h>
#include <tiny32_v3.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h> // include libraries
#include <Adafruit_SHT31.h>
#include <otaUpdate.h>
#include <TridentTD_LineNotify.h>

/**************************************/
/*          Firmware Version          */
/**************************************/
String version = "1.1";

/**************************************/
/*          Header project            */
/**************************************/
void header_print(void)
{
    Serial.printf("\r\n***********************************************************************\r\n");
    Serial.printf("* Project      :     Solar_GreenHouse_IoT\r\n");
    Serial.printf("* Description  :     Example project for test to connect MQTT server\r\n");
    Serial.printf("* Hardware     :     tiny32_v3\r\n");
    Serial.printf("* Author       :     Tenergy Innovation Co., Ltd.\r\n");
    Serial.printf("* Date         :     30/11/2022\r\n");
    Serial.printf("* Revision     :     %s\r\n", version);
    Serial.printf("* website      :     http://www.tenergyinnovation.co.th\r\n");
    Serial.printf("* Email        :     admin@innovation.co.th\r\n");
    Serial.printf("* TEL          :     +66 62-308-3299\r\n");
    Serial.printf("***********************************************************************/\r\n");
}

/**************************************/
/*             MQTT Server            */
/**************************************/
const char *mqtt_server_backup = "167.71.223.61"; // *backup mqtt server
char topic_public[20];
char topic_sub[30];

/**************************************/
/*            GPIO define             */
/**************************************/
#define SOLID_RELAY 33 // control solid relay pin

/**************************************/
/*       Constand define value        */
/**************************************/
// 10 seconds WDT
#define WDT_TIMEOUT 60
#define AP_TIMEOUT 300
const int ambientIn_number = 1;  // revise number of sensor here*
const int ambientOut_number = 1; // revise number of sensor here*
const int airflow_number = 0;    // revise number of sensor here*
const int pzem016_number = 1;    // revise number of sensor here*
const int solid_number = 1;      // revise number of sensor here*

const uint8_t AMBIENT_IN_SELECT = 0;
const uint8_t AMBIENT_OUT_SELECT = 1;
const uint8_t AIRFLOW_SELECT = 2;
const uint8_t PZEM016_SELECT = 3;
const uint8_t SOLID_SELECT = 4;
const uint8_t CONFIG_SELECT = 5;
const uint8_t STARTUP_SQL_SELECT = 6;

#define err_sht31_temperature 128.0 // for sensnor read error - SHT31
#define err_sht31_humidity 128.0    // for sensnor read error - SHT31

#define SOLID_CMD "solid"
#define AUTO_CMD "auto"
#define SPREADSHEET_TIME_CMD "spreadsheet_time"
#define MQTT_TIME_CMD "mqtt_time"
#define MYSQL_TIME_CMD "mysql_time"
#define RESET_CMD "reset"
#define ENGERGY_RESET_CMD "energy_reset"
#define CONFIG_CMD "config"
#define MYSQL_TIME_CMD "mysql_time"

const boolean SUCCESS = 1;
const boolean ERROR = 0;

const char *line_token = "krK5OA7CTgJYGN6ya8CLaWX8Dt27nE4e9ofbBDN0Zqv"; // ETE ALERT

/**************************************/
/*       eeprom address define        */
/**************************************/
#define EEPROM_SIZE 512     // กำหนดขนาดของ eeprom *สำคัญมาก
#define mqtt_server_eep 100 // 40 bytes
#define mqtt_user_eep 140   // 10 bytes
#define mqtt_pass_eep 150   // 10 bytes
#define mqtt_topic_eep 160  // 10 bytes

#define solid_status_eep 10            // 1 byte  10
#define lower_threshould_humi_eep 11   // 4 byte  11-14
#define upper_threshould_humi_eep 15   // 4 byte  15-18
#define auto_flag_eep 19               // 1 byte  19
#define spreadsheet_update_time_eep 20 // 1 byte  20
#define mqtt_update_time_eep 21        // 1 byte  21
#define mysql_update_time_eep 22       // 2 byee 22-23

/**************************************/
/*   MultiTasking function define     */
/**************************************/
void ReadSensor_Task(void *p);

/**************************************/
/*        define global variable      */
/**************************************/
char unit[15];
long lastMsg = 0;
char msg[50];
char string[40];
uint8_t wifistatus;
int WiFi_rssi;
char mqtt_Msg[100];
char Json_Msg[300];
String JsonStr = "";

int mqtt_server_field_length = 40; // global param ( for non blocking w params )
int mqtt_user_field_length = 20;   // global param ( for non blocking w params )
int mqtt_pass_field_length = 20;   // global param ( for non blocking w params )
int mqtt_topic_field_length = 20;  // global param ( for non blocking w params )
char mqtt_server_char[40];
char mqtt_user_char[20];
char mqtt_pass_char[20];
char mqtt_topic_char[20];

bool shouldSaveConfig = false;

//*** define variable of sensor ***
struct ambient
{
    char unit[20];
    String sensor;
    uint8_t sensor_id;
    uint8_t address; // rs485 id
    float temp;
    float humi;
    bool active;
};

struct pzem016
{
    char unit[20];
    String sensor;
    uint8_t sensor_id;
    uint8_t address; // rs485 id
    float volt;
    float amp;
    float power;
    float freq;
    float pf;
    uint32_t energy;
    bool active;
};

struct airflow
{
    char unit[20];
    String sensor;
    uint8_t sensor_id;
    uint8_t address; // rs485 id
    float speed;
    bool active;
};

struct switchs
{
    char unit[20];
    uint8_t sensor_id;
    String sensor;
    bool status;
};

airflow airflow_var[airflow_number];       // system has 2 sensors
ambient ambientOut_var[ambientOut_number]; // system has 1 sensor
ambient ambientIn_var[ambientIn_number];   // system has 4 sensors
pzem016 pzem016_var[pzem016_number];       // system has 1 sensor
switchs solid_var[solid_number];           // system has 1 solid state delay

bool solid_status_var;
uint8_t spreadsheet_update_time_var; // minute [1-255]
uint8_t mqtt_update_time_var;        // second [1-255]
uint16_t mysql_update_time_var;      // second [1-65535]
float temperature_sht31, humidity_sht31;
boolean mqtt_error = 0; // check mqtt value in eeprom

/**************************************/
/*           define function          */
/**************************************/
void callback(char *topic, byte *payload, unsigned int length);
void callback_backup(char *topic, byte *payload, unsigned int length);
void reconnect();
void reconnect_backup();
void saveConfigCallback();
void wifi_config();
bool sensor_variable_init();
bool read_ambientIn_sensor();
bool read_ambientOut_sensor();
bool read_airflow_sensor();
bool read_pzem016_sensor();
String jsonConvert_Sensor(uint8_t sensor_select, uint8_t id);
void eeprom_init();
void spreaddheet_sendData(String params);
boolean mqtt_response(boolean status);
boolean mqtt_response_backup(boolean status);

/**************************************/
/*        define object variable      */
/**************************************/
tiny32_v3 mcu;
WiFiManager wm;
WiFiClient espClient;
WiFiClient espClient_backup;
PubSubClient client(espClient);
PubSubClient client_backup(espClient_backup);
Adafruit_SHT31 sht = Adafruit_SHT31();
StaticJsonDocument<200> doc;

/****************************************/
/*  Google spreadsheet parameter        */
/****************************************/
String GOOGLE_SCRIPT_ID = "AKfycbwj-55zKD0UuJ1-SYtSKZCSyB4VNoCz8Q14OPNRZpbOkSyJ4r3ofcZry4l3XZ5MbnFc"; // Change Deployment ID here
/*https://docs.google.com/spreadsheets/d/15xMPGl_6bA2mmr5P8ZAJA-ktsDSkERbwwqH-83F9dsY/edit?usp=sharing*/

const char *root_ca =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDujCCAqKgAwIBAgILBAAAAAABD4Ym5g0wDQYJKoZIhvcNAQEFBQAwTDEgMB4G\n"
    "A1UECxMXR2xvYmFsU2lnbiBSb290IENBIC0gUjIxEzARBgNVBAoTCkdsb2JhbFNp\n"
    "Z24xEzARBgNVBAMTCkdsb2JhbFNpZ24wHhcNMDYxMjE1MDgwMDAwWhcNMjExMjE1\n"
    "MDgwMDAwWjBMMSAwHgYDVQQLExdHbG9iYWxTaWduIFJvb3QgQ0EgLSBSMjETMBEG\n"
    "A1UEChMKR2xvYmFsU2lnbjETMBEGA1UEAxMKR2xvYmFsU2lnbjCCASIwDQYJKoZI\n"
    "hvcNAQEBBQADggEPADCCAQoCggEBAKbPJA6+Lm8omUVCxKs+IVSbC9N/hHD6ErPL\n"
    "v4dfxn+G07IwXNb9rfF73OX4YJYJkhD10FPe+3t+c4isUoh7SqbKSaZeqKeMWhG8\n"
    "eoLrvozps6yWJQeXSpkqBy+0Hne/ig+1AnwblrjFuTosvNYSuetZfeLQBoZfXklq\n"
    "tTleiDTsvHgMCJiEbKjNS7SgfQx5TfC4LcshytVsW33hoCmEofnTlEnLJGKRILzd\n"
    "C9XZzPnqJworc5HGnRusyMvo4KD0L5CLTfuwNhv2GXqF4G3yYROIXJ/gkwpRl4pa\n"
    "zq+r1feqCapgvdzZX99yqWATXgAByUr6P6TqBwMhAo6CygPCm48CAwEAAaOBnDCB\n"
    "mTAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUm+IH\n"
    "V2ccHsBqBt5ZtJot39wZhi4wNgYDVR0fBC8wLTAroCmgJ4YlaHR0cDovL2NybC5n\n"
    "bG9iYWxzaWduLm5ldC9yb290LXIyLmNybDAfBgNVHSMEGDAWgBSb4gdXZxwewGoG\n"
    "3lm0mi3f3BmGLjANBgkqhkiG9w0BAQUFAAOCAQEAmYFThxxol4aR7OBKuEQLq4Gs\n"
    "J0/WwbgcQ3izDJr86iw8bmEbTUsp9Z8FHSbBuOmDAGJFtqkIk7mpM0sYmsL4h4hO\n"
    "291xNBrBVNpGP+DTKqttVCL1OmLNIG+6KYnX3ZHu01yiPqFbQfXf5WRDLenVOavS\n"
    "ot+3i9DAgBkcRcAtjOj4LaR0VknFBbVPFd5uRHg5h6h+u/N5GJG79G+dwfCMNYxd\n"
    "AfvDbbnvRG15RjF+Cv6pgsH/76tuIMRQyV+dTZsXjAzlAcmgQWpzU/qlULRuJQ/7\n"
    "TBj0/VLZjmmx6BEP3ojY+x1J96relc8geMJgEtslQIxq/H5COEBkEveegeGTLg==\n"
    "-----END CERTIFICATE-----\n";

/***********************************************************************
 * FUNCTION:    setup
 * DESCRIPTION: setup process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/

void setup()
{
    Serial.begin(115200);
    mcu.buzzer_beep(1);
    header_print();

    //*** Define Unit ID ***
    String _mac1 = WiFi.macAddress().c_str();
    Serial.printf("Info: macAddress = %s\r\n", WiFi.macAddress().c_str());
    String _mac2 = _mac1.substring(9, 17);
    _mac2.replace(":", "");
    String _mac3 = "tiny32-" + _mac2;
    _mac3.toCharArray(unit, _mac3.length() + 1);
    Serial.printf("Info: Unit ID => %s\r\n", unit);

  

    //*** Variable structure inital ***/
    sensor_variable_init();


    //*** EEPROM inital ***/
    Serial.println("Info: EEPROM initial");
    eeprom_init();

    //*** Reset WiFi setting ***
    if (mcu.Sw1())
    {
        Serial.printf("Info: Reset Wifi and EEPROM config .....");
        // mcu.buzzer_beep(3);
        while (mcu.Sw2())
            ;

        for (int i = 0; i < EEPROM_SIZE; i++)
        {
            EEPROM.writeByte(i, 0xFF);
            Serial.printf("Debug: Erase eeprom =>... %.0f %c\r\n", (float)((float)i / EEPROM_SIZE) * 100, 37);
            vTaskDelay(10);
        }
        EEPROM.commit();
        wm.resetSettings();
        Serial.println("done");
    }

    mcu.TickBlueLED(0.1);

    mcu.Relay(0); // ONLINE LAMP

    //*** GPIO initial ***
    Serial.print("Info: GPIO Initial ....");
    pinMode(SOLID_RELAY, OUTPUT);
    digitalWrite(SOLID_RELAY, solid_status_var);
    Serial.println("done");

    /* Initial XY-MD02 sensor */
    Serial.print("Info: Initial XY-MD02 sensor ... ");
    mcu.XY_MD02_begin(RXD2, TXD2);
    Serial.println("Done.\r\n");

    /* Initial PZEM-016 sensor */
    Serial.print("Info: Initial PZEM-016 sensor ... ");
    mcu.PZEM_016_begin(RXD2, TXD2);
    Serial.println("Done. \r\n");

    /* Initial Wind speed sensor */
    // Serial.print("Info: Initial Wind speed sensor ... ");
    // mcu.tiny32_WIND_RSFSN01_begin(RXD2, TXD2);
    // Serial.println("Done. \r\n");

    /* Initial SHT31 sensor */
    Serial.print("Info: Initial SHT31 sensor ... \r\n");
    Wire.begin();
    Wire.beginTransmission(0x45); // Addr: 0x45
    if (Wire.endTransmission() == 0)
    {
        sht.begin(0x45);
        ambientOut_var[0].humi = sht.readHumidity();
        ambientOut_var[0].temp = sht.readTemperature();
    }
    else
    {
        temperature_sht31 = err_sht31_temperature;
        humidity_sht31 = err_sht31_humidity;
        Serial.println(F("SHT31(0x44,0x45)...Not Install"));
    }
    Serial.println("Done. \r\n");

    //*** setting wifi and get mqtt config from eerpom ***
    wifi_config();

    //*** check wifi connection ***
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("Info: SSID => ");
        Serial.println(wm.getWiFiSSID().c_str());
        Serial.print("Info: Passwrod => ");
        Serial.println(wm.getWiFiPass().c_str());
        Serial.print("Info: IP Address => ");
        Serial.println(WiFi.localIP());
        Serial.print("Info Channel => ");
        Serial.println(WiFi.channel()); // wifi channel must to match with slave code
        Serial.print("Info: WiFi Mode => ");
        Serial.println(WiFi.getMode());
        Serial.printf("Info: MQTT Server => %s\r\n", mqtt_server_char);
        (mcu.Slid_sw()) ? Serial.printf("Info: *MQTT Server Backup => %s\r\n", mqtt_server_backup) : NULL;
        Serial.printf("Info: MQTT User => %s\r\n", mqtt_user_char);
        Serial.printf("Info: MQTT Pass => %s\r\n", mqtt_pass_char);
        Serial.printf("Info: MQTT Topic => %s\r\n", mqtt_topic_char);

        //*** for check paramter in eeprom ***/
        if (mqtt_server_char[0] < 33 || mqtt_server_char[0] > 126)
        {
            Serial.printf("Error: MQTT Server is blank!!\r\n");
            for (int i = 0; i < sizeof(mqtt_pass_char); i++)
            {
                Serial.printf("\tmqtt_server_char[%d] = %c[%d]\r\n", i, mqtt_server_char[i], mqtt_pass_char[i]);
            }
            mcu.buzzer_beep(3);
            vTaskDelay(500);
            mqtt_error = 1;
        }
        if (mqtt_user_char[0] < 33 || mqtt_user_char[0] > 126)
        {
            Serial.printf("\r\nError: MQTT user is blank!!\r\n");
            for (int i = 0; i < sizeof(mqtt_pass_char); i++)
            {
                Serial.printf("\tmqtt_user_char[%d] = %c[%d]\r\n", i, mqtt_user_char[i], mqtt_user_char[i]);
            }
            mcu.buzzer_beep(3);
            vTaskDelay(500);
            mqtt_error = 1;
        }
        if (mqtt_pass_char[0] < 33 || mqtt_pass_char[0] > 126)
        {
            Serial.printf("\r\nError: MQTT password is blank!!\r\n");
            for (int i = 0; i < sizeof(mqtt_pass_char); i++)
            {
                Serial.printf("\tmqtt_pass_char[%d] = %c[%d]\r\n", i, mqtt_pass_char[i], mqtt_pass_char[i]);
            }
            mcu.buzzer_beep(3);
            vTaskDelay(500);
            mqtt_error = 1;
        }
        if (mqtt_topic_char[0] < 33 || mqtt_topic_char[0] > 126)
        {
            Serial.printf("\n\rError: MQTT topic is blank!!\r\n");
            for (int i = 0; i < sizeof(mqtt_pass_char); i++)
            {
                Serial.printf("\tmqtt_topic_char[%d] = %c[%d]\r\n", i, mqtt_topic_char[i], mqtt_topic_char[i]);
            }
            mcu.buzzer_beep(3);
            vTaskDelay(500);
            mqtt_error = 1;
        }

        //*** generate topic_public ***
        String _strtmp = String(mqtt_topic_char) + "/" + String(unit);
        _strtmp.toCharArray(topic_public, _strtmp.length() + 1);
        Serial.printf("Info: Topic_pulic => %s\r\n", topic_public);

        //*** generate topic_subscribe ***
        _strtmp = String(mqtt_topic_char) + "/" + String(unit) + "/control";
        _strtmp.toCharArray(topic_sub, _strtmp.length() + 1);
        Serial.printf("Info: topic_sub => %s\r\n", topic_sub);

        //*** MQTT config server ***
        /* for example */
        // client.setServer(mqtt_server, 1883);
        // client.connect("etc/tiny32-123456", "tiny32", "tiny32");
        // client.subscribe("etc/tiny32-123456/control");
        // client.subscribe("#");

        /* main mqtt server */
        client.setServer(mqtt_server_char, 1883);
        client.connect(topic_public, mqtt_user_char, mqtt_pass_char);
        client.setCallback(callback);
        client.subscribe(topic_sub);

        /* backup mqtt server */
        if (mcu.Slid_sw())
        {
            client_backup.setServer(mqtt_server_backup, 1883);
            client_backup.connect(topic_public, mqtt_user_char, mqtt_pass_char);
            client_backup.setCallback(callback_backup);
            client_backup.subscribe(topic_sub);
        }
    }
    else
    {
        Serial.printf("Error: Can't connect to network ...\r\n");
        // mcu.buzzer_beep(3);
    }

    //*** Config WatchDog ***

    Serial.print("Info: Configuring WDT...");
    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);
    Serial.println("done");
    otaUpdateFunction();

    /*** Updtate config for this unit when power on ***/
    String _JsonStr;
    // Update Total parameter
    _JsonStr = jsonConvert_Sensor(STARTUP_SQL_SELECT, NULL);
    _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
    Serial.printf("Info: Json size = %d\r\n", _JsonStr.length());
    Serial.printf("%s\r\n\r\n", Json_Msg);
    client_backup.publish(topic_public, Json_Msg); // don't update mqtt when set time = 0
    vTaskDelay(100);

    LINE.setToken(line_token);
    LINE.notifyPicture("******** ระบบเริ่มทำงาน (" + WiFi.SSID() + ") ********", "https://www.tenergyinnovation.co.th/wp-content/uploads/2021/07/tenergy_logo-e1628182609668.png"); // revise
    vTaskDelay(300);

    mcu.Relay(1);
    mcu.TickBlueLED(1);
    mcu.buzzer_beep(2);

    /*** Intial Multitasking function ***/
    xTaskCreate(&ReadSensor_Task, "ReadSensor_Task", 2048, NULL, 10, NULL);
}

/***********************************************************************
 * FUNCTION:    loop
 * DESCRIPTION: loop process
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void loop_test()
{

Serial.printf("Hello world");
vTaskDelay(1000);
esp_task_wdt_reset();
}

void loop()
{
    // #define VERBOSE //verbose output

    float _temperature_avg = 0; // clear value
    float _humidity_avg = 0;    // clear value
    static long _interval_sensor = 0;
    static long _interval_wifi = 0;
    static long _interval_spreadsheet = 0;
    static long _interval_mysql = 0;

    String _JsonStr;
    /* main mqtt server */
    if (!client.connected())
    {
        Serial.printf("Error: Can't connect main mqtt server\r\n");
        ESP.restart();
        reconnect();
    }
    client.loop();

    /* backup mqtt server */
    if (mcu.Slid_sw())
    {
        if (!client_backup.connected())
        {
            Serial.printf("Error: Can't connect backup mqtt server\r\n");
            reconnect_backup();
        }
        client_backup.loop();
    }

    esp_task_wdt_reset();
    vTaskDelay(100);

    // Loop for update mysql
    if (mysql_update_time_var == 0)
    {
        // time to update mysql = 0
        // it mean don't want to record DB
    }
    else if (millis() - _interval_mysql > (mysql_update_time_var * 1000))
    {
        // mcu.buzzer_beep(1);
        Serial.printf("Info: Update mysql DB evergy %d second\r\n", mysql_update_time_var);
        _interval_mysql = millis();
    }

    // Loop for update mqtt and proces
    if (millis() - _interval_sensor > (mqtt_update_time_var * 1000))
    {
        // read Ambient Indoor sensor
        for (int i = 0; i < ambientIn_number; i++)
        {
            // Serial.printf("Info: ambientIn_var[%d].sensor : %s\r\n", i, ambientIn_var[i].sensor);
            // Serial.printf("\t\t- ambientIn_var[%d].unit : %s\r\n", i, ambientIn_var[i].unit);
            // Serial.printf("\t\t- ambientIn_var[%d].sensor_id : %d\r\n", i, ambientIn_var[i].sensor_id);
            // Serial.printf("\t\t- ambientIn_var[%d].address : %d\r\n", i, ambientIn_var[i].address);
            // Serial.printf("\t\t- ambientIn_var[%d].temp : %.1f\r\n", i, ambientIn_var[i].temp);
            // Serial.printf("\t\t- ambientIn_var[%d].humi : %.1f\r\n\r\n", i, ambientIn_var[i].humi);

            _JsonStr = jsonConvert_Sensor(AMBIENT_IN_SELECT, i);
            _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
            // Serial.printf("Info: Json size = %d\r\n", _JsonStr.length());
            // Serial.printf("%s\r\n\r\n", Json_Msg);
            if (mqtt_update_time_var != 0)
            {
                client.publish(topic_public, Json_Msg);                                 // don't update mqtt when set time = 0
                (mcu.Slid_sw()) ? client_backup.publish(topic_public, Json_Msg) : NULL; // update mqtt to backup mqtt server
            }
            // Serial.println("---------------------");
            vTaskDelay(100);
            _temperature_avg += ambientIn_var[i].temp;
            _humidity_avg += ambientIn_var[i].humi;
        }

        // Average of Indoor Tempeature and Humidity
        _temperature_avg = _temperature_avg / (float)ambientIn_number;
        _humidity_avg = _humidity_avg / (float)ambientIn_number;
        Serial.println("---------------------");
        Serial.printf("Info: Indoor Temperature average => %.1f C\r\n", _temperature_avg);
        // Serial.printf("Info: Indoor Humidity average => %.1f %c [%.1f %c - %.1f %c]\r\n", _humidity_avg, 37, lower_threshould_humi_var, 37, upper_threshould_humi_var, 37);
        Serial.println("---------------------");

        // read Ambient Outdoor sensor
        for (int i = 0; i < ambientOut_number; i++)
        {
            // Serial.printf("Info: ambientOut_var[%d].sensor : %s\r\n", i, ambientOut_var[i].sensor);
            // Serial.printf("\t\t- ambientOut_var[%d].unit: %s\r\n", i, ambientOut_var[i].unit);
            // Serial.printf("\t\t- ambientOut_var[%d].sensor_id : %d\r\n", i, ambientOut_var[i].sensor_id);
            // Serial.printf("\t\t- ambientOut_var[%d].address : %d\r\n", i, ambientOut_var[i].address);
            // Serial.printf("\t\t- ambientOut_var[%d].temp : %.1f\r\n", i, ambientOut_var[i].temp);
            // Serial.printf("\t\t- ambientOut_var[%d].humi : %.1f\r\n\r\n", i, ambientOut_var[i].humi);

            _JsonStr = jsonConvert_Sensor(AMBIENT_OUT_SELECT, i);
            _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
            // Serial.printf("Info: Json size = %d\r\n", _JsonStr.length());
            // Serial.printf("%s\r\n\r\n", Json_Msg);
            if (mqtt_update_time_var != 0)
            {
                client.publish(topic_public, Json_Msg);                                 // don't update mqtt when set time = 0
                (mcu.Slid_sw()) ? client_backup.publish(topic_public, Json_Msg) : NULL; // update mqtt to backup mqtt server
            }
            // Serial.println("---------------------");
            vTaskDelay(100);
        }

        // read Airflow sensor
        // for (int i = 0; i < airflow_number; i++)
        // {
        //     // Serial.printf("Info: airflow_var[%d].sensor : %s\r\n", i, airflow_var[i].sensor);
        //     // Serial.printf("\t\t- airflow_var[%d].unit : %s\r\n", i, airflow_var[i].unit);
        //     // Serial.printf("\t\t- airflow_var[%d].sensor_id : %d\r\n", i, airflow_var[i].sensor_id);
        //     // Serial.printf("\t\t- airflow_var[%d].address : %d\r\n", i, airflow_var[i].address);
        //     // Serial.printf("\t\t- airflow_var[%d].speed : %.1f\r\n\r\n", i, airflow_var[i].speed);

        //     _JsonStr = jsonConvert_Sensor(AIRFLOW_SELECT, i);
        //     _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
        //     // Serial.printf("Info: Json size = %d\r\n", _JsonStr.length());
        //     // Serial.printf("%s\r\n\r\n", Json_Msg);
        //     if (mqtt_update_time_var != 0)
        //         client.publish(topic_public, Json_Msg); // don't update mqtt when set time = 0
        //     // Serial.println("---------------------");
        //     vTaskDelay(100);
        // }

        // read Pzem-016 sensor
        for (int i = 0; i < pzem016_number; i++)
        {
            // Serial.printf("Info: pzem016_var[%d].sensor : %s\r\n", i, pzem016_var[i].sensor);
            // Serial.printf("\t\t- pzem016_var[%d].unit : %s\r\n", i, pzem016_var[i].unit);
            // Serial.printf("\t\t- pzem016_var[%d].sensor_id : %d\r\n", i, pzem016_var[i].sensor_id);
            // Serial.printf("\t\t- pzem016_var[%d].address : %d\r\n", i, pzem016_var[i].address);
            // Serial.printf("\t\t- pzem016_var[%d].volt : %.1f\r\n", i, pzem016_var[i].volt);
            // Serial.printf("\t\t- pzem016_var[%d].amp : %.1f\r\n", i, pzem016_var[i].amp);
            // Serial.printf("\t\t- pzem016_var[%d].power : %.1f\r\n", i, pzem016_var[i].power);
            // Serial.printf("\t\t- pzem016_var[%d].energy : %d\r\n", i, pzem016_var[i].energy);
            // Serial.printf("\t\t- pzem016_var[%d].freq : %.1f\r\n", i, pzem016_var[i].freq);

            _JsonStr = jsonConvert_Sensor(PZEM016_SELECT, i);
            _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
            // Serial.printf("Info: Json size = %d\r\n", _JsonStr.length());
            // Serial.printf("%s\r\n\r\n", Json_Msg);
            if (mqtt_update_time_var != 0)
            {
                client.publish(topic_public, Json_Msg);                                 // don't update mqtt when set time = 0
                (mcu.Slid_sw()) ? client_backup.publish(topic_public, Json_Msg) : NULL; // update mqtt to backup mqtt server
            }
            // Serial.println("---------------------");
            vTaskDelay(100);
        }

        _interval_sensor = millis();
    }

    // Update Google shreadsheet
    if (spreadsheet_update_time_var == 0)
    {
        // time to update spreadsheet = 0
        // it mean don't want to record google spreadsheet
    }
    else if (millis() - _interval_spreadsheet > (spreadsheet_update_time_var * 60 * 1000))
    {

        if (WiFi.status() == WL_CONNECTED)
        {
            mcu.BuildinLED(1);
            // mcu.buzzer_beep(1);
            Serial.printf("Info: Update google spreadsheet ....");

            /*
            Unit
            Indoor [1] Temp (C)
            Indoor [1] Temp (C)
            Indoor [1] Temp (C)
            Indoor [1] Temp (C)
            Indoor [1] Humi (%)
            Indoor [1] Humi (%)
            Indoor [1] Humi (%)
            Indoor [1] Humi (%)
            Outdoor Temp (C)
            Outdoor Humi (%)
            Airflow [1] (m/s)
            Airflow [2] (m/s)
            volt (V)
            current (A)
            Frequency (Hz)
            PF
            Power (W)
            Energy (kWh)
            Fan status (ON/OFF)
            Auto Mode (ON/OFF)
            */

            String DataString = "unit=";
            DataString += String(unit);

            DataString += "&Indoor_1_temp=";
            DataString += String(ambientIn_var[0].temp);

            DataString += "&Indoor_1_humi=";
            DataString += String(ambientIn_var[0].humi);

            DataString += "&Outdoor_1_temp=";
            DataString += String(ambientOut_var[0].temp);

            DataString += "&Outdoor_1_humi=";
            DataString += String(ambientOut_var[0].humi);

            DataString += "&volt=";
            DataString += String(pzem016_var[0].volt);

            DataString += "&current=";
            DataString += String(pzem016_var[0].amp);

            DataString += "&Freq=";
            DataString += String(pzem016_var[0].freq);

            DataString += "&PF=";
            DataString += String(pzem016_var[0].pf);

            DataString += "&Power=";
            DataString += String(pzem016_var[0].power);

            DataString += "&Energy=";
            DataString += String((float)pzem016_var[0].energy * 0.001);

            DataString += "&Fan=";
            DataString += String(solid_status_var);

            spreaddheet_sendData(DataString);
            Serial.println("done");
        }
        else
        {
            mcu.BuildinLED(1);
            // mcu.buzzer_beep(3);
            Serial.printf("Error: Can't update google spreadsheet !!\r\n");
        }

        mcu.BuildinLED(0);
        _interval_spreadsheet = millis();
    }

    // WiFi connection check
    static uint8_t _wifi_error = 0;
    if (millis() - _interval_wifi > 1000)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            serverOTA.handleClient();
            _wifi_error = 0;
        }
        else if (_wifi_error++ > 20)
        {
            Serial.println("Error: can't connect to WiFi router");
            Serial.println("\r\r System is on going to reset");
            // mcu.buzzer_beep(3);
            ESP.restart();
        }

        _interval_wifi = millis();
    }
}

/***********************************************************************
 * FUNCTION:    callback
 * DESCRIPTION: callback function for mqtt receive message
 * PARAMETERS:  char* topic, byte* message, unsigned int length
 * RETURNED:    nothing
 ***********************************************************************/
void callback(char *topic, byte *payload, unsigned int length)
{
    char _sensor_char[length];
    char _value_char[length];
    String _sensor = "";
    String _value = "";
    int _index = 0;
    String _JsonStr = "";

    Serial.println("Info: Message Receive");
    Serial.print("\tmsg.topic => ");
    Serial.println(topic);

    Serial.print("\tmsg.payload =>");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
        // Serial.printf("\t\tpayload[%d] => %c\r\n", i, payload[i]);
    }
    Serial.println();
    Serial.printf("\tmsg.length => %d\r\n", length);

    // clear buffer
    for (int i = 0; i < length; i++)
    {
        _sensor_char[i] = 0;
        _value_char[i] = 0;
    }

    //*** detect type of sensor
    do
    {
        _sensor_char[_index] = (char)payload[_index];
        _index++;
    } while ((char)payload[_index] != ':' && _index < length);

    _index++;

    //*** detect value ***
    int _index2 = 0;
    do
    {
        _value_char[_index2] = (char)payload[_index];
        _index++;
        _index2++;
    } while (_index < length);

    // // debug value from payload
    // Serial.printf("\t\t _sensor_char => %s\r\n", _sensor_char);
    // Serial.printf("\t\t _value_char => %s\r\n", _value_char);
    // for (int i = 0; i < length; i++)
    // {
    //     Serial.printf("\t_sensor_char[%d]: %c{%d}\t _value_char[%d]: %c{%d}\r\n", i, _sensor_char[i], _sensor_char[i], i, _value_char[i], _value_char[i]);
    // }

    _sensor = String(_sensor_char);
    _value = String(_value_char);
    Serial.printf("\t\t_sensor => %s\r\n", _sensor);
    Serial.printf("\t\t_value => %s\r\n", _value);

    //*** matching command from mqtt payload

    // Control Solid state
    if (_sensor == SOLID_CMD)
    {
        Serial.println("\t\t- solid command -");
        if (_value.toInt() == 0 || _value.toInt() == 1)
        {

            mcu.buzzer_beep(1);
            solid_status_var = _value.toInt();
            digitalWrite(SOLID_RELAY, solid_status_var);
            Serial.printf("\t\t\tvalue = %d\r\n", solid_status_var);
            solid_status_var ? Serial.println("\t\t\tSOLID STATE => ON") : Serial.println("\t\t\tSOLID STATE => OFF");
            EEPROM.writeByte(solid_status_eep, solid_status_var);
            EEPROM.commit();
            mqtt_response(SUCCESS);
        }
        else
        {
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response(ERROR);
        }
    }

    // Setting Update spreadsheet time
    else if (_sensor == SPREADSHEET_TIME_CMD)
    {
        Serial.println("\t\t- spreadsheet time command -");
        if (_value.toInt() <= 100 && _value.toInt() >= 0)
        {
            // mcu.buzzer_beep(1);
            spreadsheet_update_time_var = _value.toInt();
            Serial.printf("\t\t\tvalue = %d min\r\n", spreadsheet_update_time_var);
            EEPROM.writeByte(spreadsheet_update_time_eep, spreadsheet_update_time_var);
            EEPROM.commit();
            mqtt_response(SUCCESS);
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response(ERROR);
        }
    }

    // Setting Update MQTT time
    else if (_sensor == MQTT_TIME_CMD)
    {
        Serial.println("\t\t- mqtt time command -");
        if (_value.toInt() <= 120 && _value.toInt() >= 0)
        {
            mcu.buzzer_beep(1);
            mqtt_update_time_var = _value.toInt();
            Serial.printf("\t\t\tvalue = %d min\r\n", mqtt_update_time_var);
            EEPROM.writeByte(mqtt_update_time_eep, mqtt_update_time_var);
            EEPROM.commit();
            mqtt_response(SUCCESS);
        }
        else
        {
            // mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response(ERROR);
        }
    }

    // Setting Update MYSQL time
    else if (_sensor == MYSQL_TIME_CMD)
    {
        Serial.println("\t\t- mysql time command -");
        if (_value.toInt() <= 120 && _value.toInt() >= 0)
        {
            mcu.buzzer_beep(1);
            mysql_update_time_var = _value.toInt();
            Serial.printf("\t\t\tvalue = %d min\r\n", mysql_update_time_var);
            EEPROM.writeUShort(mysql_update_time_eep, mysql_update_time_var);
            EEPROM.commit();
            mqtt_response(SUCCESS);
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response(ERROR);
        }
    }

    // Reset System
    else if (_sensor == RESET_CMD)
    {
        if (_value.toInt() == 1)
        {
            Serial.println("\t\t- reset command -");
            mcu.buzzer_beep(4);
            mqtt_response(SUCCESS);
            ESP.restart();
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response(ERROR);
        }
    }

    // Reset Energy
    else if (_sensor == ENGERGY_RESET_CMD)
    {
        if (_value.toInt() == 1)
        {
            mcu.buzzer_beep(1);
            Serial.println("\t\t- energy reset command -");
            mcu.buzzer_beep(1);
            mcu.PZEM_016_ResetEnergy(pzem016_var[0].address);
            mqtt_response(SUCCESS);
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response(ERROR);
        }
    }

    // Config command
    else if (_sensor == CONFIG_CMD)
    {
        if (_value == "?")
        {
            Serial.println("\t\t- config command -");
            mcu.buzzer_beep(1);
            _JsonStr = jsonConvert_Sensor(CONFIG_SELECT, NULL);
            _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
            // Serial.printf("Info: Json size = %d\r\n", _JsonStr.length());
            // Serial.printf("%s\r\n\r\n", Json_Msg);
            client.publish(topic_public, Json_Msg); // don't update mqtt when set time = 0
            vTaskDelay(100);
        }
    }

    else
    {
        Serial.println("\t\t- !! Dont't match any command !! -");
        mqtt_response(ERROR);
        mcu.buzzer_beep(3);
    }
}

/***********************************************************************
 * FUNCTION:    callback_backup
 * DESCRIPTION: callback function for backup mqtt receive message
 * PARAMETERS:  char* topic, byte* message, unsigned int length
 * RETURNED:    nothing
 ***********************************************************************/
void callback_backup(char *topic, byte *payload, unsigned int length)
{
    char _sensor_char[length];
    char _value_char[length];
    String _sensor = "";
    String _value = "";
    int _index = 0;
    String _JsonStr = "";

    Serial.println("Info: Message Receive");
    Serial.print("\tmsg.topic => ");
    Serial.println(topic);

    Serial.print("\tmsg.payload =>");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
        // Serial.printf("\t\tpayload[%d] => %c\r\n", i, payload[i]);
    }
    Serial.println();
    Serial.printf("\tmsg.length => %d\r\n", length);

    // clear buffer
    for (int i = 0; i < length; i++)
    {
        _sensor_char[i] = 0;
        _value_char[i] = 0;
    }

    //*** detect type of sensor
    do
    {
        _sensor_char[_index] = (char)payload[_index];
        _index++;
    } while ((char)payload[_index] != ':' && _index < length);

    _index++;

    //*** detect value ***
    int _index2 = 0;
    do
    {
        _value_char[_index2] = (char)payload[_index];
        _index++;
        _index2++;
    } while (_index < length);

    // // debug value from payload
    // Serial.printf("\t\t _sensor_char => %s\r\n", _sensor_char);
    // Serial.printf("\t\t _value_char => %s\r\n", _value_char);
    // for (int i = 0; i < length; i++)
    // {
    //     Serial.printf("\t_sensor_char[%d]: %c{%d}\t _value_char[%d]: %c{%d}\r\n", i, _sensor_char[i], _sensor_char[i], i, _value_char[i], _value_char[i]);
    // }

    _sensor = String(_sensor_char);
    _value = String(_value_char);
    Serial.printf("\t\t_sensor => %s\r\n", _sensor);
    Serial.printf("\t\t_value => %s\r\n", _value);

    //*** matching command from mqtt payload

    // Control Solid state
    if (_sensor == SOLID_CMD)
    {
        Serial.println("\t\t- solid command -");
        if (_value.toInt() == 0 || _value.toInt() == 1)
        {

            mcu.buzzer_beep(1);
            solid_status_var = _value.toInt();
            digitalWrite(SOLID_RELAY, !solid_status_var);
            Serial.printf("\t\t\tvalue = %d\r\n", solid_status_var);
            solid_status_var ? Serial.println("\t\t\tSOLID STATE => ON") : Serial.println("\t\t\tSOLID STATE => OFF");
            EEPROM.writeByte(solid_status_eep, solid_status_var);
            EEPROM.commit();
            mqtt_response_backup(SUCCESS);
        }
        else
        {
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response_backup(ERROR);
        }
    }

    // Setting Update spreadsheet time
    else if (_sensor == SPREADSHEET_TIME_CMD)
    {
        Serial.println("\t\t- spreadsheet time command -");
        if (_value.toInt() <= 100 && _value.toInt() >= 0)
        {
            // mcu.buzzer_beep(1);
            spreadsheet_update_time_var = _value.toInt();
            Serial.printf("\t\t\tvalue = %d min\r\n", spreadsheet_update_time_var);
            EEPROM.writeByte(spreadsheet_update_time_eep, spreadsheet_update_time_var);
            EEPROM.commit();
            mqtt_response_backup(SUCCESS);
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response_backup(ERROR);
        }
    }

    // Setting Update MQTT time
    else if (_sensor == MQTT_TIME_CMD)
    {
        Serial.println("\t\t- mqtt time command -");
        if (_value.toInt() <= 120 && _value.toInt() >= 0)
        {
            mcu.buzzer_beep(1);
            mqtt_update_time_var = _value.toInt();
            Serial.printf("\t\t\tvalue = %d min\r\n", mqtt_update_time_var);
            EEPROM.writeByte(mqtt_update_time_eep, mqtt_update_time_var);
            EEPROM.commit();
            mqtt_response_backup(SUCCESS);
        }
        else
        {
            // mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response_backup(ERROR);
        }
    }

    // Setting Update MYSQL time
    else if (_sensor == MYSQL_TIME_CMD)
    {
        Serial.println("\t\t- mysql time command -");
        if (_value.toInt() <= 120 && _value.toInt() >= 0)
        {
            mcu.buzzer_beep(1);
            mysql_update_time_var = _value.toInt();
            Serial.printf("\t\t\tvalue = %d min\r\n", mysql_update_time_var);
            EEPROM.writeUShort(mysql_update_time_eep, mysql_update_time_var);
            EEPROM.commit();
            mqtt_response_backup(SUCCESS);
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response_backup(ERROR);
        }
    }

    // Reset System
    else if (_sensor == RESET_CMD)
    {
        if (_value.toInt() == 1)
        {
            Serial.println("\t\t- reset command -");
            mcu.buzzer_beep(4);
            mqtt_response_backup(SUCCESS);
            ESP.restart();
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response_backup(ERROR);
        }
    }

    // Reset Energy
    else if (_sensor == ENGERGY_RESET_CMD)
    {
        if (_value.toInt() == 1)
        {
            mcu.buzzer_beep(1);
            Serial.println("\t\t- energy reset command -");
            mcu.buzzer_beep(1);
            mcu.PZEM_016_ResetEnergy(pzem016_var[0].address);
            mqtt_response_backup(SUCCESS);
        }
        else
        {
            mcu.buzzer_beep(3);
            Serial.printf("\t\t\tError value (%d) !!\r\n", _value.toInt());
            mqtt_response_backup(ERROR);
        }
    }

    // Config command
    else if (_sensor == CONFIG_CMD)
    {
        if (_value == "?")
        {
            Serial.println("\t\t- config command -");
            mcu.buzzer_beep(1);
            _JsonStr = jsonConvert_Sensor(CONFIG_SELECT, NULL);
            _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
            // Serial.printf("Info: Json size = %d\r\n", _JsonStr.length());
            // Serial.printf("%s\r\n\r\n", Json_Msg);
            client_backup.publish(topic_public, Json_Msg); // don't update mqtt when set time = 0
            vTaskDelay(100);
        }
    }

    else
    {
        Serial.println("\t\t- !! Dont't match any command !! -");
        mqtt_response_backup(ERROR);
        mcu.buzzer_beep(3);
    }
}
/***********************************************************************
 * FUNCTION:    reconnect
 * DESCRIPTION: reconnect function for reconnect mqtt server
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void reconnect()
{
    int _cnt_server = 0;
    wifistatus = WiFi.status();
    if (wifistatus != WL_CONNECTED)
        return;

    //*** Loop until we're reconnected ***
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        //*** Attempt to connect ***
        if (client.connect(mqtt_topic_char, mqtt_user_char, mqtt_pass_char))
        {
            client.subscribe(topic_sub);
            Serial.println("connected");
            Serial.println("-------------------------------");
            Serial.printf("Info: Sending data to MQTT server ...");
        }
        else
        {
            if (_cnt_server++ > 5)
                break;

            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

/***********************************************************************
 * FUNCTION:    reconnect_backup
 * DESCRIPTION: reconnect function for reconnect backup mqtt server
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void reconnect_backup()
{
    int _cnt_server = 0;
    wifistatus = WiFi.status();
    if (wifistatus != WL_CONNECTED)
        return;

    //*** Loop until we're reconnected ***
    while (!client_backup.connected())
    {
        Serial.print("Attempting Backup MQTT connection...");
        //*** Attempt to connect ***
        if (client_backup.connect(mqtt_topic_char, mqtt_user_char, mqtt_pass_char))
        {
            client_backup.subscribe(topic_sub);
            Serial.println("connected");
            Serial.println("-------------------------------");
            Serial.printf("Info: Sending data to Backup MQTT server ...");
        }
        else
        {
            if (_cnt_server++ > 5)
                break;

            Serial.print("failed, rc=");
            Serial.print(client_backup.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

/***********************************************************************
 * FUNCTION:    saveConfigCallback
 * DESCRIPTION: saveConfigCallback for check save config after AP mode
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void saveConfigCallback()
{
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

/***********************************************************************
 * FUNCTION:    wifi_config
 * DESCRIPTION: Config wifi, AP config, Mqtt setting
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void wifi_config()
{

    //*** Connect WiFi network ***
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server_char, 40);
    WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user_char, 20);
    WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass_char, 20);
    WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", mqtt_topic_char, 20);

    wm.setSaveConfigCallback(saveConfigCallback);
    wm.addParameter(&custom_mqtt_server);
    wm.addParameter(&custom_mqtt_user);
    wm.addParameter(&custom_mqtt_pass);
    wm.addParameter(&custom_mqtt_topic);
    // wm.setConnectTimeout(30);
    wm.setConfigPortalTimeout(AP_TIMEOUT);

    //*** Connect WiFi network ***
    if (wm.autoConnect(unit, "password"))
    {
        Serial.printf("Info: Success to connected\r\n");
    }
    else
    {
        Serial.println("Error: Failed to connect wifi");
        Serial.println("\t\tSystem is resetting");
        // mcu.buzzer_beep(3);
        ESP.restart();
    }

    if (shouldSaveConfig)
    {

        if (!EEPROM.begin(EEPROM_SIZE)) // fail eeprom
        {
            Serial.println("Error: failed to initialise EEPROM");
        }
        else
        {

            //*** write to char variable นำค่าที่ได้จาก web ap mode มาเก็บไว้ในตัวแปรต่างๆ ***
            strcpy(mqtt_server_char, custom_mqtt_server.getValue());
            strcpy(mqtt_user_char, custom_mqtt_user.getValue());
            strcpy(mqtt_pass_char, custom_mqtt_pass.getValue());
            strcpy(mqtt_topic_char, custom_mqtt_topic.getValue());

            Serial.printf("Info: MQTT Server* => %s\r\n", mqtt_server_char);
            Serial.printf("Info: MQTT User* => %s\r\n", mqtt_user_char);
            Serial.printf("Info: MQTT Pass* => %s\r\n", mqtt_pass_char);
            Serial.printf("Info: MQTT Topic* => %s\r\n", mqtt_topic_char);
            String _str_tmp = String(mqtt_topic_char) + "/" + String(unit);
            Serial.printf("Info: Topic public => %s\r\n", topic_public);

            //*** write variable to eeprom เขียนค่าตัวแปรที่ได้รับเก็บไว้ที่ eeprom ***
            EEPROM.writeString(mqtt_server_eep, mqtt_server_char);
            EEPROM.writeString(mqtt_user_eep, mqtt_user_char);
            EEPROM.writeString(mqtt_pass_eep, mqtt_pass_char);
            EEPROM.writeString(mqtt_topic_eep, mqtt_topic_char);
            EEPROM.commit();
        }
    }
    else
    {
        if (!EEPROM.begin(EEPROM_SIZE)) // fail eeprom
        {
            Serial.println("Error: failed to initialise EEPROM");
        }
        else
        {
            //*** Get Mqtt server ***
            int _strlen = EEPROM.readString(mqtt_server_eep).length(); // เก็บขนาดความยาวของข้อมูล
            // Serial.printf("Debug: _strlen = %d\r\n",_strlen);
            EEPROM.readString(mqtt_server_eep).toCharArray(mqtt_server_char, _strlen + 1); // นำค่าที่ได้จาก eeprom ไปใส่ในตัวแปล char array

            //*** Get Mqtt user ***
            _strlen = EEPROM.readString(mqtt_user_eep).length(); // เก็บขนาดความยาวของข้อมูล
            // Serial.printf("Debug: _strlen = %d\r\n",_strlen);
            EEPROM.readString(mqtt_user_eep).toCharArray(mqtt_user_char, _strlen + 1); // นำค่าที่ได้จาก eeprom ไปใส่ในตัวแปล char array

            //*** Get Mqtt password ***
            _strlen = EEPROM.readString(mqtt_pass_eep).length(); // เก็บขนาดความยาวของข้อมูล
            // Serial.printf("Debug: _strlen = %d\r\n",_strlen);
            EEPROM.readString(mqtt_pass_eep).toCharArray(mqtt_pass_char, _strlen + 1); // นำค่าที่ได้จาก eeprom ไปใส่ในตัวแปล char array

            //*** Get Mqtt Topic ***
            _strlen = EEPROM.readString(mqtt_topic_eep).length(); // เก็บขนาดความยาวของข้อมูล
            // Serial.printf("Debug: _strlen = %d\r\n",_strlen);
            EEPROM.readString(mqtt_topic_eep).toCharArray(mqtt_topic_char, _strlen + 1); // นำค่าที่ได้จาก eeprom ไปใส่ในตัวแปล char array

            // Serial.printf("Info: MQTT Server => %s\r\n", mqtt_server_char);
            // Serial.printf("Info: MQTT User => %s\r\n", mqtt_user_char);
            // Serial.printf("Info: MQTT Pass => %s\r\n", mqtt_pass_char);
            // Serial.printf("Info: MQTT Topic => %s\r\n", mqtt_topic_char);
        }
    }
}

/***********************************************************************
 * FUNCTION:    sensor_variable_init
 * DESCRIPTION: Initial, Assign Modbus address, sensor name, sensor id
 *              for each structure variable
 * PARAMETERS:  nothing
 * RETURNED:    true/false
 ***********************************************************************/
bool sensor_variable_init(void)
{
    //------------- AirFLow sensor variable initial --------------
    Serial.println("Info: Airflow variable initial");

    // /* Airflow[0] */
    // strncpy(airflow_var[0].unit, unit, sizeof(unit));
    // airflow_var[0].sensor_id = 0;
    // airflow_var[0].address = 6; // change Modbus Address here *
    // airflow_var[0].sensor = "airflow";
    // Serial.printf("\t\t- airflow[0].unit : %s\r\n", airflow_var[0].unit);
    // Serial.printf("\t\t- airflow[0].sensor : %s\r\n", airflow_var[0].sensor);
    // Serial.printf("\t\t- airflow[0].sensor_id : %d\r\n", airflow_var[0].sensor_id);
    // Serial.printf("\t\t- airflow[0].address : %d\r\n", airflow_var[0].address);

    // /* Airflow[1] */
    // strncpy(airflow_var[1].unit, unit, sizeof(unit));
    // airflow_var[1].sensor_id = 1;
    // airflow_var[1].address = 7; // change Modbus Address here *
    // airflow_var[1].sensor = "airflow";
    // Serial.printf("\t\t- airflow[1].unit : %s\r\n", airflow_var[1].unit);
    // Serial.printf("\t\t- airflow[1].sensor : %s\r\n", airflow_var[1].sensor);
    // Serial.printf("\t\t- airflow[1].sensor_id : %d\r\n", airflow_var[1].sensor_id);
    // Serial.printf("\t\t- airflow[1].address : %d\r\n\r\n", airflow_var[1].address);

    //------------- Ambient Indoor sensor variable initial --------------
    Serial.println("Info: Ambient Indoor variable initial");

    /* AmbientIn_var[0] */
    strncpy(ambientIn_var[0].unit, unit, sizeof(unit));
    ambientIn_var[0].sensor_id = 0;
    ambientIn_var[0].address = 2; // change Modbus Address here *
    ambientIn_var[0].sensor = "ambientIn";
    Serial.printf("\t\t- ambientIn[0].unit : %s\r\n", ambientIn_var[0].unit);
    Serial.printf("\t\t- ambientIn[0].sensor : %s\r\n", ambientIn_var[0].sensor);
    Serial.printf("\t\t- ambientIn[0].sensor_id : %d\r\n", ambientIn_var[0].sensor_id);
    Serial.printf("\t\t- ambientIn[0].address : %d\r\n", ambientIn_var[0].address);

    // /* AmbientIn_var[1] */
    // strncpy(ambientIn_var[1].unit, unit, sizeof(unit));
    // ambientIn_var[1].sensor_id = 1;
    // ambientIn_var[1].address = 3; // change Modbus Address here *
    // ambientIn_var[1].sensor = "ambientIn";
    // Serial.printf("\t\t- ambientIn[1].unit : %s\r\n", ambientIn_var[1].unit);
    // Serial.printf("\t\t- ambientIn[1].sensor : %s\r\n", ambientIn_var[1].sensor);
    // Serial.printf("\t\t- ambientIn[1].sensor_id : %d\r\n", ambientIn_var[1].sensor_id);
    // Serial.printf("\t\t- ambientIn[1].address : %d\r\n", ambientIn_var[1].address);

    // /* AmbientIn_var[2] */
    // strncpy(ambientIn_var[2].unit, unit, sizeof(unit));
    // ambientIn_var[2].sensor_id = 2;
    // ambientIn_var[2].address = 4; // change Modbus Address here *
    // ambientIn_var[2].sensor = "ambientIn";
    // Serial.printf("\t\t- ambientIn[2].unit : %s\r\n", ambientIn_var[2].unit);
    // Serial.printf("\t\t- ambientIn[2].sensor : %s\r\n", ambientIn_var[2].sensor);
    // Serial.printf("\t\t- ambientIn[2].sensor_id : %d\r\n", ambientIn_var[2].sensor_id);
    // Serial.printf("\t\t- ambientIn[2].address : %d\r\n", ambientIn_var[2].address);

    // /* AmbientIn_var[3] */
    // strncpy(ambientIn_var[3].unit, unit, sizeof(unit));
    // ambientIn_var[3].sensor_id = 3;
    // ambientIn_var[3].address = 5; // change Modbus Address here *
    // ambientIn_var[3].sensor = "ambientIn";
    // Serial.printf("\t\t- ambientIn[3].unit : %s\r\n", ambientIn_var[3].unit);
    // Serial.printf("\t\t- ambientIn[3].sensor : %s\r\n", ambientIn_var[3].sensor);
    // Serial.printf("\t\t- ambientIn[3].sensor_id : %d\r\n", ambientIn_var[3].sensor_id);
    // Serial.printf("\t\t- ambientIn[3].address : %d\r\n\r\n", ambientIn_var[3].address);

    //------------- Ambient Outdoor sensor variable initial --------------
    Serial.println("Info: Ambient Outdoor variable initial");

    /* AmbientOut_var[0] */
    strncpy(ambientOut_var[0].unit, unit, sizeof(unit));
    ambientOut_var[0].sensor_id = 0;
    ambientOut_var[0].address = 3; // change Modbus Address here *
    ambientOut_var[0].sensor = "ambientOut";
    Serial.printf("\t\t- ambientOut[0].unit : %s\r\n", ambientOut_var[0].unit);
    Serial.printf("\t\t- ambientOut[0].sensor : %s\r\n", ambientOut_var[0].sensor);
    Serial.printf("\t\t- ambientOut[0].sensor_id : %d\r\n", ambientOut_var[0].sensor_id);
    Serial.printf("\t\t- ambientOut[0].address : %d\r\n\r\n", ambientOut_var[0].address);

    //------------- PZEM-016 sensor variable initial --------------
    Serial.println("Info: PZEM-016 sensor variable initial");

    /* PZEM-016_var[0] */
    strncpy(pzem016_var[0].unit, unit, sizeof(unit));
    pzem016_var[0].sensor_id = 0;
    pzem016_var[0].address = 1; // change Modbus Address here *
    pzem016_var[0].sensor = "pzem016";
    Serial.printf("\t\t- pzem016_var[0].unit : %s\r\n", pzem016_var[0].unit);
    Serial.printf("\t\t- pzem016_var[0].sensor : %s\r\n", pzem016_var[0].sensor);
    Serial.printf("\t\t- pzem016_var[0].sensor_id : %d\r\n", pzem016_var[0].sensor_id);
    Serial.printf("\t\t- pzem016_var[0].address : %d\r\n\r\n", pzem016_var[0].address);

    //------------- Relay Solid State variable initial --------------
    Serial.println("Info: Relay Solid State variable initial");

    /* Solid_var[0] */
    strncpy(solid_var[0].unit, unit, sizeof(unit));
    solid_var[0].sensor_id = 0;
    solid_var[0].sensor = "solid";
    Serial.printf("\t\t- solid_var[0].unit : %s\r\n", solid_var[0].unit);
    Serial.printf("\t\t- solid_var[0].sensor : %s\r\n", solid_var[0].sensor);
    Serial.printf("\t\t- solid_var[0].sensor_id : %d\r\n\r\n", solid_var[0].sensor_id);

    return true;
}

/***********************************************************************
 * FUNCTION:    ReadSensor_Task
 * DESCRIPTION: Multitasking for AmbientSensor
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void ReadSensor_Task(void *p)
{
    while (1)
    {
        read_ambientIn_sensor();
        read_ambientOut_sensor();
        // read_airflow_sensor();
        read_pzem016_sensor();
        vTaskDelay(2000);
    }
}

/***********************************************************************
 * FUNCTION:    read_ambientIn_sensor
 * DESCRIPTION: Read Ambient sensor
 * PARAMETERS:  nothing
 * RETURNED:    true/false
 ***********************************************************************/
bool read_ambientIn_sensor()
{
    ambientIn_var[0].humi = mcu.XY_MD02_humidity(ambientIn_var[0].address);
    ambientIn_var[0].temp = mcu.XY_MD02_tempeature(ambientIn_var[0].address);
    if ((ambientIn_var[0].humi == -1) || (ambientIn_var[0].temp == -1))
    {
        ambientIn_var[0].active = 0;
    }
    else
    {
        ambientIn_var[0].active = 1;
    }
    return true;

    // ambientIn_var[0].humi = random(0, 1000) * 0.1;
    // ambientIn_var[0].temp = random(100, 500) * 0.1;
    // ambientIn_var[0].active = true;

    // ambientIn_var[1].humi = random(0, 1000) * 0.1;
    // ambientIn_var[1].temp = random(100, 500) * 0.1;
    // ambientIn_var[1].active = true;

    // ambientIn_var[2].humi = random(0, 1000) * 0.1;
    // ambientIn_var[2].temp = random(100, 500) * 0.1;
    // ambientIn_var[2].active = true;

    // ambientIn_var[3].humi = random(0, 1000) * 0.1;
    // ambientIn_var[3].temp = random(100, 500) * 0.1;
    // ambientIn_var[3].active = true;

    return true;
}

/***********************************************************************
 * FUNCTION:    read_ambientOut_sensor
 * DESCRIPTION: Read Ambient sensor
 * PARAMETERS:  nothing
 * RETURNED:    true/false
 ***********************************************************************/
bool read_ambientOut_sensor()
{
    ambientOut_var[0].humi = mcu.XY_MD02_humidity(ambientOut_var[0].address);
    ambientOut_var[0].temp = mcu.XY_MD02_tempeature(ambientOut_var[0].address);
    if ((ambientOut_var[0].humi == -1) || (ambientOut_var[0].temp == -1))
    {
        ambientOut_var[0].active = 0;
    }
    else
    {
        ambientOut_var[0].active = 1;
    }
    return true;

    // ambientOut_var[0].humi = random(0, 1000) * 0.1;
    // ambientOut_var[0].temp = random(100, 500) * 0.1;
    // ambientOut_var[0].active = true;
    return true;
}

/***********************************************************************
 * FUNCTION:    read_airflow_sensor
 * DESCRIPTION: Read Airflow sensor
 * PARAMETERS:  nothing
 * RETURNED:    true/false
 ***********************************************************************/
bool read_airflow_sensor()
{
    if (mcu.tiny32_WIND_RSFSN01_SPEED(airflow_var[0].address) == -1)
    {
        airflow_var[0].active = 0;
    }
    else
    {
        airflow_var[0].speed = mcu.tiny32_WIND_RSFSN01_SPEED(airflow_var[0].address);
        airflow_var[0].active = 1;
    }

    if (mcu.tiny32_WIND_RSFSN01_SPEED(airflow_var[1].address) == -1)
    {

        airflow_var[1].active = 0;
    }
    else
    {
        airflow_var[1].speed = mcu.tiny32_WIND_RSFSN01_SPEED(airflow_var[1].address);
        airflow_var[1].active = 1;
    }
    return true;

    // airflow_var[0].speed = random(0, 20);
    // airflow_var[0].active = true;
    // airflow_var[1].speed = random(0, 20);
    // airflow_var[1].active = true;
    // return true;
}

/***********************************************************************
 * FUNCTION:    read_pzem016_sensor
 * DESCRIPTION: Read PZEM-016 power meter sensor
 * PARAMETERS:  nothing
 * RETURNED:    true/false
 ***********************************************************************/
bool read_pzem016_sensor()
{
    pzem016_var[0].volt = mcu.PZEM_016_Volt(pzem016_var[0].address);
    if (pzem016_var[0].volt == -1)
    {
        pzem016_var[0].active = 0;
        return false;
    }
    else
    {
        pzem016_var[0].volt = mcu.PZEM_016_Volt(pzem016_var[0].address);
        pzem016_var[0].amp = mcu.PZEM_016_Amp(pzem016_var[0].address);
        pzem016_var[0].pf = mcu.PZEM_016_PF(pzem016_var[0].address);
        pzem016_var[0].power = mcu.PZEM_016_Power(pzem016_var[0].address);
        pzem016_var[0].energy = mcu.PZEM_016_Energy(pzem016_var[0].address);
        pzem016_var[0].freq = mcu.PZEM_016_Freq(pzem016_var[0].address);
        pzem016_var[0].active = 1;
        return true;
    }

    // pzem016_var[0].volt = random(2100, 2500) * 0.1;
    // pzem016_var[0].amp = random(0, 20) * 0.1;
    // pzem016_var[0].pf = random(7, 9) * 0.1;
    // pzem016_var[0].power = pzem016_var[0].volt * pzem016_var[0].amp * pzem016_var[0].pf;
    // pzem016_var[0].energy = pzem016_var[0].energy + pzem016_var[0].power;
    // pzem016_var[0].freq = random(480, 510) * 0.1;
    // pzem016_var[0].active = true;
    return true;
}

/***********************************************************************
 * FUNCTION:    jsonConvert_Sensor
 * DESCRIPTION: Convert value of sensor to json
 * PARAMETERS:  nothing
 * RETURNED:    Json String
 ***********************************************************************/
String jsonConvert_Sensor(uint8_t sensor_select, uint8_t sensor_id)
{
    String _JsonStr = "";

    if (sensor_select == AMBIENT_IN_SELECT)
    {
        _JsonStr += "{";
        _JsonStr += "\"unit\":\"" + String(ambientIn_var[sensor_id].unit) + "\",";
        _JsonStr += "\"topic\":\"" + String(mqtt_topic_char) + "\",";
        _JsonStr += "\"cmd\":\"mqtt\",";
        _JsonStr += "\"sensor\":\"" + ambientIn_var[sensor_id].sensor + "\",";
        _JsonStr += "\"sensor_id\":" + String(ambientIn_var[sensor_id].sensor_id) + ",";
        _JsonStr += "\"addr\":" + String(ambientIn_var[sensor_id].address) + ",";
        _JsonStr += "\"temp\":" + String(ambientIn_var[sensor_id].temp) + ",";
        _JsonStr += "\"humi\":" + String(ambientIn_var[sensor_id].humi) + ",";
        _JsonStr += "\"solid\":" + String(solid_status_var) + ",";
        _JsonStr += "\"rssi\":" + String(WiFi.RSSI()) + ",";
        _JsonStr += "\"active\":" + String(ambientIn_var[sensor_id].active);
        _JsonStr += "}";
        return _JsonStr;
    }
    else if (sensor_select == AMBIENT_OUT_SELECT)
    {
        _JsonStr += "{";
        _JsonStr += "\"unit\":\"" + String(ambientOut_var[sensor_id].unit) + "\",";
        _JsonStr += "\"topic\":\"" + String(mqtt_topic_char) + "\",";
        _JsonStr += "\"cmd\":\"mqtt\",";
        _JsonStr += "\"sensor\":\"" + ambientOut_var[sensor_id].sensor + "\",";
        _JsonStr += "\"sensor_id\":" + String(ambientOut_var[sensor_id].sensor_id) + ",";
        _JsonStr += "\"addr\":" + String(ambientOut_var[sensor_id].address) + ",";
        _JsonStr += "\"temp\":" + String(ambientOut_var[sensor_id].temp) + ",";
        _JsonStr += "\"humi\":" + String(ambientOut_var[sensor_id].humi) + ",";
        _JsonStr += "\"active\":" + String(ambientOut_var[sensor_id].active);
        _JsonStr += "}";
        return _JsonStr;
    }
    else if (sensor_select == AIRFLOW_SELECT)
    {
        _JsonStr += "{";
        _JsonStr += "\"unit\":\"" + String(airflow_var[sensor_id].unit) + "\",";
        _JsonStr += "\"topic\":\"" + String(mqtt_topic_char) + "\",";
        _JsonStr += "\"cmd\":\"mqtt\",";
        _JsonStr += "\"sensor\":\"" + airflow_var[sensor_id].sensor + "\",";
        _JsonStr += "\"sensor_id\":" + String(airflow_var[sensor_id].sensor_id) + ",";
        _JsonStr += "\"addr\":" + String(airflow_var[sensor_id].address) + ",";
        _JsonStr += "\"speed\":" + String(airflow_var[sensor_id].speed) + ",";
        _JsonStr += "\"rssi\":" + String(WiFi.RSSI()) + ",";
        _JsonStr += "\"solid\":" + String(solid_status_var) + ",";
        _JsonStr += "\"active\":" + String(airflow_var[sensor_id].active);
        _JsonStr += "}";
        return _JsonStr;
    }
    else if (sensor_select == PZEM016_SELECT)
    {
        _JsonStr += "{";
        _JsonStr += "\"unit\":\"" + String(pzem016_var[sensor_id].unit) + "\",";
        _JsonStr += "\"topic\":\"" + String(mqtt_topic_char) + "\",";
        _JsonStr += "\"cmd\":\"mqtt\",";
        _JsonStr += "\"sensor\":\"" + pzem016_var[sensor_id].sensor + "\",";
        _JsonStr += "\"sensor_id\":" + String(pzem016_var[sensor_id].sensor_id) + ",";
        _JsonStr += "\"addr\":" + String(pzem016_var[sensor_id].address) + ",";
        _JsonStr += "\"volt\":" + String(pzem016_var[sensor_id].volt) + ",";
        _JsonStr += "\"amp\":" + String(pzem016_var[sensor_id].amp) + ",";
        _JsonStr += "\"power\":" + String(pzem016_var[sensor_id].power) + ",";
        _JsonStr += "\"energy\":" + String(pzem016_var[sensor_id].energy) + ",";
        _JsonStr += "\"pf\":" + String(pzem016_var[sensor_id].pf) + ",";
        _JsonStr += "\"freq\":" + String(pzem016_var[sensor_id].freq) + ",";
        _JsonStr += "\"active\":" + String(pzem016_var[sensor_id].active);
        _JsonStr += "}";
        return _JsonStr;
    }
    else if (sensor_select == CONFIG_SELECT)
    {
        _JsonStr += "{";
        _JsonStr += "\"unit\":\"" + String(unit) + "\",";
        _JsonStr += "\"topic\":\"" + String(mqtt_topic_char) + "\",";
        _JsonStr += "\"cmd\":\"config\",";
        _JsonStr += "\"fw\":\"" + version + "\",";
        _JsonStr += "\"server\":\"" + String(mqtt_server_char) + "\",";
        _JsonStr += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
        _JsonStr += "\"ssid\":\"" + WiFi.SSID() + "\",";
        _JsonStr += "\"rssi\":" + String(WiFi.RSSI()) + ",";
        _JsonStr += "\"spreadsheet_time\":" + String(spreadsheet_update_time_var) + ",";
        _JsonStr += "\"mysql_time\":" + String(mysql_update_time_var) + ",";
        _JsonStr += "\"mqtt_time\":" + String(mqtt_update_time_var);
        _JsonStr += "}";
        return _JsonStr;
    }

    else if (sensor_select == STARTUP_SQL_SELECT)
    {
        _JsonStr += "{";
        _JsonStr += "\"unit\":\"" + String(unit) + "\",";
        _JsonStr += "\"topic\":\"" + String(mqtt_topic_char) + "\",";
        _JsonStr += "\"cmd\":\"startup\",";
        _JsonStr += "\"fw\":\"" + version + "\",";
        _JsonStr += "\"server\":\"" + String(mqtt_server_char) + "\",";
        _JsonStr += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
        _JsonStr += "\"ssid\":\"" + WiFi.SSID() + "\",";
        _JsonStr += "\"rssi\":" + String(WiFi.RSSI()) + ",";
        _JsonStr += "\"spreadsheet_time\":" + String(spreadsheet_update_time_var) + ",";
        _JsonStr += "\"mysql_time\":" + String(mysql_update_time_var) + ",";
        _JsonStr += "\"mqtt_time\":" + String(mqtt_update_time_var);
        _JsonStr += "}";
        return _JsonStr;
    }
}

/***********************************************************************
 * FUNCTION:    eeprom_init
 * DESCRIPTION: Read EEPROM then record to variable
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
void eeprom_init()
{
    if (!EEPROM.begin(EEPROM_SIZE)) // fail eeprom
    {
        Serial.println("\r\rError: failed to initialise EEPROM");
    }
    else
    {

        solid_status_var = EEPROM.readByte(solid_status_eep);
        if (solid_status_var != 0 && solid_status_var != 1)
        {
            solid_status_var = 0;
            EEPROM.writeByte(solid_status_eep, solid_status_var);
            EEPROM.commit();
        }

        spreadsheet_update_time_var = EEPROM.readByte(spreadsheet_update_time_eep);
        if (spreadsheet_update_time_var == 0xff)
        {
            spreadsheet_update_time_var = 5; // set default for update google spreadsheet 5 minute
            EEPROM.writeByte(spreadsheet_update_time_eep, spreadsheet_update_time_var);
            EEPROM.commit();
        }

        mqtt_update_time_var = EEPROM.readByte(mqtt_update_time_eep);
        if (mqtt_update_time_var == 0xff)
        {
            mqtt_update_time_var = 1; // set default update for nodered 1 second
            EEPROM.writeByte(mqtt_update_time_eep, mqtt_update_time_var);
            EEPROM.commit();
        }

        mysql_update_time_var = EEPROM.readUShort(mysql_update_time_eep);
        if (mysql_update_time_var < 0 && mysql_update_time_var > 65565) // second
        {
            mysql_update_time_var = 30; // set default update to mysql 30 second
            EEPROM.writeUShort(mysql_update_time_eep, mysql_update_time_var);
            EEPROM.commit();
        }

        Serial.printf("\t\t- solid_status[EEPROM] => %d\r\n", solid_status_var);
        Serial.printf("\t\t- spreadsheet_update_time[EEPROM] => %d min\r\n", spreadsheet_update_time_var);
        Serial.printf("\t\t- mqtt_update_time[EEPROM] => %d sec\r\n", mqtt_update_time_var);
        Serial.printf("\t\t- mysql_update_time[EEMPROM] => %d sec\r\n\r\n", mysql_update_time_var);
    }
}

/***********************************************************************
 * FUNCTION:    spreaddheet_sendData
 * DESCRIPTION: send data to google sheet
 * PARAMETERS:  String params
 * RETURNED:    nothing
 ***********************************************************************/
void spreaddheet_sendData(String params)
{
    HTTPClient http;
    String url = "https://script.google.com/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + params;
    Serial.print(url);
    Serial.print("Making a request");
    http.begin(url, root_ca); // Specify the URL and certificate
    int httpCode = http.GET();
    http.end();
    Serial.println(": done " + httpCode);
}

/***********************************************************************
 * FUNCTION:    mqtt_response
 * DESCRIPTION: responding to user command when tiny32 get command to
 *              main mqtt server
 * PARAMETERS:  nothing
 * RETURNED:    true=success, false=unsuccess
 ***********************************************************************/
boolean mqtt_response(boolean status)
{
    String _JsonStr = "";
    // Serial.printf("mqtt_response[status] => %d\r\n",status);

    if (status == 1)
    {
        _JsonStr = String(unit) + "=> success";
        _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
        client.publish(topic_public, Json_Msg);
        return true;
    }
    else if (status == 0)
    {
        _JsonStr = String(unit) + "=> error";
        _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
        client.publish(topic_public, Json_Msg);
        return true;
    }
    return true;
}

/***********************************************************************
 * FUNCTION:    mqtt_response_backup
 * DESCRIPTION: responding to user command when tiny32 get command to
 *              backup mqtt server
 * PARAMETERS:  nothing
 * RETURNED:    true=success, false=unsuccess
 ***********************************************************************/
boolean mqtt_response_backup(boolean status)
{
    String _JsonStr = "";
    // Serial.printf("mqtt_response[status] => %d\r\n",status);

    if (status == 1)
    {
        _JsonStr = String(unit) + "=> success";
        _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
        client_backup.publish(topic_public, Json_Msg);
        return true;
    }
    else if (status == 0)
    {
        _JsonStr = String(unit) + "=> error";
        _JsonStr.toCharArray(Json_Msg, _JsonStr.length() + 1);
        client_backup.publish(topic_public, Json_Msg);
        return true;
    }
    return true;
}