/**
 * @file      MqttsBuiltlnAuth.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-11-28
 * @note
 * * * Example is suitable for A7670X/A7608X/SIM7672 series
 * * Connect MQTT Broker as https://test.mosquitto.org/
 * * Example uses a forked TinyGSM <https://github.com/lewisxhe/TinyGSM>, which will not compile successfully using the mainline TinyGSM.
 */
#define TINY_GSM_RX_BUFFER          1024 // Set RX buffer to 1Kb

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#define DEBUG

#include "utilities.h"
#include <TinyGsmClient.h>

#include <vector>
#include <algorithm>
#include <numeric>

#undef B1 // https://github.com/fmtlib/fmt/issues/3559
#include "fmt/format.h"

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// It depends on the operator whether to set up an APN. If some operators do not set up an APN,
// they will be rejected when registering for the network. You need to ask the local operator for the specific APN.
// APNs from other operators are welcome to submit PRs for filling.
// #define NETWORK_APN     "CHN-CT"             //CHN-CT: China Telecom
#define NETWORK_APN     "internet"

// MQTT details
const char *broker = "snigel.xinit.se";
const uint16_t broker_port = 12000;
const char *broker_username = "nimbus";
const char *broker_password = "26";
const char *client_id = "A7670";

const char *subscribe_topic = "GsmMqttTest/subscribe";

const char *config_topic = "homeassistant/device/{0}/config";
const char *state_topic = "homeassistant/sensor/{0}/state";

const char *config_payload = "{{ \"dev\": {{ \"ids\": \"{0}\", \"name\": \"Nimbus 26 Elin A7670\", \"mf\": \"Svante Gradén\", \"sw\": \"1.0\", \"sn\": \"{0}\" }}, \"o\": {{ \"name\":\"Lilygo T-Call\" }}, \"cmps\": {{ \"start_v\": {{ \"p\": \"sensor\", \"name\": \"Start battery voltage\", \"device_class\":\"voltage\", \"unit_of_measurement\":\"V\", \"value_template\":\"{{{{ value_json.s }}}}\", \"unique_id\":\"battery_voltage_{0}_start\", \"force_update\": \"true\", \"suggested_display_precision\": 2 }}, \"start_r\": {{ \"p\": \"sensor\", \"name\": \"Start battery raw\", \"unit_of_measurement\":\"units\", \"value_template\":\"{{{{ value_json.sr }}}}\", \"unique_id\":\"battery_raw_{0}_start\", \"force_update\": \"true\", \"suggested_display_precision\": 0 }}, \"consumable_v\": {{ \"p\": \"sensor\", \"name\": \"Consumable battery voltage\", \"device_class\":\"voltage\", \"unit_of_measurement\":\"V\", \"value_template\":\"{{{{ value_json.c }}}}\", \"unique_id\":\"battery_voltage_{0}_consumable\", \"force_update\": \"true\", \"suggested_display_precision\": 2 }}, \"consumable_r\": {{ \"p\": \"sensor\", \"name\": \"Consumable battery raw\", \"unit_of_measurement\":\"units\", \"value_template\":\"{{{{ value_json.cr }}}}\", \"unique_id\":\"battery_raw_{0}_consumable\", \"force_update\": \"true\", \"suggested_display_precision\": 0 }} }}, \"state_topic\":\"homeassistant/sensor/{0}/state\" }}";
//const char *state_payload = "{{ \"s\": {0}, \"c\": {1}, \"sr\": {2}, \"cr\": {3} }}";
const char *state_payload = "{{ \"s\": {0:.2f}, \"c\": {1:.2f}, \"sr\": {2}, \"cr\": {3} }}";

// Current connection index, range 0~1
const uint8_t mqtt_client_id = 0;
uint32_t check_connect_millis = 0;
String ueInfo = "";
String imei = "";

void mqtt_callback(const char *topic, const uint8_t *payload, uint32_t len)
{
    Serial.println();
    Serial.println("======mqtt_callback======");
    Serial.print("Topic:"); Serial.println(topic);
    Serial.println("Payload:");
    for (int i = 0; i < len; ++i) {
        Serial.print(payload[i], HEX); Serial.print(",");
    }
    Serial.println();
    Serial.println("=========================");
}

bool mqtt_connect()
{
    Serial.print("Connecting to ");
    Serial.print(broker);

    bool ret = modem.mqtt_connect(mqtt_client_id, broker, broker_port, client_id, broker_username, broker_password, 1200U);
    if (!ret) {
        Serial.println("Failed!"); return false;
    }
    Serial.println("successfully.");

    if (modem.mqtt_connected()) {
        Serial.println("MQTT has connected!");
    } else {
        return false;
    }
    // Set MQTT processing callback
    //modem.mqtt_set_callback(mqtt_callback);
    // Subscribe to topic
    //modem.mqtt_subscribe(mqtt_client_id, subscribe_topic);

    return true;
}

uint32_t getBatteryVoltage()
{
    std::vector<uint32_t> data;
    for (int i = 0; i < 64; ++i) {
        uint32_t val = analogReadRaw(GPIO_NUM_34);
        // Serial.printf("analogReadMilliVolts : %u mv \n", val * 2);
        data.push_back(val);
        delay(30);
    }
    std::sort(data.begin(), data.end());
    data.erase(data.begin());
    data.pop_back();
    int sum = std::accumulate(data.begin(), data.end(), 0);
    double average = static_cast<double>(sum) / data.size();
    return  average;
}

uint32_t getVoltageMedian(uint8_t pin)
{
    std::vector<uint32_t> data;
    for (int i = 0; i < 64; ++i) {
        uint32_t val = analogReadRaw(pin);
        // Serial.printf("analogReadMilliVolts : %u mv \n", val * 2);
        data.push_back(val);
        delay(30);
    }
    std::sort(data.begin(), data.end());
    return data[data.size()/2];
}

void setup()
{
    std::string config_topic_formatted;
    std::string config_payload_formatted;

    bool result;

    Serial.begin(115200); // Set console baud rate

    Serial.println("Start Sketch");

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

#ifdef BOARD_POWERON_PIN
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

    // Set modem reset pin, reset modem
#ifdef MODEM_RESET_PIN
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Check if the modem is online
    Serial.println("Start modem...");

    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.println(".");
        if (retry++ > 10) {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println();

    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY) {
        sim = modem.getSimStatus();
        switch (sim) {
        case SIM_READY:
            Serial.println("SIM card online");
            break;
        case SIM_LOCKED:
            Serial.println("The SIM card is locked. Please unlock the SIM card first.");
            // const char *SIMCARD_PIN_CODE = "123456";
            // modem.simUnlock(SIMCARD_PIN_CODE);
            break;
        default:
            break;
        }
        delay(1000);
    }

    //SIM7672G Can't set network mode
#ifndef TINY_GSM_MODEM_SIM7672
    if (!modem.setNetworkMode(MODEM_NETWORK_AUTO)) {
        Serial.println("Set network mode failed!");
    }
    String mode = modem.getNetworkModes();
    Serial.print("Current network mode : ");
    Serial.println(mode);
#endif

#ifdef NETWORK_APN
    Serial.printf("Set network apn : %s\n", NETWORK_APN);
    modem.sendAT(GF("+CGDCONT=1,\"IP\",\""), NETWORK_APN, "\"");
    if (modem.waitResponse() != 1) {
        Serial.println("Set network apn error !");
    }
#endif

    // Check network registration status and network signal status
    int16_t sq ;
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
        status = modem.getRegistrationStatus();
        switch (status) {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
            sq = modem.getSignalQuality();
            Serial.printf("[%lu] Signal Quality:%d\n", millis() / 1000, sq);
            delay(1000);
            break;
        case REG_DENIED:
            Serial.println("Network registration was rejected, please check if the APN is correct");
            return ;
        case REG_OK_HOME:
            Serial.println("Online registration successful");
            break;
        case REG_OK_ROAMING:
            Serial.println("Network registration successful, currently in roaming mode");
            break;
        default:
            Serial.printf("Registration Status:%d\n", status);
            delay(1000);
            break;
        }
    }
    Serial.println();


    Serial.printf("Registration Status:%d\n", status);
    delay(1000);

    if (modem.getSystemInformation(ueInfo)) {
        Serial.print("Inquiring UE system information:");
        Serial.println(ueInfo);
    }

    imei = modem.getIMEI();
    
    Serial.print("IMEI: ");
    Serial.println(imei.c_str());

    if (!modem.setNetworkActive()) {
        Serial.println("Enable network failed!");
    }

    delay(5000);

    String ipAddress = modem.getLocalIP();
    Serial.print("Network IP:"); Serial.println(ipAddress);

    analogSetPinAttenuation(GPIO_NUM_34, ADC_0db);

    // Initialize MQTT, use SSL, skip authentication server
    modem.mqtt_begin(false);

    if (!mqtt_connect()) {
        return ;
    }
    else {
        config_topic_formatted = fmt::format(config_topic, imei.c_str());
        config_payload_formatted = fmt::format(config_payload, imei.c_str());

        Serial.print(config_topic_formatted.c_str());
        Serial.print(config_payload_formatted.c_str());

        result = modem.mqtt_publish(0, config_topic_formatted.c_str(), config_payload_formatted.c_str());

        if(result) {
            Serial.print("Config publish successfully.");
        }
        else {
            Serial.print("Config publish failed.");
        }
    }
}

void loop()
{
    int consumableRaw;
    std::string state_topic_formatted;
    std::string payload_formatted;
    bool result;

    //consumableRaw = analogReadRaw(GPIO_NUM_34);
    //consumableRaw = getBatteryVoltage();
    consumableRaw = getVoltageMedian(GPIO_NUM_34);
    Serial.println(consumableRaw);

    // Debug AT
    while (SerialAT.available()) {
        Serial.write(SerialAT.read());
    }
    while (Serial.available()) {
        SerialAT.write(Serial.read());
    }

    if (!modem.mqtt_connected()) {
        mqtt_connect();
    } else {
        state_topic_formatted = fmt::format(state_topic, imei.c_str());
        payload_formatted = fmt::format(state_payload, (float)consumableRaw/224, (float)consumableRaw/224, consumableRaw, consumableRaw);

        result = modem.mqtt_publish(0, state_topic_formatted.c_str(), payload_formatted.c_str());

        if(result) {
            Serial.print("State publish successfully.");
        }
        else {
            Serial.print("State publish failed.");
        }
    }

    modem.mqtt_handle();

    delay(60000);
}

#ifndef TINY_GSM_FORK_LIBRARY
#error "No correct definition detected, Please copy all the [lib directories](https://github.com/Xinyuan-LilyGO/LilyGO-T-A76XX/tree/main/lib) to the arduino libraries directory , See README"
#endif
