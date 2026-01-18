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

void setup()
{
    Serial.begin(115200); // Set console baud rate

    Serial.println("Start Sketch");

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    Serial.println("Board started.");

    pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop()
{
  delay(1000);
// you can set the delay time by adjusting the parameter of delay();
  digitalWrite(BOARD_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_LED_PIN, LOW);
}
