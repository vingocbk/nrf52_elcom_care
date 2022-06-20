#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <SPI.h>
#include <filters.h>

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#define MODE_READ_ECG_GRAPH                 1;
#define MODE_READ_SPO2_GRAPH                2;
#define MODE_READ_HEART_RATE                3;
#define MODE_READ_SPO2                      4;
#define MODE_READ_TEMP_AND_BATTERY          5;

#define PIN_DATA_ECG  A4  //AIN4
#define PIN_VREF_ECG  A5  //AIN5
#define PIN_NEG_LO_ECG  13
#define PIN_POS_LO_ECG
#define PIN_MAX_MFIO  
#define PIN_MAX_RSTN

void TaskReadEcg(void *pvParameters);
void TaskHeartRate(void *pvParameters);
void TaskReadSpo2(void *pvParameters);
void TaskReadTempAndBattery(void *pvParameters);

// BLE Service
// BLEDfu  bledfu;  // OTA DFU service
// BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
// BLEBas  blebas;  // battery
BLEClientUart clientUart; // bleuart client

void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void startAdv(void);
void bleuart_rx_callback(uint16_t conn_handle);

QueueHandle_t xQueueSpo2;


// const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
// const float sampling_time = 0.006; //Sampling time in seconds.
// IIR::ORDER  order  = IIR::ORDER::OD4; // Order (OD1 to OD4)
const float cutoff_freq   = 37.0;  //Cutoff frequency in Hz
const float sampling_time = 0.003; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD4; // Order (OD1 to OD4)
// Low-pass filter
Filter f(cutoff_freq, sampling_time, order);

MAX30105 Max_Spo2;
byte ledBrightness = 60; //Options: 0=Off to 255=50mA
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384


uint32_t timeEcg = 0, timeHeartRate = 0;
uint8_t count_ecg = 2, count_heart_rate = 2, count_spo2 = 2;
uint16_t dataEcg = 0;
uint8_t dataSendEcg[20], dataSendSpo2Graph[20], dataSendHeartRate[20];
uint8_t dataEcg_High = 0, dataEcg_Low = 0;
uint8_t irValue_High = 0, irValue_Low = 0;
bool start_send_data_ecg = false, start_send_data_spo2 = true;

const int8_t bufferLength = 100; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
uint32_t irBuffer[bufferLength]; //infrared LED sensor data
uint32_t redBuffer[bufferLength];  //red LED sensor data
int dataBufferSpo2;
const byte RATE_SIZE = 15; //Increase this for more averaging. 4 is good.
byte heartRates[RATE_SIZE]; //Array of heart rates
byte rateSpotHearRate = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
uint16_t beatAvg;

struct max_value
{
    uint32_t red_value[100];
    uint32_t ir_value[100];
};


void TaskReadEcg(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    // Serial.println("TaskReadEcg 1");
    if(millis() - timeEcg >= 8 && start_send_data_ecg){
      timeEcg = millis();

      if(count_ecg >= 19){
      dataSendEcg[0] = 0;
      dataSendEcg[1] = MODE_READ_ECG_GRAPH;
      bleuart.write(dataSendEcg, 20);
      count_ecg = 2;
    }

      // dataEcg = analogRead(PIN_DATA_ECG);
      dataEcg = int(analogRead(PIN_DATA_ECG) - analogRead(PIN_VREF_ECG) + 500);
      float filteredval = f.filterIn(dataEcg);
      Serial.println((uint16_t)filteredval);

      dataSendEcg[count_ecg] = (uint16_t)filteredval >> 8;
      dataSendEcg[count_ecg+1] = (uint16_t)filteredval & 0x00FF;
      count_ecg += 2;
    }
    vTaskDelay(pdMS_TO_TICKS( 1/portTICK_PERIOD_MS)); // wait for one second
  }
}


void TaskHeartRate(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  int num_spo2 = 0;
  struct max_value max_send;
  uint32_t redValue;
  uint32_t irValue;
  for(int i = 0; i < RATE_SIZE; i++){
    heartRates[i] = 70;
  }
  for (;;) // A Task shall never return or exit.
  {
    // Serial.println("TaskHeartRate 2");
    vTaskDelay(pdMS_TO_TICKS(1/portTICK_PERIOD_MS)); // wait for one ms second
    if(millis() - timeHeartRate >= 10 && start_send_data_spo2){
      timeHeartRate = millis();
      if(count_heart_rate >= 19){
        dataSendSpo2Graph[0] = 0;
        dataSendSpo2Graph[1] = MODE_READ_SPO2_GRAPH;
        bleuart.write(dataSendSpo2Graph, 20);
        count_heart_rate = 2;
      }
      
      redValue = Max_Spo2.getRed();
      // irValue = Max_Spo2.getIR();
      // Max_Spo2.nextSample();
      // Serial.println(redValue); //Send raw data to plotter
      dataSendSpo2Graph[count_heart_rate] = (redValue - 50000) >> 8;
      dataSendSpo2Graph[count_heart_rate+1] = (redValue - 50000) & 0x00FF;
      count_heart_rate += 2;
      if (checkForBeat(redValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          heartRates[rateSpotHearRate++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpotHearRate %= RATE_SIZE; //Wrap variable

          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += heartRates[x];
          beatAvg /= RATE_SIZE;
        }
        // Serial.print("bmp: ");
        // Serial.println(beatAvg);
        dataSendHeartRate[0] = 0;
        dataSendHeartRate[1] = MODE_READ_HEART_RATE;
        dataSendHeartRate[2] = beatAvg >> 8;
        dataSendHeartRate[3] = beatAvg & 0x00FF;
        bleuart.write(dataSendHeartRate, 20);
      }

      
      max_send.ir_value[num_spo2] = irValue;
      max_send.red_value[num_spo2] = redValue;
      num_spo2++;
      if(num_spo2 >= bufferLength){
        // for(int i = 0; i < bufferLength; i++){
        //   Serial.print("send: ");
        //   Serial.println(max_send.ir_value[i]);
        // }
        num_spo2 = 0;
        if(xQueueSend(xQueueSpo2, &max_send, 0) != pdPASS){
          Serial.println("no queue space.");
        }
      }
    }
    // else{
    //   Max_Spo2.check();
    // }
  }
}

void TaskReadSpo2(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  struct max_value maxValue;
  uint8_t dataSendSpo2[20];
  uint8_t spo2s[RATE_SIZE];
  int32_t spo2Avg; //SPO2 avg value
  byte rateSpotSpo2 = 0;
  for(int i = 0; i < RATE_SIZE; i++){
    spo2s[i] = 98;
  }
  for (;;) // A Task shall never return or exit.
  {
    // Serial.println("TaskReadSpo2 3");
    if(xQueueReceive(xQueueSpo2, &maxValue, 10) == pdPASS){
      // for(int i = 0; i < bufferLength; i++){
      //   Serial.print("rece: ");
      //   Serial.println(maxValue.ir_value[i]);
      // }
      
      maxim_heart_rate_and_oxygen_saturation(maxValue.ir_value, bufferLength, maxValue.red_value, &spo2, &validSPO2, &heartRate, &validHeartRate);
      // Serial.println(spo2);
      if(validSPO2){
        spo2s[rateSpotSpo2++] = (uint8_t)spo2; //Store this reading in the array
        rateSpotSpo2 %= RATE_SIZE; //Wrap variable
        //Take average of readings
        spo2Avg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          spo2Avg += spo2s[x];
        spo2Avg /= RATE_SIZE;
        if(spo2Avg >= 99){
          spo2Avg = spo2Avg - 2;
        } 
        dataSendSpo2[0] = 0;
        dataSendSpo2[1] = MODE_READ_SPO2;
        dataSendSpo2[2] = spo2Avg >> 8;
        dataSendSpo2[3] = spo2Avg & 0x00FF;
        bleuart.write(dataSendSpo2, 20);
      }
    }
    vTaskDelay(pdMS_TO_TICKS( 1)); // wait for one ms second
  }
}

void TaskReadTempAndBattery(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  // Max_Spo2.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
  float temperature = 36;
  float battery_adc;
  uint32_t time_temp;
  uint8_t dataSendTemp[20];
  for (;;) // A Task shall never return or exit.
  {
    if(millis() - time_temp >= 1000 && start_send_data_spo2){
      time_temp = millis();
      // temperature = Max_Spo2.readTemperature();
      battery_adc = analogRead(A1);
      battery_adc = battery_adc/512*100;
      // Serial.println(battery_adc);
      dataSendTemp[0] = 0;
      dataSendTemp[1] = MODE_READ_TEMP_AND_BATTERY;
      dataSendTemp[2] = (uint16_t)temperature >> 8;
      dataSendTemp[3] = (uint16_t)temperature & 0x00FF;
      dataSendTemp[4] = (uint16_t)battery_adc >> 8;
      dataSendTemp[5] = (uint16_t)battery_adc & 0x00FF;
      bleuart.write(dataSendTemp, 20);
      

    }
    vTaskDelay(pdMS_TO_TICKS( 1)); // wait for one ms second
  }
}

void startAdv(void)
{
  // Bluefruit.Advertising.stop();
  // Bluefruit.Advertising.clearData();
  // Bluefruit.ScanResponse.clearData(); // add this
  // Bluefruit.setName("ElcomEcg");

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  // Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  // Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

void bleuart_rx_callback(uint16_t conn_handle)
{
  (void) conn_handle;
  // Forward data from Mobile to our peripheral
  char str[20+1] = { 0 };
  bleuart.read(str, 20);

  // Serial.print("[Prph] RX: ");
  // Serial.println(str);  
  if(str[0] == 's'){
    Serial.println("Start ECG");
    count_ecg = 2;
    start_send_data_ecg = true;
  }
  else if(str[0] == 't'){
    Serial.println("Stop ECG");
    count_ecg = 2;
    start_send_data_ecg = false;
  }
  else if(str[0] == 'n'){
    Serial.println("Start Spo2");
    count_heart_rate = 2;
    dataBufferSpo2 = 75;
    start_send_data_spo2 = true;
  }
  else if(str[0] == 'm'){
    Serial.println("Stop Spo2");
    count_heart_rate = 2;
    start_send_data_spo2 = false;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  Serial.println("Begin!!!");

  // Initialize sensor
  if (!Max_Spo2.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    // delay(1000);
    // while (1);
  }

  Max_Spo2.setup();
  Max_Spo2.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  Max_Spo2.setPulseAmplitudeGreen(0); //Turn off Green LED

  // Max_Spo2.setup();
  // Max_Spo2.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  // Max_Spo2.setPulseAmplitudeGreen(0); //Turn off Green LED

  analogReadResolution(10);

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  // Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  // Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setName("ElcomEcg"); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  bleuart.begin();
  bleuart.setRxCallback(bleuart_rx_callback);
  // Set up and start advertising
  startAdv();

  

  xQueueSpo2 = xQueueCreate( 10, sizeof( struct max_value ) );
  if( xQueueSpo2 == NULL )
  {
      /* Queue was not created and must not be used. */
      Serial.println("Can't create xQueueSpo2");
  }
  Serial.println("Start Task!!!");
  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskReadEcg
    ,  (const portCHAR *)"TaskReadEcg"   // A name just for humans
    ,  1024  // Stack size
    ,  NULL
    ,  3  // priority
    ,  NULL );

  xTaskCreate(
    TaskHeartRate
    ,  (const portCHAR *) "TaskHeartRate"
    ,  1024 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  2  // priority
    ,  NULL );

  xTaskCreate(
    TaskReadSpo2
    ,  (const portCHAR *) "TaskReadSpo2"
    ,  1024 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  2  // priority
    ,  NULL );

    xTaskCreate(
    TaskReadTempAndBattery
    ,  (const portCHAR *) "TaskReadTempAndBattery"
    ,  512 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  2  // priority
    ,  NULL );

  // vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS( 5 )); // wait for one ms second
}















/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// The example shows how to retrieve raw values from the sensor
// experimenting with the most relevant configuration parameters.
// Use the "Serial Plotter" app from arduino IDE 1.6.7+ to plot the output
// #include <Arduino.h>
// #include <Wire.h>
// #include "MAX30100.h"
// #include <SPI.h>
// #include <filters.h>

// // Sampling is tightly related to the dynamic range of the ADC.
// // refer to the datasheet for further info
// #define SAMPLING_RATE                       MAX30100_SAMPRATE_100HZ

// // The LEDs currents must be set to a level that avoids clipping and maximises the
// // dynamic range
// #define IR_LED_CURRENT                      MAX30100_LED_CURR_50MA
// #define RED_LED_CURRENT                     MAX30100_LED_CURR_27_1MA

// // The pulse width of the LEDs driving determines the resolution of
// // the ADC (which is a Sigma-Delta).
// // set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US    _16BITS
// #define PULSE_WIDTH                         MAX30100_SPC_PW_1600US_16BITS
// #define HIGHRES_MODE                        true


// // Instantiate a MAX30100 sensor class
// MAX30100 sensor;

// void setup()
// {
//     Serial.begin(115200);

//     Serial.print("Initializing MAX30100..");

//     // Initialize the sensor
//     // Failures are generally due to an improper I2C wiring, missing power supply
//     // or wrong target chip
//     if (!sensor.begin()) {
//         Serial.println("FAILED");
//         // for(;;);
//     } else {
//         Serial.println("SUCCESS");
//     }

//     // Set up the wanted parameters
//     sensor.setMode(MAX30100_MODE_SPO2_HR);
//     sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
//     sensor.setLedsPulseWidth(PULSE_WIDTH);
//     sensor.setSamplingRate(SAMPLING_RATE);
//     sensor.setHighresModeEnabled(HIGHRES_MODE);
// }

// void loop()
// {
//     uint16_t ir, red;

//     sensor.update();

//     while (sensor.getRawValues(&ir, &red)) {
//         Serial.print(ir);
//         Serial.print('\t');
//         Serial.println(red);
//     }
// }