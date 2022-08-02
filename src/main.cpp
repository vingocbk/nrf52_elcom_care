#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "filters.h"
#include "max32664.h"
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#define MODE_READ_ECG_GRAPH                 1;
#define MODE_READ_IR_VALUE                  2;
#define MODE_READ_BLOOD_PRESSURE            3;
#define MODE_READ_PROGRESS_CALIB            4;
#define MODE_READ_TEMP_AND_BATTERY          5;
#define MODE_READ_HEART_RATE_FROM_ECG       6;

#define PIN_DATA_ECG  A4  //AIN4
#define PIN_VREF_ECG  A5  //AIN5
#define PIN_NEG_LO_ECG  25
#define PIN_POS_LO_ECG  26
#define PIN_MAX_MFIO  06
#define PIN_MAX_RSTN  07
#define PIN_LED_R     A7
#define PIN_LED_G     A3
#define PIN_LED_B     A2

#define  RAWDATA_BUFFLEN 400

void TaskReadEcg(void *pvParameters);
void TaskBloodPressure(void *pvParameters);
void TaskReadIrValue(void *pvParameters);
void TaskReadTempAndBattery(void *pvParameters);
void mfioInterruptHndlr();
void enableInterruptPin();
void loadAlgomodeParameters();

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
void scanI2cAddress();
SemaphoreHandle_t xSemaphore = NULL;

max32664 MAX32664(PIN_MAX_RSTN, PIN_MAX_MFIO, RAWDATA_BUFFLEN);

const float cutoff_freq_low   = 37.0;  //Cutoff frequency in Hz
const float sampling_time_low = 0.003; //Sampling time in seconds.
IIR::ORDER  order_low  = IIR::ORDER::OD4; // Order (OD1 to OD4)
// Low-pass filter
Filter filter_low(cutoff_freq_low, sampling_time_low, order_low);

const float cutoff_freq_high   = 0.1;  //Cutoff frequency in Hz
const float sampling_time_high = 0.003; //Sampling time in seconds.
IIR::ORDER  order_high  = IIR::ORDER::OD2; // Order (OD1 to OD4)
// Low-pass filter
Filter filter_high(cutoff_freq_high, sampling_time_high, order_high, IIR::TYPE::HIGHPASS);

uint8_t count_ecg = 2;

bool start_send_data_ecg = false, start_send_data_ir_value = false, start_send_data_blood_pressure = false;
bool statusConnectBle = false;

void TaskReadEcg(void *pvParameters)  // This is a task.
{
  uint16_t dataEcg = 0;
  uint16_t buffer_data_ecg[100];
  uint8_t count_current_buffer_data_ecg = 0;
  uint8_t dataSendEcg[20];
  uint8_t dataSendHearRateEcg[20];
  uint32_t timeEcg = 0;
  uint16_t beat_new = 0, beat_old = 0;
  uint16_t current_BPM = 0;
  uint16_t beats[10] = {0};
  uint8_t beatIndex = 0;
  uint16_t max_value_ecg = 0;
  bool belowThreshold  = false;
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    if(millis() - timeEcg >= 8 && start_send_data_ecg){
      timeEcg = millis();
      
      if(count_ecg >= 19){
        dataSendEcg[0] = MODE_READ_ECG_GRAPH;
        // dataSendEcg[1] = MODE_READ_ECG_GRAPH;
        bleuart.write(dataSendEcg, 20);
        count_ecg = 2;
      }

      // dataEcg = analogRead(PIN_DATA_ECG);
      dataEcg = int(analogRead(PIN_DATA_ECG) - analogRead(PIN_VREF_ECG) + 500);
      if((digitalRead(PIN_NEG_LO_ECG) == 1)||(digitalRead(PIN_POS_LO_ECG) == 1)){
        dataEcg = 500;
      }
      float filteredval = filter_low.filterIn(dataEcg);
      Serial.println((uint16_t)filteredval);

      dataSendEcg[count_ecg] = (uint16_t)filteredval >> 8;
      dataSendEcg[count_ecg+1] = (uint16_t)filteredval & 0x00FF;
      count_ecg += 2;

      //Calculate Heart Rate
      buffer_data_ecg[count_current_buffer_data_ecg] = (uint16_t)filteredval;
      count_current_buffer_data_ecg ++;
      if(count_current_buffer_data_ecg >= 90){
        count_current_buffer_data_ecg = 0;
        max_value_ecg = buffer_data_ecg[0];

        for(int i = 0; i < 90; i++){
          if(max_value_ecg < buffer_data_ecg[i] && buffer_data_ecg[i] < 650){
            max_value_ecg = buffer_data_ecg[i];
            beat_new = i;
          }
        }
        current_BPM = 60000/((beat_new+(90-beat_old))*8);    // convert to beats per minute
        if(current_BPM > 60 && current_BPM < 120){
          beats[beatIndex] = current_BPM;  // store to array to convert the average
          beatIndex = (beatIndex + 1) % 10;
          uint16_t total = 0;
          for (int i = 0; i < 10; i++){
            if(beats[i] == 0){
              beats[i] = current_BPM;
            }
            total += beats[i];
          }
          current_BPM = uint16_t(total / 10);
          dataSendHearRateEcg[0] = MODE_READ_HEART_RATE_FROM_ECG;
          dataSendHearRateEcg[1] = current_BPM >> 8;
          dataSendHearRateEcg[2] = current_BPM & 0x00FF;
          bleuart.write(dataSendHearRateEcg, 3);
        }
      }

      // if((uint16_t)filteredval > (max_value_ecg - 15) && belowThreshold){
      //   belowThreshold = false;
      //   beat_new = millis();
      //   int diff = beat_new - beat_old;    // find the time between the last two beats
      //   beat_old = beat_new;
      //   current_BPM = 60000 / diff;    // convert to beats per minute
      //   if(current_BPM > 60 && current_BPM < 120){
      //     beats[beatIndex] = current_BPM;  // store to array to convert the average
      //     uint16_t total = 0;
      //     for (int i = 0; i < 10; i++){
      //       if(beats[i] == 0){
      //         beats[i] = current_BPM;
      //       }
      //       total += beats[i];
      //     }
      //     beatIndex = (beatIndex + 1) % 10;
      //     current_BPM = uint16_t(total / 10);
      //     dataSendHearRateEcg[0] = MODE_READ_HEART_RATE_FROM_ECG;
      //     dataSendHearRateEcg[1] = current_BPM >> 8;
      //     dataSendHearRateEcg[2] = current_BPM & 0x00FF;
      //     bleuart.write(dataSendHearRateEcg, 3);
      //   }
      // }
      // else if((uint16_t)filteredval < (max_value_ecg - 15)){
      //   belowThreshold = true;
      // }

    }
    vTaskDelay(pdMS_TO_TICKS(1)); // wait for one ms
  }
}


void TaskBloodPressure(void *pvParameters)  // This is a task.
{
  uint8_t dataSendProgess[2];
  uint8_t dataSendBloodPressure[5];
  bool statusCalib = true;
  bool startCalib = true;
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    if(!start_send_data_blood_pressure){
      startCalib = true;
    }
    if(start_send_data_blood_pressure && startCalib){
      startCalib = false;
      statusCalib = true;
      bool ret = MAX32664.startBPTcalibration();
      while(!ret){
        Serial.println("failed calib, please retsart");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ret = MAX32664.startBPTcalibration();
      }
      Serial.println("Please keep your finger on sensor untill progress reach 100%");
      vTaskDelay(pdMS_TO_TICKS(120));
      uint8_t bpStatus = MAX32664.readCalibSamples();
      while(statusConnectBle && start_send_data_blood_pressure && bpStatus != 2 && (MAX32664.max32664Output.progress != 100) ){ //bpStatus, 0x02 == success
        bpStatus = MAX32664.readCalibSamples();
        if(bpStatus == 05){
          statusCalib = false;
          dataSendProgess[0] = MODE_READ_PROGRESS_CALIB;
          dataSendProgess[1] = 255;
          bleuart.write(dataSendProgess, 2);
          Serial.println("calibration failed");
          break;
        }
        dataSendProgess[0] = MODE_READ_PROGRESS_CALIB;
        dataSendProgess[1] = MAX32664.max32664Output.progress;
        bleuart.write(dataSendProgess, 2);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      // vTaskDelay(pdMS_TO_TICKS(1000));

      //Serial.println("start in estimation mode");
      if(start_send_data_blood_pressure){
        ret = MAX32664.configAlgoInEstimationMode();
        while(!ret){
          Serial.println("failed est mode");
          ret = MAX32664.configAlgoInEstimationMode();
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
    }
    if(start_send_data_blood_pressure && statusCalib){
      // Serial.println("statusCalib true");
      uint8_t num_samples = MAX32664.readSamples();
      if(num_samples){
        dataSendBloodPressure[0] = MODE_READ_BLOOD_PRESSURE;
        dataSendBloodPressure[1] = MAX32664.max32664Output.hr;
        dataSendBloodPressure[2] = (uint8_t)MAX32664.max32664Output.spo2;
        dataSendBloodPressure[3] = MAX32664.max32664Output.dia;
        dataSendBloodPressure[4] = MAX32664.max32664Output.sys;
        bleuart.write(dataSendBloodPressure, 5);
        Serial.print("sys = ");
        Serial.print(MAX32664.max32664Output.sys);
        Serial.print(", dia = ");
        Serial.print(MAX32664.max32664Output.dia);
        Serial.print(", hr = ");
        Serial.print(MAX32664.max32664Output.hr);
        Serial.print(" spo2 = ");
        Serial.println(MAX32664.max32664Output.spo2);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // wait for one ms
  }
}

void TaskReadIrValue(void *pvParameters)  // This is a task.
{
  bool turnOnRawMode;
  int16_t irBuff[RAWDATA_BUFFLEN];
  uint8_t dataSendIrValue[20];
  dataSendIrValue[0] = MODE_READ_IR_VALUE;
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    if(!start_send_data_ir_value){
      turnOnRawMode = true;
    }
    if(start_send_data_ir_value && turnOnRawMode){
      turnOnRawMode = false;
      MAX32664.configRawdataMode();  
    }
    if(start_send_data_ir_value && !turnOnRawMode){
      uint8_t num_samples = MAX32664.readRawSamples(irBuff);
      if(num_samples > 0){
        for(int i=0; i<num_samples; i++){
          Serial.println(irBuff[i]);
        }
        if(num_samples <= 9){
          for(int i = 0, j = 2; i < num_samples; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, num_samples*2+2);
        }
        else if(num_samples <= 18){
          for(int i = 0, j = 2; i < 9; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 20);
          for(int i = 9, j = 2; i < num_samples; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 2*(num_samples-9)+2);
        }
        else if(num_samples <= 27){
          for(int i = 0, j = 2; i < 9; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 20);
          for(int i = 9, j = 2; i < 18; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 20);
          for(int i = 18, j = 2; i < num_samples; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 2*(num_samples-18)+2);
        }
        else if(num_samples <= 36){
          for(int i = 0, j = 2; i < 9; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 20);
          for(int i = 9, j = 2; i < 18; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 20);
          for(int i = 18, j = 2; i < 27; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 20);
          for(int i = 27, j = 2; i < num_samples; i++, j=j+2){
            dataSendIrValue[j] = irBuff[i] >> 8;
            dataSendIrValue[j+1] = irBuff[i] & 0x00FF;
          }
          bleuart.write(dataSendIrValue, 2*(num_samples-27)+2);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // wait for one ms
  }
}


void mfioInterruptHndlr(){
  //Serial.println("i");
}

void enableInterruptPin(){

  //pinMode(mfioPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);

}

void loadAlgomodeParameters(){

  algomodeInitialiser algoParameters;
  /*  Replace the predefined values with the calibration values taken with a reference spo2 device in a controlled environt.
      Please have a look here for more information, https://pdfserv.maximintegrated.com/en/an/an6921-measuring-blood-pressure-MAX32664D.pdf
      https://github.com/Protocentral/protocentral-pulse-express/blob/master/docs/SpO2-Measurement-Maxim-MAX32664-Sensor-Hub.pdf
  */

  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;

  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
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
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  statusConnectBle = true;
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
  statusConnectBle = false;
  start_send_data_ecg = false;
  start_send_data_ir_value = false;
  start_send_data_blood_pressure = false;
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
    start_send_data_ir_value = false;
    start_send_data_blood_pressure = false;
  }
  else if(str[0] == 't'){
    Serial.println("Stop ECG");
    count_ecg = 2;
    start_send_data_ecg = false;
  }
  else if(str[0] == 'n'){
    Serial.println("Start Read IR Value");
    start_send_data_ir_value = true;
    start_send_data_ecg = false;
    start_send_data_blood_pressure = false;
  }
  else if(str[0] == 'm'){
    Serial.println("Stop Read IR Value");
    start_send_data_ir_value = false;
  }
  else if(str[0] == 'u'){
    Serial.println("Start Bloode Pressure");
    start_send_data_blood_pressure = true;
    start_send_data_ecg = false;
    start_send_data_ir_value = false;
  }
  else if(str[0] == 'k'){
    Serial.println("Stop Bloode Pressure");
    start_send_data_blood_pressure = false;
  }
}

void scanI2cAddress(){
  byte error, address; //variable for error and I2C address
  int nDevices;
  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  Serial.println("Begin!!!");
  Wire.begin();
  scanI2cAddress();

  analogReadResolution(10);
  analogWrite(PIN_LED_R, 0);
  // analogWrite(PIN_LED_G, 0);
  // analogWrite(PIN_LED_B, 0);

  pinMode(PIN_NEG_LO_ECG, INPUT);
  pinMode(PIN_POS_LO_ECG, INPUT);

  loadAlgomodeParameters();

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

  loadAlgomodeParameters();
  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS){
    Serial.println("Sensorhub begin!");
  }else{
    while(1){
      Serial.println("Could not communicate with the sensor! please make proper connections");
      delay(5000);
    }
  }


  xSemaphore = xSemaphoreCreateMutex();
  Serial.println("Start Task!!!");
  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskReadEcg
    ,  (const portCHAR *)"TaskReadEcg"   // A name just for humans
    ,  1024  // Stack size
    ,  NULL
    ,  1  // priority
    ,  NULL );

  xTaskCreate(
    TaskBloodPressure
    ,  (const portCHAR *) "TaskBloodPressure"
    ,  1024 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL );

  xTaskCreate(
    TaskReadIrValue
    ,  (const portCHAR *) "TaskReadIrValue"
    ,  1024 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL );

  // vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS(100)); // wait for one ms second
  
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  // Serial.println("loop");
  xSemaphoreGive(xSemaphore);
}


