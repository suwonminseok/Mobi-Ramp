/* Central Mode (client) BLE UART for ESP32
 *
 * This sketch is a central mode (client) Nordic UART Service (NUS) that connects automatically to a peripheral (server)
 * Nordic UART Service. NUS is what most typical "blueart" servers emulate. This sketch will connect to your BLE uart
 * device in the same manner the nRF Connect app does.
 *
 * Once connected this sketch will switch notification on using BLE2902 for the charUUID_TX characteristic which is the
 * characteristic that our server is making data available on via notification. The data received from the server
 * characteristic charUUID_TX will be printed to Serial on this device. Every five seconds this device will send the
 * string "Time since boot: #" to the server characteristic charUUID_RX, this will make that data available in the BLE
 * uart and trigger a notifyCallback or similar depending on your BLE uart server setup.
 *
 *
 * A brief explanation of BLE client/server actions and rolls:
 *
 * Central Mode (client) - Connects to a peripheral (server).
 *   -Scans for devices and reads service UUID.
 *   -Connects to a server's address with the desired service UUID.
 *   -Checks for and makes a reference to one or more characteristic UUID in the current service.
 *   -The client can send data to the server by writing to this RX Characteristic.
 *   -If the client has enabled notifications for the TX characteristic, the server can send data to the client as
 *   notifications to that characteristic. This will trigger the notifyCallback function.
 *
 * Peripheral (server) - Accepts connections from a central mode device (client).
 *   -Advertises a service UUID.
 *   -Creates one or more characteristic for the advertised service UUID
 *   -Accepts connections from a client.
 *   -The server can send data to the client by writing to this TX Characteristic.
 *   -If the server has enabled notifications for the RX characteristic, the client can send data to the server as
 *   notifications to that characteristic. This the default function on most "Nordic UART Service" BLE uart sketches.
 *
 *
 * Copyright <2018> <Josh Campbell>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
 * notice and this permission notice shall be included in all copies or substantial portions of the Software. THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 * Based on the "BLE_Client" example by Neil Kolban:
 * https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/BLE_client/BLE_client.ino
 * With help from an example by Andreas Spiess:
 * https://github.com/SensorsIot/Bluetooth-BLE-on-Arduino-IDE/blob/master/Polar_Receiver/Polar_Receiver.ino
 * Nordic UART Service info:
 * https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v14.0.0%2Fble_sdk_app_nus_eval.html
 *
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <stdio.h>
#include <string.h>
#include "esp_adc_cal.h"

#define  BLE_COMM             false
#define  UART_COMM            true

static boolean doConnect        = false;
static boolean connected        = false;

#if BLE_COMM
// The remote service we wish to connect to.
static BLEUUID serviceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
// The characteristic of the remote service we are interested in.
static BLEUUID readUUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static BLEUUID charUUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

// static boolean doConnect        = false;
// static boolean connected        = false;
static boolean doScan           = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicRx;
static BLEAdvertisedDevice*     myDevice;
static BLEScan*                 pBLEScan; 
#endif

#if UART_COMM

#define TX1 10 //23
#define RX1 9  //27

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

char UART_TX_BUF[UART_TX_BUF_SIZE];
char UART_RX_BUF[UART_RX_BUF_SIZE];
uint8_t UART_RX_BUF_Index = 0;
uint8_t Uart_Msg_Length = 0;

bool Sensor_Started = false;
#endif


int             Relay_Count             = 0;
bool            Relay_On                = false;

String          VEHICLEDETECT_CMD       = "00";
String          DIRECTION_CMD           = "01";
String          RELAYTIMER_CMD          = "02";
String          RELAYTIMING_CMD         = "03";
String          SENSITIVITY_CMD         = "04";
String          BATTERYLEVEL_CMD        = "05";
String          OPERATIONMODE_CMD       = "06";
String          BLETXPOWER_CMD          = "07";
String          DIRECTION_CAT_CMD       = "08";
String          SENSORERROR_CMD         = "99";

String          Front_CMD;
String          Back_CMD;

const int       RelayLED                = 2;  // power LED
const int       RelayPin                = 21;  //16; //RELAY
const int       ERRLED                  = 33;   // ERR LED
const int       PowerLED                = 4;   //13; //LED

const uint8_t   notificationOff[]       = {0x0, 0x0};
const uint8_t   notificationOn[]        = {0x1, 0x0};
bool            onoff                   = true;
bool            onoff1                  = true;

const int       Operation_DipSwitch[]   = {14, 15}; // 1 - 0 : ramp, 1: bar / 2 - 0 : nc, 1 : counter
const int       DipSwitch_2             = 36; //Sensor0 Direction, 0 : L -> R, 1 : R -> L
const int       DipSwitch_3             = 32; //Direction Sensitivity 0
const int       Direction_DipSwitch[]   = {26, 39};
const int       Sensitivity_DipSwitch[] = {13, 22};

/*
const int       DipSwitch_0             = 14; //Operation Mode, 0 : ramp, 1 : bar
const int       DipSwitch_1             = 15; //One channel or Two channel
const int       DipSwitch_3             = 32; //Direction Sensitivity 0
const int       DipSwitch_4             = 26; //Direction Sensitivity 1
const int       DipSwitch_5             = 39; //Sensitivity level, 0 ~ 3
const int       DipSwitch_6             = 13; //Sensitivity level, 4 ~ 6
const int       DipSwitch_7             = 22; //Sensitivity level, 7 ~ 9
*/
const int       VariableR               = 35; //Relay Timer 0
const int       VariableR1              = 34; //Relay Timer 1

uint8_t         VEHICLEDETECT_PARAM     = 0;  //0: Off 1: On
uint8_t         VEHICLEDETECT_PARAM1    = 0;  //0: Off 1: On

uint8_t         OPERATIONMODE_PARAM     = 0;  //0: 경광등,  1: 차단기.
uint8_t         TWOCHANNLEMODE_PARAM    = 0;  //0: 1Ch,  1: 2Ch.
uint8_t         TWOINONEMODE_PARAM      = 0;  //0: Out 1&2,  1: Out 1.
uint8_t         SENSITIVITY_PARAM       = 1;  //00: Low, 1: High

uint8_t         DIRECTION_PARAM         = 0;  //0: Left->Right, 1: Right->Left
uint8_t         RELAYTIMING_PARAM       = 0;  //00: In, 01: Out

int             RELAYTIMER_PARAM        = 5;  //5 secs
uint8_t         BATTERYLEVEL_PARAM      = 9;  //00: Off, 9: Max
uint8_t         SENSORERR_PARAM         = 0;  //0: OK, 1: Error

uint8_t         DIRECTION_PARAM1        = 0;  //0: Left->Right, 1: Right->Left
uint8_t         RELAYTIMING_PARAM1      = 0;  //00: In, 01: Out

int             SENSITIVITY_VALUE       = 5;  //Sersitivity value 0 ~ 9
uint8_t         BATTERYLEVEL_PARAM1     = 9;  //00: Off, 9: Max 
uint8_t         SENSORERR_PARAM1        = 0;  //0: OK, 1: Error

uint8_t         DIRECTION_SENSITIVITY   = 0;  //
uint8_t         DIRECTION_SENSITIVITY1  = 0;  //

uint8_t         SENSITIVITY_LEVEL1      = 0;  //
uint8_t         SENSITIVITY_LEVEL2      = 0;  //
uint8_t         SENSITIVITY_LEVEL3      = 0;  //

int             OPERATION_VALUE         = 0;
int             DIRECTION_VALUE         = 0;
int             SENSITIVITY_LEVEL_VALUE = 0;

uint8_t         RelayTimerArr[9]        = {0, 3, 5, 7, 10, 12, 15, 20, 30};
uint8_t         SensitivityArr[8]       = {0, 1, 2, 3, 4, 5, 6, 7};

static boolean  sendParam               = false;

bool            Mobi_Ramp_Sensor0_Connected = false;

bool            Mobi_Ramp_Sensor0_Error = false;
bool            Mobi_Ramp_Sensor0_Error_Flag = false;

uint8_t         Vehicle_Count = 0;

//For ADC
#define         DEFAULT_VREF            1100
esp_adc_cal_characteristics_t           *adc_chars;

void Split_Word_F(String Buffer) {

  int Split_Word = Buffer.indexOf(":");

  if(Split_Word != -1)
  {
    int Split_Length = Buffer.length();

    Front_CMD = Buffer.substring(max(Split_Word - 2, 0), min(Split_Word, Split_Length));
    Back_CMD = Buffer.substring(Split_Word + 1, min(Split_Word + 3, Split_Length));

    Serial.println("Front_CMD : " + Front_CMD);
    Serial.println("Back_CMD : " + Back_CMD);
  }
  else {
    Serial.println("Invalid Command.");
  }
}

//////////////////////////////////////////////////////////////////////////////
////////////////////////--BLE Callback--//////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#if BLE_COMM

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) 
  {
    String          Buffer;

    if (sendParam == false)
      return;

    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.write(pData, length);
    Serial.println();

    String rxValue = (char*)pData;

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      Serial.print(rxValue.length());
      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
        Buffer += (char)rxValue[i];
      }
      Serial.println();
      Serial.println("*********");
      Serial.println(Buffer);
    }

    Split_Word_F(Buffer);

    if(Front_CMD == VEHICLEDETECT_CMD)             //VehicleDetect_Command Mode
    {
      //VEHICLEDETECT_PARAM = atoi(Buffer.substring(msg_start, msg_start+2).c_str());
      VEHICLEDETECT_PARAM = atoi(Back_CMD.c_str());

      if (OPERATIONMODE_PARAM == 0) {  //경광등 모드
        if (VEHICLEDETECT_PARAM == 1 && RELAYTIMING_PARAM == 0)
        {
          Serial.println("입차");
          Relay_Count = RELAYTIMER_PARAM * 10;
          Buffer = "";
          Serial.println(Relay_Count);

          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(SensorPin, HIGH); 
          }      
        }
        else if(VEHICLEDETECT_PARAM == 0 && RELAYTIMING_PARAM == 1) {
          Serial.println("출차");
          Relay_Count = RELAYTIMER_PARAM * 10;
          Buffer = "";
          Serial.println("Relay_Count : " + Relay_Count);
          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(SensorPin, HIGH); 
          }           
        }
      } else if (OPERATIONMODE_PARAM == 1) {  // 차단봉 모드 
        if (VEHICLEDETECT_PARAM == 1)
        {
          Serial.println("입차");
          Buffer = "";
          digitalWrite(RelayPin, HIGH);
          digitalWrite(SensorPin, HIGH); 

          /*
          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(RelayLed, HIGH); 
          }
          */         
        } else if (VEHICLEDETECT_PARAM == 0)
        {
          Serial.println("출차");
          Buffer = "";
          digitalWrite(RelayPin, LOW);
          digitalWrite(SensorPin, LOW); 
          //Relay_On = true;        
          //Relay_Count = RELAYTIMER_PARAM * 10;        
        }
      } else if(OPERATIONMODE_PARAM == 2 || OPERATIONMODE_PARAM == 3) { // 카운터 모드

        if (VEHICLEDETECT_PARAM == 1 && RELAYTIMING_PARAM == 0)
        {
          Serial.println("입차");
          Vehicle_Count = Vehicle_Count + 1;
          /*
          Relay_Count = 1 * 10;
          Buffer = "";
          Serial.println(Relay_Count);

          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(SensorPin, HIGH); 
          }
        */    
        }
        else if(VEHICLEDETECT_PARAM == 0 && RELAYTIMING_PARAM == 1) {
          Serial.println("출차");
          Vehicle_Count = Vehicle_Count + 1;
          /*
          Relay_Count = 1 * 10;
          Buffer = "";
          Serial.println(Relay_Count);

          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(SensorPin, HIGH); 
          }
        */          
        }
      }
    }
    else if(Front_CMD == (SENSORERROR_CMD))
    {
      //SENSORERR_PARAM = atoi(Buffer.substring(msg_start, msg_start+2).c_str());
      SENSORERR_PARAM = atoi(Back_CMD.c_str());

      if(SENSORERR_PARAM == 1)
        Mobi_Ramp_Sensor0_Error = true;
      else 
        Mobi_Ramp_Sensor0_Error = false;

      Buffer = "";
      Serial.println("mobi-ramp sensor err");
    }
    else {
      Serial.println("This command does not exist.");
      Buffer = "";
    }  
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Mobi_Ramp_Sensor0_Connected = true;
    Serial.println("Connected to device 1");
    SENSORERR_PARAM = 0;
  }

  void onDisconnect(BLEClient* pclient) {
    Mobi_Ramp_Sensor0_Connected = false;
    Mobi_Ramp_Sensor0_Error = false;
    connected = false;
    sendParam = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristicRx = pRemoteService->getCharacteristic(readUUID);
    if (pRemoteCharacteristicRx == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(readUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our Rx characteristic");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify()) {
      pRemoteCharacteristic->registerForNotify(notifyCallback);

      Serial.println("Notifications turned on");
      pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
    }
    
    connected = true;
    return true;
}


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() 
    && advertisedDevice.isAdvertisingService(serviceUUID)
    && (advertisedDevice.getName().compare("[intervoid]mobi-ramp_01") == 0)    
    ) 
    {
      Mobi_Ramp_Sensor0_Connected = true;
   
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

#endif

#if UART_COMM

static void UART_CMD_PROCESSOR (
  uint8_t* pData,
  size_t length ) 
  {
    String          Buffer;

    if (sendParam == false)
      return;

    Serial.print("Notify callback for UART ");
    Serial.print("mobi-ramp sensor");
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.write(pData, length);
    Serial.println();

    String rxValue = (char*)pData;

    if (rxValue.length() > 0) {
      // Serial.println("*********");
      // Serial.print("Received Value: ");
      // Serial.print(rxValue.length());
      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
        Buffer += (char)rxValue[i];
      }
      // Serial.println();
      // Serial.println("*********");
      // Serial.println(Buffer);
    }

    Split_Word_F(Buffer);

    if(Front_CMD == VEHICLEDETECT_CMD)             //VehicleDetect_Command Mode
    {
      //VEHICLEDETECT_PARAM = atoi(Buffer.substring(msg_start, msg_start+2).c_str());
      VEHICLEDETECT_PARAM = atoi(Back_CMD.c_str());

      if (OPERATIONMODE_PARAM == 0) {  //경광등 모드
        if (VEHICLEDETECT_PARAM == 1 && RELAYTIMING_PARAM == 0)
        {
          Serial.println("입차");
          Relay_Count = RELAYTIMER_PARAM * 10;
          Buffer = "";
          Serial.println(Relay_Count);

          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(RelayLED, HIGH); 
          }      
        }
        else if(VEHICLEDETECT_PARAM == 0 && RELAYTIMING_PARAM == 1) {
          Serial.println("출차");
          Relay_Count = RELAYTIMER_PARAM * 10;
          Buffer = "";
          Serial.println("Relay_Count : " + Relay_Count);
          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(RelayLED, HIGH); 
          }           
        }
      } else if (OPERATIONMODE_PARAM == 1) {  // 차단봉 모드 
        if (VEHICLEDETECT_PARAM == 1)
        {
          Serial.println("입차");
          Buffer = "";
          digitalWrite(RelayPin, HIGH);
          digitalWrite(RelayLED, HIGH); 

          /*
          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(RelayLed, HIGH); 
          }
          */         
        } else if (VEHICLEDETECT_PARAM == 0)
        {
          Serial.println("출차");
          Buffer = "";
          digitalWrite(RelayPin, LOW);
          digitalWrite(RelayLED, LOW); 
          //Relay_On = true;        
          //Relay_Count = RELAYTIMER_PARAM * 10;        
        }
      } else if(OPERATIONMODE_PARAM == 2 || OPERATIONMODE_PARAM == 3) { // 카운터 모드

        if (VEHICLEDETECT_PARAM == 1 && RELAYTIMING_PARAM == 0)
        {
          Serial.println("입차");
          Vehicle_Count = Vehicle_Count + 1;
          /*
          Relay_Count = 1 * 10;
          Buffer = "";
          Serial.println(Relay_Count);

          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(SensorPin, HIGH); 
          }
        */    
        }
        else if(VEHICLEDETECT_PARAM == 0 && RELAYTIMING_PARAM == 1) {
          Serial.println("출차");
          Vehicle_Count = Vehicle_Count + 1;
          /*
          Relay_Count = 1 * 10;
          Buffer = "";
          Serial.println(Relay_Count);

          if(RELAYTIMER_PARAM != 0)
          {
            Relay_On = true;
            digitalWrite(RelayPin, HIGH);
            digitalWrite(SensorPin, HIGH); 
          }
        */          
        }
      }
    }
    else if(Front_CMD == (SENSORERROR_CMD))
    {
      //SENSORERR_PARAM = atoi(Buffer.substring(msg_start, msg_start+2).c_str());
      SENSORERR_PARAM = atoi(Back_CMD.c_str());

      if(SENSORERR_PARAM == 1)
        Mobi_Ramp_Sensor0_Error = true;
      else 
        Mobi_Ramp_Sensor0_Error = false;

      Buffer = "";
      Serial.println("mobi-ramp sensor err");
    }
    else {
      Serial.println("This command does not exist.");
      Buffer = "";
    }  
}

#endif


/////////////////////////////////////////////////////////////////////////
//Read Dip Switch Values
/////////////////////////////////////////////////////////////////////////

void readDipSwitchVal()
{
  for(int i = 0; i <2 ; i++) {
    pinMode(Operation_DipSwitch[i], INPUT_PULLUP);
  }

  pinMode(DipSwitch_2, INPUT_PULLUP);
  pinMode(DipSwitch_3, INPUT_PULLUP);
  
  for(int i = 0; i <2 ; i++) {
    pinMode(Direction_DipSwitch[i], INPUT_PULLUP);
  }
  
  for(int j = 0; j < 2; j++) {
    pinMode(Sensitivity_DipSwitch[j], INPUT_PULLUP);
  }

   for(int i = 1; i >= 0; i--) {
    int operation_switch = digitalRead(Operation_DipSwitch[i]);
    operation_switch = !operation_switch;
    OPERATION_VALUE = OPERATION_VALUE | (operation_switch << i);
  }
  OPERATIONMODE_PARAM = OPERATION_VALUE;

  Serial.printf("Operation_value = ");
  Serial.println(OPERATION_VALUE);

  DIRECTION_PARAM         = digitalRead(DipSwitch_2) == 1 ? 0 : 1;
  RELAYTIMING_PARAM       = digitalRead(DipSwitch_3) == 1 ? 0 : 1;

  Serial.printf("DIRECTION_PARAM = ");
  Serial.println(DIRECTION_PARAM);

  Serial.printf("RELAYTIMING_PARAM = ");
  Serial.println(RELAYTIMING_PARAM);

  for(int i = 1; i >= 0; i--) {
    int direction_switch = digitalRead(Direction_DipSwitch[i]);
    direction_switch = !direction_switch;
    DIRECTION_VALUE = DIRECTION_VALUE | (direction_switch << i);
  }

  Serial.printf("Direction_Sensitivity_value = ");
  Serial.println(DIRECTION_VALUE);

  for(int j = 1; j >= 0; j--) {
    int Sensitivity_switch = digitalRead(Sensitivity_DipSwitch[j]);
    Sensitivity_switch = !Sensitivity_switch;
    SENSITIVITY_LEVEL_VALUE = SENSITIVITY_LEVEL_VALUE | (Sensitivity_switch << j);
  }

  Serial.printf("Sensitivity_level_value = ");
  Serial.println(SENSITIVITY_LEVEL_VALUE);

  //ADC Settings
  //Range 0-4096
  adc1_config_width(ADC_WIDTH_12Bit);
  
  //full voltage range
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_11db); 
  //adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db); 

  /*
  //get the ADC characteristics
  esp_adc_cal_characterize(
    ADC_UNIT_1,
    ADC_ATTEN_DB_11,
    ADC_WIDTH_12Bit,
    DEFAULT_VREF,
    adc_chars
  );
  */

  RELAYTIMER_PARAM        = adc1_get_raw(ADC1_CHANNEL_7);
  //SENSITIVITY_VALUE       = adc1_get_raw(ADC1_CHANNEL_6);

  //RELAYTIMER_PARAM        = analogRead(VariableR);
  Serial.println((String)"VariableR: " + RELAYTIMER_PARAM);
  //RELAYTIMER_PARAM1       = analogRead(VariableR1);
 // Serial.println((String)"Sensitivity: " + SENSITIVITY_VALUE);

  
  //uint8_t         RelayTimerArr[9]       = {0, 3, 5, 7, 10, 12, 15, 20, 30};
  if (RELAYTIMER_PARAM < 500)
    RELAYTIMER_PARAM = RelayTimerArr[0];
  else if (RELAYTIMER_PARAM >= 500 && RELAYTIMER_PARAM < 1000)  
    RELAYTIMER_PARAM = RelayTimerArr[1];
  else if (RELAYTIMER_PARAM >= 1000 && RELAYTIMER_PARAM < 1500)  
    RELAYTIMER_PARAM = RelayTimerArr[2];
  else if (RELAYTIMER_PARAM >= 1500 && RELAYTIMER_PARAM < 2000)  
    RELAYTIMER_PARAM = RelayTimerArr[3];
  else if (RELAYTIMER_PARAM >= 2000 && RELAYTIMER_PARAM < 2500)  
    RELAYTIMER_PARAM = RelayTimerArr[4];
  else if (RELAYTIMER_PARAM >= 2500 && RELAYTIMER_PARAM < 3000)  
    RELAYTIMER_PARAM = RelayTimerArr[5];
  else if (RELAYTIMER_PARAM >= 3000 && RELAYTIMER_PARAM < 3500)  
    RELAYTIMER_PARAM = RelayTimerArr[6];
  else if (RELAYTIMER_PARAM >= 3500 && RELAYTIMER_PARAM < 4000)  
    RELAYTIMER_PARAM = RelayTimerArr[7];
  else if (RELAYTIMER_PARAM >= 4000)  
    RELAYTIMER_PARAM = RelayTimerArr[8];                        

//SensitivityTimerArr
/*

  if (SENSITIVITY_VALUE < 400)
    SENSITIVITY_VALUE = SensitivityTimerArr[0];
  else if (SENSITIVITY_VALUE >= 400 && SENSITIVITY_VALUE < 800)  
    SENSITIVITY_VALUE = SensitivityTimerArr[1];
  else if (SENSITIVITY_VALUE >= 800 && SENSITIVITY_VALUE < 1200)  
    SENSITIVITY_VALUE = SensitivityTimerArr[2];
  else if (SENSITIVITY_VALUE >= 1200 && SENSITIVITY_VALUE < 1600)  
    SENSITIVITY_VALUE = SensitivityTimerArr[3];
  else if (SENSITIVITY_VALUE >= 1600 && SENSITIVITY_VALUE < 2000)  
    SENSITIVITY_VALUE = SensitivityTimerArr[4];
  else if (SENSITIVITY_VALUE >= 2000 && SENSITIVITY_VALUE < 2400)  
    SENSITIVITY_VALUE = SensitivityTimerArr[5];
  else if (SENSITIVITY_VALUE >= 2400 && SENSITIVITY_VALUE < 2800)  
    SENSITIVITY_VALUE = SensitivityTimerArr[6];
  else if (SENSITIVITY_VALUE >= 2800 && SENSITIVITY_VALUE < 3200)  
    SENSITIVITY_VALUE = SensitivityTimerArr[7];
  else if (SENSITIVITY_VALUE >= 3200 && SENSITIVITY_VALUE < 3600)  
    SENSITIVITY_VALUE = SensitivityTimerArr[8];
  else if (SENSITIVITY_VALUE >= 3600)  
    SENSITIVITY_VALUE = SensitivityTimerArr[9]; 
  */

  Serial.println((String)"Read DipSW Value");

  Serial.println((String)"DIRECTION_PARAM: " + DIRECTION_PARAM);
  Serial.println((String)"RELAYTIMER_PARAM: " + RELAYTIMER_PARAM);
  Serial.println((String)"RELAYTIMING_PARAM: " + RELAYTIMING_PARAM);
  Serial.println((String)"SENSITIVITY_PARAM: " + SENSITIVITY_PARAM);
  Serial.println((String)"BATTERTLEVEL: " + BATTERYLEVEL_PARAM);
  Serial.println((String)"OPERATIONMODE_PARAM: " + OPERATIONMODE_PARAM);
  Serial.println((String)"SENSORERROR_PARAM: " + SENSORERR_PARAM);

  //Serial.println((String)"SENSITIVITY_VALUE: " + SENSITIVITY_VALUE); 
  
  Serial.println("Read DipSwitch Mode finished...");
}

String converter(uint8_t val) {
  char c_str[2];
  sprintf(c_str, "%2d", val);
  return String(c_str);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////-- Arduino Code--////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

#if BLE_COMM
  BLEDevice::init("");
#endif

#if UART_COMM
  Serial2.begin(115200, SERIAL_8N1, RX1, TX1);
#endif

  delay(500);  
  readDipSwitchVal();
  delay(500); 

#if BLE_COMM
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  
  // Specify that we want active scanning and start the
  // scan to run for 5 seconds.

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  //pBLEScan->start(10);
#endif

  pinMode(RelayPin, OUTPUT);
  pinMode(RelayLED, OUTPUT);
  pinMode(ERRLED, OUTPUT);
  pinMode(PowerLED, OUTPUT);

  delay(1000);  
}

void loop() {
  
  if (Relay_Count <= 0 && !Relay_On && Vehicle_Count > 0)
  {
    Relay_Count = 1 * 10;
    Serial.println(Relay_Count);
    Relay_On = true;
    digitalWrite(RelayPin, HIGH);
    digitalWrite(RelayLED, HIGH); 
  } else if (Relay_Count <= 0 && Relay_On)
  {
    digitalWrite(RelayPin, LOW);
    digitalWrite(RelayLED, LOW);
    Relay_On = false;
    Vehicle_Count = Vehicle_Count > 0 ? Vehicle_Count - 1 : 0;
  }
  else if (Relay_Count > 0) {
    Serial.printf("[%d] : ", Vehicle_Count);
    Serial.println(Relay_Count);
    Relay_Count--;
  }

#if BLE_COMM
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    if (sendParam == false)
    {
      delay(500);

      String newValue = (OPERATIONMODE_CMD + ":" + converter(OPERATIONMODE_PARAM) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      pRemoteCharacteristicRx->writeValue(newValue.c_str(), newValue.length());
      delay(500);

      newValue = (DIRECTION_CMD + ":" + converter(DIRECTION_PARAM) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      pRemoteCharacteristicRx->writeValue(newValue.c_str(), newValue.length());
      delay(500);

      newValue = (RELAYTIMER_CMD + ":" + converter(RELAYTIMER_PARAM) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      pRemoteCharacteristicRx->writeValue(newValue.c_str(), newValue.length());
      delay(500);

      newValue = (SENSITIVITY_CMD + ":" + converter(SENSITIVITY_LEVEL_VALUE) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      pRemoteCharacteristicRx->writeValue(newValue.c_str(), newValue.length());
      delay(500);

      newValue = (DIRECTION_CAT_CMD + ":" + converter(DIRECTION_VALUE) + "\n");
      //Serial.println("Setting new characteristic value to \"" + DIRECTION_CAT_CMD + ": " + String(DIRECTION_VALUE, BIN) + "\"");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      pRemoteCharacteristicRx->writeValue(newValue.c_str(), newValue.length());
      delay(500);
     
      sendParam = true;
    }
  }
    
  if(Relay_On)
  {
    pBLEScan->stop();
  } else {
    if (!connected)
    {
      pBLEScan->start(1);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
      delay(50); // Delay a second between loops.
    }
  }

#endif

#if UART_COMM

  if (connected) {
    if (sendParam == false)
    {
      delay(500);

      String newValue = (OPERATIONMODE_CMD + ":" + converter(OPERATIONMODE_PARAM) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      Serial2.write(newValue.c_str());
      delay(500);

      newValue = (DIRECTION_CMD + ":" + converter(DIRECTION_PARAM) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      Serial2.write(newValue.c_str());
      delay(500);

      newValue = (RELAYTIMER_CMD + ":" + converter(RELAYTIMER_PARAM) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      Serial2.write(newValue.c_str());
      delay(500);

      newValue = (SENSITIVITY_CMD + ":" + converter(SENSITIVITY_LEVEL_VALUE) + "\n");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      Serial2.write(newValue.c_str());
      delay(500);

      newValue = (DIRECTION_CAT_CMD + ":" + converter(DIRECTION_VALUE) + "\n");
      //Serial.println("Setting new characteristic value to \"" + DIRECTION_CAT_CMD + ": " + String(DIRECTION_VALUE, BIN) + "\"");
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      Serial2.write(newValue.c_str());
      delay(500);
     
      sendParam = true;
    }
  } else {
    if (Sensor_Started)
    {
      Serial.println("send _mobi-ramp msg to sensor\n");    
      Serial2.write("_mobi-ramp\n");
      delay(1000);
    }
  }

  //Receive UART Msg From mobi-ramp sensor
  Uart_Msg_Length = Serial2.available();

  if( Uart_Msg_Length > 0 )
  {
    char ch;
    memset(&UART_RX_BUF, 0, UART_RX_BUF_SIZE);
    for (int i = 0; i < Uart_Msg_Length; i++)
    {
      Serial2.read(&ch, 1);
      UART_RX_BUF[i] = ch;
    }
    Serial.printf("received %s from sensor\n", UART_RX_BUF);      
    String recv_Str = UART_RX_BUF;

    if (!connected)
    {
      if (recv_Str.indexOf("start") > -1)
      {
        Sensor_Started = true;
        Serial.printf("Sensor_Started is True\n"); 
      }
      else if (recv_Str.indexOf("sensor") > -1)
      {
        connected = true;
        Serial.write("mobi-ramp sensor connected\n");
      }
    } else {
      if (recv_Str.indexOf("start") > -1)
      {
        connected = false;
        sendParam = false;
        Sensor_Started = true;

        digitalWrite(RelayLED, LOW);
        digitalWrite(RelayPin, LOW);
        digitalWrite(ERRLED, LOW);
        digitalWrite(PowerLED, LOW);

        Serial.printf("Sensor_Started is True\n"); 
      } else if (recv_Str.indexOf("sensor") == -1)
      {
        UART_CMD_PROCESSOR((uint8_t *)&UART_RX_BUF[0], Uart_Msg_Length);
      }
    }
  }
#endif

  if (!connected) {    
    if (onoff)
      digitalWrite(PowerLED, LOW);
    else
      digitalWrite(PowerLED, HIGH); 
  } else {
    digitalWrite(PowerLED, HIGH);
  }

  // Err Function
  if (connected) {    
    if(Mobi_Ramp_Sensor0_Error)
    {
      if (onoff){
        digitalWrite(ERRLED, LOW);
        delay(400);
      }
      else{
        digitalWrite(ERRLED, HIGH);
        delay(400);
      }
      Mobi_Ramp_Sensor0_Error_Flag = true;
    }
    else {
      if(Mobi_Ramp_Sensor0_Error_Flag){
        digitalWrite(ERRLED, LOW);
        Mobi_Ramp_Sensor0_Error_Flag = false;
      }
    }
  } else {
    digitalWrite(ERRLED, LOW);
    Mobi_Ramp_Sensor0_Error = false;
  }

  onoff = !onoff;
  delay(100); // Delay a second between loops.
}