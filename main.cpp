/*
  Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016
  https://github.com/sparkfun/MAX30105_Breakout
  This demo shows heart rate and SPO2 levels.
  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.
  This example is based on MAXREFDES117 and RD117_LILYPAD.ino from Maxim. Their example
  was modified to work with the SparkFun MAX30105 library and to compile under Arduino 1.6.11
  Please see license file for more info.
  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

uint8_t txValue = 0;
BLEServer *pServer = NULL;                   //BLEServer指针 pServer
BLECharacteristic *pTxCharacteristic = NULL; //BLECharacteristic指针 pTxCharacteristic
bool deviceConnected = false;                //本次连接状态
bool oldDeviceConnected = false;             //上次连接状态
int MPU6050_Addr = 0x68; 
int XH,XL,YH,YL,ZH,ZL;


MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
#define SERVICE_UUID "12a59900-17cc-11ec-9621-0242ac130002" // UART service UUID
#define CHARACTERISTIC_UUID_RX "12a59e0a-17cc-11ec-9621-0242ac130002"
#define CHARACTERISTIC_UUID_TX "12a5a148-17cc-11ec-9621-0242ac130002"
#define X_Axis_DATAXH 0x3B // Hexadecima address for the DATAX internal register.
#define X_Axis_DATAXL 0x3C 
#define Y_Axis_DATAXH 0x3D 
#define Y_Axis_DATAXL 0x3E 
#define Z_Axis_DATAXH 0x3F 
#define Z_Axis_DATAXL 0x40 

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue(); //接收信息

        if (rxValue.length() > 0)
        { //向串口输出收到的值
            Serial.print("RX: ");
            for (int i = 0; i < rxValue.length(); i++)
                Serial.print(rxValue[i]);
            Serial.println();
        }
    }
};


void setup()
{
  Wire.begin();
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:

	Wire.beginTransmission(MPU6050_Addr);
  Wire.write(0x6B);// Bit D3 High for measuring enable (0000 1000)
  Wire.write(0);  
  Wire.endTransmission(true);


  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  //BLE connection
  // setup BLE 
  BLEDevice::init("ESP32_BLE");

  // setup a BLE service
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks()); 
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // setup BLE characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks()); 

  pService->start();                  
  pServer->getAdvertising()->start(); 
  Serial.println(" waiting for a client connection to notify... ");

}

void loop()
{ 
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

      //BLE
      // deviceConnected 已连接
      if (deviceConnected)
      {   txValue=redBuffer[i];
          pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
          pTxCharacteristic->notify();              // 广播                               // 指针地址自加1
          //delay(2000);                              // 如果有太多包要发送，蓝牙会堵塞
          txValue=irBuffer[i];
          pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
          pTxCharacteristic->notify();

          txValue=heartRate;
          pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
          pTxCharacteristic->notify();

          txValue=spo2;
          pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
          pTxCharacteristic->notify();
      }

      // disconnecting  断开连接
      if (!deviceConnected && oldDeviceConnected)
      {
        delay(500);                  // 留时间给蓝牙缓冲
        pServer->startAdvertising(); // 重新广播
        Serial.println(" 开始广播 ");
        oldDeviceConnected = deviceConnected;
      }

      // connecting  正在连接
      if (deviceConnected && !oldDeviceConnected)
      {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
      }
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    
    //accelerator
    Wire.beginTransmission(MPU6050_Addr); // Begin transmission to the Sensor  
    Wire.write(X_Axis_DATAXH);
    Wire.endTransmission(true); // Ends the transmission and transmits the data from the two registers
    Wire.requestFrom(MPU6050_Addr,6); // Request the transmitted two bytes from the two registers

    if(Wire.available()<=6) 
    {  // 
      XH = Wire.read(); // Reads the data from the register
      XL = Wire.read();   
      YH = Wire.read(); // Reads the data from the register
      YL = Wire.read();   
      ZH = Wire.read(); // Reads the data from the register
      ZL = Wire.read(); 
    }
    Serial.print("XH= ");
    Serial.print(XH);
    Serial.print("XL= ");
    Serial.println(XL);
    Serial.print("YH= ");
    Serial.print(YH);
    Serial.print("YL= ");
    Serial.println(YL);
    Serial.print("ZH= ");
    Serial.print(ZH);
    Serial.print("ZL= ");
    Serial.println(ZL);

    if (deviceConnected)
    {   txValue=XH;
      pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
      pTxCharacteristic->notify();              // 广播                               // 指针地址自加1
      //delay(2000);                              // 如果有太多包要发送，蓝牙会堵塞
      txValue=XL;
      pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
      pTxCharacteristic->notify();

      txValue=YH;
      pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
      pTxCharacteristic->notify();

      txValue=YL;
      pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
      pTxCharacteristic->notify();

      txValue=ZH;
      pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
      pTxCharacteristic->notify();

      txValue=ZL;
      pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
      pTxCharacteristic->notify();
      }

      // disconnecting  断开连接
      if (!deviceConnected && oldDeviceConnected)
      {
        delay(500);                  // 留时间给蓝牙缓冲
        pServer->startAdvertising(); // 重新广播
        Serial.println(" 开始广播 ");
        oldDeviceConnected = deviceConnected;
      }

      // connecting  正在连接
      if (deviceConnected && !oldDeviceConnected)
      {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
      }
  }
}
