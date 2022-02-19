#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "common.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "../lib/ArduinoJson/ArduinoJson.h"
#include <Wire.h>

uint8_t txValue = 0;
BLEServer *pServer = NULL;                   //BLEServer指针 pServer
BLECharacteristic *pTxCharacteristic = NULL; //BLECharacteristic指针 pTxCharacteristic
bool deviceConnected = false;                //本次连接状态
bool oldDeviceConnected = false;             //上次连接状态
int mode=0; //0->normal mode(1 per min); 1->emergency mode(1 per sec)
hw_timer_t *timer = NULL; //counter
//I2C
int MPU6050_Addr = 0x68; 
int MAX30102_Addr= 0x57;
#define X_Axis_DATAXH 0x3B // Hexadecima address for the DATAX internal register.
#define X_Axis_DATAXL 0x3C 
#define Y_Axis_DATAXH 0x3D 
#define Y_Axis_DATAXL 0x3E 
#define Z_Axis_DATAXH 0x3F 
#define Z_Axis_DATAXL 0x40 
//#define Power_Register 0x2D // Power Control Register

int XH,XL,YH,YL,ZH,ZL;

// to generate UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID "12a59900-17cc-11ec-9621-0242ac130002" // UART service UUID
#define CHARACTERISTIC_UUID_RX "12a59e0a-17cc-11ec-9621-0242ac130002"
#define CHARACTERISTIC_UUID_TX "12a5a148-17cc-11ec-9621-0242ac130002"

void IRAM_ATTR diffMode() //enter interrupt and read and forward the data
{
    Serial.println("read here");
}

void changeMode()
{
   mode=1; //change into emergent mode
   Serial.println("enter emergent mode");
}

void WiFi_Connect()
{
	WiFi.begin("HuurinleuvenT25", "ihaveinternet");
	while (WiFi.status() != WL_CONNECTED)
	{ 
		delay(300);
		Serial.print(".");
	}
}

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
{   Wire.begin();
    Serial.begin(115200);
	
	//attach interrupt from sudden falling
	pinMode(32, INPUT_PULLUP);
    attachInterrupt(32, changeMode, FALLING); //GPIO32 is used for accelerator

	//timer->counter
	timer = timerBegin(0, 80000, true); //timer count per 1ms (count up)
	timerAttachInterrupt(timer, &diffMode, true);//trigger by pulse
#if 0==mode //emergent
    timerAlarmWrite(timer, 60000, true); //read per min
#elif 1==mode
    timerAlarmWrite(timer, 1000, true); //read per sec
#endif
    timerAlarmEnable(timer); //	使能定时器

    //wifi connection
	Serial.print("Connecting.. ");
	WiFi_Connect();
	Serial.println("WiFi connected");
    //BLE connection
    // 创建一个 BLE 设备
    BLEDevice::init("UART_BLE");

    // 创建一个 BLE 服务
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); //设置回调
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // 创建一个 BLE 特征
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks()); //设置回调

    pService->start();                  // 开始服务
    pServer->getAdvertising()->start(); // 开始广播
    Serial.println(" waiting... ");
}

void loop()
{
    // deviceConnected 已连接
    if (deviceConnected)
    {
        pTxCharacteristic->setValue(&txValue, 1); // 设置要发送的值为1
        pTxCharacteristic->notify();              // 广播
        txValue++;                                // 指针地址自加1
        delay(2000);                              // 如果有太多包要发送，蓝牙会堵塞
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

    Wire.beginTransmission(MPU6050_Addr); // Begin transmission to the Sensor 
    //Ask the particular registers for data
    Wire.write(X_Axis_DATAXH);
    Wire.write(X_Axis_DATAXL);
    Wire.write(Y_Axis_DATAXH);
    Wire.write(Y_Axis_DATAXL);
    Wire.write(Z_Axis_DATAXH);
    Wire.write(Z_Axis_DATAXL);
  
    Wire.requestFrom(MPU6050_Addr,2,true); // Request the transmitted two bytes from the two registers
    Wire.endTransmission(true); // Ends the transmission and transmits the data from the two registers
  
    if(Wire.available()<=2) 
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
}