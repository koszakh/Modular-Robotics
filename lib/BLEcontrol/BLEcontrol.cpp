#include <BLEcontrol.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
BLECharacteristic *pCharacteristic1;
void (*BLEDataProcessing) (std::string);


void BLEcontrol::initialize(void parseBLEData(std::string valueFromJoystick), std::string SERV_UUID, std::string CHAR_UUID)
{
  #define SERVICE_UUID        SERV_UUID
  #define CHARACTERISTIC_UUID CHAR_UUID
  BLEDataProcessing = parseBLEData;
  BLEDevice::init("MyESP");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE 
                                       );

  pCharacteristic->setCallbacks(new BLEcontrol());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();
}


void BLEcontrol::onWrite(BLECharacteristic *pCharacteristic)
{
  std::string valueFromJoystick = pCharacteristic->getValue();
  BLEDataProcessing(valueFromJoystick);
}


