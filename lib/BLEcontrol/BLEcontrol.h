#ifndef BLEcontrol_h
#define BLEcontrol_h
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


class BLEcontrol: public BLECharacteristicCallbacks 
{
    public:
    void onWrite(BLECharacteristic *pCharacteristic); 
    void initialize(void parseBLEData(std::string valueFromJoystick), std::string SERV_UUID, std::string CHAR_UUID);
};

#endif
