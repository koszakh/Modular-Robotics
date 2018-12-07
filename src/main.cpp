#include <Arduino.h>
#include <BLEjoystick.h>
#include "BLEDevice.h"

std::string SERV_UUID = "ad108f46-f88e-11e8-8eb2-f2801f1b9fd1";
std::string CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

uint8_t JoystickxPin = 33;
uint8_t JoystickyPin = 32;
uint8_t SolidPin = 23;

BLEjoystick Joystick;

void setup() {

	//Joystick.SetupUUID(ServiceUUID,CharacteristicUUID);
	Joystick.BLEjoystickSetup();
	//pinMode(JoystickxPin, INPUT_PULLDOWN);
	//pinMode(JoystickyPin, INPUT_PULLDOWN);

} 
void loop() {

	Joystick.GetDataFromJoystick(JoystickxPin, JoystickyPin);

} 