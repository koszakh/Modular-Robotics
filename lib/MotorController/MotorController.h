#ifndef MotorController_h
#define MotorController_h
#include "Arduino.h"//стандартная библиотека  Arduino.h
#include <SPI.h>     //библиотека для работы с шиной SPI

class MotorController //создаем класс RadioJoystick
{

private:
	struct motor // создаем объект класса
	{
		// инициализируем  пины для управления двигателям
		uint8_t pin_forward;
		uint8_t pin_back;
		uint8_t pin_speed;
		uint8_t channel;
	};
	motor* MotorRight;
	motor* MotorLeft;
	// функция считывания сигналов c пинов Arduino
	void setPinsValuesMotorDriver (motor* motorStruct, bool up, bool down, short speed);

public:
//Задаем подключеные соответствующие контакты, для правого и левого двигателей, в очередности pinUpRight, pinDownRight, pinSpeedRight, pinUpLeft, pinDownLeft, pinSpeedLeft
	void setup(uint8_t pinEnableR, uint8_t pinForwardR, uint8_t pinBackR , uint8_t pinForwardL, uint8_t pinBackL, uint8_t pinEnableL, uint8_t channel_Right, uint8_t channel_Left);

	void rotation(motor* motorStruct, short Speed, short Side);
	// контроль вращения двигателем на основе данных с джойстика
	void controlByCamera(short correctValue, float reduceSpeed, float reduceSpeedSide);

	void rotateLeft(short correctValue);
	// номер канала
	void rotateRight(short correctValue);

	void stop(short correctValue);

	int channel;

	void setupMotorDriver(uint8_t channel_Right, uint8_t channel_Left, short frequency, uint8_t resolution);

};
#endif
