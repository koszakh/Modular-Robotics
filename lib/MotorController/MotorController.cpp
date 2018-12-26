#include "MotorController.h"     //библиотека для управления двигателями
#include "Arduino.h"            //стандартная библиотека  Arduino.h
#include <SPI.h>                //библиотека для работы с шиной SPI
#include "esp32-hal-ledc.h" //библиотека ШИМ
#include <cmath>    

//Задаем подключеные соответствующие контакты, для правого и левого двигателей, в очередности pinUpRight, pinDownRight, pinSpeedRight, pinUpLeft, pinDownLeft, pinSpeedLeft
void MotorController::setup(uint8_t pinEnableR, uint8_t pinForwardR, uint8_t pinBackR , uint8_t pinForwardL, uint8_t pinBackL, uint8_t pinEnableL, uint8_t channel_Right, uint8_t channel_Left)
{
	MotorRight = new motor();
	MotorLeft = new motor();
// инициализируем все пины для управления двигателями
	MotorRight->pin_forward = pinForwardR;
	MotorRight->pin_back = pinBackR;
	MotorRight->pin_speed = pinEnableR;
	MotorLeft->pin_forward = pinForwardL;
	MotorLeft->pin_back = pinBackL;
	MotorLeft->pin_speed = pinEnableL;

// обозначаем порты как выходы
	pinMode(MotorRight->pin_forward, OUTPUT);
	pinMode(MotorRight->pin_back, OUTPUT);
	pinMode(MotorRight->pin_speed, OUTPUT);
	pinMode(MotorLeft->pin_forward, OUTPUT);
	pinMode(MotorLeft->pin_back, OUTPUT);
	pinMode(MotorLeft->pin_speed, OUTPUT);



}
// функция задает номер канала, частоту, разрешение
void MotorController::setupMotorDriver(uint8_t channel_Right, uint8_t channel_Left, short frequency, uint8_t resolution)
{
	MotorRight->channel = channel_Right;
	MotorLeft->channel = channel_Left;

	ledcSetup(channel_Right,frequency,resolution);
	ledcSetup(channel_Left,frequency,resolution);
	ledcAttachPin(MotorRight->pin_speed, channel_Right);
    ledcAttachPin(MotorLeft->pin_speed, channel_Left);
}

// функция считывания сигналов c пинов Arduino
void  MotorController::setPinsValuesMotorDriver (motor* motorStruct, bool up, bool down, short speed)
{
    digitalWrite(motorStruct->pin_forward, up);
	digitalWrite(motorStruct->pin_back, down);
	ledcWrite(motorStruct->channel, speed);
	//Serial.println(speed);
}
// эта функция обеспечит вращение двигателей в двух направлениях на установленной скорости
void MotorController::rotation(motor* motorStruct, short Speed, short Side)
{
	//if (v>100) v = 100;
	//if (v<-100) v = -100;
	if (Side>0) {
		setPinsValuesMotorDriver(motorStruct, HIGH, LOW, Speed); //настройки запуска двигателя в одну сторону
	}
	else if (Side<0) {
		setPinsValuesMotorDriver(motorStruct, LOW, HIGH, Speed);//настройки запуска двигателя в обратную  сторону
	}
	else {
		setPinsValuesMotorDriver(motorStruct, LOW, LOW, 0); //двигатель отключен
	}
}
// контроль вращения двигателем на основе данных с джойстика
void MotorController::controlByCamera(short correctValue, float reduceSpeed, float reduceSpeedSide)
{
	rotation(MotorRight, 110, 1);
	rotation(MotorLeft, 110, 1);
	delay(correctValue*12);
	rotation(MotorLeft, 150, 0);
	rotation(MotorRight,150, 0);

	// if (correctValue == 0) {
	// 	rotation(MotorLeft, 150, -1);
	// 	rotation(MotorRight,150, 1);
	// 	//Serial.println("Moving forward");
	// }
	// else if (correctValue == 1) {
	// 	rotation(MotorRight, 110, 1);
	// 	rotation(MotorLeft, 110, 1);
	// 	//Serial.println("Moving right");
	// }
	// else if (correctValue == 2) {
	// 	rotation(MotorRight,150, -1);
	// 	rotation(MotorLeft, 150, -1);
	// 	//Serial.println("Moving left");
	// }
	// else {
	// 	rotation(MotorRight,100, 0);
	// 	rotation(MotorLeft, 100, 0);
	// 	//Serial.println("Stop");
	
	/*if ((xCoord > -500 && xCoord < 500) && yCoord < -500) {

	    rotation(MotorLeft, abs(yCoord/reduceSpeed), -yCoord);
		rotation(MotorRight,abs(yCoord/reduceSpeed), yCoord);
		Serial.println("Moving forward");
	}
	
	else if (xCoord >= 500 && yCoord <= 500) {
		rotation(MotorRight, abs(xCoord/reduceSpeedSide), yCoord);
		rotation(MotorLeft, abs(xCoord/reduceSpeedSide), yCoord);
		Serial.println("Moving right");
	}

	else if (xCoord < -500 && yCoord <= 500) {
		rotation(MotorRight,abs(xCoord/reduceSpeedSide), -yCoord);
		rotation(MotorLeft, abs(xCoord/reduceSpeedSide), -yCoord);
		Serial.println("Moving left");
	}

	else if ((xCoord > -1000 && xCoord < 1000) && yCoord > 50) {
	    rotation(MotorLeft,  abs(yCoord/reduceSpeed), -yCoord);
		rotation(MotorRight, abs(yCoord/reduceSpeed), yCoord);
		Serial.println("Moving back");
	}
	//else if (xCoord > 500 && yCoord >= -500) {
		//rotation(MotorLeft,0/*-yCoord/ (1+(log(10+abs(xCoord)))/4), -yCoord);
		//rotation(MotorRight, yCoord/reduceSpeed, yCoord);
		//Serial.println("Moving back left");
	//}
	//else if (xCoord < -150 && yCoord <= 0) {
	   // rotation(MotorLeft, -yCoord/reduceSpeed, -yCoord);
		//rotation(MotorRight,0/*-yCoord/ (1+(log(10+abs(xCoord)))/4), yCoord);
		//Serial.println("Moving back right");
	//}
	else {
		rotation(MotorLeft, 0, 0);
		rotation(MotorRight, 0, 0);
	}*/
	
}

void MotorController::rotateLeft(short correctValue)
{	
	rotation(MotorRight, 300, 1);
	rotation(MotorLeft, 300, 1);
	delay(120);
	rotation(MotorLeft, 150, 0);
	rotation(MotorRight,150, 0);

}

void MotorController::rotateRight(short correctValue)
{	
	rotation(MotorRight, 300, -1);
	rotation(MotorLeft, 300, -1);
	delay(120);
	// delay(-correctValue*12);
	rotation(MotorLeft, 150, 0);
 	rotation(MotorRight,150, 0);

}

void MotorController::stop(short correctValue)
{
	rotation(MotorLeft, 180, -1);
	rotation(MotorRight, 180, 1);	
}
