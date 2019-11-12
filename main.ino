/* Including needed libraries and declaring global variables */

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Kalman.h"

static const int RXPin = 8, TXPin = 9;
static const uint32_t GPSBaud = 38400;

Kalman kalmanX;
Kalman kalmanY;

uint8_t IMUAddress = 0x68;

TinyGPSPlus gps;

int16_t gyroX; int16_t gyroX1;
int16_t gyroY; int16_t gyroY1;
int16_t gyroZ;

double gyroXangle = 180; double gyroX1angle = 180; // Angle calculate using the gyro
double gyroYangle = 180; double gyroY1angle = 180;

double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;

uint32_t timer1;

SoftwareSerial ss(3, 4);
SoftwareSerial mySerial(RXPin, TXPin);

int v1;

byte strttmr = 1;
byte warncounter = 0;
byte cond = 0;
byte condvf = 0;// unblocked default



void setup() {

	Wire.begin();

	Serial.begin(19200);

	ss.begin(GPSBaud);

	mySerial.begin(19200);  //Скорость порта для связи Arduino с GSM модулем

	mySerial.println("AT");

	i2cWrite(0x6B,0x00); // Disable sleep mode

	kalmanX.setAngle(180); // Set starting angle

	kalmanY.setAngle(180);

	timer1 = micros();

}


void warning() {

	unsigned long timer;

	float lat = (gps.location.lat(), gps.location.isValid(), 11, 6);

	float lng = (gps.location.lng(), gps.location.isValid(), 12, 6);

	if (strttmr != 0)
	    timer = millis();

	if (((millis() - timer) > 1000000) || (warncounter == 0)) {   // Appr. 17 minutes

		String latlon = String(lat,6) + String(' ') + String(lng,6) ;

		sms(latlon,"+79125143223");   // Sending current latitude and longtitude by SMS
	}

	warncounter = 1;

	strttmr = 0;

}


void blocked() {

	 // Checking gyro values, starting to use GPS in case of big difference between previous and current

	 uint8_t* data = i2cRead(0x3B,14);

	 gyroX1 = ((data[8] << 8) | data[9]);

	 gyroY1 = ((data[10] << 8) | data[11]);

	 double gyroXrate = (double)gyroX1 / 131.0;

	 double gyroYrate = -((double)gyroY1 / 131.0);

	 gyroX1angle += kalmanX.getRate() * ((double)(micros() - timer1) / 1000000); // Calculating gyro angle using the unbiased rate

	 gyroY1angle += kalmanY.getRate() * ((double)(micros() - timer1) / 1000000);

	 if ((abs(gyroX1angle - gyroXangle) > 45) || (abs(gyroY1angle - gyroYangle) > 45)) {
	     cond = 0;
	     warncounter = 0;
	     strttmr = 1;
	 }

}



void loop() {

	if (mySerial.available())
	    Serial.write(mySerial.read());

	if (Serial.available())
	    mySerial.write(Serial.read());

	uint8_t* data = i2cRead(0x3B,14);

	gyroX = ((data[8] << 8) | data[9]);

	gyroY = ((data[10] << 8) | data[11]);

	gyroZ = ((data[12] << 8) | data[13]);

	double gyroXrate = (double)gyroX / 131.0;

	double gyroYrate = -((double)gyroY / 131.0);

	gyroXangle += kalmanX.getRate() * ((double)(micros() - timer1) / 1000000); // Calculate gyro angle using the unbiased rate

	gyroYangle += kalmanY.getRate() * ((double)(micros() - timer1) / 1000000);

	kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer1) / 1000000); // Calculate the angle using a Kalman filter

	kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer1) / 1000000);

	timer1 = micros();

	v1 = digitalRead(2);

  	if (v1 == 1) {

	   if (cond == 0)
	       cond = 2;

	   if (cond == 2)
	       cond = 0;

	   if (cond == 1) {
	       cond = 0;
	       warncounter = 0;
	       strttmr = 1;
	   }

	}

	switch(cond) {

	    case 1:	warning();
	     	break;

	    case 2: if (condvf == 0) { sms("Blocked","+79125143223"); condvf = 2; }
		 	blocked();
	     	break;

	    default: if (condvf == 2) { sms("Unblocked","+79125143223"); condvf = 0; }
	    	break;

	  }

	 /*
	 printInt(gps.satellites.value(), gps.satellites.isValid(), 5);

	 printInt(gps.hdop.value(), gps.hdop.isValid(), 5);

	 printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);

	 printFloat(gps.location.lng(), gps.location.isValid(), 12, 6); */

}


void sms(String text, String phone) {

	Serial.println("SMS send started");

	mySerial.println("AT+CMGS=\"" + phone + "\"");

	delay(1000);

	mySerial.print(text);

	delay(300);

	mySerial.print((char)26);

	delay(300);

	Serial.println("SMS send finished");

	delay(3000);

}


void i2cWrite(uint8_t registerAddress, uint8_t data){

	Wire.beginTransmission(IMUAddress);

  	Wire.write(registerAddress);

  	Wire.write(data);

  	Wire.endTransmission(); // Stop sending

}


uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {

 	uint8_t data[nbytes];

 	Wire.beginTransmission(IMUAddress);

 	Wire.write(registerAddress);

 	Wire.endTransmission(false); // Don't release the bus

 	Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading

 	for (uint8_t i = 0; i < nbytes; ++i)
    	data [i] = Wire.read();

  	return data;

}
