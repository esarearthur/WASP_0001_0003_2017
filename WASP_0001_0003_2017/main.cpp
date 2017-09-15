/*
   WASP_0001_0003_2017.cpp

   Created: 9/5/2017 9:23:36 PM
   Author : Ronald Esare Arthur
 
   Copyright (c) Drone Technologies.
   All right reserved.

   This is not a free software and can not be redistributed or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   
        MCU: ATMega328P / #ATMega1284P
       XTAL: 16MHz
   ADC VREF: 5V
   
 */

#define TINY_GSM_MODEM_SIM900

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_MQTT.h>
#include <EEPROM.h>
#include <port.h>
#include <util/delay.h>

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }

#define LM35    0x00
#define DS18B20 0x01
#define pH_SENSOR 0x02

#define INTERVAL_5000 5000
#define INTERVAL_10000 10000

#define Vref 5.00

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS A2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature TEMP_PROBE(&oneWire);

// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
// Use Software Serial on Uno, Nano
#ifdef ARDUINO_AVR_UNO
	SoftwareSerial SerialAT(2, 3); // RX, TX
#else
	#define SerialAT Serial1
#endif

// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 0

// GPRS connection parameters
#define GPRS_APN "internet"
#define GPRS_LOGIN NULL
#define GPRS_PASSWORD NULL

// MQTT variable setup
#define AIO_SERVER "iot.eclipse.org"
#define AIO_SERVERPORT 1883                  // use 8883 for SSL
#define AIO_CHANNEL_SUB "DEV_ID/SUB/DATA"
#define AIO_CHANNEL_PUB "DEV_ID/PUB/DATA"

// Create modem serial port
TinyGsm modem(SerialAT);

// Create GSM GPRS client
TinyGsmClient client(modem);

// Setup the MQTT client class by passing in the client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);

// Topic publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish client_pub = Adafruit_MQTT_Publish(&mqtt, AIO_CHANNEL_PUB);

// Topic subscription.
Adafruit_MQTT_Subscribe client_sub = Adafruit_MQTT_Subscribe(&mqtt, AIO_CHANNEL_SUB);

// Settings structure
struct StoreStruct {
	uint8_t version[4];
	uint32_t epoch;
	} storage = {
	CONFIG_VERSION,
	0
};

uint16_t ADC_0_measurement;

void loadConfig() {
	if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] && EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] && EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
		for (uint16_t t=0; t<sizeof(storage); t++)
			*((uint8_t*)&storage + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {
	for (unsigned int t=0; t<sizeof(storage); t++)
		EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
}

void MQTT_Connect() {
	int8_t mqtt_conn;
	uint8_t retries = 3;
	
	// Stop if already connected.
	if (mqtt.connected()) {
		return;
	}
	
	while((mqtt_conn = mqtt.connect()) != 0) {
		Serial.println(mqtt.connectErrorString(mqtt_conn));
		if(retries > 0) {
			Serial.println("Retrying MQTT connection in 5 seconds...");
			mqtt.disconnect();
		}
		if (retries == 0) {
			Serial.println("MQTT connection failed check parameters. :=(");
			while(1);
		}
		retries--;
		delay(5000);  // wait 5 seconds
	}
	
	Serial.println("MQTT Connected!");
}

float getTemp(char sensor_type){
	float temperature = 0;

	switch(sensor_type)
	{
		case LM35:
		ADC_0_measurement = analogRead(LM35);
		temperature = ((float)ADC_0_measurement * Vref /(1023))/0.01;
		break;
		
		case DS18B20:
		TEMP_PROBE.requestTemperatures();
		temperature = TEMP_PROBE.getTempCByIndex(0);
		break;
	}
	
	return temperature;
}

float getPH()
{
	ADC_0_measurement = 0;
	uint8_t m = 10;
	
	do {
		ADC_0_measurement += analogRead(pH_SENSOR);
	}while(--m);
	
	ADC_0_measurement = ADC_0_measurement / 10;
	return (7 - 1000 * (ADC_0_measurement - 372) * Vref / 59.16 /1023);
}

uint32_t tGetTemperatureMillis = 0;
float g[2];
void TaskGetTemperature(void)
{
	uint32_t cMillis = millis();
	
	if (cMillis - tGetTemperatureMillis >= INTERVAL_5000)
	{
		g[0] = getTemp(LM35);
		g[1] = getTemp(DS18B20);
		tGetTemperatureMillis = cMillis;
	}
}

uint32_t tSampleFluidMillis = 0;
float pH_VALUE;
void TaskGetpH(void)
{
	uint32_t cMillis = millis();
	if (cMillis - tSampleFluidMillis >= INTERVAL_10000)
	{
		pH_VALUE = getPH();
		tSampleFluidMillis = cMillis;
	}
}

static uint16_t DaysToMonth365[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };
static uint16_t DaysToMonth366[] = { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 };
static const uint32_t TicksInMillisecond = 1000L;
static const uint32_t TicksInSecond = TicksInMillisecond / 1000L;

bool IsLeapYear(int year)
{
	if ((year % 4) != 0)
	return false;

	if ((year % 100) == 0)
	return ((year % 400) == 0);

	return true;
}

uint32_t DateToTicks(uint8_t year, uint8_t month, uint8_t day)
{
	if (((year >= 1) && (year <= 9999)) && ((month >= 1) && (month <= 12)))
	{
		uint16_t *daysToMonth = (IsLeapYear(year)) ? DaysToMonth366 : DaysToMonth365;
		if ((day >= 1) && (day <= (daysToMonth[month] - daysToMonth[month - 1])))
		{
			uint16_t previousYear = year - 1;
			uint16_t daysInPreviousYears = ((((previousYear * 365) + (previousYear / 4)) - (previousYear / 100)) + (previousYear / 400));

			uint16_t totalDays = ((daysInPreviousYears + daysToMonth[month - 1]) + day) - 1;
			return (totalDays * 86400);
		}
	}
	return 0;
}

uint32_t TimeToTicks(uint8_t hour, uint8_t minute, uint8_t second)
{
	uint32_t totalSeconds = ((hour * 3600L) + (minute * 60L)) + second;
	//if ((totalSeconds > 0xd6bf94d5e5L) || (totalSeconds < -922337203685L))
	//return (totalSeconds * TicksInSecond);
	return totalSeconds;
}

uint32_t GetTimeStamp(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t milliseconds = 0)
{
	uint32_t timestamp = DateToTicks(year, month, day) + TimeToTicks(hour, minute, second);
	return (timestamp + milliseconds) * TicksInMillisecond;
}

uint32_t tPublishDataMillis = 0;
void TaskPublishData(void)
{
	struct DateTime {
		uint16_t Year, Month, Day, Hour, Minute, Second;
	} NTP = {
		0, 0, 0, 0, 0, 0
	};
	
	uint32_t cMillis = millis();
	if (cMillis - tPublishDataMillis >= INTERVAL_5000)
	{
		if (!modem.gprsConnect(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD)) 
		{
			Serial.println(" fail");
			while (true);
		}
		
		MQTT_Connect();
		
		String m = modem.getNetworkTime();
		char charBuf[m.length() + 1];
		m.toCharArray(charBuf, m.length());
	
		sscanf(charBuf, "%d/%d/%d,%d:%d:%d+00", &NTP.Year, &NTP.Month, &NTP.Day, &NTP.Hour, &NTP.Minute, &NTP.Second);
		Serial.println(charBuf);
		
		uint32_t rTimeStamp = GetTimeStamp(NTP.Year, NTP.Month, NTP.Day, NTP.Hour, NTP.Minute, NTP.Second);
	
		if(rTimeStamp >= storage.epoch)
		{	
 			char payload[40];
 			sprintf(payload, "{\"BTEMP\":\"%.2f\",\"PTEMP\":\"%.2f\",\"pH\":\"%.2f\"}", g[0], g[1], pH_VALUE);
			if (client_pub.publish(payload)) 
			{
				storage.epoch = rTimeStamp + (TimeToTicks(3, 0, 0) * TicksInMillisecond);
				saveConfig();
			}
			else
			{
				Serial.println("Publishing payload failed");
			}
		}
		tPublishDataMillis = cMillis;
	}
}
	
int main(void)
{
	mcu_init();
	init();
	initVariant();

#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) 
	{
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}

void setup()
{	
	/* Enable TIMER0 for delay to work */
	PRR0 &= ~(1 << PRTIM0);
	
	/* Set pin direction to input */
	PORTD_set_pin_dir(0, PORT_DIR_IN);
	/* Turn off pull mode */
	PORTD_set_pin_pull_mode(0, PORT_PULL_OFF);
	/* Set pin direction to output */
	PORTD_set_pin_dir(1, PORT_DIR_OUT);
	/* Set pin level to low */
	PORTD_set_pin_level(1, false);
	/* Enable USART0 */
	PRR0 &= ~(1 << PRUSART0);
	Serial.begin(9600);
	
	TEMP_PROBE.begin();
	
	/* Set pin direction to input */
	PORTD_set_pin_dir(2, PORT_DIR_IN);
	/* Turn off pull mode */
	PORTD_set_pin_pull_mode(2, PORT_PULL_OFF);
	/* Set pin direction to output */
	PORTD_set_pin_dir(3, PORT_DIR_OUT);
	/* Set pin level to low */
	PORTD_set_pin_level(3, false);
	/* Enable USART1 */
	PRR0 &= ~(1 << PRUSART1);
	SerialAT.begin(9600);
	while(!SerialAT);
	
	delay(1000);
	
	loadConfig();
	modem.factoryDefault();
	
	if(storage.epoch == 0)
	{
		storage.epoch = GetTimeStamp(17, 9, 1, 0, 0, 0);
		saveConfig();
	}

	// Restart takes quite some time
	// To skip it, call init() instead of restart()
	modem.restart();

	// Unlock your SIM card with a PIN
	//modem.simUnlock("1234");

	Serial.print("Waiting for network ...");
	if (!modem.waitForNetwork()) 
	{
		Serial.println(" ... Fail");
		while (true);
	}
	Serial.println(" OK");

	if (!modem.gprsConnect(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD)) 
	{
		Serial.println(" ... Fail");
		while (true);
	}
	
	Serial.println("Connection established");
	mqtt.subscribe(&client_sub);
}

/*Adafruit_MQTT_Subscribe *subscription;*/
void loop()
{
	TaskGetTemperature();
	TaskGetpH();
	TaskPublishData();
}

