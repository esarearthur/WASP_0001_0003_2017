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
   
        MCU: #ATMega328P / ATMega1284P
       XTAL: 16MHz
   ADC VREF: 5V
   
 */

#define TINY_GSM_MODEM_SIM900

#include <Arduino.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_MQTT.h>
#include <EEPROM.h>

#define LM35    0x00
#define DS18B20 0x01
#define pH_SENSOR 0x02

#define INTERVAL_5000 5000
#define INTERVAL_10000 10000

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS A1

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature TEMP_PROBE(&oneWire);

// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
// Use Software Serial on Uno, Nano
#if defined(__AVR_ATmega328P__)
	SoftwareSerial SerialAT(2, 3); // RX, TX
	#define Vref 5000
#else
	#define SerialAT Serial1
	#define Vref 1100
#endif

// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 0

// GPRS connection parameters
#define GPRS_APN "internet"
#define GPRS_LOGIN NULL
#define GPRS_PASSWORD NULL

#define ADAFRUIT

// MQTT variable setup
#ifdef ECLIPSE
#define AIO_SERVER "iot.eclipse.org"
#endif
#ifdef ADAFRUIT
#define AIO_SERVER "io.adafruit.com"
#endif
#define AIO_SERVERPORT 1883                  // use 8883 for SSL
#define AIO_USERNAME    "esarearthur"
#define AIO_KEY         "4f111091fbfd45d09c24e5c75df850da"
#define AIO_CHANNEL_P1 AIO_USERNAME "/feeds/wasp_boardTemp"
#define AIO_CHANNEL_P2 AIO_USERNAME "/feeds/wasp_probeTemp"
#define AIO_CHANNEL_P3 AIO_USERNAME "/feeds/wasp_probePH"
#define AIO_CHANNEL_S1 AIO_USERNAME "/feeds/wasp_sub"

// Create modem serial port
TinyGsm modem(SerialAT);

// Create GSM GPRS client
TinyGsmClient client(modem);

// Setup the MQTT client class by passing in the client and MQTT server and login details.
#ifdef ADAFRUIT
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);
#else
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);
#endif

// Topic publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish board_temp_sub = Adafruit_MQTT_Publish(&mqtt, AIO_CHANNEL_P1);
Adafruit_MQTT_Publish probe_temp_sub = Adafruit_MQTT_Publish(&mqtt, AIO_CHANNEL_P2);
Adafruit_MQTT_Publish probe_pH_sub = Adafruit_MQTT_Publish(&mqtt, AIO_CHANNEL_P3);

// Topic subscription.
Adafruit_MQTT_Subscribe client_sub = Adafruit_MQTT_Subscribe(&mqtt, AIO_CHANNEL_S1);

// Settings structure
struct StoreStruct {
	uint8_t version[4];
	uint32_t epoch;
	} storage = {
	CONFIG_VERSION,
	0
};

int ADC_0_measurement;

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
	uint8_t retries = 3U;
	
	// Stop if already connected.
	if (mqtt.connected()) {
		return;
	}
	else{
		modem.sendAT(GF("+CIPCLOSE"));
		modem.waitResponse();
	}
	
	Serial.println("Connecting to MQTT server");
	while((mqtt_conn = mqtt.connect()) != 0) {
		Serial.println(mqtt.connectErrorString(mqtt_conn));
		if(retries > 0) {
			Serial.println("Retrying MQTT connection in 5 seconds...");
			mqtt.disconnect();
			retries--;
			delay(5000);  // wait 5 seconds
		}
		if (retries == 0) {
			Serial.println("MQTT connection failed check parameters. :=(");
			while(1);
		}
	}
	
	Serial.println("MQTT Connected!");
}

float getTemp(char sensor_type){
	float temperature;
	ADC_0_measurement = 0;
	
	switch(sensor_type)
	{
		case LM35:
		ADC_0_measurement = analogRead(A0);
		temperature = ((float)ADC_0_measurement * Vref / (1023.f)) / 10.f;
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
		ADC_0_measurement += analogRead(A2);
	}while(--m);
	
	ADC_0_measurement = ADC_0_measurement / 10;
	return (7 - 1000 * (ADC_0_measurement - 372) * Vref / 59.16 /1023);
}

uint32_t tGetTemperatureMillis = 0;
float MQTTData[3];
void TaskGetTemperature(void)
{
	uint32_t cMillis = millis();
	
	if (cMillis - tGetTemperatureMillis >= INTERVAL_5000)
	{
		MQTTData[0] = getTemp(LM35);
		MQTTData[1] = getTemp(DS18B20);
		tGetTemperatureMillis = cMillis;
	}
}

uint32_t tSampleFluidMillis = 0;
void TaskGetpH(void)
{
	uint32_t cMillis = millis();
	if (cMillis - tSampleFluidMillis >= INTERVAL_10000)
	{
		MQTTData[2] = getPH();
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
			return (totalDays * 86400UL);
		}
	}
	return 0;
}

uint32_t TimeToTicks(uint8_t hour, uint8_t minute, uint8_t second)
{
	uint32_t totalSeconds = ((hour * 3600UL) + (minute * 60U)) + second;
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

		String m = modem.getNetworkTime();
		char charBuf[m.length() + 1];
		m.toCharArray(charBuf, m.length());
	
		sscanf(charBuf, "%d/%d/%d,%d:%d:%d+00", &NTP.Year, &NTP.Month, &NTP.Day, &NTP.Hour, &NTP.Minute, &NTP.Second);
		Serial.println(charBuf);
		
		uint32_t rTimeStamp = GetTimeStamp(NTP.Year, NTP.Month, NTP.Day, NTP.Hour, NTP.Minute, NTP.Second);
	
		if(rTimeStamp >= storage.epoch)
		{	
			char result[7] = "";		 
			dtostrf(MQTTData[0], 3, 1, result);
			if(!board_temp_sub.publish(result))
			{
				Serial.println("Publishing board temperature failed");
				return;
			}
			
			dtostrf(MQTTData[1], 3, 1, result);
			if(!probe_temp_sub.publish(result))
			{
				Serial.println("Publishing probe temperature failed");
				return;
			}
			
			dtostrf(MQTTData[2], 3, 1, result);
			if(!probe_pH_sub.publish(result))
			{
				Serial.println("Publishing probe pH failed");
				return;
			}
			
			storage.epoch = rTimeStamp;
		}
		else
		{
			// ping the server to keep the mqtt connection alive
			// NOT required if you are publishing once every KEEPALIVE seconds
			if(! mqtt.ping()) {
				mqtt.disconnect();
			}
		}
		tPublishDataMillis = cMillis;
	}
}

void setup()
{	
	Serial.begin(9600);
	while(!Serial);
	pinMode(A0, INPUT);
	pinMode(A1, OUTPUT);
	pinMode(A2, INPUT);
	
	#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
	analogReference(DEFAULT);
	#else
	analogReference(INTERNAL1V1);
	#endif
	
	// locate devices on the bus
	TEMP_PROBE.begin();

	SerialAT.begin(9600);
	while(!SerialAT);
	
	delay(1000);
	
	loadConfig();
	Serial.println("Resetting modem");
	while(!modem.factoryDefault());
	
	if(storage.epoch == 0)
	{
		storage.epoch = GetTimeStamp(17, 9, 1, 0, 0, 0);
		saveConfig();
	}

	// Restart takes quite some time
	// To skip it, call init() instead of restart()
	modem.restart();

	Serial.print("Waiting for network ...");
	if (!modem.waitForNetwork()) 
	{
		Serial.println(" ... Fail");
		while (true);
	}
	Serial.println(" OK");
	
	Serial.println("Initiating GPRS connection");
	if (!modem.gprsConnect(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD)) 
	{
		Serial.println(" ... Fail");
		while (true);
	}
	
	Serial.println("Connection established");
	
	mqtt.subscribe(&client_sub);
}

void loop()
{
	// Ensure the connection to the MQTT server is alive (this will make the first
	// connection and automatically reconnect when disconnected).  See the MQTT_connect
	// function definition further below.
	MQTT_Connect();
	
	TaskGetTemperature();
	TaskGetpH();
	TaskPublishData();
}

