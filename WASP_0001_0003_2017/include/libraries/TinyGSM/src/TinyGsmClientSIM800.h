/**
 * @file       TinyGsmClientSIM800.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef TinyGsmClientSIM800_h
#define TinyGsmClientSIM800_h

//#define TINY_GSM_DEBUG Serial
//#define TINY_GSM_USE_HEX

#if !defined(TINY_GSM_RX_BUFFER)
  #define TINY_GSM_RX_BUFFER 100
#endif

#define TINY_GSM_MUX_COUNT 5

#include <TinyGsmCommon.h>

#define RESP_OK 1
#define RESP_ERR 2

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;

enum SimStatus {
	SIM_ERROR = 0,
	SIM_READY = 1,
	SIM_LOCKED = 2,
};

enum RegStatus {
	REG_UNREGISTERED = 0,
	REG_SEARCHING    = 2,
	REG_DENIED       = 3,
	REG_OK_HOME      = 1,
	REG_OK_ROAMING   = 5,
	REG_UNKNOWN      = 4,
};

class TinyGsm
{
/*
GSMClient Class
*/
public:
	class GsmClient : public Client
	{
		friend class TinyGsm;
		typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

	public:
		GsmClient() {}

		GsmClient(TinyGsm& modem, uint8_t mux = 1) {
			init(&modem, mux);
		}

		bool init(TinyGsm* modem, uint8_t mux = 1) {
			this->at = modem;
			this->mux = mux;
			sock_available = 0;
			prev_check = 0;
			sock_connected = false;
			got_data = false;

			at->sockets[mux] = this;

			return true;
		}

		virtual int connect(const char *host, uint16_t port) {
			TINY_GSM_YIELD();
			rx.clear();
			sock_connected = at->modemConnect(host, port, mux);
			return sock_connected;
		}

		virtual int connect(IPAddress ip, uint16_t port) {
			String host; host.reserve(16);
			host += ip[0];
			host += ".";
			host += ip[1];
			host += ".";
			host += ip[2];
			host += ".";
			host += ip[3];
			return connect(host.c_str(), port);
		}

		virtual void stop() {
			TINY_GSM_YIELD();
			at->sendAT(GF("+CIPCLOSE="), mux);
			sock_connected = false;
			at->waitResponse();
		}

		virtual size_t write(const uint8_t *buf, size_t size) {
			TINY_GSM_YIELD();
			at->maintain();
			return at->modemSend(buf, size, mux);
		}

		virtual size_t write(uint8_t c) {
			return write(&c, 1);
		}

		virtual int available() {
		TINY_GSM_YIELD();
		if (sock_connected && !rx.size()) {
			// Workaround: sometimes SIM800 forgets to notify about data arrival.
			// TODO: Currently we ping the module periodically,
			// but maybe there's a better indicator that we need to poll
			if (millis() - prev_check > 500) {
				got_data = true;
				prev_check = millis();
			}
			at->maintain();
		}
		return rx.size() + sock_available;
		}

		virtual int read(uint8_t *buf, size_t size) {
		TINY_GSM_YIELD();
		at->maintain();
		size_t cnt = 0;
		while (cnt < size) {
			size_t chunk = TinyGsmMin(size-cnt, rx.size());
			if (chunk > 0) {
				rx.get(buf, chunk);
				buf += chunk;
				cnt += chunk;
				continue;
			}
			// TODO: Read directly into user buffer?
			at->maintain();
			if (sock_available > 0) {
				at->modemRead(rx.free(), mux);
			} else {
				break;
			}
		}
		return cnt;
		}

		virtual int read() {
			uint8_t c;
			if (read(&c, 1) == 1) {
				return c;
			}
			return -1;
		}

		virtual int peek() { return -1; } //TODO
		virtual void flush() { at->stream.flush(); }

		virtual uint8_t connected() {
			if (available()) {
				return true;
			}
			return sock_connected;
		}
		virtual operator bool() { return connected(); }
	  
	private:
		TinyGsm*      at;
		uint8_t       mux;
		uint16_t      sock_available;
		uint32_t      prev_check;
		bool          sock_connected;
		bool          got_data;
		RxFifo        rx;
	};
/*
End GSMClient Class
*/


/*
TinyGSM Class
*/
public:
	TinyGsm(Stream& stream) : stream(stream)
	{
		setCommandCounter(1);
		memset(sockets, 0, sizeof(sockets));
	}
	
	/** Get command counter
		@return command counter
		*/
	uint8_t getCommandCounter(){return commandCounter;};
		
	/** Set command counter
		@param c			Initial value
		*/
	void setCommandCounter(uint8_t c){commandCounter=c;};

	/**
		Basic functions
		*/
	bool begin() {
		return init();
	}
	
	/** Initialize modem
		@return true or false
		*/
	bool init() {
		if (!autoBaud()) {
			return false;
		}
		sendAT(GF("&FZ"));  // Factory + Reset
		waitResponse();
		sendAT(GF("E0"));   // Echo Off
		if (waitResponse() != RESP_OK) {
			return false;
		}
		getSimStatus();
		return true;
	}
	
	/** Set modem as auto baud rate
		@param timeout			timeout value
		@return true or false
		*/
	bool autoBaud(unsigned long timeout = 10000UL) {
		//streamWrite(GF("AAAAA" GSM_NL));  // TODO: extra A's to help detect the baud rate
		for (unsigned long start = millis(); millis() - start < timeout; ) {
			sendAT(GF(""));
			if (waitResponse(200UL) == 1) {
				delay(100);
				return true;
			}
			delay(100);
		}
		return false;
	}

	void maintain() {
		for (int mux = 0; mux < TINY_GSM_MUX_COUNT; mux++) {
			GsmClient* sock = sockets[mux];
			if (sock && sock->got_data) {
				sock->got_data = false;
				sock->sock_available = modemGetAvailable(mux);
			}
		}
		while (stream.available()) {
			waitResponse(10UL, NULL, NULL);
		}
	}

	bool factoryDefault() {
		uint8_t cc = getCommandCounter();
		uint8_t commandDone = false;
		switch(cc){
			case 1:
			sendAT(GF("&FZE0&W"));  // Factory + Reset + Echo Off + Write
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(2);
			}
			break;
			
			case 2:
			sendAT(GF("E&W")); 
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(3);
			}
			break;
			
			case 3:
			sendAT(GF("E&W")); 
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(4);
			}
			break;
			
			case 4:
			sendAT(GF("+IPR=0")); 
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(5);
			}
			break;
			
			case 5:
			sendAT(GF("+IFC=0,0")); 
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(6);
			}
			break;
			
			case 6:
			sendAT(GF("+ICF=3,3")); 
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(7);
			}
			break;
			
			case 7:
			sendAT(GF("+CSCLK=0"));
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(8);
			}
			break;
			
			case 8:
			sendAT(GF("+CLTS=1"));
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(9);
			}else{
				Serial.println("Mobile network does not support NITZ");
				setCommandCounter(11);
			}
			break;
			
			case 9:
			sendAT(GF("+CENG=3"));
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(10);
			}
			break;
			
			case 10:
			sendAT(GF("+CTZU=1"));
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(11);
			}
			break;
			
			case 11:
			sendAT(GF("&W"));
			if (waitResponse(3000UL) == RESP_OK) {
				setCommandCounter(1);
				commandDone = true;
			}
			break;
		}
		return commandDone;
	}

  /*
   * Power functions
   */

	bool restart() {
		if (!autoBaud()) {
			return false;
		}
		sendAT(GF("+CFUN=0"));
		if (waitResponse(10000L) != RESP_OK) {
			return false;
		}
		sendAT(GF("+CFUN=1,1"));
		if (waitResponse(10000L) != RESP_OK) {
			return false;
		}
		delay(3000);
		return init();
	}

	bool radioOff() {
		if (!autoBaud()) {
			return false;
		}
		sendAT(GF("+CFUN=0"));
		if (waitResponse(10000L) != RESP_OK) {
			return false;
		}
		delay(3000);
		return true;
	}

  /*
   * SIM card functions
   */

	bool simUnlock(const char *pin) {
		sendAT(GF("+CPIN=\""), pin, GF("\""));
		return waitResponse() == RESP_OK;
	}

	String getSimCCID() {
		sendAT(GF("+ICCID"));
		if (waitResponse(GF(GSM_NL "+ICCID:")) != RESP_OK) {
			return "";
		}
		String res = stream.readStringUntil('\n');
		waitResponse();
		res.trim();
		return res;
	}

	String getIMEI() {
		sendAT(GF("+GSN"));
		if (waitResponse(GF(GSM_NL)) != RESP_OK) {
			return "";
		}
		String res = stream.readStringUntil('\n');
		waitResponse();
		res.trim();
		return res;
	}

	SimStatus getSimStatus(unsigned long timeout = 10000L) {
		for (unsigned long start = millis(); millis() - start < timeout; ) {
			sendAT(GF("+CPIN?"));
			if (waitResponse(GF(GSM_NL "+CPIN:")) != RESP_OK) {
				delay(1000);
				continue;
			}
			int status = waitResponse(GF("READY"), GF("SIM PIN"), GF("SIM PUK"), GF("NOT INSERTED"));
			waitResponse();
			switch (status) {
				case 2:
				case 3:  return SIM_LOCKED;
				case 1:  return SIM_READY;
				default: return SIM_ERROR;
			}
		}
		return SIM_ERROR;
	}

	RegStatus getRegistrationStatus() {
	sendAT(GF("+CREG?"));
	if (waitResponse(GF(GSM_NL "+CREG:")) != RESP_OK) {
		return REG_UNKNOWN;
	}
	streamSkipUntil(','); // Skip format (0)
	int status = stream.readStringUntil('\n').toInt();
	waitResponse();
	return (RegStatus)status;
	}

	String getOperator() {
	sendAT(GF("+COPS?"));
	if (waitResponse(GF(GSM_NL "+COPS:")) != RESP_OK) {
		return "";
	}
	streamSkipUntil('"'); // Skip mode and format
	String res = stream.readStringUntil('"');
	waitResponse();
	return res;
	}

  /*
   * Generic network functions
   */

	int getSignalQuality() {
		sendAT(GF("+CSQ"));
		if (waitResponse(GF(GSM_NL "+CSQ:")) != RESP_OK) {
			return 99;
		}
		int res = stream.readStringUntil(',').toInt();
		waitResponse();
		return res;
	}

	bool waitForNetwork(unsigned long timeout = 60000L) {
		for (unsigned long start = millis(); millis() - start < timeout; ) {
			RegStatus s = getRegistrationStatus();
			if (s == REG_OK_HOME || s == REG_OK_ROAMING) {
				return true;
			}
			delay(1000);
		}
		return false;
	}

  /*
   * GPRS functions
   */
	bool gprsConnect(const char* apn, const char* user, const char* pwd) {
	sendAT(GF("+CIFSR;E0"));
	String res;
	if (waitResponse(5000L) != 2) {
		return true;
	}

	gprsDisconnect();

	sendAT(GF("+SAPBR=3,1,\"Contype\",\"GPRS\""));
	waitResponse();

	sendAT(GF("+SAPBR=3,1,\"APN\",\""), apn, '"');
	waitResponse();

	if (user) {
		sendAT(GF("+SAPBR=3,1,\"USER\",\""), user, '"');
		waitResponse();
	}
	if (pwd) {
		sendAT(GF("+SAPBR=3,1,\"PWD\",\""), pwd, '"');
		waitResponse();
	}

	sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"');
	waitResponse();

	sendAT(GF("+CGACT=1,1"));
	waitResponse(60000L);

	// Open a GPRS context
	sendAT(GF("+SAPBR=1,1"));
	waitResponse(85000L);
	// Query the GPRS context
	sendAT(GF("+SAPBR=2,1"));
	if (waitResponse(30000L) != 1)
		return false;

	sendAT(GF("+CGATT=1"));
	if (waitResponse(60000L) != 1)
		return false;

	// TODO: wait AT+CGATT?

	sendAT(GF("+CIPMUX=1"));
	if (waitResponse() != 1) {
		return false;
	}

	sendAT(GF("+CIPQSEND=1"));
	if (waitResponse() != 1) {
		return false;
	}

	sendAT(GF("+CIPRXGET=1"));
	if (waitResponse() != 1) {
		return false;
	}

	sendAT(GF("+CSTT=\""), apn, GF("\",\""), user, GF("\",\""), pwd, GF("\""));
	if (waitResponse(60000L) != 1) {
		return false;
	}

	sendAT(GF("+CIICR"));
	if (waitResponse(60000L) != 1) {
		return false;
	}

	sendAT(GF("+CIFSR;E0"));
	String data;
	if (waitResponse(10000L, data) != 1) {
		data.replace(GSM_NL, "");
		return false;
	}

	sendAT(GF("+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\""));
	if (waitResponse() != 1) {
		return false;
	}

	return true;
	}

	bool gprsDisconnect() {
		sendAT(GF("+CIPSHUT"));
		return waitResponse(60000L) == 1;
	}

  /*
   * Phone Call functions
   */

	bool setGsmBusy(bool busy = true) {
		sendAT(GF("+GSMBUSY="), busy ? 1 : 0);
		return waitResponse() == 1;
	}

	bool callAnswer() {
		sendAT(GF("A"));
		return waitResponse() == 1;
	}

	bool callNumber(const String& number) {
		sendAT(GF("D"), number);
		return waitResponse() == 1;
	}

	bool callRedial() {
		sendAT(GF("DL"));
		return waitResponse() == 1;
	}

	bool callHangup(const String& number) {
		sendAT(GF("H"), number);
		return waitResponse() == 1;
	}

  /*
   * Messaging functions
   */

	void sendUSSD() TINY_GSM_ATTR_NOT_IMPLEMENTED;

	bool sendSMS(const String& number, const String& text) {
		sendAT(GF("+CMGF=1"));
		waitResponse();
		sendAT(GF("+CMGS=\""), number, GF("\""));
		if (waitResponse(GF(">")) != 1) {
			return false;
		}
		stream.print(text);
		stream.write((char)0x1A);
		stream.flush();
		return waitResponse(60000L) == 1;
	}

	bool sendSMS_UTF16(const String& number, const void* text, size_t len) {
		sendAT(GF("+CMGF=1"));
		waitResponse();
		sendAT(GF("+CSCS=\"HEX\""));
		waitResponse();
		sendAT(GF("+CSMP=17,167,0,8"));
		waitResponse();

		sendAT(GF("+CMGS=\""), number, GF("\""));
		if (waitResponse(GF(">")) != 1) {
			return false;
		}

		uint16_t* t = (uint16_t*)text;
		for (size_t i=0; i<len; i++) {
			uint8_t c = t[i] >> 8;
			if (c < 0x10) { stream.print('0'); }
			stream.print(c, HEX);
			c = t[i] & 0xFF;
			if (c < 0x10) { stream.print('0'); }
			stream.print(c, HEX);
		}
		stream.write((char)0x1A);
		stream.flush();
		return waitResponse(60000L) == 1;
	}

  /*
   * Network time function
   */
  
	bool syncNetworkTime() {
		sendAT(GF("+SAPBR=1,1"));
		waitResponse(85000L);
		
		sendAT(GF("+CNTPCID=1"));
		waitResponse(85000L);
		
		sendAT(GF("+CNTP=\"pool.ntp.org\",0"));
		waitResponse(85000L);
		
		sendAT(GF("+CNTP"));
		return waitResponse(60000L) == true;
	}
	
	String getNetworkTime() {
		sendAT(GF("+CCLK?"));
		if (waitResponse(5000L, GF(GSM_NL "+CCLK: \"")) != 1) {
			return "";
		}
		String res = stream.readStringUntil('\n');
		waitResponse();
		res.trim();
		return res;
	}

  /*
   * Location functions
   */

	String getGsmLocation() {
		sendAT(GF("+CIPGSMLOC=1,1"));
		if (waitResponse(10000L, GF(GSM_NL "+CIPGSMLOC:")) != 1) {
			return "";
		}
		String res = stream.readStringUntil('\n');
		waitResponse();
		res.trim();
		return res;
	}

  /*
   * Battery functions
   */
  // Use: float vBatt = modem.getBattVoltage() / 1000.0;
	uint16_t getBattVoltage() {
		sendAT(GF("+CBC"));
		if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
			return 0;
		}
		streamSkipUntil(','); // Skip
		streamSkipUntil(','); // Skip

		uint16_t res = stream.readStringUntil(',').toInt();
		waitResponse();
		return res;
	}

	int getBattPercent() {
		if (!autoBaud()) {
			return false;
		}
		sendAT(GF("+CBC"));
		if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
			return false;
		}
		stream.readStringUntil(',');
		int res = stream.readStringUntil(',').toInt();
		waitResponse();
		return res;
	}

private:
	// Counts the steps by the command
	uint8_t commandCounter;
	
	int modemConnect(const char* host, uint16_t port, uint8_t mux) {
		sendAT(GF("+CIPSTART="), mux, ',', GF("\"TCP"), GF("\",\""), host, GF("\","), port);
		int rsp = waitResponse(75000L,
		GF("CONNECT OK" GSM_NL),
		GF("CONNECT FAIL" GSM_NL),
		GF("ALREADY CONNECT" GSM_NL));
		return (1 == rsp);
	}

	int modemSend(const void* buff, size_t len, uint8_t mux) {
		sendAT(GF("+CIPSEND="), mux, ',', len);
		if (waitResponse(GF(">")) != 1) {
			return -1;
		}
		stream.write((uint8_t*)buff, len);
		stream.flush();
		if (waitResponse(GF(GSM_NL "DATA ACCEPT:")) != 1) {
			return -1;
		}
		streamSkipUntil(','); // Skip mux
		return stream.readStringUntil('\n').toInt();
	}

	size_t modemRead(size_t size, uint8_t mux) {
		#ifdef TINY_GSM_USE_HEX
		sendAT(GF("+CIPRXGET=3,"), mux, ',', size);
		if (waitResponse(GF("+CIPRXGET:")) != 1) {
			return 0;
		}
		#else
		sendAT(GF("+CIPRXGET=2,"), mux, ',', size);
		if (waitResponse(GF("+CIPRXGET:")) != 1) {
			return 0;
		}
		#endif
		streamSkipUntil(','); // Skip mode 2/3
		streamSkipUntil(','); // Skip mux
		size_t len = stream.readStringUntil(',').toInt();
		sockets[mux]->sock_available = stream.readStringUntil('\n').toInt();

		for (size_t i=0; i<len; i++) {
			#ifdef TINY_GSM_USE_HEX
				while (stream.available() < 2) { TINY_GSM_YIELD(); }
				char buf[4] = { 0, };
				buf[0] = stream.read();
				buf[1] = stream.read();
				char c = strtol(buf, NULL, 16);
			#else
				while (!stream.available()) { TINY_GSM_YIELD(); }
				char c = stream.read();
			#endif
			sockets[mux]->rx.put(c);
		}
		waitResponse();
		return len;
	}

	size_t modemGetAvailable(uint8_t mux) {
		sendAT(GF("+CIPRXGET=4,"), mux);
		size_t result = 0;
		if (waitResponse(GF("+CIPRXGET:")) == 1) {
			streamSkipUntil(','); // Skip mode 4
			streamSkipUntil(','); // Skip mux
			result = stream.readStringUntil('\n').toInt();
			waitResponse();
		}
		if (!result) {
			sockets[mux]->sock_connected = modemGetConnected(mux);
		}
		return result;
	}

	bool modemGetConnected(uint8_t mux) {
		sendAT(GF("+CIPSTATUS="), mux);
		int res = waitResponse(GF(",\"CONNECTED\""), GF(",\"CLOSED\""), GF(",\"CLOSING\""), GF(",\"INITIAL\""));
		waitResponse();
		return 1 == res;
	}

public:
	/* Utilities */

	template<typename T>
	void streamWrite(T last) {
		stream.print(last);
	}

	template<typename T, typename... Args>
	void streamWrite(T head, Args... tail) {
		stream.print(head);
		streamWrite(tail...);
	}

	bool streamSkipUntil(char c) { //TODO: timeout
		while (true) {
			while (!stream.available()) { TINY_GSM_YIELD(); }
			if (stream.read() == c)
			return true;
		}
		return false;
	}

	template<typename... Args>
	void sendAT(Args... cmd) {
		streamWrite("AT", cmd..., GSM_NL);
		stream.flush();
		TINY_GSM_YIELD();
		#ifdef TINY_GSM_DEBUG
		DBG("AT", cmd...);
		#endif
	}

	// TODO: Optimize this!
	uint8_t waitResponse(uint32_t timeout, String& data,
						GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
						GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
	{
		/*String r1s(r1); r1s.trim();
		String r2s(r2); r2s.trim();
		String r3s(r3); r3s.trim();
		String r4s(r4); r4s.trim();
		String r5s(r5); r5s.trim();
		DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
		data.reserve(64);
		int index = 0;
		unsigned long startMillis = millis();
		do {
			TINY_GSM_YIELD();
			while (stream.available() > 0) {
				char a = stream.read();
				if (a <= 0) continue; // Skip 0x00 bytes, just in case
				data += a;
				if (r1 && data.endsWith(r1)) {
					index = 1;
					goto finish;
				} else if (r2 && data.endsWith(r2)) {
					index = 2;
					goto finish;
				} else if (r3 && data.endsWith(r3)) {
					index = 3;
					goto finish;
				} else if (r4 && data.endsWith(r4)) {
					index = 4;
					goto finish;
				} else if (r5 && data.endsWith(r5)) {
					index = 5;
					goto finish;
				} else if (data.endsWith(GF(GSM_NL "+CIPRXGET:"))) {
					String mode = stream.readStringUntil(',');
					if (mode.toInt() == 1) {
						int mux = stream.readStringUntil('\n').toInt();
					
						if (mux >= 0 && mux < TINY_GSM_MUX_COUNT) {
							sockets[mux]->got_data = true;
						}
						data= "";
					} else {
						data += mode;
					}
				} else if (data.endsWith(GF("CLOSED" GSM_NL))) {
					int nl = data.lastIndexOf(GSM_NL, data.length()-8);
					int coma = data.indexOf(',', nl+2);
					int mux = data.substring(nl+2, coma).toInt();
				
					if (mux >= 0 && mux < TINY_GSM_MUX_COUNT) {
						sockets[mux]->sock_connected = false;
					}
					data = "";
				}
			}
		} while (millis() - startMillis < timeout);
		finish:
		if (!index) {
			data.trim();
			if (data.length()) {
				#ifdef TINY_GSM_DEBUG
				DBG("### Unhandled:", data);
				#endif
			}
			data = "";
		}
		return index;
	}

	uint8_t waitResponse(uint32_t timeout,
						GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
						GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
	{
		String data;
		return waitResponse(timeout, data, r1, r2, r3, r4, r5);
	}

	uint8_t waitResponse(GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
						GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
	{
		return waitResponse(1000, r1, r2, r3, r4, r5);
	}

private:
	Stream&       stream;
	GsmClient*    sockets[TINY_GSM_MUX_COUNT];
	
/*
End TinyGSM Class
*/
};

typedef TinyGsm::GsmClient TinyGsmClient;

#endif
