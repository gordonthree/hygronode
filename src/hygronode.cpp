#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h> // be sure to use the forked version

#define CLS "\033[2J"
#define HOME "\033[H"


// WiFi network setup - put your password and network name before,
// or include them in a file named WiFiSETUP.h

// #define WIFIPASSWORD "xxxxxxxxxx" // your WiFi password
// #define WIFISSID "Your SSID here" // your WiFi network

#ifndef WIFIPASSWORD // password not defined, load it from external file
#include <WiFiSETUP.h> // private wifi network details defined here
#endif

#ifndef HASDOW
#define HASDOW 0 // device has 1-wire device attached
#define DOWPIN 4 // 1-wire data pin
#define DOWPWR 5 // 1-wire power pin
#endif

#ifndef pinSDA
#define pinSDA 12 // i2c data pin
#define pinSCL 13 // i2c clock oin
#endif

#ifndef i2cPWR
#define i2cPWR 14 // i2c bus power control pin
#endif

#define ADC 0x48
#define ADCPWR 0x20
#define PUMP 0x21
#define OFF 0x0
#define ON 0x1
#define IODIR 0x00
#define GPIO 0x09

const char* mqtt_server = "mypi3";
const char* mqttAnnounce = "home/msg"; // general messages
const char* mqttPub = "home/hygro/livingroom_window/msg"; // general messages
const char* mqttSub = "home/hygro/livingroom_window/cmd"; // general commands
const char* mqttTemperature = "home/hygro/livingroom_window/temperature"; // messages
const char* mqttBattery = "home/hygro/livingroom_window/battery"; // messages
const char* mqttRSSI = "home/hygro/livingroom_window/rssi"; // messages
const char* mqttADC = "home/hygro/livingroom_window/probe"; // messages
const char* nodeName = "livingroom_window"; // hostname

uint16_t lastReconnectAttempt = 0;
uint16_t lastMsg = 0;
char msg[200];
char msgTemp[10];
char msgRSSI[10];
char msgAccel[24];
char str[60];
uint8_t i2cbuff[30];
int16_t adc[4]={0,0,0,0};


char devId[6];
double celsius;
bool serialDebug = true; // yes serial debugging
bool hasTout = false; // no temp out right now
bool hasRSSI = true; // report our signal strength
bool setPolo = false; bool doReset = false; bool getTime = false;
bool useMQTT = false; // flag for mqtt available
boolean skipSleep = false;

Adafruit_ADS1115 ads(ADC);
WiFiClient espClient;
PubSubClient mqtt(espClient);
#ifdef HASDOW
OneWire oneWire(DOWPIN);
DallasTemperature ds18b20(&oneWire);
#endif

void i2c_wordwrite(uint8_t address, uint8_t cmd, uint16_t theWord) {
  //  Send output register address
  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.write(highByte(theWord));  //  high byte
  Wire.write(lowByte(theWord));  //  send low byte of word data
  Wire.endTransmission();
}

void i2c_write(uint8_t address, uint8_t cmd, uint8_t data) {
  //  Send output register address
  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.write(data);  //  send byte data
  Wire.endTransmission();
}

int i2c_wordread(uint8_t address, uint8_t cmd) {
  uint16_t result;
  uint8_t xlo, xhi;

  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.endTransmission();

  uint16_t readbytes = Wire.requestFrom(address, (size_t)2); // request two bytes
  xhi = Wire.read();
  xlo = Wire.read();

  result = xhi << 8; // hi byte
  result = result | xlo; // add in the low byte

  return result;
}

byte i2c_read(uint8_t devaddr, uint8_t regaddr) {

  uint8_t result = 0;

  Wire.beginTransmission(devaddr);
  Wire.write(regaddr); // control register
  Wire.endTransmission();

  size_t readbytes = Wire.requestFrom(devaddr, (size_t)1, (bool)true); // request cnt bytes

  result  = Wire.read();

  return result;
}

void i2c_readbytes(uint8_t address, uint8_t cmd, uint8_t bytecnt) {

  Wire.beginTransmission(address);
  Wire.write(cmd); // control register
  Wire.endTransmission();

  Wire.requestFrom(address, bytecnt); // request cnt bytes
  for (uint8_t x = 0; x < bytecnt; x++) {
    i2cbuff[x] = Wire.read();
  }
}

void i2c_scan() { // scan entire i2c address space for devices
  uint8_t error, address;
  uint8_t nDevices;

  if (i2cPWR>0) digitalWrite(i2cPWR, HIGH);
  if (serialDebug) Serial.println("Scanning I2C Bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      if (serialDebug) Serial.print("Device found at 0x");
      if (address<16)
      if (serialDebug) Serial.print("0");
      if (serialDebug) Serial.print(address,HEX);
      if (serialDebug) Serial.println();

      nDevices++;
    }
  }
  if (i2cPWR>0) digitalWrite(i2cPWR, LOW);
}

void handleMsg(char* cmdStr) { // handle commands from mqtt or websocket
  // using c string routines instead of Arduino String routines ... a lot faster
  char* cmdTxt = strtok(cmdStr, "=");
  char* cmdVal = strtok(NULL, "=");

  if (strcmp(cmdTxt, "marco")==0) setPolo = true;
  else if (strcmp(cmdTxt, "reboot")==0) doReset = true;
}

void mqttCallback(char* topic, uint8_t* payload, uint16_t len) { // handle MQTT events
  skipSleep=true; // don't go to sleep if we receive mqtt message
  char tmp[200];
  strncpy(tmp, (char*)payload, len);
  tmp[len] = 0x00;
  handleMsg(tmp);
}

boolean mqttReconnect() { // connect or reconnect to MQTT server
  if (mqtt.connect(nodeName)) {
    if (serialDebug) Serial.println("Established MQTT connection.");
    // Once connected, publish an announcement...
    sprintf(msg,"Hello from %s", nodeName);
    mqtt.publish(mqttPub, msg);
    // ... and resubscribe
    mqtt.subscribe(mqttSub);
    useMQTT = true;
  }
  return mqtt.connected();
}

void setup_wifi() { // setup wifi and network stuff
  delay(10);
  // We start by connecting to a WiFi network
  if (serialDebug) Serial.println();
  if (serialDebug) Serial.print("Connecting to ");
  if (serialDebug) Serial.println(WIFISSID);

  String hostname(nodeName);
  WiFi.hostname(hostname);
  WiFi.begin(WIFISSID, WIFIPASSWORD);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    if (serialDebug) Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  if (serialDebug) Serial.println("");
  if (serialDebug) Serial.print("WiFi connected, my IP address ");
  if (serialDebug) Serial.println(WiFi.localIP());

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(nodeName);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    if (serialDebug) Serial.print("Start");
  });
  ArduinoOTA.onEnd([]() {
    if (serialDebug) Serial.println("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (serialDebug) Serial.print(".");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if (serialDebug) Serial.printf("Error[%u]: ", error);

    if (error == OTA_AUTH_ERROR) if (serialDebug) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) if (serialDebug) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) if (serialDebug) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) if (serialDebug) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) if (serialDebug) Serial.println("End Failed");

  });
  ArduinoOTA.begin();
  if (serialDebug) Serial.println("OTA is ready");
}


void setup() {
  if (serialDebug) Serial.begin(115200);
  delay(500); // let micro settle after booting
  if (serialDebug) {
    Serial.print(CLS); // clear screen
    Serial.print(HOME); // home cursor
    Serial.println("Greetings program!");
    Serial.println("");
  }

  if (i2cPWR>0) pinMode(i2cPWR, OUTPUT);

  if (HASDOW) {
    hasTout = true;
    if (DOWPWR>0) pinMode(DOWPWR, OUTPUT);
    ds18b20.begin(); // start one wire temp probe
  }

  Wire.begin(pinSDA, pinSCL); // setup i2c bus

  // setup WiFi and OTA
  setup_wifi();

  // setup MQTT
  lastReconnectAttempt = 0;
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(mqttCallback);

  mqttReconnect(); // establish mqtt connection

  //i2c_write(PUMP, IODIR, 0x00); // all ports output
  i2c_write(ADCPWR, IODIR, 0x01); // all ports except 0 output

  //i2c_write(PUMP, GPIO, 0x00); // all ports low
  i2c_write(ADCPWR, GPIO, 0x00); // all ports low

  ads.begin();
  ads.setGain(GAIN_ONE);
  ads.setSPS(ADS1115_DR_128SPS);
}

void doRSSI() { // store RSSI in global char array
  int16_t rssi = WiFi.RSSI();
  memset(msgRSSI,0,sizeof(msgRSSI));
  sprintf(msgRSSI, "%d", rssi);
  mqtt.publish(mqttRSSI, msgRSSI);
}

uint16_t getAdc(uint8_t chan) {
  uint8_t gp = 1 << (chan + 1);
  uint8_t adcc = chan;
  uint16_t result;

  i2c_write(ADCPWR, GPIO, gp); // power up port
  delay(10); // wait for stable
  result = ads.readADC_SingleEnded(adcc); // take reading
  i2c_write(ADCPWR, GPIO, 0); // power down

  return result; // done
}

void doTout() { // store DOW temperature in char array
  String vStr;
  if (!HASDOW) return;

  memset(msgTemp,0,sizeof(msgTemp));
  if (DOWPWR>0) {
    digitalWrite(DOWPWR, HIGH); // ow on
    delay(5); // wait for powerup
  }

  ds18b20.requestTemperatures();
  byte retry = 5;
  float temp=0.0;
  do {
    temp = ds18b20.getTempCByIndex(0);
    retry--;
    delay(2);
  } while (retry > 0 && (temp == 85.0 || temp == (-127.0)));

  if (DOWPWR>0) {
    digitalWrite(DOWPWR, LOW); // ow off
  }

  vStr = String(temp,4);
  vStr.toCharArray(msgTemp, vStr.length()+1);
  mqtt.publish(mqttTemperature, msgTemp);
}

void doVout() {
  uint16_t vBat = analogRead(A0); // read the TOUT pin
  // uint16_t vBat = getAdc(0); // adc input 0 is tied to supply voltage
  //int  vBat = ESP.getVcc(); // internal voltage reference (Vcc)
  float voltage = vBat * (5.545 / 1023.0); // adjust value, set 5.45 equal to your maximum expected input voltage
  voltage += 0.2F; // correction offset due to cheap divider resistors
  // float vDiv = 0.000125F;
  // float voltage = (float)vBat * (4.096 / 32767.0); // gain of one, PGA = 4.096 volts, 32765 is full scale

  String vStr = String(voltage);
  char myChr[8];
  vStr.toCharArray(myChr, vStr.length()+1);
  mqtt.publish(mqttBattery, myChr);
}

void loop() {
  ArduinoOTA.handle();

  if (!mqtt.connected()) {
    if (serialDebug) Serial.println("MQTT connect failed!");
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    mqtt.loop();
  }

  String vStr;
  char myChr[6];
  char myMQTT[32];

  if (serialDebug) {
    Serial.print(CLS); // home cursor
    Serial.print(HOME); // home cursor
  }

  for (int x=0; x<4; x++) {
    adc[x] = getAdc(x);
    if (serialDebug) {
      Serial.print(x);  Serial.print("="); Serial.println(adc[x]);
    }
    sprintf(myChr, "%u", adc[x]);
    sprintf(myMQTT, "%s/%u", mqttADC, x + 1);
    mqtt.publish(myMQTT, myChr);
  }

  // scan i2c bus
  // i2c_scan();

  doRSSI();
  doTout();
  doVout();

  mqtt.publish(mqttPub, "Sleeping in 60 sec");

  uint16_t cnt = 240; // 60,000 msec
  while(cnt--) {
    ArduinoOTA.handle();
    mqtt.loop();
    if (setPolo) {
      setPolo = false;
      mqtt.publish(mqttPub, "Polo!");
      skipSleep = true;
    }
    delay(250);
  }

  if (!skipSleep) {
    mqtt.publish(mqttPub, "Back in half an hour");
    mqtt.loop();
    uint32_t sleepyTime = 3600000000;

    ESP.deepSleep(sleepyTime, WAKE_RF_DEFAULT); // sleep for 60 min

  }
}
