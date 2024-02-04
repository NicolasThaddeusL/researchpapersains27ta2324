#include <LiquidCrystal_I2C.h>
#include "WiFiS3.h"
#include <ArduinoMqttClient.h>
#include <DFRobot_ENS160.h>

#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
  DFRobot_ENS160_I2C ENS160(&Wire, 0x53);
#else
  uint8_t csPin = D3;
  DFRobot_ENS160_SPI ENS160(&SPI, csPin);
#endif

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
 
char ssid[] = "CC Junior High School";
char pass[] = "Canisian64";
int status = WL_IDLE_STATUS;
char mqtt_user[] = ""; 
char mqtt_pass[] = "";

#define lampu 2
#define ldr 34
int nilaiLdr = 0;

float temp;
float hum;
const char broker[] = "broker.emqx.io";
int port = 1883;
unsigned char bufferRTT[32] = {};
char tempStr[15];
char HumStr[15];
char str[8];
char col;
unsigned int PMSa1 = 0, PMSa2_5 = 0, PMSa10 = 0, FMHDSa = 0, TPSa = 0, HDSa = 0, PMSb1 = 0, PMSb2_5 = 0, PMSb10 = 0, FMHDSb = 0, TPSb = 0, HDSb = 0;
unsigned int PMS1 = 0, PMS2_5 = 0, PMS10 = 0, FMHDS = 0, TPS = 0, HDS = 0, CR1 = 0, CR2 = 0;

char clientId[] = "LetsMakeThingsEverylthing";
char topic_led[] = "mqtt/roomtest/lampu";
char topic_ldr[] = "mqtt/roomtest/ldr";
char topic_temp[] = "mqtt/roomtest/temperature";
char topic_hum[] = "mqtt/roomtest/humidity";
char topic_pm_satu[] = "mqtt/roomtest/pmsatu";
char topic_pm_dua_lima[] = "mqtt/roomtest/pmdualima";
char topic_pm_sepuluh[] = "mqtt/roomtest/pmsepuluh";
char topic_eCO2[] = "mqtt/roomtest/ecodua";
char topic_TVOC[] = "mqtt/roomtest/tvoc";
char topic_AQI[] = "mqtt/roomtest/aqi";
char topic_status[] = "mqtt/roomtest/status";
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void wifiConnect() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true)
      ;
  }
  
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();
}

void printWifiData() {
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);
  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength (RSSI):");
  Serial.println(rssi);
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600); 
  wifiConnect();
  lcd.begin(20,4);
  lcd.init();
  lcd.backlight(); 
  
  while( NO_ERR != ENS160.begin() ){
    Serial.println("Communication with ENS160 failed, please check the connection.");
    delay(3000);
  }
  
  Serial.println("Begin ok!");
  ENS160.setPWRMode(ENS160_STANDARD_MODE);
  ENS160.setTempAndHum(25.0, 50.0);
  
  Serial.print("Attempting to connect to the MQTT broker.");
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1)
      ;
  }

  Serial.println("You're connected to the MQTT broker!");
  // mqttClient.onMessage(onMqttMessage);
  Serial.println(topic_led);
  Serial.print("Subscribing to topic: ");
  int subscribeQos = 0;
  mqttClient.subscribe(topic_led, subscribeQos);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(topic_led);
}

void loop() {
  mqttClient.poll();

  Serial.print("Temperatur : ");
  sprintf(tempStr, "%d%d.%d", TPS / 100, (TPS / 10) % 10, TPS % 10);
  Serial.print(tempStr);
  Serial.println(" C");
  mqttClient.beginMessage(topic_temp);
  mqttClient.print(tempStr);
  mqttClient.endMessage();

  Serial.print("Humiditas   : ");
  sprintf(tempStr, "%d%d.%d", HDS / 100, (HDS / 10) % 10, HDS % 10);
  Serial.print(tempStr);
  Serial.println(" %");
  mqttClient.beginMessage(topic_hum);
  mqttClient.print(tempStr);
  mqttClient.endMessage();

  Serial.print("PM 1.0: ");
  Serial.print(PMS1);
  Serial.println(" ug/m3");
  mqttClient.beginMessage(topic_pm_satu);
  mqttClient.print(PMS1);
  mqttClient.endMessage();

  Serial.print("PM 2.5: ");
  Serial.print(PMS2_5);
  Serial.println(" ug/m3");
  mqttClient.beginMessage(topic_pm_dua_lima);
  mqttClient.print(PMS2_5);
  mqttClient.endMessage();

  Serial.print("PM 10: ");
  Serial.print(PMS10);
  Serial.println(" ug/m3");
  mqttClient.beginMessage(topic_pm_sepuluh);
  mqttClient.print(PMS10);
  mqttClient.endMessage();

  uint8_t Status = ENS160.getENS160Status();
  Serial.print("Status pengoperasian sensor : ");
  Serial.println(Status);
  mqttClient.beginMessage(topic_status);
  mqttClient.print(Status);
  mqttClient.endMessage();
  
  uint8_t AQI = ENS160.getAQI();
  Serial.print("Air Quality Index : ");
  Serial.println(AQI);
  lcd.setCursor(11, 2);
  lcd.print(AQI);
  mqttClient.beginMessage(topic_AQI);
  mqttClient.print(AQI);
  mqttClient.endMessage();
  
  uint16_t TVOC = ENS160.getTVOC();
  Serial.print("Konsentrasi Total Senyawa Organik yang Mudah Menguap (TVOC) : ");
  Serial.print(TVOC);
  Serial.println(" ppb");
  mqttClient.beginMessage(topic_TVOC);
  mqttClient.print(TVOC);
  mqttClient.endMessage();
  
  uint16_t ECO2 = ENS160.getECO2();
  Serial.print("Konsentrasi Karbon Dioksida Setara (eCO2) : ");
  Serial.print(ECO2);
  Serial.println(" ppm");
  mqttClient.beginMessage(topic_eCO2);
  mqttClient.print(ECO2);
  mqttClient.endMessage();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("eCO2:");
  lcd.setCursor(7, 0);
  lcd.print(ECO2);
  lcd.setCursor(12, 0);
  lcd.print("ppm");
  lcd.setCursor(0, 1); 
  lcd.print("TVOC:");
  lcd.setCursor(7, 1);
  lcd.print(TVOC);
  lcd.setCursor(12, 1);
  lcd.print("ppb");
  lcd.setCursor(0, 2); 
  lcd.print("AQI Index:");
  lcd.setCursor(11,2);
  lcd.print(AQI);
  lcd.setCursor(0, 3);
  lcd.print("PM2.5:");
  lcd.setCursor(7,3);
  lcd.print(PMS2_5);

  while (!Serial1.available());
  while (Serial1.available() > 0)
  {
    for (int i = 0; i < 32; i++) {
      col = Serial1.read();
      bufferRTT[i] = (char)col;
      delay(2);
    }

    Serial1.flush();

    CR1 = (bufferRTT[30] << 8) + bufferRTT[31];
    CR2 = 0;
    for (int i = 0; i < 30; i++)
      CR2 += bufferRTT[i];
    if (CR1 == CR2)
    {
      PMSa1 = bufferRTT[10];
      PMSb1 = bufferRTT[11]; 
      PMS1 = (PMSa1 << 8) + PMSb1;

      PMSa2_5 = bufferRTT[12];
      PMSb2_5 = bufferRTT[13];
      PMS2_5 = (PMSa2_5 << 8) + PMSb2_5;

      PMSa10 = bufferRTT[14];
      PMSb10 = bufferRTT[15];
      PMS10 = (PMSa10 << 8) + PMSb10;

      TPSa = bufferRTT[24];
      TPSb = bufferRTT[25];
      TPS = (TPSa << 8) + TPSb;

      HDSa = bufferRTT[26];
      HDSb = bufferRTT[27];
      HDS = (HDSa << 8) + HDSb;
    } else {
      PMS1 = 0;
      PMS2_5 = 0;
      PMS10 = 0;
      TPS = 0;
      HDS = 0;
    }
  }

  mqttClient.beginMessage(topic_ldr);
  mqttClient.print(random(1000));
  mqttClient.endMessage();
  
  delay(10000);
  lcd.clear();
}
