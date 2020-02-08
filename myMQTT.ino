#include <PubSubClient.h>
#include <SPI.h>
#include <Ethernet.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define MAX_ATTACHED_DS18B20 16
#define TEMPERATURE_PRECISION 12         // Точность измерений в битах (по умолчанию 12)


OneWire ds(7);
DallasTemperature sensors(&ds);

byte inputPIRPin = 2;  // инициализируем пин для получения сигнала от пироэлектрического датчика движения
byte inputDOORPin = 3;  // инициализируем пин для получения сигнала от датчика двери
byte inputSMOKEPin = 6;  // инициализируем пин для получения сигнала от датчика дыма
byte inputRELAYPin = 9;  // инициализируем пин для получения сигнала от датчика дыма

byte inputW5100Reset = 8; // инициализируем пин для сброса W5100

const char* mqtt_server = "192.168.8.100";


DeviceAddress deviceAddresses[MAX_ATTACHED_DS18B20];

float lastTemperature[MAX_ATTACHED_DS18B20];

byte numSensors = 0;

long previousMillis = 0;

bool g_pins[10];

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,8, 80);

EthernetClient ethClient;
PubSubClient client(ethClient);

boolean to_publish_first_time = true;

byte timeInterval = 1000;

byte addr[8] = {0X28,0XFF,0X05,0XCB,0X50,0X16,0X04,0X6B};

void setup() {
    // put your setup code here, to run once:
    pinMode(inputPIRPin, INPUT);  // объявляем PIR датчик в качестве INPUT
    pinMode(inputDOORPin, INPUT);  // объявляем датчик двери в качестве INPUT
    pinMode(inputSMOKEPin, INPUT);  // объявляем датчик дыма в качестве INPUT
    pinMode(7, INPUT);  // pin7 - датчик температуры в качестве INPUT
    // подготавливаем реле-модуль:
    pinMode(inputRELAYPin, OUTPUT);
    digitalWrite(inputRELAYPin, LOW);
    // preparing w5100 for reset
    pinMode(inputW5100Reset, OUTPUT);
    digitalWrite(inputRELAYPin, HIGH);    
    // открываем последовательную коммуникацию на скорости 9600 бод:
    Serial.begin(9600);
   

//    sensors.setWaitForConversion(false);
    sensors.setResolution(10);
    sensors.begin();
    numSensors = sensors.getDeviceCount();
    for (int i = 0; i < numSensors; i++) {
      if (!sensors.getAddress(deviceAddresses[i], i)) {                     // Если не удалось получить идентификатор
        Serial.println("Unable to find address for Device " + String(i));   // Выдаем сообщение об этом
      } else {
        sensors.setResolution(deviceAddresses[i], TEMPERATURE_PRECISION);
      }
    }
     
    Serial.println("Found " + String(numSensors) + " ds18b20 sensors");
    
    resetW5100(false);
}



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if ((char)payload[0] == '1') {
    digitalWrite(inputRELAYPin, HIGH);   // Turn the RElay on 
  } else {
    digitalWrite(inputRELAYPin, LOW);  // Turn the relay off 
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "BoilerRoomClient-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "Connected");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try to reset eth borad and connect again in 5 seconds");
      // Wait 5 seconds before retrying
      resetW5100(true);
      delay(1000);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // получаем данные с шины DS18B20
  if (millis() - previousMillis > timeInterval) 
  {
    // Fetch temperatures from Dallas sensors
    for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {  
      // Fetch and round temperature to one decimal
      //float temperature = sensors.getTempCByIndex(i);
      float temperature = sensors.getTempC(deviceAddresses[i]);
      // Only send data if temperature has changed and no error
      if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
        String tmpTopic = "temperature/sensor" + String(i);
        char str_temp[6];
        dtostrf(temperature, 2, 1, str_temp);
        client.publish(tmpTopic.c_str(), str_temp);    
        lastTemperature[i]=temperature;
      }
    }
    sensors.requestTemperatures();
    previousMillis = millis(); 
  }
  
  
  for (int pin = 2; pin < 10; pin++) {
    if (pin != 7){
      bool val = digitalRead(pin) == HIGH;
      if (val != g_pins[pin]){
        String tmpTopic = "data/pin" + String(pin);
        String a = val ? "ON" : "OFF";
        client.publish(tmpTopic.c_str(), a.c_str());    
        g_pins[pin] = val;        
      }
    }
  }

}

void resetW5100(boolean rst) {
    if(rst){
      digitalWrite(inputW5100Reset, LOW);  //low resets the W5100 chip
      delay(250);  //Keep this short
      digitalWrite(inputW5100Reset, HIGH);
      delay(500); 
    }
    Ethernet.begin(mac, ip);  
    delay(50); 
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}
  
