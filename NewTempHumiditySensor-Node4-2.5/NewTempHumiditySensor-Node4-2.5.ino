// HVAC Zone Controller
// (c) Paul Zavracky, May 2018
#define VERSION "2.5"

// DESCRIPTION
/*
 * This zone controller is used to control the louvers of the HVAC vents in each room or zone of the home.  It uses a DHT22 sensor to determine the temperature and humidity locally.
 * The zone controller comprises an Arduino Nano, a NRF24 radio, the DHT sensor, a stepper motor, control circuit boards, and the mechanics necessary to open and close the louvers.  
 * Temperture and Humidity data is retreived from the DHT22 and averaged over a given number of samples.  Once averaged, the data is sent to the HomeAssistant controller and to the 
 * thermostat.  The controller also listens for requests to open and close the louvers.  While capable of proportionally opening the louvers, this implementation is set to fully open
 * and close the louvers using an optical limit switch to prevent over driving.  The status of the limit switches is sent to the controllerand to the Thermostat as well. In the event 
 * the louvers do not open or close all the way, the controller may attempt second and third times.
 */

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached 
#define MY_RADIO_NRF24
#define MY_NODE_ID 4 // Master Bedroom
#define THERMOSTAT_NODE 1
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC 
#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_MOTOR 2
#define CHILD_ID_CLOSED_FLAG 3
#define CHILD_ID_OPEN_FLAG 4

// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 3
#define OPEN_FLAG_PIN 18 // A4
#define CLOSED_FLAG_PIN 19 // A5

#define HUM_ADJUST -1.44 // calibration for humidity sensor
#define TEMP_ADJUST -2.1 // calibration for temperature sensor

#include <SPI.h>
#include <MySensors.h>
#include <MyConfig.h>
#include <DHT.h>

static const uint8_t DHTcounterMax = 30; // number of samples taken of temp and hum in averaging (30 = ~1min)
static const uint8_t MOTOR_PIN[] {4,5,6,7};

float humidity, temperature;
float lastTemp;
float lastHum;
float sumTemperature = 0.00;
float sumHumidity = 0.00;
float a = 0.0; // used for the motor control
bool metric = true;
uint64_t DHTmin; // set after acquiring info diretly from the sensor
String DHTerror; 
unsigned long oldMillis;
int DHTcounter;

int CW_CCW = 1; // if 1, direction is CW, else 0 is reverse
char convBuffer[10];

int delayTime = 5;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgMotorForRev(CHILD_ID_MOTOR, V_LIGHT); // used to indicate motor direction
MyMessage msgMotorPos(CHILD_ID_MOTOR, V_LIGHT_LEVEL); // recieve an integer from 0 to 255 from HomeAssistant slider
MyMessage msgOpenFlag(CHILD_ID_OPEN_FLAG, V_TRIPPED);
MyMessage msgClosedFlag(CHILD_ID_CLOSED_FLAG, V_TRIPPED);

DHT dht;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void presentation()  
{ 
  //Serial.println("made it to presentation");
  // Send the sketch version information to the gateway
  sendSketchInfo("TemperatureAndHumidity", VERSION);

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_MOTOR, S_LIGHT_LEVEL);
  present(CHILD_ID_MOTOR, S_LIGHT);
  present(CHILD_ID_OPEN_FLAG, S_DOOR);
  present(CHILD_ID_CLOSED_FLAG, S_DOOR);
  
  metric = getControllerConfig().isMetric;
}

void setup() {
  Serial.begin(115200);
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  DHTmin = dht.getMinimumSamplingPeriod(); // minimum time to collect data
  Serial.print("dhtmin = "); Serial.println(int(DHTmin));
  for (int i = 0; i < 4; i++ ) { pinMode(MOTOR_PIN[i], OUTPUT); }
  pinMode(OPEN_FLAG_PIN, INPUT);
  pinMode(CLOSED_FLAG_PIN, INPUT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {  
 
  if(updateTempHumidity()) {
    if (DHTerror == "OK") {
      if (temperature != lastTemp) {  // forces the node to send data after so many reads
        lastTemp = temperature;
        send(msgTemp.setDestination(MY_PARENT_NODE_ID).set(temperature, 1)); // this message is specifically for the controler, so sent directly to the gateway
        send(msgTemp.setDestination(THERMOSTAT_NODE).set(temperature, 1)); // send message through gateway to the thermostat
        sendFlags();  // use this opportunity to update the flags periodically
      }
      if (humidity != lastHum) {
        // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
        lastHum = humidity;
        send(msgHum.setDestination(MY_PARENT_NODE_ID).set(humidity, 1)); // this message is specifically for the controler, so sent directly to the gateway
        send(msgHum.setDestination(THERMOSTAT_NODE).set(humidity, 1)); // send message through gateway to the thermostat
      }
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void receive(const MyMessage &message) {

  Serial.print("recieved a message, "); Serial.println(message.type);
  if (message.type == V_LEVEL) {  // this should be the only type of message we recieve
    int steps = atoi(message.getString(convBuffer));
    Serial.print("steps = "); Serial.println(steps);
    if (CW_CCW == 1) {
      steps = steps;
      openLouvers(steps);
    }
    else {
      closeLouvers(steps);      
    }
  }
  if (message.type  == V_STATUS) {
    CW_CCW = atoi(message.getString(convBuffer));
    Serial.print("forRev = ");Serial.println(CW_CCW);
  }
} 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool updateTempHumidity() {
  bool dataReady;

  dataReady = false;
  if(oldMillis > millis()) { oldMillis = 0; } // clock rolled over!!!
  if ((millis() - oldMillis) > DHTmin + 1000) {
    oldMillis = millis();
    dht.readSensor(true); // Force reading sensor, so it works also after sleep()
    DHTerror = dht.getStatusString();
    if (DHTerror == "OK") {
      humidity = dht.getHumidity();
      temperature = dht.toFahrenheit(dht.getTemperature());
      sumTemperature += temperature;
      sumHumidity += humidity;
      DHTcounter += 1; // increment counter     
    }
    if (DHTcounter >= DHTcounterMax) {
      temperature =sumTemperature/DHTcounterMax + TEMP_ADJUST; // final average; only place where indoorTemperature is updated
      humidity = sumHumidity/DHTcounterMax + HUM_ADJUST;
      sumTemperature = 0.00;
      sumHumidity = 0.00;
      DHTcounter = 0;
      dataReady = true;
    }
  }
  return(dataReady);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void openLouvers(int steps) { // open louvers
  int finalSteps = 20;  // number of steps after optical switch detects position

  if(digitalRead(OPEN_FLAG_PIN)) {  // are the louvers already open
    sendFlags();  
    return; }
  while (steps > 0) {
    for ( int i = 0; i < 4; i++) {
      for ( int j = 0; j < 4; j++) {
        a = float((i*4+j))/5;
        digitalWrite(MOTOR_PIN[3-j], ((a - int(a)) > 0) ? 0 : 1);
      }
      if(digitalRead(OPEN_FLAG_PIN)) { // are the louvers open
        if (finalSteps < 0) {  // exit loop
          i = 4; steps = 0;
        }
        else {
          finalSteps--; // decriment finalSteps
          steps = 2; // keep moving until finalSteps run out
        }
      }
      delay(delayTime);
    }
    steps--;
  }
  for (int i = 0; i < 4; i++) {
    digitalWrite(MOTOR_PIN[i], LOW); // turn off all pins
  }
  sendFlags();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void closeLouvers(int steps) { // close louvers
  int finalSteps = 20;  // number of steps after optical switch detects position
  
  if(digitalRead(CLOSED_FLAG_PIN)) {// are the louvers already closed
    sendFlags();   
    return; }
  while (steps > 0) {
    for ( int i = 0; i < 4; i++) {
      for ( int j = 3; j > -1; j--) {
        a = float((i*4+j))/5;
        digitalWrite(MOTOR_PIN[j], ((a - int(a)) > 0) ? 0 : 1);    
      }
      if(digitalRead(CLOSED_FLAG_PIN)) { // are the louvers closed
        if (finalSteps < 0) {  // exit loop
          i = 4; steps = 0;
        }
        else {
          finalSteps--; // decriment finalSteps
          steps = 2; // keep moving until finalSteps run out
        }
      }
      delay(delayTime);
    }
    steps--;
  }
    for (int i = 0; i < 4; i++) {
    digitalWrite(MOTOR_PIN[i], LOW); // turn off all pins
  }
  sendFlags();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void sendFlags() { // send louver status to HomeAssistant and the Thermostat
  
  send(msgOpenFlag.setDestination(MY_PARENT_NODE_ID).set(!digitalRead(OPEN_FLAG_PIN)));  
  send(msgClosedFlag.setDestination(MY_PARENT_NODE_ID).set(!digitalRead(CLOSED_FLAG_PIN)));
  send(msgOpenFlag.setDestination(THERMOSTAT_NODE).set(!digitalRead(OPEN_FLAG_PIN))); 
  send(msgClosedFlag.setDestination(THERMOSTAT_NODE).set(!digitalRead(CLOSED_FLAG_PIN)));
}


