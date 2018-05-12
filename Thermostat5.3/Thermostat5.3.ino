// Wireless Thermostat System
//
// (c) Paul Zavracky, January, 2018
#define VERSION "5.3"
// This software began using Touchscreen software from Adafruit as an example drawing program.  Elements of that program included a color selection menu.  That concept
// was carried over to this program.  Afterwards, a sub menu was added.  Then cell selection capability and cell modification using arrow keys.  Additionally. a realtime
// clock has been added as well as a MySensors interface with NRF24 radio.  Finally a temperature/humidity sensor was added.
// The intent of this software is to create a wireless thermostat that interfaces with HomeAssistant.  It collects data from around the home and adjusts the heat, or
// cooling for optimum result.  The intention is to add automated vent controllers so that the air to individual rooms can be controlled.  In this way, the thermostat
// will be able to choose portions of the home to heat and cool, thus saving energy and providing a more comfortable environment.  The controller can also be placed in
// humidity control mode.  In this mode, a humidity set point is chosen and the controller controls to humidity rather than temperature.

// EEPROM Memory Usage:
//      Memory is filled as follows:
//      0 - 71    Allocated to the schedules of weekend and weekday set points.  See function WriteLongTermData() below.
//      72 - 75   temperature and humidity manual set points
//      76 - 78   modes (heat, fan, cool)
//      79        runMode
//      80        tempHumdity
//      81 - 85   zones

#define EEPROM_CHILD_ID_START_OF_MODES 76 // these two constants are used to establish the starting point for restoring Modes and Zones
#define EEPROM_CHILD_ID_START_OF_ZONES 81

// definitions for MySensors.  These must be defined before calling the MySensors library!
#define MY_NODE_ID 1 // This node is the entire thermostat, including the relays and the humidity and temporature sensor
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

// Enable debug prints
// #define MY_DEBUG
// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53
#define RF24_PA_LEVEL RF24_PA_HIGH

// the following are the child ids for the sensors integral to the thermostat
#define CHILD_ID_HUM 1
#define CHILD_ID_TEMP 2
#define CHILD_ID_SET_POINT_HUM 3
#define CHILD_ID_HVAC 5
#define CHILD_ID_START_OF_MODES 10
#define CHILD_ID_FAN_MODE 10
#define CHILD_ID_COOL_MODE 11
#define CHILD_ID_HEAT_MODE 12
#define CHILD_ID_HAVC_ACTUATION_STATE 13
#define CHILD_ID_HVAC_REFRESH 14
#define CHILD_ID_START_OF_BUTTONS 20
#define CHILD_ID_RUN_BUTTON 20
#define CHILD_ID_TH_BUTTON 21
#define CHILD_ID_START_OF_ZONES 30
#define CHILD_ID_ZONE1 30
#define CHILD_ID_ZONE2 31
#define CHILD_ID_ZONE3 32
#define CHILD_ID_ZONE4 33
#define CHILD_ID_ZONE5 34
#define CHILD_ID_START_OF_RELAY_INDICATORS 40
#define CHILD_ID_FAN_RELAY_INDICATOR 40
#define CHILD_ID_COOL_RELAY_INDICATOR 41
#define CHILD_ID_HEAT_RELAY_INDICATOR 42

// the following are external (to the thermostat) nodes and children
#define CHILD_ID_MOTOR 2 // this is the childID for the motor at each of the 5 zone sensors
#define START_OF_ZONE_SENSOR_NODE_IDS 2 //  just for clarity START_OF_ZONE_SENSOR_CHILD_IDS
#define NODE_ID_LIVING_ROOM 2  // NODES for zones ("living","dining","master","guest ","study ")
#define NODE_ID_DINING_ROOM 3
#define NODE_ID_MASTER 4
#define NODE_ID_GUEST 5
#define NODE_ID_STUDY 6

#define TFT_CS 10
#define TFT_DC 9

// Assign human-readable names to some common 16-bit color values: (R4,R3,R2,R1,R0,G5,G4,G3,G2,G1,G0,B4.B3,B2,B1,B0)
// So HEX are (R4,R3,R2,R1; R0,G5,G4,G3; G2,G1,G0,B4; B3,B2,B1,B0)
// Then (0; 0; 1; F) is all blue!
#define BLACK       0x0000
#define LTBLUE      0x001F
#define DKBLUE      0x002F
#define RED         0xF800
#define GREEN       0x07E0
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define DKGREEN     0x03E0
#define LTGRAY      0xC618 // (1100 0110 0001 1000) or (11000, 110000, 11000)

// the following definitions are unique to this thermostat's operation
#define MENUHEIGHT 20
#define MENUWIDTH 80
#define LABLE0 "OPERATE"
#define LABLE1 "SCHEDULE"
#define LABLE2 "MANUAL"
#define LABLE3 "ZONE DATA"

// Arduino Digital I/O pin number for first relay is 32 as seen below and corresponds to "HEAT_RELAY" (second on pin+2 etc)
#define NUMBER_OF_RELAYS 3 // Total number of attached relays
#define NUMBER_OF_MODES 3 // Modes determine what kind of function is required, fan, cool or heat, whereas, the relays turn on and off the appropriate actuator
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay
#define FAN 0
#define COOL 1
#define HEAT 2
#define OFF 3
#define TEMP 1
#define HUM 0
#define NUMBER_OF_ZONES 5 // Total number of zones

// the following definitions and variables are used for the thermistat without regard for the radio interface
#define WAIT 200
#define HUMIDITY_SENSOR_PIN 30
#define HUM_ADJUST -4.22 // calibration for humidity sensor
#define TEMP_ADJUST -3.87 // calibration for temperature sensor
#define START_OF_RELAY_PINS 32
#define FAN_RELAY_PIN 32
#define COOL_RELAY_PIN 34
#define HEAT_RELAY_PIN 36

#define TEMP_ANTICIPATION 0.7 // this variable provides a range of operation for the system.  It controls the frequency of on and off cycles. ( a value of 0.5 could be increased to 0.7 comfortably)
#define HUM_ANTICIPATION 3.5 // similar to the above, but for humidity (a value of 3.5 seems best based upon experimentation)
#define FAN_DELAY_TIME 60000  //milliseconds - 60000 = 1 minute
#define SUSTAIN_TIME 70000 // milliseconds of time louvers and A/C unit will remain on or off after a state change has been effected by thermostat
#define MAX_LOUVER_ATTEMPT_COUNT 3 // number of times the controller will attempt to open or close a louver.  To correct, must reset counter at HomeAssistant.

//display backlight variables
#define BACKLIGHT_PIN 5
#define BACKLIGHT_DELAY_TIME 300000 //milliseconds - 300000 = 5 minutes

// inclusion of libraries
#include <MySensors.h>
#include <MyConfig.h>
#include <EEPROM.h>
#include <DS3231.h> // RTC Library
#include <Wire.h> // I2C library
#include <DHT.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>       // this is needed for display
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>

// this section defines all the global constants and variables
const float humCal[5] = {0.0,0.0,0.0,0.0,0.0}; // the actual reading plus this value for each zone will yeild the calibrated humidity (".0" to insure assignment as float)
const float tempCal[5] = {0.0,0.0,0.0,0.0,0.0}; // the actual reading plus this value for each zone will yeild the calibrated temperature

const char* modeButton[3] = {" FAN "," COOL"," HEAT"}; // String arrays for Screen3
const char* zoneButton[5] = {"living","dining","master","guest ","study "};
const char* shortDOW[7] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
const char* longDOW[7] = {"MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY", "SUNDAY"};
int DOWindex = 0;

int cellFrame[4][6][4] =  {{{10,65,70,25},  // definition of schedule array positions on screen (column, row, box dimensions)
                        {10,95,70,25},
                        {10,125,70,25},
                        {10,155,70,25},
                        {10,185,70,25},
                        {10,215,70,25}},
                        {{90,65,70,25},  
                        {90,95,70,25},
                        {90,125,70,25},
                        {90,155,70,25},
                        {90,185,70,25},
                        {90,215,70,25}},
                        {{170,65,70,25},
                        {170,95,70,25},
                        {170,125,70,25},
                        {170,155,70,25},
                        {170,185,70,25},
                        {170,215,70,25}},
                        {{250,65,70,25},
                        {250,95,70,25},
                        {250,125,70,25},
                        {250,155,70,25},
                        {250,185,70,25},
                        {250,215,70,25}}};

int CellDataPos[4][6][2] =  {{{15,70},  // define the position of the text within the cells of the Plan matrix
                                {15,100},
                                {15,130},
                                {15,160},
                                {15,190},
                                {15,220}},
                                {{95,70},  
                                {95,100},
                                {95,130},
                                {95,160},
                                {95,190},
                                {95,220}},
                                {{175,70},
                                {175,100},
                                {175,130},
                                {175,160},
                                {175,190},
                                {175,220}},
                                {{255,70},
                                {255,100},
                                {255,130},
                                {255,160},
                                {255,190},
                                {255,220}}};

int CellLabelPos[4][2] = {{5,45},{95,45},{170,45},{245,45}};
float tempHumidityData[2][5] = {{0,0,0,0,0},{0,0,0,0,0}}; // initial values for tempHumidityData - the temp [1][zone] and humidity [0][zone] for each zone ("living","dining","master","guest ","study ")
int boxIncrmt = 10;
int oldscreen, currentscreen;
int oldsubscreen = 0, currentsubscreen = 0;
bool firstTime = true; // used to set up HA for the first time.
bool weekdays = true; // if true, set for weekdays, if false, set for weekends
bool mode[] = {false, true, false}; // initial state of fan, cool, and heat
bool zone[5] = {true, true, true, true, true}; // initial activity mode of zones - all louvers that are 'true' will be controlled automatically, otherwise they are off (closed). 
bool louverState[5][2]; // state of five zones; either open, closed, inbetween, or an impossible state (both open and closed at the same time). Louver..[x][0] is CLOSED_FLAG
byte louverAttemptCount[5][2]; // attempt counter for louvers - see MAX_LOUVER_ATTEMPT_COUNT.  louver..[x][0] corresponds to the CLOSED_FLAG
bool runMode = true; // if true, system is running on the programmed schedule.  If false, the system is controlling to the setpoint.
bool fanMode = false; // if true, the system is running in fan mode -- not heating or cooling
bool tempHumidity = true; // if true the system is running temperature control.  If false, the system is controlling the humidity.
bool oldTempHumidity; // used to monitor changes in this mode of operation specifically for louverAdjust
bool timeSet = false; // if true, then when on screen 1, the subscreen for setting the time and date is displayed
bool heatCoolPlan = false; // if true, then when planning, the displayed schedule will be for heating, else cooling.  Different schedules required!

// fan delay variables
bool fanDelay = false;  // forces delayed fan relay turn off from within main loop
unsigned long fanDelayStartTime; // to be compared with FAN_DELAY_TIME (amount of delay) and the current time


// backlight delay variables
unsigned long backlightDelayStartTime; 

// following set of values used for 'schedule' screen
int row;
int column;
int oldRowColumn; // save the old row collumn as an integer ( row * 10 + column ) 
int newRowColumn;
int oldRow;
int oldColumn;

char Toutdoor[] = "48";
char Houtdoor[] = "21";
char convBuffer[10]; // used to receive humidity setpoint

float indoorHumidity; // humidity data that is averaged over DHTcounterMax data points - see humMeasurement
float lastIndoorHumidity;
float indoorTemperature; // temperature data that is averaged over DHTcounterMax data points - see tempMeasurement
float lastIndoorTemperature;
float sumTemperature; // used for averaging temperature readings
float sumHumidity;
float tempMeasurement; // unaveraged data from sensor
float humMeasurement;
int DHTcounter; // also used for averaging temp and humidity
int DHTcounterMax = 30; // number of temperatures or humidities summed; 30 counts takes about 1 minute
String DHTerror; // string to store errors received from the DHT sensor
String lastError; // DHTerror saved for comparison with current error.  If not equal, new error will be displayed on screen

byte Setpt[2][2] = {{50,74},{99,72}};  //Setpt for temperature and humidity -- Setpt[0][1] is cool setpt, Setpt[1][1] is heat setpt, Setpt[0][0] is humidity setpt, Setpt[1][0] is 99% humidity!!
 
byte DTPlanTemp[2][2][6] = {{{72,72,72,72,72,72},{72,72,72,72,72,72}}, // daily plan for temperature in six values, for weekdays and weekends
                            {{75,75,75,75,75,75},{75,75,75,75,75,75}}}; // indices are [heatCoolPlan][weekdays][data]
byte DTPlanHours[2][2][6] = {{{8,9,11,13,15,21},{8,9,11,13,15,21}},
                             {{7,8,10,13,17,21},{7,8,10,13,17,21}}};
byte DTPlanMins[2][2][6] = {{{0,15,30,45,0,15},{15,45,0,15,30,45}},
                            {{15,30,15,30,45,0},{45,15,45,55,15,15}}};
byte planIndex; // the index of the current temperature plan
byte oldPlanIndex = 0; // used to update the display on screen1.  do not wnat display to flicker!
byte planTemp; // active temperature from plan.  Humidity control only works in manual mode!


int monthDayYear[3] = {1,1,2018}; // here only temporarily while setting up the real time clock
int hourMin[2] = {7,45};
int oldMin;
int oldHour;
unsigned long oldMillis; // used to slow humidity/temp updates
unsigned long thermostatOldMillis; // used to prevent the thermostat from switching on and off once the state changes
unsigned long zoneOldMillis[5]; // used to keep zones opened or closed upon a state change for the duration 'SUSTAIN_TIME'
int DHTdelay; // delay for DHT to collect data

String timeString; // use as local variable and temporary
String dateString; // use as local variable and temporary
String DOW;  // day of week string
String oldDOW; // needed for louverAdjust

char TimeTime[6]; // this variable will store the times the required display format "00:00"
byte RTC_I2C_address = 0x68; // location of first register in RTC
byte disk1 = 0x57;    //Address of 24C32N eeprom chip
byte I2Cerror;
bool RTCavail; // used to by-pass date time.  must set mode to "set point" rather than "run"
bool oldRTCavail; // used to insure a screen 0 refresh if the RTC is recovered from a loss

// stuff for MySensors HVAC
int POWER_STATE;
int TEMP_STATE;
int FAN_STATE;
int MODE_STATE;
int VDIR_STATE;
int HDIR_STATE;

// start library functionality
DS3231  rtc(SDA, SCL); // real time clock setup
DHT dht; // temperature and humidity sensors

Adafruit_FT6206 ctp = Adafruit_FT6206();
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

MyMessage msgHum(CHILD_ID_HUM, V_HUM); // these messages are transmitted using the MySensors radio routines from the MySensors library
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP); // internal humdity and temperature sensors
MyMessage msgHumSetPt(CHILD_ID_SET_POINT_HUM, V_DIMMER); // we'll use this construct to pass the humidity setpoint from the controller to this node (NODE 1)
MyMessage msgHumSetPtStatus(CHILD_ID_SET_POINT_HUM, V_STATUS); // same as above
MyMessage msgHVACsetptHeat(CHILD_ID_HVAC, V_HVAC_SETPOINT_HEAT);
MyMessage msgHVACsetptCool(CHILD_ID_HVAC, V_HVAC_SETPOINT_COOL);
MyMessage msgHVACSpeed(CHILD_ID_HVAC, V_HVAC_SPEED);
MyMessage msgHVACFlowState(CHILD_ID_HVAC, V_HVAC_FLOW_STATE);
MyMessage msgHVACTemp(CHILD_ID_HVAC, V_TEMP); // temperature used for control signal
MyMessage msgMode[NUMBER_OF_MODES]; // set up an array of messages for the modes - you can be in cool mode without having the cool relay on!
MyMessage msgRelay[NUMBER_OF_RELAYS]; // set up an array of message for the relays, to inform which relays are actually on
MyMessage msgHVACRefresh(CHILD_ID_HVAC_REFRESH, V_LIGHT); // used to get HomeAssistant thermostat refreshed
MyMessage msgRunMode(CHILD_ID_RUN_BUTTON, V_LIGHT); // sets runMode (boolean)
MyMessage msgTHBut(CHILD_ID_TH_BUTTON, V_LIGHT); // sets tempHumidity (boolean)
MyMessage msgZone[NUMBER_OF_ZONES]; // set up an array of messages for the zones
MyMessage msgMotoropenClose[NUMBER_OF_ZONES]; // used to indicate motor direction
MyMessage msgMotorPos[NUMBER_OF_ZONES]; // send an integer from 0 to 255; 100 will fully open or close the louver

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void) {
  int nDevices;
  byte error, address; 

  Serial.begin(115200);
      
  Serial.println(F("My Thermostat"));
  // Initialize the rtc object
  rtc.begin(); // Real Time Clock

  dht.setup(HUMIDITY_SENSOR_PIN);
  DHTdelay = dht.getMinimumSamplingPeriod();
  Serial.print("the minimum data refresh time for the DHT sensor is: ");Serial.println(DHTdelay);
  
  Wire.begin();  // I2C library used to determine if RTC is available
  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
        nDevices++;
      }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
  // ready to read long term data - last know operating position
  // WriteLongTermData(); // UNCOMMENT THIS LINE TO INITIALIZE LONG TERM DATA!!!!!!!
  ReadLongTermData(); // get last known state
  
  // set up relays
  for (int i = 0, pin = START_OF_RELAY_PINS; i < NUMBER_OF_RELAYS; i++, pin = pin + 2) {
    // Then set relay pins in output mode
    pinMode(pin, OUTPUT); // will setup the FAN_RELAY_PIN, COOL_RELAY_PIN, and HEAT_RELAY_PIN pins as outputs!
    // Set relay to last known state (using eeprom storage) 
  }
  // set up mode parameters for presentation
  for (int i = 0; i < NUMBER_OF_MODES; i++) {
    // Register all sensors to gw (they will be created as child devices)
    msgMode[i].sensor = i + CHILD_ID_START_OF_MODES; // required because we set up an array of messages for relays           
    msgMode[i].type = V_LIGHT;    
  }
  // set up relay indicators for presentation
  for (int i = 0; i < NUMBER_OF_RELAYS; i++) {
    // Register all sensors to gw (they will be created as child devices)
    msgRelay[i].sensor = i + CHILD_ID_START_OF_RELAY_INDICATORS; // required because we set up an array of messages for relays           
    msgRelay[i].type = V_TRIPPED;    
  } 
  // similarly for zones
  for (int i = 0; i < NUMBER_OF_ZONES; i++) {
    // Register all sensors to gw (they will be created as child devices)
    msgZone[i].sensor = i + CHILD_ID_START_OF_ZONES; // required because we set up an array of messages for zones.  This defines the possible sensor CHILD_IDS           
    msgZone[i].type = V_LIGHT; 
    msgMotoropenClose[i].sensor = CHILD_ID_MOTOR; // message for a zone sensor to adjust louver
    msgMotoropenClose[i].type = V_LIGHT; // used to indicate motor direction
    msgMotorPos[i].sensor = CHILD_ID_MOTOR; // message for a zone sensor to adjust louver   
    msgMotorPos[i].type = V_LEVEL; // recieve an integer from 0 to 255 (100 will fully open or close louver)
    louverState[i][0] = true; louverState[i][1] = false; // initialize with all louvers open (louvers themselves initialize open!)
    zoneOldMillis[i] = millis(); // initiate timer for louvers
  }

  tft.begin();
  if (! ctp.begin(40)) {  // pass in 'sensitivity' coefficient
    Serial.println("Couldn't start FT6206 touchscreen controller");
    while (1);
  }
  Serial.println("Capacitive touchscreen started");
  
  tft.setRotation(1);
  tft.fillScreen(LTBLUE);
  tft.fillRect(0, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
  tft.drawRect(0, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
  tft.setCursor(15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE0);
  tft.fillRect(MENUWIDTH, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
  tft.drawRect(MENUWIDTH, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
  tft.setCursor(MENUWIDTH + 15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE1);
  tft.fillRect(MENUWIDTH*2, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
  tft.drawRect(MENUWIDTH*2, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
  tft.setCursor(MENUWIDTH*2 + 15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE2);
  tft.fillRect(MENUWIDTH*3, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
  tft.drawRect(MENUWIDTH*3, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
  tft.setCursor(MENUWIDTH*3 + 15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE3);
   
  tft.drawRect(0+1, 0+1, MENUWIDTH-2, MENUHEIGHT-2, WHITE);
  currentscreen = 0;
  Screen0SetUp();
 
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT); // backlight control pin

  digitalWrite(BACKLIGHT_PIN, 1); // turn on backlight
  backlightDelayStartTime = millis();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void presentation()  
{ 
  // Send the Sketch Version Information to the Gateway
  sendSketchInfo("Boca Thermostat", VERSION);

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present (CHILD_ID_SET_POINT_HUM, S_DIMMER); // we'll use this construct to pass the humidity setpoint from the controller to this node (NODE 1)

  for (int i = 0; i < NUMBER_OF_MODES; i++) {
    // Register all relays to gw (they will be created as child devices)   
    present(CHILD_ID_START_OF_MODES + i, S_LIGHT);
  }

  for (int i = 0; i < NUMBER_OF_RELAYS; i++) {
    // Register all relays to gw (they will be created as child devices)   
    present(CHILD_ID_START_OF_RELAY_INDICATORS + i, S_DOOR);
  }
  
  for (int i = 0; i < NUMBER_OF_ZONES; i++) {
    // Register all zones to gw (they will be created as child devices)  
    present(CHILD_ID_START_OF_ZONES + i, S_LIGHT);
  }

  present(CHILD_ID_HVAC, S_HVAC, "MyThermostat");
  present(CHILD_ID_HVAC_REFRESH, S_LIGHT); // switch used to request an update of MySensors data to HomeAssistant
  present(CHILD_ID_RUN_BUTTON, S_LIGHT);
  present(CHILD_ID_TH_BUTTON, S_LIGHT); 

}
int count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  int steps = 100; 

  if (firstTime) {
    // next is based on the data from the initial read of the EEPROM
    HAupdate();
    // open all louvers
    for (int i = 0; i < 5; i++) {
      openLouver(i);
    }
    firstTime = false; 
  }

  if ((millis() - backlightDelayStartTime) > BACKLIGHT_DELAY_TIME) { // down time has been exceeded, turn off display backlight
    digitalWrite(BACKLIGHT_PIN,0);
  }
  
  TS_Point p = ctp.getPoint();
  // flip it around to match the screen.
  p.y = map(p.y, 0, 320, 320, 0);
  
  if (ctp.touched()) {  
    if (digitalRead(BACKLIGHT_PIN) == 0) { // backlight is not on; turn it on
      digitalWrite(BACKLIGHT_PIN, 1); // turn on backlight
      backlightDelayStartTime = millis();
    } // here we exit the if statement without checking where on the screen the touch occurred
    else { // this will prevent the touch that turns on the backlight from being acted upon
      if (p.x < MENUHEIGHT && timeSet == false) { // If currently setting the time, ignor menu selections - only "SET" button will exit screen
        checkMenus(p.y);  // check menus and execute any selected screen changes
      }
      if ( currentscreen == 0 ) {
        Screen0Chk(p.x, p.y);
      }
      if ( currentscreen == 1 ) {
        Screen1Chk(p.x, p.y);
      }
      if ( currentscreen == 2 ) {
        Screen2Chk(p.x, p.y);
      }
    }
  } 
  // Screen3 is only updated after receiving data from within the "receive" routine directly below
  // the below allow screens to display things like time or sensor readings whether or not the touchscreen has been pressed.
  // it should be noted that x.p and y.p will not be valid without touchscreen input!!
  if ( currentscreen == 0 ) { // this screen has many elements that need to be updated even when there is no screen press
    Screen0Chk(100, 0); // this arbitrary location is not an active area on screen0.  The only active area is the up and down arrows
  }
  if ( currentscreen == 1) { updateScreen1Period; } // the planIndex is updated each cycle within Thermostat.  This will change the highlighted period if on screen 1.
  WriteLongTermData(); // store data in EEPROM
  updateTempHumidity(); // regurlar update of temp/hum average - timer within this subroutine sets update rate
  Thermostat();
  fanDelayChk();
} 

// end of loop

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HAupdate() {
  int pause = 0; // delay for system reaction time
  
  if (mode[COOL]) { // cool mode
    send(msgHVACFlowState.set("CoolOn"));
  } // options for FlowState are CoolOn, HeatOn, AutoChangeOver, and Off.
  else if (mode[HEAT]) { // heat mode
    send(msgHVACFlowState.set("HeatOn"));
  }
  else { // either in fan mode or off.  either way, we will tell HA we are off
    send(msgHVACFlowState.set("Off"));
  }
  wait(pause);
  send(msgHVACsetptHeat.set(Setpt[HEAT - 1][TEMP])); // set up the correct setpoint for either temp mode (shouldn't be in Humdiity cntrl mode).  In both cases, system should be cooling.
  send(msgHVACsetptCool.set(Setpt[COOL - 1][TEMP])); // set up the correct setpoint for either temp mode.
  send(msgHumSetPtStatus.set(1)); // set point is on
  send(msgHumSetPt.set(int(Setpt[COOL - 1][HUM]))); // send humidity setpoint for slider control
  send(msgRunMode.set(runMode)); // Set run mode button on HA 
  send(msgTHBut.set(tempHumidity)); // sets tempHumidity (boolean)
  send(msgHVACRefresh.set(false)); // start with refresh switch off, and turn off switch each time through
  for (int i=0; i<3; i++) {
    send(msgMode[i].set(mode[i])); // update modes
    wait(pause);
    bool state = (digitalRead(START_OF_RELAY_PINS + 2*i) > 0);
    send(msgRelay[i].set(state)); // send relay state
  }
  for (int i=0; i<5; i++) {
    send(msgZone[i].set(zone[i])); // update zones
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkMenus(int py) {
 oldscreen = currentscreen;
 if (py < MENUWIDTH) { 
   currentscreen = 0; 
   tft.drawRect(0+1, 0+1, MENUWIDTH-2, MENUHEIGHT-2, WHITE);
   Screen0SetUp();
 } else if (py < MENUWIDTH*2) {
   currentscreen = 1;
   tft.drawRect(MENUWIDTH+1, 0+1, MENUWIDTH-2, MENUHEIGHT-2, WHITE);
   Screen1SetUp();
 } else if (py < MENUWIDTH*3) {
   currentscreen = 2;
   tft.drawRect(MENUWIDTH*2+1, 0+1, MENUWIDTH-2, MENUHEIGHT-2, WHITE);
   Screen2SetUp();
 } else if (py < MENUWIDTH*4) {
   currentscreen = 3;
   tft.drawRect(MENUWIDTH*3+1, 0+1, MENUWIDTH-2, MENUHEIGHT-2, WHITE);
   Screen3SetUp();
 }
 if (oldscreen != currentscreen) {
    if (oldscreen == 0) {
      tft.fillRect(0, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
      tft.drawRect(0, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
      tft.setCursor(15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE0);
    }
    if (oldscreen == 1) {
      tft.fillRect(MENUWIDTH, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
      tft.drawRect(MENUWIDTH, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
      tft.setCursor(MENUWIDTH + 15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE1);
    }
    if (oldscreen == 2) {
      tft.fillRect(MENUWIDTH*2, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
      tft.drawRect(MENUWIDTH*2, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
      tft.setCursor(MENUWIDTH*2 + 15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE2);
    }
    if (oldscreen == 3) {
      tft.fillRect(MENUWIDTH*3, 0, MENUWIDTH, MENUHEIGHT, DKBLUE);
      tft.drawRect(MENUWIDTH*3, 0, MENUWIDTH, MENUHEIGHT, LTBLUE);
      tft.setCursor(MENUWIDTH*3 + 15,7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println(LABLE3);
    }        
  }
}
        
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void receive(const MyMessage &message) {
  int i, temp, row, zoneX, sensor;
  bool autochangeover = false;

  if (message.isAck()) {
     Serial.println("This is an ack from gateway");
     return;
  }
  
  String recvData = message.data;
  recvData.trim();
  
  switch (message.type) {
    case V_HVAC_SPEED:
      Serial.println("V_HVAC_SPEED");
      if(recvData.equalsIgnoreCase("auto")) FAN_STATE = 0;
      else if(recvData.equalsIgnoreCase("min")) FAN_STATE = 1;
      else if(recvData.equalsIgnoreCase("normal")) FAN_STATE = 2;
      else if(recvData.equalsIgnoreCase("max")) FAN_STATE = 3;
    break;

    case V_HVAC_SETPOINT_COOL:
      Serial.println("V_HVAC_SETPOINT_COOL "); Serial.println(message.getFloat());
      if (Setpt[COOL - 1][TEMP] != message.getFloat()) { // COOL - 1 = 0 and TEMP = 1: just to emphasize we are using the temperature setpoint for cooling
        Setpt[COOL - 1][TEMP] = message.getFloat(); // cool is on for both humidity and temperature control, but humidity setpt is dealt with separately
      }
      if (mode[COOL]) { // in cool mode and tempHumidity == TEMP, TEMP = 1; COOL = 1.  If in cool mode, then we may need to update the screens
          if (currentscreen == 0 ) {Screen0updateSetpt();} else if (currentscreen == 2) { Screen2updateSetpt(); }
      }
      if (runMode) { // check to see if its manual already, or if it's in program mode, switch to manual with a simulated button press.
        if (currentscreen != 1) { checkMenus(MENUWIDTH); }// if not on screen, must change to screen to keep display functioning normally
        Screen1Chk(CellDataPos[3][5][1],CellDataPos[3][5][0]);  // switch to screen1 and/or update.  Use CellDataPos to indicate correct position of imagined button push.
      }
    break;

    case V_HVAC_SETPOINT_HEAT:
      Serial.print("V_HVAC_SETPOINT_HEAT "); Serial.println(message.getFloat());
      if (Setpt[HEAT - 1][TEMP] != message.getFloat()) { // HEAT - 1 = 1 and TEMP = 1: just to emphasize we are using the temperature setpoint for heating
        Setpt[HEAT - 1][TEMP] = message.getFloat();
      }
      if (mode[HEAT]) { // in heat mode: HEAT = 2 .  If in heat mode, then we may need to update the screens          
        if (currentscreen == 0 ) {Screen0updateSetpt();} else if (currentscreen == 2) { Screen2updateSetpt(); }
        }
      if (runMode) { // check to see if its manual already, or if it's in program mode, switch to manual with a simulated button press.
        if (currentscreen != 1) {checkMenus(MENUWIDTH); } // if not on screen 1, must change to screen to keep display functioning normally
        Screen1Chk(CellDataPos[3][5][1],CellDataPos[3][5][0]);  // switch to screen1 and/or update.  Use CellDataPos to indicate correct position of imagined button push.
      }
    break;
    
    case V_HVAC_FLOW_STATE:
      Serial.print("V_HVAC_FLOW_STATE "); Serial.println(recvData);
      if (recvData.equalsIgnoreCase("AutoChangeOver")) {
        // do nothing important until I decide what to do with this
        autochangeover = true;
      }
      else {
        if (currentscreen != 2) {
          checkMenus(MENUWIDTH*2); // if not on screen, must change to screen to keep display functioning normally
        }
        if (recvData.equalsIgnoreCase("CoolOn")) {
          temp = 2;
          if(mode[COOL]) { temp = 0; } // if we are already in the cool mode, we don't need to do anything
        }
        else if (recvData.equalsIgnoreCase("HeatOn")) {
          temp = 3;
          if(mode[HEAT]) { temp = 0; } // if we are already in the heat mode, we don't need to do anything
        }
        else if (recvData.equalsIgnoreCase("Off")){
          temp = 0;  // initialize to do nothing value
          for (i = 0; i < NUMBER_OF_MODES; i++) {  // find out what node changed stat and store in temp
            if(mode[i]) { temp = i+1;}  // which mode is on; can at most be one?  could be fan!
          }
        }
        if (temp != 0) { // make sure we are not already in the off position, or already set appropriately
          Screen2Chk(CellDataPos[0][temp][1],CellDataPos[0][temp][0]);  // switch to screen2 and/or update.  Use CellDataPos to indicate correct position of imagined button push.
        }                   
      }
    break;
    
    case V_PERCENTAGE:  // this corresponds to the humidity setpoint
      Setpt[0][0] = atoi(message.getString(convBuffer)); // humidity setpoint
      if (tempHumidity == HUM) { // in humidity mode
        if (currentscreen == 0) {Screen0updateSetpt();} else if (currentscreen == 2) {Screen2updateSetpt();} // if on screen 0 or 2, then must update
      }
    break;

    // Start with relays and buttons.  Use fake button push technique!!!
    case V_LIGHT:
      if (message.sender == 0) {  // this corresponds to the humidity setpoint
        // could use this to transfer two setpoints instead of just one.  however, here we will just ignor this
      }
      if ( message.sensor == CHILD_ID_HEAT_MODE || message.sensor == CHILD_ID_FAN_MODE || message.sensor == CHILD_ID_COOL_MODE ) {
        if (currentscreen != 2) {checkMenus(MENUWIDTH*2);} // if not on screen, must change to screen to keep display functioning normally
        for (i = 0; i < NUMBER_OF_MODES; i++) {  // find out what node changed stat and store in temp
          if((message.sensor - CHILD_ID_START_OF_MODES) == i) { temp = i; } // which mode?
        }
        Screen2Chk(CellDataPos[0][temp+1][1],CellDataPos[0][temp+1][0]);  // switch to screen2 and/or update.  Use CellDataPos to indicate correct position of imagined button push.                   
      }
      if ( message.sensor == CHILD_ID_ZONE1 || message.sensor == CHILD_ID_ZONE2 || message.sensor == CHILD_ID_ZONE3 || message.sensor == CHILD_ID_ZONE4 || message.sensor == CHILD_ID_ZONE5 ){
        if (currentscreen != 2) {checkMenus(MENUWIDTH*2);} // if not on screen, must change to screen to keep display functioning normally
        for (i = 0; i < NUMBER_OF_ZONES; i++) {  // find out what node changed state and store in temp
          if((message.sensor - CHILD_ID_START_OF_ZONES) == i) { temp = i; } // which zone?
        }
        Screen2Chk(CellDataPos[2][temp][1],CellDataPos[2][temp][0]);  // switch to screen2 and/or update.  Use CellDataPos to indicate correct position of imagined button selection.
      }
      if ( message.sensor == CHILD_ID_RUN_BUTTON) {
        if (currentscreen != 1) {checkMenus(MENUWIDTH);} // if not on screen, must change to screen to keep display functioning normally; mimic touchscreen push on menu(screen) 1
        Screen1Chk(CellDataPos[3][5][1],CellDataPos[3][5][0]);  // Use CellDataPos to indicate correct position of imagined run button selection.      
      }
      if (message.sensor == CHILD_ID_TH_BUTTON) {
        if (currentscreen != 2) {checkMenus(MENUWIDTH*2);} // if not on screen, must change to screen to keep display functioning normally; mimic touchscreen push on menu (screen) 2
        Screen2Chk(CellDataPos[1][4][1],CellDataPos[1][4][0]);  // Use CellDataPos to indicate correct position of imagined T/H button selection.
      }
      if (message.sensor == CHILD_ID_HVAC_REFRESH) {
        HAupdate();
      }
    break;
    
    case V_HUM: // ("living","dining","master","guest ","study ") tempHumidityData[2][5] (1 is temp, 0 is hum; in room order )
      // must be a zone
      // Write some debug info
      row = message.sender - START_OF_ZONE_SENSOR_NODE_IDS; // presumes the zone nodes are in order starting from START_OF_ZONE_SENSOR_CHILD_IDS
      tempHumidityData[0][row] = message.getFloat() + humCal[row]; // places value in correct memory location
      if ( currentscreen == 3 ) { // must update data if on Screen 3
         Screen3Chk(3, row); // column is 3 because it's humidity
      }
    break;

    case V_TEMP:
      // must be a zone
      // Write some debug info
      row = message.sender - START_OF_ZONE_SENSOR_NODE_IDS; // presumes the zone nodes are in order starting from START_OF_ZONE_SENSOR_CHILD_IDS
      tempHumidityData[1][row] = message.getFloat() + tempCal[row]; // places value in correct memory location
      if ( currentscreen == 3 ) { // must update data if on Screen 3
        Screen3Chk(2, row); // column is 3 because it's temperature
      }
    break;

    case V_TRIPPED:
      // This will be a louver position indicator corresponding to the variable louverState[zone][open/closed sensor] (this is preset to indicate open as the default)
      zoneX = message.sender - 2; // zone NODE_IDs are 2 thru 6 corresponding to zones 0 thru 4
      sensor = message.sensor - 3; //zone CHILD_IDs are 3 and 4 for open and closed
      louverState[zoneX][sensor] = !message.getBool(); // update the louver states (louver states are inverted by zone control to make HomeAssistant's display more 
                                                      // intuitive - 'open' is 'on' translates to 0 is true?? in HA.  So, to be clear.  The sensor hardware is designed to 
                                                      // output a high when the physical flag intersects the optical sensor.  When it doesn't the sensor pulls the signal
                                                      // low.  So a true FLAG means the physical flag is in position.  However, since I am using the door sensor, it 
                                                      // turns yellow in HomeAssistant when the door is open (false).  To compensate, the zone controller inverts the
                                                      // output, making the FLAG false when closed.  This code inverts it back when stored in 'louverState' so that 
                                                      // true has the intuitive sense that it means the physical flag is indeed blocking the optical sensor.
      if (zone[zoneX] && (sensor==1) && message.getBool()) { // if the zone was switched to open (true), sensor is OPEN_FLAG (1) and indicating true
        louverAttemptCount[zoneX][sensor] = 0; // counter corresponding zone and sensor must be reset when xFLAG corresponds
      }
      if (!zone[zoneX] && (sensor==0) && message.getBool()) { // if the zone was switched to closed (false), sensor is CLOSED_FLAG (0) and indicating true
        louverAttemptCount[zoneX][sensor] = 0; // counter corresponding zone and sensor must be reset when xFLAG corresponds
      }
    break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkRTC() {
  // let's check to see if clock is responding
  Wire.beginTransmission(RTC_I2C_address);
  I2Cerror = Wire.endTransmission();
  if (I2Cerror>0) {
    RTCavail = false; 
    runMode = false;
  }
  else {
    RTCavail = true;
    DOW = rtc.getDOWStr(1);
    timeString = rtc.getTimeStr(); // timeString is a temporary time variable
    hourMin[0] = timeString.substring(0,2).toInt(); // subsection of hours 
    hourMin[1] = timeString.substring(3,5).toInt(); // subsection of minutes
    dateString = rtc.getDateStr();
    monthDayYear[0] = dateString.substring(0,2).toInt();
    monthDayYear[1] = dateString.substring(3,5).toInt();
    monthDayYear[2] = dateString.substring(6,10).toInt(); 
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int getPlanTemp() { // obtain the current temperature set point based on the plan
  bool weekendTest = (DOW == "Sun" || DOW == "Sat"); // if true, it's a weekend (true = 1?);
  
  getPlanIndex(); // note: planIndex is global, the getPlanIndex() is used to update planIndex

  if (!(planIndex == 6)) { // RTC is available, so we can continue running in runMode
    planTemp = DTPlanTemp[heatCoolPlan][!weekendTest][planIndex]; // go to the plan and get the appropriate temperature
  }
  // Serial.print("Plan Index = ");Serial.print(planIndex);Serial.print("    Plan Temp = ");Serial.println(planTemp);
  // if RTC not available, checkRTC() will already have made runMode false.  checkRTC() executed in getPlanIndex()! 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getPlanIndex() { // obtain the index of the current set point based on the current time from the schedule
  int hours;
  int mins;
  int i;
  long currentTime; 
  long scheduleTime;
  // note: planIndex is global

  checkRTC();
  if (RTCavail) { // if RTC not available, must exit automatic mode
    hours = hourMin[0]; // subsection of hours 
    mins = hourMin[1]; // subsection of minutes
    bool weekendTest = (DOW == "Sun" || DOW == "Sat"); // if true, it's a weekend (true = 1?)
    currentTime = hours*60 + mins;
    
    for (i = 0; i < 6; i++) {
      scheduleTime = DTPlanHours[heatCoolPlan][!weekendTest][i]*60 + DTPlanMins[heatCoolPlan][!weekendTest][i]; // initial test value (time in minutes!)
      if (scheduleTime > currentTime) { // we've passed the active shedule time
        if (i > 0) {
          planIndex = i - 1;
        }
        else {
          planIndex = 5; // if i is 0 then i - 1 = -1, so must change to 5
        }
        i = 6; // exit loop
      }
    }
    if (scheduleTime < currentTime) {  // scheduleTime is at index 5, so if its less than the current time it must be active
      planIndex = 5;
    }
  }
  else {planIndex = 6;} // default value for room temperature and offscale value for planIndex to prevent incorrect display modification
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setRelays(int onPin) { // 0 = fan relay, 1 = cool relay, 2 = heat relay, and 3 == all off
  
  switch(onPin) {
    case 0:  //fan relay on
      digitalWrite(FAN_RELAY_PIN,RELAY_ON);
      digitalWrite(COOL_RELAY_PIN, RELAY_OFF);
      digitalWrite(HEAT_RELAY_PIN, RELAY_OFF);
      fanDelay = false;
      break;
    case 1: // cool relay on
      digitalWrite(FAN_RELAY_PIN,RELAY_ON);
      digitalWrite(COOL_RELAY_PIN, RELAY_ON);
      digitalWrite(HEAT_RELAY_PIN, RELAY_OFF);
      fanDelay = false;
      break;
    case 2: // heat relay on
      digitalWrite(FAN_RELAY_PIN,RELAY_ON);
      digitalWrite(COOL_RELAY_PIN, RELAY_OFF);
      digitalWrite(HEAT_RELAY_PIN, RELAY_ON);
      fanDelay = false;
      break;
    case 3:   
      if (digitalRead(FAN_RELAY_PIN)) { // if the fan is on, we delay in turning it off.  This is true even if it is the only relay that's on.
        fanDelayStartTime = millis(); // to be compared with fanDelayTime (amount of delay) and the current time
        fanDelay = true;  // forces delayed fan relay turn off from within main loop
      }
      digitalWrite(COOL_RELAY_PIN, RELAY_OFF);
      digitalWrite(HEAT_RELAY_PIN, RELAY_OFF);
      break;
    }
  for (int i=0; i<3; i++) {  // update relay indicators
    bool state = (digitalRead(START_OF_RELAY_PINS + 2*i) > 0);
    send(msgRelay[i].set(state)); // send relay state
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void fanDelayChk() { //after heat or cool relay are shut off, this routine delays before shutting off the fan

  if (fanDelay) { // Check fan delay flag to see if cool or heat has been turned off and fan needs to be turned off after a delay
    if ((millis() - fanDelayStartTime) > FAN_DELAY_TIME) {
      digitalWrite(FAN_RELAY_PIN, RELAY_OFF);
      bool state = (digitalRead(FAN_RELAY_PIN) > 0);
      send(msgRelay[FAN].set(state)); // send relay state
      fanDelay = false;
    }
  }
  if ((digitalRead(HEAT_RELAY_PIN) == 1) || (digitalRead(COOL_RELAY_PIN) == 1)) { // under no condition will the fan be off when the heat or cool are on!!!
    if (digitalRead(FAN_RELAY_PIN) == 0) { // check if it is off, then turn on!
      digitalWrite(FAN_RELAY_PIN, RELAY_ON); 
      bool state = (digitalRead(FAN_RELAY_PIN) > 0); // just double checking!?
      send(msgRelay[FAN].set(state)); // send relay state
    }
  } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thermostat() { // code to control the heater and air conditioner
  int onPin;
  int i;

  if (mode[FAN]) { // FAN mode active
    return; // nothing to do but leave the fan running.  Must turn fan off in "MANUAL" screen!
  }
  if ((millis() - thermostatOldMillis) < 0) { thermostatOldMillis = 0; } // millis() rolled over!
  if ((millis() - thermostatOldMillis) < SUSTAIN_TIME) { return; } // prevents the termostat from turning on an off the AC unless sufficient time has past
  else if (mode[COOL]) { // COOL mode active
    if (tempHumidity) { // if true, we are in temperature mode
      getPlanTemp(); // updates planTemp (a gobal variable)
      if (runMode) { // determine set point based on schedule first 
        if (indoorTemperature > (planTemp + TEMP_ANTICIPATION)) {  // calling for cooling: ( 0, 1, 2, 3 ) = (fan, cool, heat, off)
          if (!digitalRead(COOL_RELAY_PIN)) { setRelays(COOL); thermostatOldMillis = millis(); }
          louverAdjust(planTemp);
        }
        else if (indoorTemperature < (planTemp - TEMP_ANTICIPATION)) { // not calling for cooling
          if (digitalRead(COOL_RELAY_PIN)) { setRelays(OFF); thermostatOldMillis = millis(); } // if cool relay is on, turn it and all others off
          louverAdjust(planTemp);
        }
      }
      else {  // set point mode
        if (indoorTemperature > (Setpt[mode[HEAT]][1] + TEMP_ANTICIPATION)) {  // calling for cooling
          if (!digitalRead(COOL_RELAY_PIN)) { setRelays(COOL); thermostatOldMillis = millis(); }
          louverAdjust(Setpt[mode[HEAT]][1]);
        }
        else if (indoorTemperature < (Setpt[mode[HEAT]][1] - TEMP_ANTICIPATION)) { // not calling for cooling
          if (digitalRead(COOL_RELAY_PIN)) { setRelays(OFF); thermostatOldMillis = millis(); }// if cool relay is on, turn it and all others off
          louverAdjust(Setpt[mode[HEAT]][1]);
        }
      }      
    }
    else { // we are in humidity mode
      if (indoorHumidity > (Setpt[0][0] + HUM_ANTICIPATION)) {  // calling for cooling
        if (!digitalRead(COOL_RELAY_PIN)) { setRelays(COOL); thermostatOldMillis = millis(); } 
        louverAdjust(Setpt[0][0]);
      }
      else if (indoorHumidity < (Setpt[0][0] - HUM_ANTICIPATION)) { // not calling for cooling
        if (digitalRead(COOL_RELAY_PIN)) { setRelays(OFF); thermostatOldMillis = millis(); } // if cool relay is on, turn it and all others off
        louverAdjust(Setpt[0][0]);
      }  
    }
  }
  else if (mode[HEAT]) { // HEAT mode active
    getPlanTemp(); // updates planTemp (a gobal variable)
    if (runMode) { // determine set point based on schedule first 
      if (indoorTemperature > (planTemp + TEMP_ANTICIPATION)) {  // not calling for heat
        if (digitalRead(HEAT_RELAY_PIN)) { setRelays( OFF ); thermostatOldMillis = millis(); } // if the heat is on, turn it off
        louverAdjust(planTemp);
      }
      else if (indoorTemperature < (planTemp - TEMP_ANTICIPATION)) { // calling for heat
        if (!digitalRead(HEAT_RELAY_PIN)) { setRelays(HEAT); thermostatOldMillis = millis(); }
        louverAdjust(planTemp);
      }
    }
    else {  // set point mode
      if (indoorTemperature > (Setpt[mode[HEAT]][1] + TEMP_ANTICIPATION)) {  // not calling for heat
        if (digitalRead(HEAT_RELAY_PIN)) { setRelays( OFF ); thermostatOldMillis = millis(); } // if the heat is on, turn it off
        louverAdjust(Setpt[mode[HEAT]][1]);
      }
      else if (indoorTemperature < (Setpt[mode[HEAT]][1] - TEMP_ANTICIPATION)) { // calling for heat
        if (!digitalRead(HEAT_RELAY_PIN)) { setRelays(HEAT); thermostatOldMillis = millis(); }
        louverAdjust(Setpt[mode[HEAT]][1]);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void louverAdjust(float Setpt) { // Here the Setpt is the exact temperature at which the louver will open or close - no offsets!

  bool sustain; 
  int steps = 100; // number of steps for the stepping motor
  int i;

  if (mode[FAN] || !(mode[FAN] || mode[COOL] || mode[HEAT])) { // in fan mode or no other mode is on, then exit subroutine, the system is in off mode
    return; 
  }
  if (mode[HEAT]) { // we are in heating mode
    for (i = 0; i < 5; i++) { // scan through zones
      if (zone[i]) { // only adjust zones that are active
        if (millis() - zoneOldMillis[i] < 0) { zoneOldMillis[i] = 0; } // millis() rolled over!
        sustain = (millis() - zoneOldMillis[i] < SUSTAIN_TIME); // if sustain is true then the louver was just opened or closed
        if (sustain) {return; } // if the louver is closed and we are not sustaining, then open the louver
        if (tempHumidityData[1][i] < (Setpt - TEMP_ANTICIPATION)) { 
          openLouver(i);
          zoneOldMillis[i] = millis(); // if the zone louver just opened, must start timer and only close louver if it hasn't just been shut
        }
        else if (tempHumidityData[1][i] > (Setpt + TEMP_ANTICIPATION)) { 
          closeLouver(i);
          zoneOldMillis[i] = millis(); // if the zone louver just closed, must start timer and only open louver if it hasn't just been shut
        }
      }
    }
  }
  if (mode[COOL]) { // we are in cooling mode
    for (i = 0; i < 5; i++) { // scan through zones
      if (zone[i]) { // only adjust zones that are active
        if (millis() - zoneOldMillis[i] < 0) { zoneOldMillis[i] = 0; } // millis() rolled over!
        sustain = (millis() - zoneOldMillis[i] < SUSTAIN_TIME); // if sustain is true then the louver was just opened or closed
        if (sustain) {return; } // if the louver is closed and we are not sustaining, then open the louver
        if (tempHumidityData[tempHumidity][i] > (Setpt + TEMP_ANTICIPATION*tempHumidity + HUM_ANTICIPATION*(1 - tempHumidity))) { 
          openLouver(i); 
          zoneOldMillis[i] = millis(); // if the zone louver just opened, must start timer and only close louver if it hasn't just been shut
        }
        if (tempHumidityData[tempHumidity][i] < (Setpt - TEMP_ANTICIPATION*tempHumidity - HUM_ANTICIPATION*(1 - tempHumidity))) {
          closeLouver(i);
          zoneOldMillis[i] = millis(); // if the zone louver just closed, must start timer and only open louver if it hasn't just been shut
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void closeLouver(int zoneX) {
  int steps = 100;
  int error;
  bool openClose = false;
  
  if (louverState[zoneX][0]) {  // if the closed louver flag is true, the louver is already closed. louver..[x][0] is the CLOSED_FLAG.
    louverAttemptCount[zoneX][0] = 0; // counter corresponding to CLOSED_FLAG.  This should be redundant - it's changed in the 'receive' routine
    }
  else { // louver is not closed
    if(louverAttemptCount[zoneX][0] < MAX_LOUVER_ATTEMPT_COUNT) {
      send(msgMotoropenClose[zoneX].setDestination(START_OF_ZONE_SENSOR_NODE_IDS + zoneX).set(openClose)); // close louver direction
      send(msgMotorPos[zoneX].setDestination(START_OF_ZONE_SENSOR_NODE_IDS + zoneX).set(steps)); // number of steps (100 will be more than enough)
      louverAttemptCount[zoneX][0] += 1; // increment counter
    }
    else { // send louver error 
      error = louverState[zoneX][0] + 2*louverState[zoneX][1]; // binary indication of the state of the louver
      // send error to controller
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void openLouver(int zoneX) {
  int steps = 100;
  int error;
  bool openClose = true;
  
  if (louverState[zoneX][1]) {  // if the open louver flag is true, the louver is already open. louver..[x][0] is the OPEN_FLAG.
    louverAttemptCount[zoneX][1] = 0; // counter corresponding to OPEN_FLAG.  This should be redundant - it's changed in the 'receive' routine
    }
  else { // louver is not open
    if(louverAttemptCount[zoneX][1] < MAX_LOUVER_ATTEMPT_COUNT) {
      send(msgMotoropenClose[zoneX].setDestination(START_OF_ZONE_SENSOR_NODE_IDS + zoneX).set(openClose)); // open louver direction
      send(msgMotorPos[zoneX].setDestination(START_OF_ZONE_SENSOR_NODE_IDS + zoneX).set(steps)); // number of steps (100 will be more than enough)
      louverAttemptCount[zoneX][1] += 1; // increment counter
    }
    else { // send louver error 
      error = louverState[zoneX][0] + 2*louverState[zoneX][1]; // binary indication of the state of the louver
      // send error to controller
    }
  }
}
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void i2c_eeprom_write( int deviceaddress, unsigned int eeaddress, byte data ) {
    int rdata = data;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(rdata);
    Wire.endTransmission();
    wait(5); // ran into storage problem
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte i2c_eeprom_read( int deviceaddress, unsigned int eeaddress ) {
    byte rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void WriteLongTermData() { // data stored as bytes
  int i;
  int j;
  int k;
  int dataCount = 86;
  byte data[dataCount]; // 86 bytes to store.  You have to keep track of the memory map so as not to over write existing data.  
  byte storedData[dataCount];
  
  for (k=0; k<2; k++) {
    for (j=0; j<2; j++) {
      for (i=0; i<6; i++) {
        data[i+j*6+k*12] = DTPlanTemp[k][j][i]; // 24 values --> 2 * 2 * 6
        data[i+j*6+k*12+24] = DTPlanHours[k][j][i]; // this index must start at 24
        data[i+j*6+k*12+48] = DTPlanMins[k][j][i]; // this index must start at 48 to accommodate previous values
      }
    }
  }
  
  data[72] = Setpt[0][0];
  data[73] = Setpt[0][1];
  data[74] = Setpt[1][0];
  data[75] = Setpt[1][1];

  // the last data are booleans which will be translated to 1 or 0 before storing.  hopefully, upon reading the data, they will be converted back to booleans!
  for (i=0; i<3; i++) { // save modes
    data[i+76] = mode[i]; // data[76] - mode[FAN]
  }

  data[79] = runMode;
  data[80] = tempHumidity;

  for (i=0; i<5; i++) { // save zones
    data[i+81] = zone[i]; // data[81] - zone[0]
  } 
  
  for (i = 0; i < dataCount; i++ ) { // index on data points rather than bytes
    if(data[i] != storedData[i]) {
      i2c_eeprom_write(disk1, i, data[i]); 
    }
  }
  // stored 86 values
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReadLongTermData() {
  int i;
  int j;
  int k;
  int dataCount = 84;
  int data[dataCount];

  for (i=0; i<dataCount; i++) {
    data[i] = i2c_eeprom_read(disk1, i);
  }

  for (k=0; k<2; k++) {
    for (j=0; j<2; j++) {
      for (i=0; i<6; i++) {
        DTPlanTemp[k][j][i] = data[i+j*6+k*12];
        DTPlanHours[k][j][i] = data[i+j*6+k*12+24];
        DTPlanMins[k][j][i] = data[i+j*6+k*12+48];
      }
    }
  }

  Setpt[0][0] = data[72];
  Setpt[0][1] = data[73];
  Setpt[1][0] = data[74];
  Setpt[1][1] = data[75];

    // the last data are booleans which will be translated to 1 or 0 before storing.  hopefully, upon reading the data, they will be converted back to booleans!
  for (i=0; i<3; i++) { // restore modes
     mode[i] = data[i+76]; // data[74] - mode[FAN]
  }

  runMode = data[79];
  tempHumidity  = data[80];

  for (i=0; i<5; i++) { // restore zones
    zone[i] = data[i+81]; // data[81] - zone[0]
  } 
  // 86 recovered values
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateTempHumidity() {
  float humidity, temperature;

  //Serial.print("made it to updateTempHumidity   "); Serial.print("oldMillis = ");Serial.print(oldMillis); Serial.print("    Millis() = "); Serial.println(millis());
  if(oldMillis > millis()) { oldMillis = 0; } // clock rolled over!!!
  if ((millis() - oldMillis) > DHTdelay + 1000) {
    oldMillis = millis();
    dht.readSensor(true); // Force reading sensor, so it works also after sleep()
    DHTerror = dht.getStatusString();
    //Serial.print("DHT error = ");Serial.println(DHTerror);
    if (DHTerror == "OK") {
      humidity = dht.getHumidity();
      temperature = dht.toFahrenheit(dht.getTemperature());
      //Serial.print("humidity = ");Serial.print(humidity);Serial.print("    temperature = ");Serial.println(temperature);
      if (DHTcounter == DHTcounterMax) {
        DHTcounter = 0;
        indoorTemperature = (sumTemperature + temperature)/DHTcounterMax + TEMP_ADJUST; // final average; only place where indoorTemperature is updated
        indoorHumidity = (sumHumidity + humidity)/DHTcounterMax + HUM_ADJUST;
        sumTemperature = 0;
        sumHumidity = 0;
        send(msgTemp.set(indoorTemperature, 1)); // send wireless info to gateway. Sends only a single decimal point
        send(msgHum.set(indoorHumidity, 1));
        send(msgHVACTemp.set(indoorTemperature, 1)); // must update the HA thermostat separately
      }
      else {
        sumTemperature += temperature;
        sumHumidity += humidity;
        DHTcounter += 1; // increment counter
        DHTerror = "Summing";
      }
    }
    else {
      
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen0updateStatus() {
  tft.fillRect(0, 195, 80, 35, LTBLUE); // clear screen area for status settings
  tft.setCursor(15,195); tft.setTextColor(WHITE); tft.setTextSize(1);
  if (mode[FAN]) {tft.print("fan only");} else if (mode[COOL]) {tft.print("cooling");} else if (mode[HEAT]) {tft.print("heating");}
  else if (!(mode[FAN] || mode[COOL] || mode[HEAT])) {tft.print("off");}
  tft.setCursor(15,205);
  if (runMode) {tft.print("running");} else {tft.print("set pt");}
  tft.setCursor(15,215);
  if (tempHumidity) {tft.print("temp cntrl");} else {tft.print("humidity cntrl");}
}  

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen0updateTempHumidity() {
  String error;

  //1Serial.println("made it to Screen0updateTempHumidity");
  //if (DHTerror == "Summing") { return; } // wait for temperature to be averaged
  if ((DHTerror != "OK") && (DHTerror != lastError)) { // otherwise, if the error is not equal to OK, must not be getting temp/hum data
    lastError = DHTerror;
    if (currentscreen == 0) { 
      tft.fillRect(190, 100, 120, 20, LTBLUE);
      tft.setTextColor(RED); tft.setTextSize(1);  
      tft.setCursor(190,100); tft.setTextSize(1); tft.print("DHT ERROR ,"); tft.print(DHTerror);
    }
  }
  else {
    if ((indoorTemperature != lastIndoorTemperature) && currentscreen == 0) {
      lastIndoorTemperature = indoorTemperature;
      tft.fillRect(190, 60, 120, 40, LTBLUE);
      tft.setTextColor(WHITE); tft.setTextSize(3); 
      tft.setCursor(200,60); tft.print(indoorTemperature, 1); tft.print("F"); // prints temperature with one decimal place
    } 
    if ((indoorHumidity != lastIndoorHumidity) && currentscreen == 0) {
      lastIndoorHumidity = indoorHumidity;
      tft.fillRect(190, 100, 120, 20, LTBLUE);
      tft.setTextColor(WHITE); tft.setTextSize(1);
      tft.setCursor(190,100); tft.setTextSize(1); tft.print(indoorHumidity); tft.print("% Humidity"); // prints more than one decimal place
    }
  }  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen0updateClock() {
  checkRTC();
  if (RTCavail) { // update time display on screen
    // at the turn of the hour, oldMin = 59 and hourMin[1] = 0, so the difference is -59!!!
    if (abs(hourMin[1]-oldMin) > 1) { (oldMin = hourMin[1] - 1); } // if for whatever reason the system gets our of sync, and on the hour, this should correct.
    if (((hourMin[1] - oldMin) == 1 ) || (!oldRTCavail)) {
      tft.fillRect(0, 20, 150, 80, LTBLUE);
      tft.setCursor(15,40); tft.setTextColor(WHITE); tft.setTextSize(1); tft.print(rtc.getDateStr(2,3,47)); // get date from Real Time Clock
      tft.setCursor(15,55); tft.setTextSize(2); tft.print(timeString.substring(0,5)); // get time from Real Time Clock but only print hour:min 
      tft.setCursor(15,80); tft.setTextSize(2); tft.print(rtc.getDOWStr(0)); // get DOW from Real Time Clock  
      oldMin = hourMin[1];  
    }
  }
  else {
    if (oldRTCavail) { // if oldRTCavail is true, but now the RTC has failed, we need to update the screen
      tft.fillRect(0, 20, 150, 80, LTBLUE);
      tft.setCursor(15,40); tft.setTextColor(RED); tft.setTextSize(1); tft.print("RTC UNAVAILABLE"); // show error RTC unavailable
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen0SetUp() {

  tft.fillRect(0,20,320,240,LTBLUE); // clear screen
  checkRTC();
  if (RTCavail) {
    oldMin = hourMin[1] - 1; // making oldMin 1 minute less than the current time will force thte Screen0updataClock() routine to display the clock
  }
  oldRTCavail = RTCavail; // this sets up the time for a refresh upon a temporary loss of the real time clock
  Screen0updateClock();
  
  tft.setCursor(190,40); tft.setTextColor(YELLOW); tft.setTextSize(2); tft.print("INDOOR");
  tft.setCursor(200,60); tft.setTextColor(WHITE); tft.setTextSize(3); tft.print(indoorTemperature, 1); tft.print("F"); // prints temperature with one decimal place
  tft.setCursor(190,100); tft.setTextSize(1); tft.print(indoorHumidity); tft.print("% Humidity");
  
  tft.setCursor(15,110); tft.setTextColor(YELLOW); tft.setTextSize(2); tft.print("OUTDOOR");
  tft.setCursor(15,135); tft.setTextColor(WHITE); tft.setTextSize(1); tft.print(Toutdoor); tft.print(" Degrees F"); 
  tft.setCursor(15,145); tft.print(Houtdoor); tft.print("% Humidity");
  
  tft.setCursor(15,170); tft.setTextColor(YELLOW); tft.setTextSize(2); tft.print("STATUS");
  Screen0updateStatus();
  
  tft.setCursor(190,130); tft.setTextColor(YELLOW); tft.setTextSize(2); tft.print("SET POINT");
  if (runMode) { // if runMode is true, then you need the planTemp for the setpoint, otherwise, you need the manual set point
    tft.setCursor(240,190); tft.setTextColor(WHITE); tft.setTextSize(4); tft.print(planTemp); tft.print("F");
  }
  else { // this is for manual mode.  Humidity mode is manual!
    tft.setCursor(240,190); tft.setTextColor(WHITE); tft.setTextSize(4); tft.print(Setpt[mode[HEAT]][tempHumidity]); 
    if (tempHumidity) {tft.print("F");} else {tft.print("%");}
  }

  tft.fillTriangle(210, 170, 190, 190, 230, 190, DKBLUE);
  tft.fillTriangle(210, 230, 190, 210, 230, 210, DKBLUE); 

  tft.drawLine(160,40,160,220,WHITE); // vertical line
  tft.drawLine(15,130,120,130,WHITE); // line below 'OUTDOOR'
  tft.drawLine(15,190,120,190,WHITE); //line below 'STATUS'
  tft.drawLine(180,120,300,120,WHITE); // line above 'SET POINT'
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen0updateSetpt() {
  
  tft.fillRect(240, 190, 80, 40, LTBLUE); 
  tft.setCursor(240,190); tft.setTextColor(WHITE); tft.setTextSize(4); tft.print(Setpt[mode[HEAT]][tempHumidity]); 
  if (tempHumidity) {tft.print("F");} else {tft.print("%");}

  // now we need to update HomeAssistant
  if (mode[COOL]) { // cooling
    send(msgHVACFlowState.set("CoolOn"));
    if (tempHumidity) { // must send the correct setpoint in msg
      send(msgHVACsetptCool.set(Setpt[COOL - 1][tempHumidity])); // could be either in humidity or temperature control mode, however, Setpt[0][x] is cool mode
    }
    else {
      send(msgHumSetPtStatus.set(1)); // set point is on
      send(msgHumSetPt.set(int(Setpt[COOL - 1][HUM]))); // send humidity setpoint for slider control
    }
  }
  else if (mode[HEAT]) { // heating
    send(msgHVACsetptHeat.set(Setpt[HEAT - 1][TEMP]));  // tempHumidity must be true --> temp mode, so Setpt[1][1] is heat mode, temp
    send(msgHVACFlowState.set("HeatOn"));
  }
  wait(WAIT);
  Screen0updateStatus();
}
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen0Chk(int py, int px) { // for touch screen x and y axis are opposite -- x==y, y==x
  
  Screen0updateClock();
  Screen0updateTempHumidity(); // get updated temperature and humidity (temperature in celcius)
  if (mode[FAN] || (py == 100)) { return; } // if it's in fan mode or the screen was not pressed, then exit.  If py = 100 it means this is a main loop update - no screen press!
  if ((px < 240) && (px > 180) && (py < 200) && (py > 160)) { // up triangle
    Setpt[mode[HEAT]][tempHumidity] = Setpt[mode[HEAT]][tempHumidity] + 1; // mode[HEAT] is true if the heat mode is selected, but if mode[HEAT] is false, then in cool mode since FAN already tested.
    if (Setpt[mode[HEAT]][tempHumidity] > 85) { Setpt[mode[HEAT]][tempHumidity] = 85; }
    runMode = false; // changing setpt takes system out of automated mode
    Screen0updateSetpt(); // this will also update HomeAssistant
  }
  if ((px < 240) && (px > 180) && (py < 239) && (py > 200)) { // down triangle
    Setpt[mode[HEAT]][tempHumidity] = Setpt[mode[HEAT]][tempHumidity] - 1; 
    if (Setpt[mode[HEAT]][tempHumidity] < 40) { Setpt[mode[HEAT]][tempHumidity] = 40; } // lower limit must accomodate humidity range
    runMode = false; // changing setpt takes system out of automated mode
    Screen0updateSetpt(); // this will also update HomeAssistant
  }
}   

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen1SetUp() {
  int i;
  
  // submenu items
  tft.fillRect(0, 20, 320, 240, LTBLUE); // clear screen area below main menu
  tft.fillRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE); // weekdays
  tft.drawRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, WHITE);
  tft.fillRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE); // weekends
  tft.drawRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
  tft.fillRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE); // timeset
  tft.drawRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
  tft.setCursor(15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("weekdays");
  tft.setCursor(MENUWIDTH + 15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("weekends");
  tft.setCursor(MENUWIDTH*2 + 15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("date/time?");

  // labels
  tft.setCursor(5,45); tft.setTextColor(YELLOW); tft.setTextSize(2); tft.print("PERIOD");  
  tft.setCursor(95,45); tft.print("TIME"); 
  tft.setCursor(170,45); tft.print("TEMP");
  tft.setCursor(245,45); tft.print("SCHED?");  
  tft.setCursor(250,135); tft.print(" ADJ");
  for (i = 0; i < 6; i++ ) { // start of PERIOD COLUMN
    tft.setCursor(35,70 + i*30); tft.print(String(i+1));
  }

  // up down arrows
  tft.fillTriangle(280, 155, 260, 180, 300, 180, DKBLUE);
  tft.fillTriangle(280, 210, 260, 185, 300, 185, DKBLUE);

  // heat plan button
  tft.fillRect(250, 65, 60, 25, GREEN*heatCoolPlan + RED*(1-heatCoolPlan));
  tft.drawRect(250, 65, 60, 25, DKGREEN);
  tft.setCursor(258,70); tft.setTextColor(DKGREEN); tft.setTextSize(2); tft.println("HEAT");

  // cool plan button
  tft.fillRect(250, 95, 60, 25, RED*heatCoolPlan + GREEN*(1-heatCoolPlan));
  tft.drawRect(250, 95, 60, 25, DKGREEN);
  tft.setCursor(258,100); tft.setTextColor(DKGREEN); tft.setTextSize(2); tft.println("COOL");
  
  // run button
  tft.fillRect(250, 217, 60, 26, GREEN*runMode + RED*(1-runMode));
  tft.drawRect(250, 217, 60, 26, DKGREEN);
  tft.setCursor(263,222); tft.setTextColor(DKGREEN); tft.setTextSize(2); tft.println("RUN");

  // separation lines
  tft.drawLine(230,60,230,220,WHITE); // vertical line
  tft.drawLine(250,125,310,125,WHITE); // horizontal line above ADJ arrows
  tft.drawLine(250,212,310,212,WHITE); // horizontal line above RUN button
   
  Screen1Data();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen1Data() {
  int i;

  getPlanIndex(); // we only want the planIndex here to update the PERIOD display section
  
// clear cell contents and rewrite latest data
  tft.setTextColor(WHITE); tft.setTextSize(2); 
  for (i=0; i<6; i++) {
    tft.fillRect(cellFrame[1][i][0],cellFrame[1][i][1],cellFrame[1][i][2],cellFrame[1][i][3], LTBLUE); // TIME COLUMN
    tft.fillRect(cellFrame[2][i][0],cellFrame[2][i][1],cellFrame[2][i][2]-20,cellFrame[2][i][3], LTBLUE); // TEMP COLUMN
    tft.drawRect(cellFrame[1][i][0],cellFrame[1][i][1],cellFrame[1][i][2],cellFrame[1][i][3], DKBLUE); // TIME COLUMN
    tft.drawRect(cellFrame[2][i][0],cellFrame[2][i][1],cellFrame[2][i][2]-20,cellFrame[2][i][3], DKBLUE); // TEMP COLUMN
    // setup time array display    
    if (i>0 && ((DTPlanHours[heatCoolPlan][weekdays][i]+DTPlanMins[heatCoolPlan][weekdays][i]/60) < DTPlanHours[heatCoolPlan][weekdays][i-1]+DTPlanMins[heatCoolPlan][weekdays][i-1]/60)) {
      tft.setTextColor(RED);
    }
    else {tft.setTextColor(WHITE);}
    sprintf_P(TimeTime, (PGM_P)F("%02d:%02d"), DTPlanHours[heatCoolPlan][weekdays][i] , DTPlanMins[heatCoolPlan][weekdays][i]); // convert time to string
    tft.setCursor(CellDataPos[1][i][0],CellDataPos[1][i][1]); tft.print(TimeTime);
    // setup temperature array display
    tft.setCursor(CellDataPos[2][i][0]+boxIncrmt,CellDataPos[2][i][1]); tft.print(DTPlanTemp[heatCoolPlan][weekdays][i]);
  }
  updateScreen1Period();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateScreen1Period() {

  if (planIndex == oldPlanIndex) { return; } // prevents screen flicker
  else { oldPlanIndex = planIndex; } // update oldPlanIndex
  for (int i=0; i<6; i++) {
    // update the PERIOD display
    bool weekdayWeekend = !(DOW == "Sun" || DOW == "Sat"); // if weekdayWeekend is true, the current day (today) is a weekday, otherwise it's a weekend
    bool weekdaySched = weekdayWeekend && weekdays; // if weekdaySched is true, we are on a weekday schedule and we are displaying weekday data
    bool weekendSched = !weekdayWeekend && !weekdays; // if weekendSched is true, we are on a weekend schedule and we are displaying weekend data
    // overwrite all PERIOD numbers
    tft.setTextSize(2);
    tft.fillRect(cellFrame[0][i][0],cellFrame[0][i][1],cellFrame[0][i][2],cellFrame[0][i][3], LTBLUE); // clear PERIOD column cell
    tft.drawRect(cellFrame[0][i][0],cellFrame[0][i][1],cellFrame[0][i][2],cellFrame[0][i][3], DKBLUE);
    if (planIndex == i && (weekdaySched || weekendSched)) { // we only change color if the schedule displayed corrresponds to the current state (either weekday or weekend)
      tft.setTextColor(GREEN);  // current period will be highlighted in GREEN rather than YELLOW
      tft.setCursor(35,70 + i*30); tft.print(String(i+1));
    }
    else {
      tft.setTextColor(YELLOW);  // 
      tft.setCursor(35,70 + i*30); tft.print(String(i+1));
    }
    tft.setTextColor(WHITE);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setupTimeDateSubscreen() {
  int i;
  
  tft.fillRect(0, 40, 240, 240, LTBLUE); // clear screen area below main menu leaving the arrows and "SET" button
  tft.fillRect(240, 40, 80, 120, LTBLUE);
  
  if (!RTCavail) {
    tft.setTextColor(RED); tft.setTextSize(2);
    tft.setCursor(15,95); tft.print("Error: RTC ERROR");
    tft.setCursor(15,110); tft.print("  press set to exit");
  }
  // set up labels
  tft.setTextColor(YELLOW); tft.setTextSize(2);
  tft.setCursor(CellLabelPos[1][0],CellLabelPos[1][1]); tft.print(" Hour");
  tft.setCursor(CellLabelPos[2][0],CellLabelPos[2][1]); tft.print(" Min");
  tft.setCursor(CellDataPos[0][2][0],CellDataPos[0][2][1]); tft.print("Month");
  tft.setCursor(CellDataPos[1][2][0],CellDataPos[1][2][1]); tft.print(" Day");
  tft.setCursor(CellDataPos[2][2][0],CellDataPos[2][2][1]); tft.print("Year");
  tft.setCursor(CellDataPos[1][4][0],CellDataPos[1][4][1]); tft.print(" DOW");
        
  // set up time cells (hours and minutes)
  tft.setTextColor(WHITE); tft.setTextSize(2);
  for (i=1; i<3; i++) {// hours and minutes in separate cells
    tft.fillRect(cellFrame[i][0][0],cellFrame[i][0][1],cellFrame[i][0][2],cellFrame[i][0][3], LTBLUE); // TIME SET ROW/COLUMN
    tft.drawRect(cellFrame[i][0][0],cellFrame[i][0][1],cellFrame[i][0][2],cellFrame[i][0][3], DKBLUE); // TIME SET ROW/COLUMN
    tft.setCursor(CellDataPos[i][0][0]+((5-(String(hourMin[i-1]).length()))/2)*12,CellDataPos[i][0][1]); tft.print(hourMin[i-1]);
  }
  //date
  for (i=0; i<3; i++) { // set up the three columns in the date set row (3)
    tft.fillRect(cellFrame[i][3][0],cellFrame[i][3][1],cellFrame[i][3][2],cellFrame[i][3][3], LTBLUE);
    tft.drawRect(cellFrame[i][3][0],cellFrame[i][3][1],cellFrame[i][3][2],cellFrame[i][3][3], DKBLUE); 
    tft.setCursor(CellDataPos[i][3][0]+((5-(String(monthDayYear[i]).length()))/2)*12,CellDataPos[i][3][1]); tft.print(monthDayYear[i]);
  }
  // Day of Week
  tft.fillRect(cellFrame[1][5][0],cellFrame[1][5][1],cellFrame[1][5][2],cellFrame[1][5][3], LTBLUE);
  tft.drawRect(cellFrame[1][5][0],cellFrame[1][5][1],cellFrame[1][5][2],cellFrame[1][5][3], DKBLUE); 
  tft.setCursor(CellDataPos[1][5][0]+boxIncrmt,CellDataPos[1][5][1]); tft.print(shortDOW[0]);
      
  // change "RUN" to "SET" button
  tft.fillRect(250, 217, 60, 26, GREEN);
  tft.drawRect(250, 217, 60, 26, DKGREEN);
  tft.setCursor(263,222); tft.setTextColor(DKGREEN); tft.setTextSize(2); tft.println("SET");  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void changeTime(int px, int py) {
  int i;
  
  row = ((py - 65)/30); // row cacluation based on height of cell ( first row is 0 )
  column = (px/80); // column calculation based on width of cell ( first column is 0 )
  // exit on non-active cells
  if (row < 0 || column < 0) { return; } // return on invalid data
  if (column != 3 && !(row == 0 || row == 3 || row == 5)) { return; } // selection not active on this screen
  if (row == 0 && column == 0) { return; } // this cell is not active on this screen
  // OK, good to go
  oldRow = oldRowColumn/10;
  oldColumn = oldRowColumn - oldRow*10;
  newRowColumn = row*10+column; // definition of the newRowColumn variable
  
  if ((oldRowColumn != newRowColumn) && column < 3)  { // new row and column selected and column is either 1, 2 or 3
    // change highlighted cell
    tft.drawRect(cellFrame[column][row][0],cellFrame[column][row][1],cellFrame[column][row][2],cellFrame[column][row][3], WHITE);
    if (!(oldRow == 0 && oldColumn == 0)) {
      tft.drawRect(cellFrame[oldColumn][oldRow][0],cellFrame[oldColumn][oldRow][1],cellFrame[oldColumn][oldRow][2],cellFrame[oldColumn][oldRow][3], DKBLUE);
    }
    oldRowColumn = newRowColumn; 
  }
  else if (column == 3 && oldRow == 0) { // we are in the hour and minutes adjust section
    tft.setTextColor(WHITE); tft.setTextSize(2); // get ready to write text
    if (row == 3) { // increase arrow
      if (oldColumn == 1) { // hour set point
        hourMin[0] += 1; // increment hours
        if (hourMin[0] > 24) { hourMin[0] = 1; }
      }
      if (oldColumn == 2) { // minutes set point
        hourMin[1] += 1; // increment minutes
        if (hourMin[1] > 59) {
          hourMin[1] = 0;
          hourMin[0] += 1;
          if (hourMin[0] > 24) { // use 24 hour clock
            hourMin[0] = 1;
          }
        }
      }
      for (i=0; i<2; i++) { // write both the hour and the minutes regardless
        tft.fillRect(cellFrame[i+1][0][0]+1,cellFrame[i+1][0][1]+1,cellFrame[i+1][0][2]-2,cellFrame[i+1][0][3]-2, LTBLUE);
        tft.setCursor(CellDataPos[i+1][0][0]+((5-(String(hourMin[i]).length()))/2)*12, CellDataPos[i+1][0][1]); tft.print(hourMin[i]);
      }
      wait(WAIT); // delay to make setting times and dates easier 
    }      
    if (row == 4) { // decrease arrow
      if (oldColumn == 1) { // hour set point
        hourMin[0] -= 1; // decrement hours
        if (hourMin[0] < 1) { hourMin[0] = 24; }
      }
      if (oldColumn == 2) { // minutes set point
        hourMin[1] -= 1; // decrement minutes
        if (hourMin[1] < 1) {
          hourMin[1] = 59;
          hourMin[0] -= 1;
          if (hourMin[0] < 1) {
            hourMin[0] = 24;
          }
        }
      }
      for (i=0; i<2; i++) { // write both the hour and the minutes regardless
        tft.fillRect(cellFrame[i+1][0][0]+1,cellFrame[i+1][0][1]+1,cellFrame[i+1][0][2]-2,cellFrame[i+1][0][3]-2, LTBLUE);
        tft.setCursor(CellDataPos[i+1][0][0]+((5-(String(hourMin[i]).length()))/2)*12,CellDataPos[i+1][0][1]); tft.print(hourMin[i]);
      }
      wait(WAIT); // delay to make setting times and dates easier  
    }
    if ((row == 5)) { // "SET" button selected
      timeSet = false;
    }
  }
  else if (column == 3 && oldRow == 3) { // we are in the date adjust section
    tft.setTextColor(WHITE); tft.setTextSize(2); // get ready to write text
    if (row == 3) { // increase arrow
      if (oldColumn == 0) { // month set point
        monthDayYear[0] += 1; // increment month
        if (monthDayYear[0] == 13) { monthDayYear[0] = 1; }
      }
      if (oldColumn == 1) { // day set point
        monthDayYear[1] += 1; // increment dat
        if (monthDayYear[1] == 32) { monthDayYear[1] = 1; }
      }
      if (oldColumn == 2) { //year set point
        monthDayYear[2] += 1; // increment year
      }        
      for (i=0; i<3; i++) { // write the entire date regardless
        tft.fillRect(cellFrame[i][3][0]+1,cellFrame[i][3][1]+1,cellFrame[i][3][2]-2,cellFrame[i][3][3]-2, LTBLUE);
        tft.setCursor(CellDataPos[i][3][0]+((5-(String(monthDayYear[i]).length()))/2)*12,CellDataPos[i][3][1]); tft.print(monthDayYear[i]);
      }
      wait(WAIT); // delay to make setting times and dates easier
    }
    
    if (row == 4) { // decrease arrow
      if (oldColumn == 0) { // month set point
        monthDayYear[0] -= 1; // increment month
        if (monthDayYear[0] == 0) { monthDayYear[0] = 12; }
      }
      if (oldColumn == 1) { // day set point
        monthDayYear[1] -= 1; // increment dat
        if (monthDayYear[1] == 0) { monthDayYear[1] = 31; }
      }
      if (oldColumn == 2) { //year set point
        monthDayYear[2] -= 1; // increment year
      }        
      for (i=0; i<3; i++) { // write the entire date regardless
        tft.fillRect(cellFrame[i][3][0]+1,cellFrame[i][3][1]+1,cellFrame[i][3][2]-2,cellFrame[i][3][3]-2, LTBLUE);
        tft.setCursor(CellDataPos[i][3][0]+((5-(String(monthDayYear[i]).length()))/2)*12,CellDataPos[i][3][1]); tft.print(monthDayYear[i]);
      }
      wait(WAIT); // delay to make setting times and dates easier 
    }
  }
  else if (column == 3 && oldRow == 5) { // DOW update
    if (row == 3) { // increase arrow
      DOWindex += 1; // increment DOW index
      if (DOWindex > 6) {DOWindex = 0;}
    }
    if (row == 4) { // decrease arrow
      DOWindex -= 1; // decrement DOW index
      if (DOWindex < 0) {DOWindex = 6;}      
    }
    tft.setTextColor(WHITE); tft.setTextSize(2);
    tft.fillRect(cellFrame[1][5][0]+1,cellFrame[1][5][1]+1,cellFrame[1][5][2]-2,cellFrame[1][5][3]-2, LTBLUE);
    tft.setCursor(CellDataPos[1][5][0]+boxIncrmt,CellDataPos[1][5][1]); tft.print(shortDOW[DOWindex]);
    wait(WAIT); // delay to make setting times and dates easier 
  }
  if ((column == 3 && row == 5)) { // "SET" button selected
    if (RTCavail) {
      rtc.setDOW(DOWindex+1); // Set Day-of-Week
      rtc.setTime(hourMin[0], hourMin[1], 0); // Set the time (24hr format)
      rtc.setDate(monthDayYear[1], monthDayYear[0], monthDayYear[2]);   // Set the date
    }
    else {
      tft.setTextColor(RED); tft.setTextSize(2);
      tft.setCursor(CellDataPos[1][0][0],CellDataPos[1][0][1]); tft.print("Error: RTC Unavailable");
    }
    timeSet = false;
    Screen1SetUp();
  }
}    

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen1Chk(int py, int px) { // for touch screen x and y axis are oposite -- x==y, y==x
  int i;
  
  if (timeSet) {
    changeTime(px, py); // submenu 2 ("Set Time") was previously selected
    return; // we're done here!
  }
  if (py < 41 && py > 20 && px < MENUWIDTH) { // weekdays submenu selected 
    weekdays = true;
    tft.fillRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, WHITE);
    tft.fillRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
    tft.fillRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
    tft.setCursor(15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("weekdays");
    tft.setCursor(MENUWIDTH + 15,MENUHEIGHT + 7); tft.println("weekends");
    tft.setCursor(MENUWIDTH*2 + 15,MENUHEIGHT + 7); tft.println("Time/Date?");
    Screen1Data();
    return; // do nothing more on this round
    }
  if (py < 41 && py > 20 && px < MENUWIDTH*2 && px > MENUWIDTH) { // weekends submenu selected
    weekdays = false;
    tft.fillRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
    tft.fillRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, WHITE);
    tft.fillRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
    tft.setCursor(15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("weekdays");
    tft.setCursor(MENUWIDTH + 15,MENUHEIGHT + 7); tft.println("weekends");
    tft.setCursor(MENUWIDTH*2 + 15,MENUHEIGHT + 7); tft.println("Time/Date?");
    Screen1Data();
    return; // do nothing more on this round
  }
  if (py<41 && py > 20 && px < MENUWIDTH*3 && px > MENUWIDTH*2) { // timeSet submenu selected
    timeSet = true;
    tft.fillRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(0, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
    tft.fillRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(MENUWIDTH, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, LTBLUE);
    tft.fillRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, DKBLUE);
    tft.drawRect(MENUWIDTH*2, MENUHEIGHT, MENUWIDTH, MENUHEIGHT, WHITE);
    tft.setCursor(15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("weekdays");
    tft.setCursor(MENUWIDTH + 15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("weekends");
    tft.setCursor(MENUWIDTH*2 + 15,MENUHEIGHT + 7); tft.setTextColor(WHITE); tft.setTextSize(1); tft.println("Time/Date?");
    setupTimeDateSubscreen();
    return; // do nothing more on this round 
  }
// if none of the above, then this could be a cell selection.  Let's determine which cell by calculation
  updateScreen1Period(); // highlights active schedule period
  row = ((py - 65)/30); // row cacluation based on height of cell ( first row is 0 )
  column = (px/80); // column calculation based on width of cell ( first column is 0 )
  if (row < 0 || column == 0) {return;} // return on invalid data
  oldRow = oldRowColumn/10;
  oldColumn = oldRowColumn - oldRow*10;
  if (oldColumn == 0) {oldColumn = 1;} // initialization issue.  Can't have oldColumn or column equal to zero.
  newRowColumn = row*10+column; // definition of the newRowColumn variable
  
  if ((oldRowColumn != newRowColumn) && column < 3)  { // new row and column selected and column is either 1 or 2 as if column is 0 subroutine already exited
    // change highlighted cell
    tft.drawRect(cellFrame[column][row][0],cellFrame[column][row][1],cellFrame[column][row][2]-(column-1)*20,cellFrame[column][row][3], WHITE);
    tft.drawRect(cellFrame[oldColumn][oldRow][0],cellFrame[oldColumn][oldRow][1],cellFrame[oldColumn][oldRow][2]-(oldColumn-1)*20,cellFrame[oldColumn][oldRow][3], DKBLUE);
    oldRowColumn = newRowColumn;
  
    return; // must return to avoid lower section of routine when not necessary  
  }
  if (column == 3) {
    if (row == 3) { // increase arrow
      if (oldColumn == 1) { // time set point
        DTPlanMins[heatCoolPlan][weekdays][oldRow] += 15;
        if (DTPlanMins[heatCoolPlan][weekdays][oldRow] > 59) {
          DTPlanMins[heatCoolPlan][weekdays][oldRow] = 0;
          DTPlanHours[heatCoolPlan][weekdays][oldRow] += 1;
          if (DTPlanHours[heatCoolPlan][weekdays][oldRow] > 24) { // use 24 hour clock
            DTPlanHours[heatCoolPlan][weekdays][oldRow] = 1;
          }
        }
        sprintf_P(TimeTime, (PGM_P)F("%02d:%02d"), DTPlanHours[heatCoolPlan][weekdays][oldRow] , DTPlanMins[heatCoolPlan][weekdays][oldRow]); // convert time to string
        tft.fillRect(cellFrame[1][oldRow][0],cellFrame[1][oldRow][1],cellFrame[1][oldRow][2],cellFrame[1][oldRow][3], LTBLUE);
        tft.drawRect(cellFrame[1][oldRow][0],cellFrame[1][oldRow][1],cellFrame[1][oldRow][2],cellFrame[1][oldRow][3], WHITE);
        tft.setCursor(CellDataPos[1][oldRow][0],CellDataPos[1][oldRow][1]); tft.setTextColor(WHITE); tft.setTextSize(2); tft.print(TimeTime);        
      }
      if (oldColumn == 2) { // temperature set point
        DTPlanTemp[heatCoolPlan][weekdays][oldRow] += 1; // increment temperature by 1 degree Fahrenheit
        if (DTPlanTemp[heatCoolPlan][weekdays][oldRow] > 99) { DTPlanTemp[heatCoolPlan][weekdays][oldRow] = 99; }
        tft.fillRect(cellFrame[2][oldRow][0],cellFrame[2][oldRow][1],cellFrame[2][oldRow][2]-20,cellFrame[2][oldRow][3], LTBLUE); // erase current contents
        tft.drawRect(cellFrame[2][oldRow][0],cellFrame[2][oldRow][1],cellFrame[2][oldRow][2]-20,cellFrame[2][oldRow][3], WHITE);
        tft.setCursor(CellDataPos[2][oldRow][0]+boxIncrmt,CellDataPos[2][oldRow][1]); tft.setTextColor(WHITE); tft.setTextSize(2); tft.print(DTPlanTemp[heatCoolPlan][weekdays][oldRow]);
      }
    }
    else if (row == 4) { // decrease arrow
      if (oldColumn == 1) { // time set point
        DTPlanMins[heatCoolPlan][weekdays][oldRow] -= 15;
        if (DTPlanMins[heatCoolPlan][weekdays][oldRow] > 45) { // if you subtract 15 from a 0 value byte, you will get 241 or (256 - 15).
          DTPlanMins[heatCoolPlan][weekdays][oldRow] = 45;
          DTPlanHours[heatCoolPlan][weekdays][oldRow] -= 1; // once again, if you subtract 1 from a 0 value byte, you will get 255
          if (DTPlanHours[heatCoolPlan][weekdays][oldRow] > 24) { // use 24 hour clock
            DTPlanHours[heatCoolPlan][weekdays][oldRow] = 24;
          }
        }
        sprintf_P(TimeTime, (PGM_P)F("%02d:%02d"), DTPlanHours[heatCoolPlan][weekdays][oldRow] , DTPlanMins[heatCoolPlan][weekdays][oldRow]); // convert time to string
        tft.fillRect(cellFrame[1][oldRow][0],cellFrame[1][oldRow][1],cellFrame[1][oldRow][2],cellFrame[1][oldRow][3], LTBLUE);
        tft.drawRect(cellFrame[1][oldRow][0],cellFrame[1][oldRow][1],cellFrame[1][oldRow][2],cellFrame[1][oldRow][3], WHITE);
        tft.setCursor(CellDataPos[1][oldRow][0],CellDataPos[1][oldRow][1]); tft.setTextColor(WHITE); tft.setTextSize(2); tft.print(TimeTime);        
      }
      if (oldColumn == 2) { // temperature set point
        DTPlanTemp[heatCoolPlan][weekdays][oldRow] -= 1; // increment temperature by 1 degree Fahrenheit
        if (DTPlanTemp[heatCoolPlan][weekdays][oldRow] < 0) { DTPlanTemp[heatCoolPlan][weekdays][oldRow] = 0; }
        tft.fillRect(cellFrame[2][oldRow][0],cellFrame[2][oldRow][1],cellFrame[2][oldRow][2]-20,cellFrame[2][oldRow][3], LTBLUE);
        tft.drawRect(cellFrame[2][oldRow][0],cellFrame[2][oldRow][1],cellFrame[2][oldRow][2]-20,cellFrame[2][oldRow][3], WHITE);
        tft.setCursor(CellDataPos[2][oldRow][0]+boxIncrmt,CellDataPos[2][oldRow][1]); tft.setTextColor(WHITE); tft.setTextSize(2); tft.print(DTPlanTemp[heatCoolPlan][weekdays][oldRow]);
      }      
    }
    else if (row == 0 || row == 1) {
      if (row == 0) {
        heatCoolPlan = true;}
      else  {
        heatCoolPlan = false;}
    
      // heat plan button
      tft.fillRect(250, 65, 60, 25, GREEN*heatCoolPlan + RED*(1-heatCoolPlan));
      tft.drawRect(250, 65, 60, 25, DKGREEN);
      tft.setCursor(258,70); tft.setTextColor(DKGREEN); tft.setTextSize(2); tft.println("HEAT");
    
      // cool plan button
      tft.fillRect(250, 95, 60, 25, RED*heatCoolPlan + GREEN*(1-heatCoolPlan));
      tft.drawRect(250, 95, 60, 25, DKGREEN);
      tft.setCursor(258,100); tft.setTextColor(DKGREEN); tft.setTextSize(2); tft.println("COOL"); 
       
      Screen1Data(); // refill data with that corresponding to heatCoolPlan   
    }
    else if (row == 5) { // run button
      if (tempHumidity && !mode[FAN]) { // as long as we are not in Humidity Control and not Fan Mode, we can enter run mode
        runMode = !runMode; // invert runMode
        send(msgRunMode.set(runMode));  // update runMode on Controller
        // update run button
        tft.fillRect(250, 217, 60, 26, GREEN*runMode + RED*(1-runMode));
        tft.drawRect(250, 217, 60, 26, DKGREEN);
        tft.setCursor(263,222); tft.setTextColor(DKGREEN); tft.setTextSize(2); tft.println("RUN");
      }
    }
    for (i=1; i<6; i++) {
      // check time values to see if they are in order 
      if ((DTPlanHours[heatCoolPlan][weekdays][i]+DTPlanMins[heatCoolPlan][weekdays][i]/60) < DTPlanHours[heatCoolPlan][weekdays][i-1]+DTPlanMins[heatCoolPlan][weekdays][i-1]/60) {
        tft.setTextColor(RED);
      }
      else {tft.setTextColor(WHITE);}
      sprintf_P(TimeTime, (PGM_P)F("%02d:%02d"), DTPlanHours[heatCoolPlan][weekdays][i] , DTPlanMins[heatCoolPlan][weekdays][i]); // convert time to string
      tft.setCursor(CellDataPos[1][i][0],CellDataPos[1][i][1]); tft.print(TimeTime);
      // setup temperature array display
      tft.setCursor(CellDataPos[2][i][0]+boxIncrmt,CellDataPos[2][i][1]); tft.print(DTPlanTemp[heatCoolPlan][weekdays][i]);   
    }
    wait(WAIT); // delay to make setting temps and times easier
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen2SetUp() {
  int i;
  
  tft.fillRect(0, 20, 320, 240, LTBLUE); // clear screen area below main menu
  
// setup MANUAL buttons display

  tft.setTextColor(YELLOW); tft.setTextSize(2); 
  tft.setCursor(20,70); tft.print("MODE"); 
  tft.setCursor(90,100); tft.print("SET PT"); 
  tft.setCursor(170,45); tft.print(" ZONE");
  tft.setCursor(245,45); tft.print("  ADJ");

  tft.fillTriangle(285, 70, 265, 120, 305, 120, DKBLUE);
  tft.fillTriangle(285, 180, 265, 130, 305, 130, DKBLUE);
  tft.setCursor(90,125); tft.setTextColor(WHITE); tft.setTextSize(4); tft.print(Setpt[mode[HEAT]][tempHumidity]);
  if (tempHumidity) {tft.print("F");} else {tft.print("%");}

// clear screen
  for (i=1; i<4; i++) {
    tft.fillRect(cellFrame[0][i][0],cellFrame[0][i][1],cellFrame[0][i][2],cellFrame[0][i][3], RED*(1-mode[i-1])+GREEN*mode[i-1]); // MODE COLUMN (fan, cool, heat)
    tft.drawRect(cellFrame[0][i][0],cellFrame[0][i][1],cellFrame[0][i][2],cellFrame[0][i][3], BLACK);
    send(msgMode[i-1].set(mode[i-1])); // update modes
  }
    tft.fillRect(cellFrame[1][4][0],cellFrame[1][4][1],cellFrame[1][4][2],cellFrame[1][4][3], LTGRAY); // Set Point COLUMN (T/H button)
    tft.drawRect(cellFrame[1][4][0],cellFrame[1][4][1],cellFrame[1][4][2],cellFrame[1][4][3], BLACK);
  for (i=0; i<5; i++) {
    tft.fillRect(cellFrame[2][i][0],cellFrame[2][i][1],cellFrame[2][i][2]+boxIncrmt,cellFrame[2][i][3], RED*(1-zone[i])+GREEN*zone[i]); // ZONE COLUMN
    tft.drawRect(cellFrame[2][i][0],cellFrame[2][i][1],cellFrame[2][i][2]+boxIncrmt,cellFrame[2][i][3], BLACK);
    send(msgZone[i].set(zone[i])); // update zones
  }
// setup button lables

  tft.setTextColor(WHITE); tft.setTextSize(2);
  for (i=0; i<3; i++) {
    tft.setCursor(CellDataPos[0][i+1][0],CellDataPos[0][i+1][1]); tft.print(modeButton[i]);
  }
  tft.setCursor(CellDataPos[1][4][0],CellDataPos[1][4][1]); tft.print(" T/H");
  for (i=0; i<5; i++) {
    tft.setCursor(CellDataPos[2][i][0],CellDataPos[2][i][1]); tft.print(zoneButton[i]);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen2updateSetpt() {
  tft.fillRect(cellFrame[1][2][0],cellFrame[1][2][1],cellFrame[1][2][2],cellFrame[1][2][3]+20, LTBLUE); // clear set point text
  tft.setCursor(90,125); tft.setTextColor(WHITE); tft.setTextSize(4); tft.print(Setpt[mode[HEAT]][tempHumidity]);  // rewrite set point text
  if (tempHumidity) {tft.print("F");} else {tft.print("%");}
  if (mode[COOL]) { // cooling
    send(msgHVACFlowState.set("CoolOn"));
    if (tempHumidity) { // must send the correct setpoint in msg
      send(msgHVACsetptCool.set(Setpt[COOL - 1][tempHumidity])); // could be either in humidity or temperature control mode, however, Setpt[0][x] is cool mode.
    }
    else {
      send(msgHumSetPtStatus.set(1)); // set point is on
      send(msgHumSetPt.set(int(Setpt[COOL - 1][HUM])), true); // send humidity setpoint for slider control
    }
  }
  else if (mode[HEAT]) { // heating
    send(msgHVACsetptHeat.set(Setpt[HEAT][TEMP]));  // tempHumidity must be true --> temp mode, so Setpt[1][1] is heat mode, temp
    send(msgHVACFlowState.set("HeatOn"));
  }
  runMode = false; // changing setpt takes system out of automated mode
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen2Chk(int py, int px) { // for touch screen x and y axis are oposite -- x==y, y==x
  int i;
  
// as above, we'll determine which cell by calculation
  row = ((py - 65)/30); // row cacluation based on height of cell ( first row is 0 ), row and column are integers
  column = (px/80); // column calculation based on width of cell ( first column is 0 )
  if (row < 0) {return;} // exit when invalid data
  if (column == 1 && row != 4) {return;} // no active cells in this column except for row 5 (T/H)
  else if (column == 1 && row == 4) { // tempHumdity button
    tempHumidity = !tempHumidity; // invert control parameter
    send(msgTHBut.set(tempHumidity)); // update tempHumidity on HASS
    if (!tempHumidity && !mode[COOL]) { column = 0; row = 2; } // forces change to cool mode whenever switching to humidity control
    else { // all this stuff will be done when changing to cool mode, but not otherwise
      Screen2updateSetpt();
      runMode = false; // manually changing the H/T button forces system into runMode regardless
      send(msgRunMode.set(runMode)); // send runmode button change
    }   
  }
  if (column == 3) {  // if column three, updown buttons may have been pressed (row 4 is exception - not an arrow)
    if (row == 0 || row == 1) { // increase arrow
      Setpt[mode[HEAT]][tempHumidity] += 1; // increment setpoint
      if (Setpt[mode[HEAT]][tempHumidity] > 99) {Setpt[mode[HEAT]][tempHumidity] = 99;}
      Screen2updateSetpt();
    }
    if (row == 2 || row == 3) { // decrease arrow
      Setpt[mode[HEAT]][tempHumidity] -= 1; // decrement setpoint
      if (Setpt[mode[HEAT]][tempHumidity] < 0) {Setpt[mode[HEAT]][tempHumidity]  = 0;}
      Screen2updateSetpt();
    }
    if (row != 4) { // if row is 4, it means a touch beneath the down button.  So no action was taken above and therefore, must skip changing runMode.
      runMode = false;  // if you select a mode manually, it switches the system to set point mode
      send(msgRunMode.set(runMode)); // send runmode button change
    }
  }
  tft.setTextColor(WHITE); tft.setTextSize(2);
  if (column == 2) { // ZONES
    zone[row] = !zone[row]; // invert selected cell
    if (!zone[row]) {closeLouver(row);}  // if the zone[row] is false (not active), then must close the appropriate louver (louver routine will check if open or closed)
    else if (zone[row]) {openLouver(row);} // if the zone[row] is true (active), then must open the appropriate louver (louver routine will check if open or closed)
    tft.fillRect(cellFrame[2][row][0],cellFrame[2][row][1],cellFrame[2][row][2]+boxIncrmt,cellFrame[2][row][3], RED*(1-zone[row]) + GREEN*zone[row]); // ZONE COLUMN
    tft.drawRect(cellFrame[2][row][0],cellFrame[2][row][1],cellFrame[2][row][2]+boxIncrmt,cellFrame[2][row][3], BLACK);
    tft.setCursor(CellDataPos[2][row][0],CellDataPos[2][row][1]); tft.print(zoneButton[row]);
    send(msgZone[row].set(zone[row]));  // update homeassistant
    wait(WAIT);   
  }
  if (column == 0) { // MODES
    if (row == 0 || row > 3) {return;} // this area not active
    fanDelay = false; // if switching modes, the fan delay needs to be disabled.  It may immediately be turned back on
    mode[row-1] = !mode[row-1]; // invert selected cell
    if (row == 1 && mode[row-1]) {mode[COOL] = false; mode[HEAT] = false;} // fan mode
    if (row == 2 && mode[row-1]) {mode[FAN] = false; mode[HEAT] = false; heatCoolPlan = false;} // cool mode
    if (row == 3 && mode[row-1]) {mode[FAN] = false; mode[COOL] = false; heatCoolPlan = true; tempHumidity = true;} // heat mode and can't be in Humidity control
    for (i=1; i<4; i++) {
      tft.fillRect(cellFrame[0][i][0],cellFrame[0][i][1],cellFrame[0][i][2],cellFrame[0][i][3], RED*(1-mode[i-1]) + GREEN*mode[i-1]); // MODE COLUMN
      tft.drawRect(cellFrame[0][i][0],cellFrame[0][i][1],cellFrame[0][i][2],cellFrame[0][i][3], BLACK);
      tft.setCursor(CellDataPos[0][i][0],CellDataPos[0][i][1]); tft.print(modeButton[i-1]);
    }
    runMode = false;  // if you select a mode manually, it switches the system to set point mode
    send(msgRunMode.set(runMode)); // send runmode button change
    int j = 3; // if final result is 3, it means all relays are off
    for (int i = 0; i < 3; i++ ) {
      send(msgMode[i].set(mode[i]));
      if (mode[i]) {j = i;} // if one of the modes is true, then j will be set to that mode, if not it will remain equal to 3!
    }
    setRelays(j); // set relays for appropriate function
    if (mode[COOL]) { send(msgHVACFlowState.set("CoolOn")); }
    if (mode[HEAT]) { send(msgHVACFlowState.set("HeatOn")); }
    if (!mode[COOL] && !mode[HEAT]) { send(msgHVACFlowState.set("Off")); } // it's either in fan or off mode.  For HA, fan might as well be off - no control required
    Screen2updateSetpt();
    wait(WAIT);  
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen3SetUp() {
  int i;
  
  tft.fillRect(0, 20, 320, 240, LTBLUE); // clear screen area below main menu
  
// setup Data buttons display

  tft.setTextColor(YELLOW); tft.setTextSize(2); 
  tft.setCursor(90,45); tft.print("ZONE"); 
  tft.setCursor(170,45); tft.print(" TEMP");
  tft.setCursor(245,25); tft.print(" HUMID");
  tft.setCursor(245,45); tft.print("  ITY");

// set up data cells
  for (i=0; i<5; i++) {
    tft.fillRect(cellFrame[2][i][0],cellFrame[2][i][1],cellFrame[2][i][2],cellFrame[2][i][3], LTBLUE); // TEMP COLUMN
    tft.drawRect(cellFrame[2][i][0],cellFrame[2][i][1],cellFrame[2][i][2]-5,cellFrame[2][i][3], BLACK);
  }
  for (i=0; i<5; i++) {
    tft.fillRect(cellFrame[3][i][0],cellFrame[3][i][1],cellFrame[3][i][2],cellFrame[3][i][3], LTBLUE); // HUMIDITY COLUMN
    tft.drawRect(cellFrame[3][i][0],cellFrame[3][i][1],cellFrame[3][i][2]-5,cellFrame[3][i][3], BLACK);
  }
// setup data values
  tft.setTextColor(WHITE); tft.setTextSize(2);
  for (i=0; i<5; i++) {
    tft.setCursor(CellDataPos[2][i][0],CellDataPos[2][i][1]); tft.print(tempHumidityData[1][i], 1); tft.print("F");
    tft.setCursor(CellDataPos[3][i][0],CellDataPos[3][i][1]); tft.print(tempHumidityData[0][i], 1); tft.print("%");
  }
  tft.setTextColor(GREEN); tft.setTextSize(2);
  for (i=0; i<5; i++) {
    tft.setCursor(CellDataPos[1][i][0]-10,CellDataPos[2][i][1]); tft.print(zoneButton[i]);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Screen3Chk(int column, int row) { // all we need is the column and row to be updated.  Remember that order here is [column][row][spacial data]
  
// clear data cell
    tft.fillRect(cellFrame[column][row][0],cellFrame[column][row][1],cellFrame[column][row][2],cellFrame[column][row][3], LTBLUE); // TEMP COLUMN
    tft.drawRect(cellFrame[column][row][0],cellFrame[column][row][1],cellFrame[column][row][2]-5,cellFrame[column][row][3], BLACK);
// update data values
  tft.setTextColor(WHITE); tft.setTextSize(2);
  if (column == 2 ) { // must be a temperature
    tft.setCursor(CellDataPos[2][row][0],CellDataPos[2][row][1]); tft.print(tempHumidityData[1][row], 1); tft.print("F");
  }
  else { // must be a humidity
    tft.setCursor(CellDataPos[3][row][0],CellDataPos[3][row][1]); tft.print(tempHumidityData[0][row], 1); tft.print("%");
  }  
}


