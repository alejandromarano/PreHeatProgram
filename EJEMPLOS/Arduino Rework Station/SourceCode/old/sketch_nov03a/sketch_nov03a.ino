

/*DUAL LOOP PID CONTROLLER
 Coded badly by Norcal Reballer
 Facebook.com/norcal.reballer
 */


#include <EEPROM.h>
#include <Adafruit_MAX31855.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 10, 5, 4, 3, 2);

//ssr pins. Any variables ending in 1 have to do with top heater
//Any variables ending in 2 have to do with bottom heater
#define RelayPin1 6
#define RelayPin2 7

int buzzerPin = 8; 

int backLight = 13;

//declaring which pins buttons are connected to
int upSwitchPin = 22;
int downSwitchPin = 24;
int editSwitchPin = 30;
int cancelSwitchPin = 32;
int okSwitchPin = 34;

//declaring switch state
int upSwitchState = 0;
int downSwitchState = 0;
int leftSwitchState = 0;
int rightSwitchState = 0;
int editSwitchState = 0;
int cancelSwitchState = 0;
int okSwitchState = 0;

//profile stuff
byte currentProfile = 1;
byte currentStep = 1;
byte profileSteps;
double rampRateStep1;
double rampRateStep2;
double rampRateStep3;
double rampRateStep4;
double rampRateStep5;
double rampRateStep6;
double rampRateStep7;
double rampRateStep8;
double rampRateStep9;

int dwellTimerStep1;
int dwellTimerStep2;
int dwellTimerStep3;
int dwellTimerStep4;
int dwellTimerStep5;
int dwellTimerStep6;
int dwellTimerStep7;
int dwellTimerStep8;
int dwellTimerStep9;

int kp1;
int ki1;
int kd1;
int kp2;
int ki2;
int kd2;

int setpointRamp;
int startTemp;

int temperatureStep1;
int temperatureStep2;
int temperatureStep3;
int temperatureStep4;
int temperatureStep5;
int temperatureStep6;
int temperatureStep7;
int temperatureStep8;
int temperatureStep9;

int eepromAddress = 0;//starting eepromaddress

long previousMillis; //these are for counters
double counter;

//these are the different states of the sketch. We call different ones depending on conditions
// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_MENU_STEPS,
  REFLOW_STATE_MENU_BOTTOM_HEAT,
  REFLOW_STATE_MENU_STEP_1_RAMP,
  REFLOW_STATE_MENU_STEP_1_TARGET,
  REFLOW_STATE_MENU_STEP_1_DWELL,
  REFLOW_STATE_MENU_STEP_2_RAMP,
  REFLOW_STATE_MENU_STEP_2_TARGET,
  REFLOW_STATE_MENU_STEP_2_DWELL,
  REFLOW_STATE_MENU_STEP_3_RAMP,
  REFLOW_STATE_MENU_STEP_3_TARGET,
  REFLOW_STATE_MENU_STEP_3_DWELL,
  REFLOW_STATE_MENU_STEP_4_RAMP,
  REFLOW_STATE_MENU_STEP_4_TARGET,
  REFLOW_STATE_MENU_STEP_4_DWELL,
  REFLOW_STATE_MENU_STEP_5_RAMP,
  REFLOW_STATE_MENU_STEP_5_TARGET,
  REFLOW_STATE_MENU_STEP_5_DWELL,
  REFLOW_STATE_MENU_STEP_6_RAMP,
  REFLOW_STATE_MENU_STEP_6_TARGET,
  REFLOW_STATE_MENU_STEP_6_DWELL,
  REFLOW_STATE_MENU_STEP_7_RAMP,
  REFLOW_STATE_MENU_STEP_7_TARGET,
  REFLOW_STATE_MENU_STEP_7_DWELL,
  REFLOW_STATE_MENU_STEP_8_RAMP,
  REFLOW_STATE_MENU_STEP_8_TARGET,
  REFLOW_STATE_MENU_STEP_8_DWELL,
  REFLOW_STATE_MENU_STEP_9_RAMP,
  REFLOW_STATE_MENU_STEP_9_TARGET,
  REFLOW_STATE_MENU_STEP_9_DWELL,
  REFLOW_STATE_MENU_BOTTOM_P,
  REFLOW_STATE_MENU_BOTTOM_I,
  REFLOW_STATE_MENU_BOTTOM_D,
  REFLOW_STATE_MENU_TOP_P,
  REFLOW_STATE_MENU_TOP_I,
  REFLOW_STATE_MENU_TOP_D,
  REFLOW_STATE_STEP_1_RAMP,
  REFLOW_STATE_STEP_1,
  REFLOW_STATE_STEP_1_DWELL,
  REFLOW_STATE_STEP_2_RAMP,
  REFLOW_STATE_STEP_2,
  REFLOW_STATE_STEP_2_DWELL,
  REFLOW_STATE_STEP_3_RAMP,
  REFLOW_STATE_STEP_3,
  REFLOW_STATE_STEP_3_DWELL,  
  REFLOW_STATE_STEP_4_RAMP,
  REFLOW_STATE_STEP_4,
  REFLOW_STATE_STEP_4_DWELL,
  REFLOW_STATE_STEP_5_RAMP,
  REFLOW_STATE_STEP_5,
  REFLOW_STATE_STEP_5_DWELL,
  REFLOW_STATE_STEP_6_RAMP,
  REFLOW_STATE_STEP_6,
  REFLOW_STATE_STEP_6_DWELL,
  REFLOW_STATE_STEP_7_RAMP,
  REFLOW_STATE_STEP_7,
  REFLOW_STATE_STEP_7_DWELL,
  REFLOW_STATE_STEP_8_RAMP,
  REFLOW_STATE_STEP_8,
  REFLOW_STATE_STEP_8_DWELL,
  REFLOW_STATE_STEP_9_RAMP,
  REFLOW_STATE_STEP_9,
  REFLOW_STATE_STEP_9_DWELL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_ERROR
}
reflowState_t;

typedef enum REFLOW_STATUS //this is simply to check if reflow should be running or no
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
}
reflowStatus_t;

#define SENSOR_SAMPLING_TIME 1000 //read tc every second


reflowStatus_t reflowStatus;
// Reflow oven controller state machine state variable
reflowState_t reflowState;


//TC read timer variables
unsigned long nextCheck1;
unsigned long nextRead1;
//PID stuff

double Setpoint1, Input1, Output1;
//Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint1,kp1,ki1,kd1, DIRECT);
int WindowSize = 2000;
unsigned long windowStartTime;
//PID stuff
double Setpoint2, Input2, Output2;
//Specify the links and initial tuning parameters
PID myPID2(&Input2, &Output2, &Setpoint2,kp2,ki2,kd2,DIRECT);

//Alarm state boolean
boolean alarmOn=false;

//31855 stuff - can be easily swapped for 6675
int thermoCLK = 53;
int thermoCS = 51;
int thermoDO = 49;
//31855 stuff - can be easily swapped for 6675
int thermoCLK2 = 47;
int thermoCS2 = 45;
int thermoDO2 = 43;  
Adafruit_MAX31855 thermocouple1(thermoCLK, thermoCS, thermoDO);//top heater thermocouple
Adafruit_MAX31855 thermocouple2(thermoCLK2, thermoCS2, thermoDO2);//bottom heater thermocouple

void loadProfile()//this function loads whichever profile currentProfile variable is set to
{
  if (currentProfile == 1)
  {
    double rampRateStep1Read = EEPROM.read(eepromAddress + 2);
    double rampRateStep2Read = EEPROM.read(eepromAddress + 3);
    double rampRateStep3Read = EEPROM.read(eepromAddress + 4);
    double rampRateStep4Read = EEPROM.read(eepromAddress + 5);
    double rampRateStep5Read = EEPROM.read(eepromAddress + 6);
    double rampRateStep6Read = EEPROM.read(eepromAddress + 7);
    double rampRateStep7Read = EEPROM.read(eepromAddress + 8);
    double rampRateStep8Read = EEPROM.read(eepromAddress + 9);
    double rampRateStep9Read = EEPROM.read(eepromAddress + 10);

    profileSteps = EEPROM.read(eepromAddress);
    Setpoint2 = EEPROM.read(eepromAddress + 1);

    rampRateStep1 = rampRateStep1Read / 20;
    rampRateStep2 = rampRateStep2Read / 20;
    rampRateStep3 = rampRateStep3Read / 20;
    rampRateStep4 = rampRateStep4Read / 20;
    rampRateStep5 = rampRateStep5Read / 20;
    rampRateStep6 = rampRateStep6Read / 20;
    rampRateStep7 = rampRateStep7Read / 20;
    rampRateStep8 = rampRateStep8Read / 20;
    rampRateStep9 = rampRateStep9Read / 20;

    dwellTimerStep1 = EEPROM.read(eepromAddress + 11) * 5;
    dwellTimerStep2 = EEPROM.read(eepromAddress + 12) * 5;
    dwellTimerStep3 = EEPROM.read(eepromAddress + 13) * 5;
    dwellTimerStep4 = EEPROM.read(eepromAddress + 14) * 5;
    dwellTimerStep5 = EEPROM.read(eepromAddress + 15) * 5;
    dwellTimerStep6 = EEPROM.read(eepromAddress + 16) * 5;
    dwellTimerStep7 = EEPROM.read(eepromAddress + 17) * 5;
    dwellTimerStep8 = EEPROM.read(eepromAddress + 18) * 5;
    dwellTimerStep9 = EEPROM.read(eepromAddress + 19) * 5;

    temperatureStep1 = EEPROM.read(eepromAddress + 20);
    temperatureStep2 = EEPROM.read(eepromAddress + 21);
    temperatureStep3 = EEPROM.read(eepromAddress + 22);
    temperatureStep4 = EEPROM.read(eepromAddress + 23);
    temperatureStep5 = EEPROM.read(eepromAddress + 24);
    temperatureStep6 = EEPROM.read(eepromAddress + 25);
    temperatureStep7 = EEPROM.read(eepromAddress + 26);
    temperatureStep8 = EEPROM.read(eepromAddress + 27);
    temperatureStep9 = EEPROM.read(eepromAddress + 28);

    kp1 = EEPROM.read(eepromAddress + 122);
    ki1 = EEPROM.read(eepromAddress + 123);
    kd1 = EEPROM.read(eepromAddress + 124);
    kp2 = EEPROM.read(eepromAddress + 125);
    ki2 = EEPROM.read(eepromAddress + 126);
    kd2 = EEPROM.read(eepromAddress + 127);

  }

  if (currentProfile == 2)
  {
    double rampRateStep1Read = EEPROM.read(eepromAddress + 31);
    double rampRateStep2Read = EEPROM.read(eepromAddress + 32);
    double rampRateStep3Read = EEPROM.read(eepromAddress + 33);
    double rampRateStep4Read = EEPROM.read(eepromAddress + 34);
    double rampRateStep5Read = EEPROM.read(eepromAddress + 35);
    double rampRateStep6Read = EEPROM.read(eepromAddress + 36);
    double rampRateStep7Read = EEPROM.read(eepromAddress + 37);
    double rampRateStep8Read = EEPROM.read(eepromAddress + 38);
    double rampRateStep9Read = EEPROM.read(eepromAddress + 39);

    profileSteps = EEPROM.read(eepromAddress + 29);
    Setpoint2 = EEPROM.read(eepromAddress + 30);

    rampRateStep1 = rampRateStep1Read / 20;
    rampRateStep2 = rampRateStep2Read / 20;
    rampRateStep3 = rampRateStep3Read / 20;
    rampRateStep4 = rampRateStep4Read / 20;
    rampRateStep5 = rampRateStep5Read / 20;
    rampRateStep6 = rampRateStep6Read / 20;
    rampRateStep7 = rampRateStep7Read / 20;
    rampRateStep8 = rampRateStep8Read / 20;
    rampRateStep9 = rampRateStep9Read / 20;

    dwellTimerStep1 = EEPROM.read(eepromAddress + 40) * 5;
    dwellTimerStep2 = EEPROM.read(eepromAddress + 41) * 5;
    dwellTimerStep3 = EEPROM.read(eepromAddress + 42) * 5;
    dwellTimerStep4 = EEPROM.read(eepromAddress + 43) * 5;
    dwellTimerStep5 = EEPROM.read(eepromAddress + 44) * 5;
    dwellTimerStep6 = EEPROM.read(eepromAddress + 45) * 5;
    dwellTimerStep7 = EEPROM.read(eepromAddress + 46) * 5;
    dwellTimerStep8 = EEPROM.read(eepromAddress + 47) * 5;
    dwellTimerStep9 = EEPROM.read(eepromAddress + 48) * 5;

    temperatureStep1 = EEPROM.read(eepromAddress + 49);
    temperatureStep2 = EEPROM.read(eepromAddress + 50);
    temperatureStep3 = EEPROM.read(eepromAddress + 51);
    temperatureStep4 = EEPROM.read(eepromAddress + 52);
    temperatureStep5 = EEPROM.read(eepromAddress + 53);
    temperatureStep6 = EEPROM.read(eepromAddress + 54);
    temperatureStep7 = EEPROM.read(eepromAddress + 55);
    temperatureStep8 = EEPROM.read(eepromAddress + 56);
    temperatureStep9 = EEPROM.read(eepromAddress + 57);

    kp1 = EEPROM.read(eepromAddress + 128);
    ki1 = EEPROM.read(eepromAddress + 129);
    kd1 = EEPROM.read(eepromAddress + 130);
    kp2 = EEPROM.read(eepromAddress + 131);
    ki2 = EEPROM.read(eepromAddress + 132);
    kd2 = EEPROM.read(eepromAddress + 133);

  }
  if (currentProfile == 3)
  {
    double rampRateStep1Read = EEPROM.read(eepromAddress + 60);
    double rampRateStep2Read = EEPROM.read(eepromAddress + 61);
    double rampRateStep3Read = EEPROM.read(eepromAddress + 62);
    double rampRateStep4Read = EEPROM.read(eepromAddress + 63);
    double rampRateStep5Read = EEPROM.read(eepromAddress + 64);
    double rampRateStep6Read = EEPROM.read(eepromAddress + 65);
    double rampRateStep7Read = EEPROM.read(eepromAddress + 66);
    double rampRateStep8Read = EEPROM.read(eepromAddress + 67);
    double rampRateStep9Read = EEPROM.read(eepromAddress + 68);

    profileSteps = EEPROM.read(eepromAddress + 58);
    Setpoint2 = EEPROM.read(eepromAddress + 59);

    rampRateStep1 = rampRateStep1Read / 20;
    rampRateStep2 = rampRateStep2Read / 20;
    rampRateStep3 = rampRateStep3Read / 20;
    rampRateStep4 = rampRateStep4Read / 20;
    rampRateStep5 = rampRateStep5Read / 20;
    rampRateStep6 = rampRateStep6Read / 20;
    rampRateStep7 = rampRateStep7Read / 20;
    rampRateStep8 = rampRateStep8Read / 20;
    rampRateStep9 = rampRateStep9Read / 20;

    dwellTimerStep1 = EEPROM.read(eepromAddress + 69) * 5;
    dwellTimerStep2 = EEPROM.read(eepromAddress + 70) * 5;
    dwellTimerStep3 = EEPROM.read(eepromAddress + 71) * 5;
    dwellTimerStep4 = EEPROM.read(eepromAddress + 72) * 5;
    dwellTimerStep5 = EEPROM.read(eepromAddress + 73) * 5;
    dwellTimerStep6 = EEPROM.read(eepromAddress + 74) * 5;
    dwellTimerStep7 = EEPROM.read(eepromAddress + 75) * 5;
    dwellTimerStep8 = EEPROM.read(eepromAddress + 76) * 5;
    dwellTimerStep9 = EEPROM.read(eepromAddress + 77) * 5;

    temperatureStep1 = EEPROM.read(eepromAddress + 78);
    temperatureStep2 = EEPROM.read(eepromAddress + 79);
    temperatureStep3 = EEPROM.read(eepromAddress + 80);
    temperatureStep4 = EEPROM.read(eepromAddress + 81);
    temperatureStep5 = EEPROM.read(eepromAddress + 82);
    temperatureStep6 = EEPROM.read(eepromAddress + 83);
    temperatureStep7 = EEPROM.read(eepromAddress + 84);
    temperatureStep8 = EEPROM.read(eepromAddress + 85);
    temperatureStep9 = EEPROM.read(eepromAddress + 86);

    kp1 = EEPROM.read(eepromAddress + 134);
    ki1 = EEPROM.read(eepromAddress + 135);
    kd1 = EEPROM.read(eepromAddress + 136);
    kp2 = EEPROM.read(eepromAddress + 137);
    ki2 = EEPROM.read(eepromAddress + 138);
    kd2 = EEPROM.read(eepromAddress + 139);

  }
  if (currentProfile == 4)
  {
    double rampRateStep1Read = EEPROM.read(eepromAddress + 89);
    double rampRateStep2Read = EEPROM.read(eepromAddress + 90);
    double rampRateStep3Read = EEPROM.read(eepromAddress + 91);
    double rampRateStep4Read = EEPROM.read(eepromAddress + 92);
    double rampRateStep5Read = EEPROM.read(eepromAddress + 93);
    double rampRateStep6Read = EEPROM.read(eepromAddress + 94);
    double rampRateStep7Read = EEPROM.read(eepromAddress + 95);
    double rampRateStep8Read = EEPROM.read(eepromAddress + 96);
    double rampRateStep9Read = EEPROM.read(eepromAddress + 97);

    profileSteps = EEPROM.read(eepromAddress + 87);
    Setpoint2 = EEPROM.read(eepromAddress + 88);

    rampRateStep1 = rampRateStep1Read / 20;
    rampRateStep2 = rampRateStep2Read / 20;
    rampRateStep3 = rampRateStep3Read / 20;
    rampRateStep4 = rampRateStep4Read / 20;
    rampRateStep5 = rampRateStep5Read / 20;
    rampRateStep6 = rampRateStep6Read / 20;
    rampRateStep7 = rampRateStep7Read / 20;
    rampRateStep8 = rampRateStep8Read / 20;
    rampRateStep9 = rampRateStep9Read / 20;

    dwellTimerStep1 = EEPROM.read(eepromAddress + 98) * 5;
    dwellTimerStep2 = EEPROM.read(eepromAddress + 99) * 5;
    dwellTimerStep3 = EEPROM.read(eepromAddress + 100) * 5;
    dwellTimerStep4 = EEPROM.read(eepromAddress + 101) * 5;
    dwellTimerStep5 = EEPROM.read(eepromAddress + 102) * 5;
    dwellTimerStep6 = EEPROM.read(eepromAddress + 103) * 5;
    dwellTimerStep7 = EEPROM.read(eepromAddress + 104) * 5;
    dwellTimerStep8 = EEPROM.read(eepromAddress + 105) * 5;
    dwellTimerStep9 = EEPROM.read(eepromAddress + 106) * 5;

    temperatureStep1 = EEPROM.read(eepromAddress + 107);
    temperatureStep2 = EEPROM.read(eepromAddress + 108);
    temperatureStep3 = EEPROM.read(eepromAddress + 109);
    temperatureStep4 = EEPROM.read(eepromAddress + 110);
    temperatureStep5 = EEPROM.read(eepromAddress + 111);
    temperatureStep6 = EEPROM.read(eepromAddress + 112);
    temperatureStep7 = EEPROM.read(eepromAddress + 113);
    temperatureStep8 = EEPROM.read(eepromAddress + 114);
    temperatureStep9 = EEPROM.read(eepromAddress + 115);

    kp1 = EEPROM.read(eepromAddress + 116);
    ki1 = EEPROM.read(eepromAddress + 117);
    kd1 = EEPROM.read(eepromAddress + 118);
    kp2 = EEPROM.read(eepromAddress + 119);
    ki2 = EEPROM.read(eepromAddress + 120);
    kd2 = EEPROM.read(eepromAddress + 121);

  }

  return;  
}


void setup()
{
  //setup pins as input for buttons
  pinMode (upSwitchPin, INPUT);
  pinMode (downSwitchPin, INPUT);
  pinMode (editSwitchPin, INPUT);
  pinMode (cancelSwitchPin, INPUT);
  pinMode (okSwitchPin, INPUT);
  pinMode (backLight, OUTPUT);
  pinMode (buzzerPin, OUTPUT);
  digitalWrite(backLight, HIGH);
  lcd.begin(20, 4);//setup lcd
  lcd.clear();                
  lcd.setCursor(1, 1);
  lcd.print("ARDUINO REWORK 1.1");
  //Welcome melody
  tone(buzzerPin, 523);
  delay(200);
  tone(buzzerPin, 659);
  delay(200);
  tone(buzzerPin, 784);
  delay(200);
  tone(buzzerPin, 1046);
  delay(200);
  noTone(buzzerPin);
  // wait for MAX chips to stabilize and splash screen
  delay(2000);
  lcd.clear();

  pinMode(RelayPin1, OUTPUT);//setup ssr pins as output
  pinMode(RelayPin2, OUTPUT);

  windowStartTime = millis();//Just total time sketch has been running
  // Initialize time keeping variable for TC1
  nextCheck1 = millis();
  // Initialize  top thermocouple reading variable
  nextRead1 = millis();
  //initialize soak timer variable


  myPID1.SetOutputLimits(0, WindowSize);//myPID1 = top heater PID loop
  myPID2.SetOutputLimits(0, WindowSize);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
}



void loop()
{
  //these variables read switch pins
  upSwitchState = digitalRead(upSwitchPin);
  downSwitchState = digitalRead(downSwitchPin);
  editSwitchState = digitalRead(editSwitchPin);
  cancelSwitchState = digitalRead(cancelSwitchPin);
  okSwitchState = digitalRead(okSwitchPin);

  unsigned long currentMillis = millis();

  int sv1 = Setpoint1;
  int sv2 = Setpoint2;

  int tc1 = Input1;
  int tc2 = Input2;

  if (upSwitchState==HIGH || downSwitchState==HIGH || editSwitchState==HIGH || cancelSwitchState==HIGH || okSwitchState==HIGH) {
    tone(buzzerPin, 523);
    delay(100);
    noTone(buzzerPin);
  }

  
  if (reflowState==REFLOW_STATE_COMPLETE || alarmOn){
    int i=0;
    if (i<15 && cancelSwitchState==LOW) {
      alarmOn=true;
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(400);
      i++;
    } else {
      alarmOn=false;
    }    
    
  }


  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:

    if (millis() > nextRead1)
    {

      // Read thermocouples next sampling period
      nextRead1 += SENSOR_SAMPLING_TIME;

      lcd.setCursor(16, 2);
      lcd.print("    ");
      lcd.setCursor(16, 2);
      lcd.print(tc1);
      lcd.setCursor(16, 3);
      lcd.print("    ");
      lcd.setCursor(16, 3);
      lcd.print(tc2);
    }

    //setup idle screen
    lcd.setCursor(0, 0);
    lcd.print("        IDLE        ");
    lcd.setCursor(0, 1);
    lcd.print(" PTN:");
   lcd.print(currentProfile);
   // lcd.setCursor(13, 1);
   lcd.print("      STEP:1 ");
    //lcd.print(currentStep);
  //  lcd.print(1);
    lcd.setCursor(1, 2);
    lcd.print("TH");
    lcd.setCursor(5, 2);
    lcd.print("SV:");
    lcd.print(temperatureStep1);
    lcd.print(" ");    
    lcd.setCursor(13, 2);
    lcd.print("PV:");  
    lcd.setCursor(1, 3);
    lcd.print("BH");
    lcd.setCursor(5, 3);
    lcd.print("SV:");
    lcd.print(sv2);
    lcd.setCursor(13, 3);
    lcd.print("PV:");
    windowStartTime = millis();
    if (upSwitchState == HIGH)//if up switch is pressed go to next profile
    {
      currentProfile = currentProfile + 1;
      delay(25);
      if (currentProfile >= 5)//if currentProfile = 4 and up is pressed go back to profile 1
      {
        currentProfile = 1;

      }
    }
    if (downSwitchState == HIGH)//same as above just go down one profile
    {
      currentProfile = currentProfile - 1;
      delay(25);
      if (currentProfile <= 0)
      {
        currentProfile = 4;
      }

    }
    loadProfile();//call the loadProfile function to load from eeprom

    if (editSwitchState == HIGH ) //if edit is pressed go to menu
    {
      delay(25);
      reflowState = REFLOW_STATE_MENU_STEPS;
    }


    if (okSwitchState == HIGH)
    {
      reflowStatus = REFLOW_STATUS_ON;
      reflowState = REFLOW_STATE_STEP_1_RAMP;

    }

    break;
  case REFLOW_STATE_MENU_STEPS:

    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(2, 2);
    lcd.print("Profile Steps: ");
    lcd.print(profileSteps);


    if (upSwitchState == HIGH)
    {
      profileSteps = profileSteps + 1;
      delay(25);
      if (profileSteps >= 10) {
        profileSteps = 1;
      }
    }
    if (downSwitchState == HIGH)
    {
      profileSteps = profileSteps - 1;
      delay(25);
      if (profileSteps <= 0) {
        profileSteps = 9;

      }
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(2, 2);
      lcd.print("                  ");

      reflowState = REFLOW_STATE_MENU_BOTTOM_HEAT;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }

    break;

  case REFLOW_STATE_MENU_BOTTOM_HEAT:

    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Bottom Heat: ");
    lcd.print(sv2);
    if (upSwitchState == HIGH)
    {
      Setpoint2 = Setpoint2 + 10;
      delay(25);
      if (Setpoint2 >= 350)
      {
        Setpoint2 = 350;
      }
    }
    if (downSwitchState == HIGH)
    {
      Setpoint2 = Setpoint2 - 10;
      delay(25);
      if (Setpoint2 <= 100)
      {
        Setpoint2 = 100;

      }
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(2, 2);
      lcd.print("                  ");

      reflowState = REFLOW_STATE_MENU_STEP_1_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }

    break;

  case REFLOW_STATE_MENU_STEP_1_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 1 Ramp:");
    lcd.print(rampRateStep1);
    if (upSwitchState == HIGH)
    {
      rampRateStep1 = rampRateStep1 + .25;
      delay(25);
      if (rampRateStep1 >= 9)
      {
        rampRateStep1 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep1 = rampRateStep1 - .25;
      delay(25);
      if (rampRateStep1 <= .25)
      {
        rampRateStep1 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_1_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_1_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 1 Target: ");
    lcd.print(temperatureStep1);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep1 = temperatureStep1 + 2;
      delay(25);
      if (temperatureStep1 >= 250)
      {
        temperatureStep1 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep1 = temperatureStep1 - 2;
      delay(25);
      if (temperatureStep1 <= 0)
      {
        temperatureStep1 = 0;
      }
      if (temperatureStep1 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep1 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_1_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_1_DWELL:

    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 1 Dwell: ");
    lcd.print(dwellTimerStep1);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep1 = dwellTimerStep1 + 5;
      delay(25);
      if (dwellTimerStep1 >= 1000)
      {
        dwellTimerStep1 = 1000;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep1 = dwellTimerStep1 - 5;
      delay(25);
      if (dwellTimerStep1 <= 0)
      {
        dwellTimerStep1 = 0;
      }
      if (dwellTimerStep1 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep1 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 1)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 2)
      {
        reflowState = REFLOW_STATE_MENU_STEP_2_RAMP;
      }
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_MENU_STEP_2_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 2 Ramp:");
    lcd.print(rampRateStep2);
    if (upSwitchState == HIGH)
    {
      rampRateStep2 = rampRateStep2 + .25;
      delay(25);
      if (rampRateStep2 >= 9)
      {
        rampRateStep2 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep2 = rampRateStep2 - .25;
      delay(25);
      if (rampRateStep2 <= .25)
      {
        rampRateStep2 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_2_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_2_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 2 Target: ");
    lcd.print(temperatureStep2);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep2 = temperatureStep2 + 2;
      delay(25);
      if (temperatureStep2 >= 250)
      {
        temperatureStep2 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep2 = temperatureStep2 - 2;
      delay(25);
      if (temperatureStep2 <= 0)
      {
        temperatureStep2 = 0;
      }
      if (temperatureStep2 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep2 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_2_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_2_DWELL:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 2 Dwell: ");
    lcd.print(dwellTimerStep2);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep2 = dwellTimerStep2 + 5;
      delay(25);
      if (dwellTimerStep2 >= 999)
      {
        dwellTimerStep2 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep2 = dwellTimerStep2 - 5;
      delay(25);
      if (dwellTimerStep2 <= 0)
      {
        dwellTimerStep2 = 0;
      }
      if (dwellTimerStep2 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep2 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 2)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 3)
      {
        reflowState = REFLOW_STATE_MENU_STEP_3_RAMP;
      }

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_3_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 3 Ramp:");
    lcd.print(rampRateStep3);
    if (upSwitchState == HIGH)
    {
      rampRateStep3 = rampRateStep3 + .25;
      delay(25);
      if (rampRateStep3 >= 9)
      {
        rampRateStep3 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep3 = rampRateStep3 - .25;
      delay(25);
      if (rampRateStep3 <= .25)
      {
        rampRateStep3 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_3_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_3_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 3 Target: ");
    lcd.print(temperatureStep3);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep3 = temperatureStep3 + 2;
      delay(25);
      if (temperatureStep3 >= 250)
      {
        temperatureStep3 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep3 = temperatureStep3 - 2;
      delay(25);
      if (temperatureStep3 <= 0)
      {
        temperatureStep3 = 0;
      }
      if (temperatureStep3 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep3 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_3_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_3_DWELL:

    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 3 Dwell: ");
    lcd.print(dwellTimerStep3);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep3 = dwellTimerStep3 + 5;
      delay(25);
      if (dwellTimerStep3 >= 999)
      {
        dwellTimerStep3 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep3 = dwellTimerStep3 - 5;
      delay(25);
      if (dwellTimerStep3 <= 0)
      {
        dwellTimerStep3 = 0;
      }
      if (dwellTimerStep3 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep3 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 3)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 4)
      {
        reflowState = REFLOW_STATE_MENU_STEP_4_RAMP;
      }

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_4_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 4 Ramp:");
    lcd.print(rampRateStep4);
    if (upSwitchState == HIGH)
    {
      rampRateStep4 = rampRateStep4 + .25;
      delay(25);
      if (rampRateStep4 >= 9)
      {
        rampRateStep4 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep4 = rampRateStep4 - .25;
      delay(25);
      if (rampRateStep4 <= .25)
      {
        rampRateStep4 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_4_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_4_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 4 Target: ");
    lcd.print(temperatureStep4);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep4 = temperatureStep4 + 2;
      delay(25);
      if (temperatureStep4 >= 250)
      {
        temperatureStep4 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep4 = temperatureStep4 - 2;
      delay(25);
      if (temperatureStep4 <= 0)
      {
        temperatureStep4 = 0;
      }
      if (temperatureStep4 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep4 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_4_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_4_DWELL:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 4 Dwell: ");
    lcd.print(dwellTimerStep4);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep4 = dwellTimerStep4 + 5;
      delay(25);
      if (dwellTimerStep4 >= 999)
      {
        dwellTimerStep4 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep4 = dwellTimerStep4 - 5;
      delay(25);
      if (dwellTimerStep4 <= 0)
      {
        dwellTimerStep4 = 0;
      }
      if (dwellTimerStep4 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep4 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 4)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 5)
      {
        reflowState = REFLOW_STATE_MENU_STEP_5_RAMP;
      }

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_5_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 5 Ramp:");
    lcd.print(rampRateStep5);
    if (upSwitchState == HIGH)
    {
      rampRateStep5 = rampRateStep5 + .25;
      delay(25);
      if (rampRateStep5 >= 9)
      {
        rampRateStep5 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep5 = rampRateStep5 - .25;
      delay(25);
      if (rampRateStep5 <= .25)
      {
        rampRateStep5 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_5_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_5_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 5 Target: ");
    lcd.print(temperatureStep5);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep5 = temperatureStep5 + 2;
      delay(25);
      if (temperatureStep5 >= 250)
      {
        temperatureStep5 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep5 = temperatureStep5 - 2;
      delay(25);
      if (temperatureStep5 <= 0)
      {
        temperatureStep5 = 0;
      }
      if (temperatureStep5 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep5 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_5_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_5_DWELL:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 5 Dwell: ");
    lcd.print(dwellTimerStep5);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep5 = dwellTimerStep5 + 5;
      delay(25);
      if (dwellTimerStep5 >= 999)
      {
        dwellTimerStep5 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep5 = dwellTimerStep5 - 5;
      delay(25);
      if (dwellTimerStep5 <= 0)
      {
        dwellTimerStep5 = 0;
      }
      if (dwellTimerStep5 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep5 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 5)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 6)
      {
        reflowState = REFLOW_STATE_MENU_STEP_6_RAMP;
      }

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_6_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 6 Ramp:");
    lcd.print(rampRateStep6);
    if (upSwitchState == HIGH)
    {
      rampRateStep6 = rampRateStep6 + .25;
      delay(25);
      if (rampRateStep6 >= 9)
      {
        rampRateStep6 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep6 = rampRateStep6 - .25;
      delay(25);
      if (rampRateStep6 <= .25)
      {
        rampRateStep6 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_6_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_6_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 6 Target: ");
    lcd.print(temperatureStep6);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep6 = temperatureStep6 + 2;
      delay(25);
      if (temperatureStep6 >= 250)
      {
        temperatureStep6 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep6 = temperatureStep6 - 2;
      delay(25);
      if (temperatureStep6 <= 0)
      {
        temperatureStep6 = 0;
      }
      if (temperatureStep6 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep6 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_6_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_6_DWELL:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 6 Dwell: ");
    lcd.print(dwellTimerStep6);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep6 = dwellTimerStep6 + 5;
      delay(25);
      if (dwellTimerStep6 >= 999)
      {
        dwellTimerStep6 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep6 = dwellTimerStep6 - 5;
      delay(25);
      if (dwellTimerStep6 <= 0)
      {
        dwellTimerStep6 = 0;
      }
      if (dwellTimerStep6 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep6 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 6)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 7)
      {
        reflowState = REFLOW_STATE_MENU_STEP_7_RAMP;
      }

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_7_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 7 Ramp:");
    lcd.print(rampRateStep7);
    if (upSwitchState == HIGH)
    {
      rampRateStep7 = rampRateStep7 + .25;
      delay(25);
      if (rampRateStep7 >= 9)
      {
        rampRateStep7 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep7 = rampRateStep7 - .25;
      delay(25);
      if (rampRateStep7 <= .25)
      {
        rampRateStep7 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_7_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_7_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 7 Target: ");
    lcd.print(temperatureStep7);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep7 = temperatureStep7 + 2;
      delay(25);
      if (temperatureStep7 >= 250)
      {
        temperatureStep7 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep7 = temperatureStep7 - 2;
      delay(25);
      if (temperatureStep7 <= 0)
      {
        temperatureStep7 = 0;
      }
      if (temperatureStep7 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep7 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_7_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_7_DWELL:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 7 Dwell: ");
    lcd.print(dwellTimerStep7);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep7 = dwellTimerStep7 + 5;
      delay(25);
      if (dwellTimerStep7 >= 999)
      {
        dwellTimerStep7 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep7 = dwellTimerStep7 - 5;
      delay(25);
      if (dwellTimerStep7 <= 0)
      {
        dwellTimerStep7 = 0;
      }
      if (dwellTimerStep7 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep7 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 7)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 8)
      {
        reflowState = REFLOW_STATE_MENU_STEP_8_RAMP;
      }

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_8_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 8 Ramp:");
    lcd.print(rampRateStep8);
    if (upSwitchState == HIGH)
    {
      rampRateStep8 = rampRateStep8 + .25;
      delay(25);
      if (rampRateStep8 >= 9)
      {
        rampRateStep8 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep8 = rampRateStep8 - .25;
      delay(25);
      if (rampRateStep8 <= .25)
      {
        rampRateStep8 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_8_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_8_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 8 Target: ");
    lcd.print(temperatureStep8);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep8 = temperatureStep8 + 2;
      delay(25);
      if (temperatureStep8 >= 250)
      {
        temperatureStep8 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep8 = temperatureStep8 - 2;
      delay(25);
      if (temperatureStep8 <= 0)
      {
        temperatureStep8 = 0;
      }
      if (temperatureStep8 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep8 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_8_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_8_DWELL:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 8 Dwell: ");
    lcd.print(dwellTimerStep8);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep8 = dwellTimerStep8 + 5;
      delay(25);
      if (dwellTimerStep8 >= 999)
      {
        dwellTimerStep8 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep8 = dwellTimerStep8 - 5;
      delay(25);
      if (dwellTimerStep8 <= 0)
      {
        dwellTimerStep8 = 0;
      }
      if (dwellTimerStep8 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep8 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      if (profileSteps == 8)
      {
        lcd.setCursor(0, 2);
        lcd.print("                   ");
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      }
      else if(profileSteps >= 9)
      {
        reflowState = REFLOW_STATE_MENU_STEP_9_RAMP;
      }

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_9_RAMP:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 9 Ramp:");
    lcd.print(rampRateStep9);
    if (upSwitchState == HIGH)
    {
      rampRateStep9 = rampRateStep9 + .25;
      delay(25);
      if (rampRateStep9 >= 9)
      {
        rampRateStep9 = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep9 = rampRateStep9 - .25;
      delay(25);
      if (rampRateStep9 <= .25)
      {
        rampRateStep9 = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);

      reflowState = REFLOW_STATE_MENU_STEP_9_TARGET;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;
  case REFLOW_STATE_MENU_STEP_9_TARGET:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(1, 2);
    lcd.print("Step 9 Target: ");
    lcd.print(temperatureStep9);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep9 = temperatureStep9 + 2;
      delay(25);
      if (temperatureStep9 >= 250)
      {
        temperatureStep9 = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep9 = temperatureStep9 - 2;
      delay(25);
      if (temperatureStep9 <= 0)
      {
        temperatureStep9 = 0;
      }
      if (temperatureStep9 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep9 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");

      reflowState = REFLOW_STATE_MENU_STEP_9_DWELL;
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_STEP_9_DWELL:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(2, 2);
    lcd.print("Step 9 Dwell: ");
    lcd.print(dwellTimerStep9);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep9 = dwellTimerStep9 + 5;
      delay(25);
      if (dwellTimerStep9 >= 999)
      {
        dwellTimerStep9 = 999;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep9 = dwellTimerStep9 - 5;
      delay(25);
      if (dwellTimerStep9 <= 0)
      {
        dwellTimerStep9 = 0;
      }
      if (dwellTimerStep9 <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep9 <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(0, 2);
      lcd.print("                   ");
      reflowState = REFLOW_STATE_MENU_BOTTOM_P;

    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_MENU_BOTTOM_P:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(0, 2);
    lcd.print("                  ");
    lcd.setCursor(3, 2);
    lcd.print("Bottom Heater:");
    lcd.setCursor(8, 3);
    lcd.print("P=");
    lcd.print(kp2);
    if (upSwitchState == HIGH)
    {
      kp2 = kp2 + 1;
      delay(25);
      if (kp2 >= 500)
      {
        kp2 = 500;
      }
    }
    if (downSwitchState == HIGH)
    {
      kp2 = kp2 - 1;
      delay(25);
      if (kp2 <= 0)
      {
        kp2 = 0;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(2, 2);
      lcd.print("                  ");
      lcd.setCursor(6, 3);
      lcd.print("          ");

      reflowState = REFLOW_STATE_MENU_BOTTOM_I;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_BOTTOM_I:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(3, 2);
    lcd.print("Bottom Heater:");
    lcd.setCursor(8, 3);
    lcd.print("I=");
    lcd.print(ki2);
    if (upSwitchState == HIGH)
    {
      ki2 = ki2 + 1;
      delay(25);
      if (ki2 >= 500)
      {
        ki2 = 500;
      }
    }
    if (downSwitchState == HIGH)
    {
      ki2 = ki2 - 1;
      delay(25);
      if (ki2 <= 0)
      {
        ki2 = 0;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(2, 2);
      lcd.print("                  ");
      lcd.setCursor(6, 3);
      lcd.print("          ");

      reflowState = REFLOW_STATE_MENU_BOTTOM_D;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;

  case REFLOW_STATE_MENU_BOTTOM_D:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(3, 2);
    lcd.print("Bottom Heater:");
    lcd.setCursor(8, 3);
    lcd.print("D=");
    lcd.print(kd2);
    if (upSwitchState == HIGH)
    {
      kd2 = kd2 + 1;
      delay(25);
      if (kd2 >= 500)
      {
        kd2 = 500;
      }
    }
    if (downSwitchState == HIGH)
    {
      kd2 = kd2 - 1;
      delay(25);
      if (kd2 <= 0)
      {
        kd2 = 0;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(2, 2);
      lcd.print("                  ");
      lcd.setCursor(6, 3);
      lcd.print("          ");

      reflowState = REFLOW_STATE_MENU_TOP_P;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;

  case REFLOW_STATE_MENU_TOP_P:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(5, 2);
    lcd.print("Top Heater:");    
    lcd.setCursor(8, 3);
    lcd.print("P=");
    lcd.print(kp1);
    if (upSwitchState == HIGH)
    {
      kp1 = kp1 + 1;
      delay(25);
      if (kp1 >= 500)
      {
        kp1 = 500;
      }
    }
    if (downSwitchState == HIGH)
    {
      kp1 = kp1 - 1;
      delay(25);
      if (kp1 <= 0)
      {
        kp1 = 0;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(2, 2);
      lcd.print("                  ");
      lcd.setCursor(7, 3);
      lcd.print("          ");

      reflowState = REFLOW_STATE_MENU_TOP_I;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_TOP_I:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(5, 2);
    lcd.print("Top Heater:");
    lcd.setCursor(8, 3);
    lcd.print("I=");
    lcd.print(ki1);
    if (upSwitchState == HIGH)
    {
      ki1 = ki1 + 1;
      delay(25);
      if (ki1 >= 500)
      {
        ki1 = 500;
      }
    }
    if (downSwitchState == HIGH)
    {
      ki1 = ki1 - 1;
      delay(25);
      if (ki1 <= 0)
      {
        ki1 = 0;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      lcd.setCursor(2, 2);
      lcd.print("                  ");
      lcd.setCursor(6, 3);
      lcd.print("          ");

      reflowState = REFLOW_STATE_MENU_TOP_D;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;

  case REFLOW_STATE_MENU_TOP_D:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Profile ");
    lcd.print(currentProfile);
    lcd.print(" Edit");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(5, 2);
    lcd.print("Top Heater:");
    lcd.setCursor(8, 3);
    lcd.print("D=");
    lcd.print(kd1);
    if (upSwitchState == HIGH)
    {
      kd1 = kd1 + 1;
      delay(25);
      if (kd1 >= 500)
      {
        kd1 = 500;
      }
    }
    if (downSwitchState == HIGH)
    {
      kd1 = kd1 - 1;
      delay(25);
      if (kd1 <= 0)
      {
        kd1 = 0;
      }

    }
    if (okSwitchState == HIGH)
    {
      int rampRateStep1Write = rampRateStep1 * 20;
      int rampRateStep2Write = rampRateStep2 * 20;
      int rampRateStep3Write = rampRateStep3 * 20;
      int rampRateStep4Write = rampRateStep4 * 20;
      int rampRateStep5Write = rampRateStep5 * 20;
      int rampRateStep6Write = rampRateStep6 * 20;
      int rampRateStep7Write = rampRateStep7 * 20;
      int rampRateStep8Write = rampRateStep8 * 20;
      int rampRateStep9Write = rampRateStep9 * 20;

      if (currentProfile == 1)
      {
        EEPROM.write(eepromAddress, profileSteps);
        EEPROM.write((eepromAddress) + 1, Setpoint2);
        EEPROM.write((eepromAddress) + 2, rampRateStep1Write);
        EEPROM.write((eepromAddress) + 3, rampRateStep2Write);
        EEPROM.write((eepromAddress) + 4, rampRateStep3Write);    
        EEPROM.write((eepromAddress) + 5, rampRateStep4Write);
        EEPROM.write((eepromAddress) + 6, rampRateStep5Write);
        EEPROM.write((eepromAddress) + 7, rampRateStep6Write);
        EEPROM.write((eepromAddress) + 8, rampRateStep7Write);
        EEPROM.write((eepromAddress) + 9, rampRateStep8Write);
        EEPROM.write((eepromAddress) + 10, rampRateStep9Write);

        EEPROM.write((eepromAddress) + 11, dwellTimerStep1 / 5);
        EEPROM.write((eepromAddress) + 12, dwellTimerStep2 / 5);
        EEPROM.write((eepromAddress) + 13, dwellTimerStep3 / 5);
        EEPROM.write((eepromAddress) + 14, dwellTimerStep4 / 5);
        EEPROM.write((eepromAddress) + 15, dwellTimerStep5 / 5);
        EEPROM.write((eepromAddress) + 16, dwellTimerStep6 / 5);
        EEPROM.write((eepromAddress) + 17, dwellTimerStep7 / 5);
        EEPROM.write((eepromAddress) + 18, dwellTimerStep8 / 5);
        EEPROM.write((eepromAddress) + 19, dwellTimerStep9 / 5);

        EEPROM.write((eepromAddress) + 20, temperatureStep1);
        EEPROM.write((eepromAddress) + 21, temperatureStep2);
        EEPROM.write((eepromAddress) + 22, temperatureStep3);
        EEPROM.write((eepromAddress) + 23, temperatureStep4);
        EEPROM.write((eepromAddress) + 24, temperatureStep5);
        EEPROM.write((eepromAddress) + 25, temperatureStep6);
        EEPROM.write((eepromAddress) + 26, temperatureStep7);
        EEPROM.write((eepromAddress) + 27, temperatureStep8);
        EEPROM.write((eepromAddress) + 28, temperatureStep9);

        EEPROM.write((eepromAddress) + 122, kp1);
        EEPROM.write((eepromAddress) + 123, ki1);
        EEPROM.write((eepromAddress) + 124, kd1);
        EEPROM.write((eepromAddress) + 125, kp2);
        EEPROM.write((eepromAddress) + 126, ki2);
        EEPROM.write((eepromAddress) + 127, kd2);    
      }
      if (currentProfile == 2)
      {
        EEPROM.write(eepromAddress + 29, profileSteps);
        EEPROM.write((eepromAddress) + 30, Setpoint2);
        EEPROM.write((eepromAddress) + 31, rampRateStep1Write);
        EEPROM.write((eepromAddress) + 32, rampRateStep2Write);
        EEPROM.write((eepromAddress) + 33, rampRateStep3Write);    
        EEPROM.write((eepromAddress) + 34, rampRateStep4Write);
        EEPROM.write((eepromAddress) + 35, rampRateStep5Write);
        EEPROM.write((eepromAddress) + 36, rampRateStep6Write);
        EEPROM.write((eepromAddress) + 37, rampRateStep7Write);
        EEPROM.write((eepromAddress) + 38, rampRateStep8Write);
        EEPROM.write((eepromAddress) + 39, rampRateStep9Write);

        EEPROM.write((eepromAddress) + 40, dwellTimerStep1 / 5);
        EEPROM.write((eepromAddress) + 41, dwellTimerStep2 / 5);
        EEPROM.write((eepromAddress) + 42, dwellTimerStep3 / 5);
        EEPROM.write((eepromAddress) + 43, dwellTimerStep4 / 5);
        EEPROM.write((eepromAddress) + 44, dwellTimerStep5 / 5);
        EEPROM.write((eepromAddress) + 45, dwellTimerStep6 / 5);
        EEPROM.write((eepromAddress) + 46, dwellTimerStep7 / 5);
        EEPROM.write((eepromAddress) + 47, dwellTimerStep8 / 5);
        EEPROM.write((eepromAddress) + 48, dwellTimerStep9 / 5);

        EEPROM.write((eepromAddress) + 49, temperatureStep1);
        EEPROM.write((eepromAddress) + 50, temperatureStep2);
        EEPROM.write((eepromAddress) + 51, temperatureStep3);
        EEPROM.write((eepromAddress) + 52, temperatureStep4);
        EEPROM.write((eepromAddress) + 53, temperatureStep5);
        EEPROM.write((eepromAddress) + 54, temperatureStep6);
        EEPROM.write((eepromAddress) + 55, temperatureStep7);
        EEPROM.write((eepromAddress) + 56, temperatureStep8);
        EEPROM.write((eepromAddress) + 57, temperatureStep9);

        EEPROM.write((eepromAddress) + 128, kp1);
        EEPROM.write((eepromAddress) + 129, ki1);
        EEPROM.write((eepromAddress) + 130, kd1);
        EEPROM.write((eepromAddress) + 131, kp2);
        EEPROM.write((eepromAddress) + 132, ki2);
        EEPROM.write((eepromAddress) + 133, kd2);    

      }
      if (currentProfile == 3)
      {
        EEPROM.write(eepromAddress + 58, profileSteps);
        EEPROM.write((eepromAddress) + 59, Setpoint2);
        EEPROM.write((eepromAddress) + 60, rampRateStep1Write);
        EEPROM.write((eepromAddress) + 61, rampRateStep2Write);
        EEPROM.write((eepromAddress) + 62, rampRateStep3Write);    
        EEPROM.write((eepromAddress) + 63, rampRateStep4Write);
        EEPROM.write((eepromAddress) + 64, rampRateStep5Write);
        EEPROM.write((eepromAddress) + 65, rampRateStep6Write);
        EEPROM.write((eepromAddress) + 66, rampRateStep7Write);
        EEPROM.write((eepromAddress) + 67, rampRateStep8Write);
        EEPROM.write((eepromAddress) + 68, rampRateStep9Write);

        EEPROM.write((eepromAddress) + 69, dwellTimerStep1 / 5);
        EEPROM.write((eepromAddress) + 70, dwellTimerStep2 / 5);
        EEPROM.write((eepromAddress) + 71, dwellTimerStep3 / 5);
        EEPROM.write((eepromAddress) + 72, dwellTimerStep4 / 5);
        EEPROM.write((eepromAddress) + 73, dwellTimerStep5 / 5);
        EEPROM.write((eepromAddress) + 74, dwellTimerStep6 / 5);
        EEPROM.write((eepromAddress) + 75, dwellTimerStep7 / 5);
        EEPROM.write((eepromAddress) + 76, dwellTimerStep8 / 5);
        EEPROM.write((eepromAddress) + 77, dwellTimerStep9 / 5);

        EEPROM.write((eepromAddress) + 78, temperatureStep1);
        EEPROM.write((eepromAddress) + 79, temperatureStep2);
        EEPROM.write((eepromAddress) + 80, temperatureStep3);
        EEPROM.write((eepromAddress) + 81, temperatureStep4);
        EEPROM.write((eepromAddress) + 82, temperatureStep5);
        EEPROM.write((eepromAddress) + 83, temperatureStep6);
        EEPROM.write((eepromAddress) + 84, temperatureStep7);
        EEPROM.write((eepromAddress) + 85, temperatureStep8);
        EEPROM.write((eepromAddress) + 86, temperatureStep9);

        EEPROM.write((eepromAddress) + 134, kp1);
        EEPROM.write((eepromAddress) + 135, ki1);
        EEPROM.write((eepromAddress) + 136, kd1);
        EEPROM.write((eepromAddress) + 137, kp2);
        EEPROM.write((eepromAddress) + 138, ki2);
        EEPROM.write((eepromAddress) + 139, kd2);    

      }
      if (currentProfile == 4)
      {
        EEPROM.write(eepromAddress + 87, profileSteps);
        EEPROM.write((eepromAddress) + 88, Setpoint2);
        EEPROM.write((eepromAddress) + 89, rampRateStep1Write);
        EEPROM.write((eepromAddress) + 90, rampRateStep2Write);
        EEPROM.write((eepromAddress) + 91, rampRateStep3Write);    
        EEPROM.write((eepromAddress) + 92, rampRateStep4Write);
        EEPROM.write((eepromAddress) + 93, rampRateStep5Write);
        EEPROM.write((eepromAddress) + 94, rampRateStep6Write);
        EEPROM.write((eepromAddress) + 95, rampRateStep7Write);
        EEPROM.write((eepromAddress) + 96, rampRateStep8Write);
        EEPROM.write((eepromAddress) + 97, rampRateStep9Write);

        EEPROM.write((eepromAddress) + 98, dwellTimerStep1 / 5);
        EEPROM.write((eepromAddress) + 99, dwellTimerStep2 / 5);
        EEPROM.write((eepromAddress) + 100, dwellTimerStep3 / 5);
        EEPROM.write((eepromAddress) + 101, dwellTimerStep4 / 5);
        EEPROM.write((eepromAddress) + 102, dwellTimerStep5 / 5);
        EEPROM.write((eepromAddress) + 103, dwellTimerStep6 / 5);
        EEPROM.write((eepromAddress) + 104, dwellTimerStep7 / 5);
        EEPROM.write((eepromAddress) + 105, dwellTimerStep8 / 5);
        EEPROM.write((eepromAddress) + 106, dwellTimerStep9 / 5);

        EEPROM.write((eepromAddress) + 107, temperatureStep1);
        EEPROM.write((eepromAddress) + 108, temperatureStep2);
        EEPROM.write((eepromAddress) + 109, temperatureStep3);
        EEPROM.write((eepromAddress) + 110, temperatureStep4);
        EEPROM.write((eepromAddress) + 111, temperatureStep5);
        EEPROM.write((eepromAddress) + 112, temperatureStep6);
        EEPROM.write((eepromAddress) + 113, temperatureStep7);
        EEPROM.write((eepromAddress) + 114, temperatureStep8);
        EEPROM.write((eepromAddress) + 115, temperatureStep9);

        EEPROM.write((eepromAddress) + 116, kp1);
        EEPROM.write((eepromAddress) + 116, kp1);
        EEPROM.write((eepromAddress) + 117, ki1);
        EEPROM.write((eepromAddress) + 118, kd1);
        EEPROM.write((eepromAddress) + 119, kp2);
        EEPROM.write((eepromAddress) + 120, ki2);
        EEPROM.write((eepromAddress) + 121, kd2);
      }


      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;


  case REFLOW_STATE_STEP_1_RAMP:
    currentStep = 1;
    lcd.setCursor(8, 0);
    lcd.print("RUN ");
    startTemp = tc1;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    lcd.setCursor(8, 3);
    lcd.print(sv2);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep1) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print("   ");
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep1) {
      lcd.setCursor(8,2);
      lcd.print(temperatureStep1);
      reflowState = REFLOW_STATE_STEP_1;
    }
    if (cancelSwitchState == HIGH)
    {

      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_1:
    Setpoint1 = temperatureStep1;    
    if (Input1 >= temperatureStep1)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_1_DWELL;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_1_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep1) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 1) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else
        reflowState = REFLOW_STATE_STEP_2_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_STEP_2_RAMP:
    currentStep = 2;
    startTemp = temperatureStep1;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep2) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep2) {
      lcd.setCursor(8,2);
      lcd.print(temperatureStep2);
      reflowState = REFLOW_STATE_STEP_2;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_2:
    Setpoint1 = temperatureStep2;    
    if (Input1 >= temperatureStep2)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_2_DWELL;
    }

    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_2_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep2) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 2) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else
        reflowState = REFLOW_STATE_STEP_3_RAMP;
    }

    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_3_RAMP:
    currentStep = 3;
    startTemp = temperatureStep2;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep3) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = temperatureStep3;
    }

    if (setpointRamp >= temperatureStep3) {
      lcd.setCursor(8,2);
      lcd.print(temperatureStep3);
      reflowState = REFLOW_STATE_STEP_3;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_3:
    Setpoint1 = temperatureStep3;    
    if (Input1 >= temperatureStep3)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_3_DWELL;
    }

    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_3_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep3) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 3) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else
        reflowState = REFLOW_STATE_STEP_4_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_STEP_4_RAMP:
    currentStep = 4;
    startTemp = temperatureStep3;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep4) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep4){
      lcd.setCursor(8,2);
      lcd.print(temperatureStep4);
      reflowState = REFLOW_STATE_STEP_4;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_4:
    Setpoint1 = temperatureStep4;    
    if (Input1 >= temperatureStep4)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_4_DWELL;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_4_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep4) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 4) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else
        reflowState = REFLOW_STATE_STEP_5_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_5_RAMP:
    currentStep = 5;
    startTemp = temperatureStep4;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep5) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep5) {
      lcd.setCursor(8,2);
      lcd.print(temperatureStep5);
      reflowState = REFLOW_STATE_STEP_5;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_5:
    Setpoint1 = temperatureStep5;    
    if (Input1 >= temperatureStep5)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_5_DWELL;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_5_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep5) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 5) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else
        reflowState = REFLOW_STATE_STEP_6_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_6_RAMP:
    currentStep = 6;
    startTemp = temperatureStep5;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep6) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep6){
      lcd.setCursor(8,2);
      lcd.print(temperatureStep6);
      reflowState = REFLOW_STATE_STEP_6;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_6:
    Setpoint1 = temperatureStep6;    
    if (Input1 >= temperatureStep6)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_6_DWELL;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_6_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep6) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 6) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else  
        reflowState = REFLOW_STATE_STEP_7_RAMP;

    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_7_RAMP:
    currentStep = 7;
    startTemp = temperatureStep6;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep7) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep7){
      lcd.setCursor(8,2);
      lcd.print(temperatureStep7);
      reflowState = REFLOW_STATE_STEP_7;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_7:
    Setpoint1 = temperatureStep7;    
    if (Input1 >= temperatureStep7)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_7_DWELL;
    }

    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_7_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep7) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 7) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else
        reflowState = REFLOW_STATE_STEP_8_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_8_RAMP:
    currentStep = 8;
    startTemp = temperatureStep7;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep8) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep8){
      lcd.setCursor(8,2);
      lcd.print(temperatureStep8);
      reflowState = REFLOW_STATE_STEP_8;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_8:
    Setpoint1 = temperatureStep8;    
    if (Input1 >= temperatureStep8)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_8_DWELL;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_8_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep8) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 8) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else
        reflowState = REFLOW_STATE_STEP_9_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_9_RAMP:
    currentStep = 9;
    startTemp = temperatureStep8;
    lcd.setCursor(18, 1);
    lcd.print(currentStep);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep9) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep9){
      lcd.setCursor(8,2);
      lcd.print(temperatureStep9);
      reflowState = REFLOW_STATE_STEP_9;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_9:
    Setpoint1 = temperatureStep9;    
    if (Input1 >= temperatureStep9)
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_9_DWELL;
    }

    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_STEP_9_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep9) {
      counter = 0;
      setpointRamp = 0;
      reflowState = REFLOW_STATE_COMPLETE;
    }
    if (cancelSwitchState == HIGH)
    {
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;


  case REFLOW_STATE_COMPLETE:

    reflowStatus = REFLOW_STATUS_OFF;
    reflowState = REFLOW_STATE_IDLE;
  }

  Input1 = thermocouple1.readCelsius();
  Input2 = thermocouple2.readCelsius();  

  if (reflowStatus == REFLOW_STATUS_ON)
  {
    if (millis() > nextRead1)
    {

      // Read thermocouples next sampling period
      nextRead1 += SENSOR_SAMPLING_TIME;

      lcd.setCursor(16, 2);
      lcd.print("    ");
      lcd.setCursor(16, 2);
      lcd.print(tc1);
      lcd.setCursor(16, 3);
      lcd.print("    ");
      lcd.setCursor(16, 3);
      lcd.print(tc2);
    }
    myPID1.SetTunings(kp1,ki1,kd1);
    myPID2.SetTunings(kp2,ki2,kd2);
    myPID1.Compute();
    myPID2.Compute();  

    if(millis() - windowStartTime>WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if(Output1 > millis() - windowStartTime) digitalWrite(RelayPin1,HIGH);  

    else digitalWrite(RelayPin1,LOW);

    if(Output2 > millis() - windowStartTime) digitalWrite(RelayPin2,HIGH);  

    else digitalWrite(RelayPin2,LOW);
  }
  else
  {
    digitalWrite(RelayPin1, LOW);
    digitalWrite(RelayPin2, LOW);
  }
}



