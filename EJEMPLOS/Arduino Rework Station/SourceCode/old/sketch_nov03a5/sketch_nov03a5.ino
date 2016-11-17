

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

int editStep = 0;

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
int currentStep = 1;
byte profileSteps;

double rampRateStep[9];

int dwellTimerStep[9];

int kp1;
int ki1;
int kd1;
int kp2;
int ki2;
int kd2;

int setpointRamp;
int startTemp;

int temperatureStep[9];

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
  //new stuff {
  REFLOW_STATE_MENU_STEP_RAMP,
  REFLOW_STATE_MENU_STEP_TARGET,
  REFLOW_STATE_MENU_STEP_DWELL,
  //}
  REFLOW_STATE_MENU_BOTTOM_P,
  REFLOW_STATE_MENU_BOTTOM_I,
  REFLOW_STATE_MENU_BOTTOM_D,
  REFLOW_STATE_MENU_TOP_P,
  REFLOW_STATE_MENU_TOP_I,
  REFLOW_STATE_MENU_TOP_D,
  //new stuff {
  REFLOW_STATE_STEP_RAMP,
  REFLOW_STATE_STEP,
  REFLOW_STATE_STEP_DWELL,
  //} 

  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_ERROR
}
reflowState_t;

typedef enum REFLOW_STATUS //this is simply to check if reflow should be running or not
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

//Update whole screen boolean
boolean updateScreen = true;

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
    profileSteps = EEPROM.read(eepromAddress);
    Setpoint2 = EEPROM.read(eepromAddress + 1);
    //new stuff {
    for (int i=0; i<9; i+1) {
      rampRateStep[i] = EEPROM.read(eepromAddress + i + 2)/20;
      i++;
    }
    //}
    //new stuff {
    for (int i=0; i<9; i+1) {
      dwellTimerStep[i] = EEPROM.read(eepromAddress + i + 11)*5;
      i++;
    }
    //}
    //new stuff {
    for (int i=0; i<9; i+1) {
      temperatureStep[i] = EEPROM.read(eepromAddress + i + 20);
      i++;
    }
    //}
    kp1 = EEPROM.read(eepromAddress + 122);
    ki1 = EEPROM.read(eepromAddress + 123);
    kd1 = EEPROM.read(eepromAddress + 124);
    kp2 = EEPROM.read(eepromAddress + 125);
    ki2 = EEPROM.read(eepromAddress + 126);
    kd2 = EEPROM.read(eepromAddress + 127);

  }

  if (currentProfile == 2)
  {
    //new stuff {
    for (int i=0; i<9; i+1) {
      rampRateStep[i] = EEPROM.read(eepromAddress + i + 31)/20;
      i++;
    }
    //}
    profileSteps = EEPROM.read(eepromAddress + 29);
    Setpoint2 = EEPROM.read(eepromAddress + 30);
    //new stuff {
    for (int i=0; i<9; i+1) {
      dwellTimerStep[i] = EEPROM.read(eepromAddress + i + 40)*5;
      i++;
    }
    //}  
    //new stuff {
    for (int i=0; i<9; i+1) {
      temperatureStep[i] = EEPROM.read(eepromAddress + i + 49);
      i++;
    }
    //}
    kp1 = EEPROM.read(eepromAddress + 128);
    ki1 = EEPROM.read(eepromAddress + 129);
    kd1 = EEPROM.read(eepromAddress + 130);
    kp2 = EEPROM.read(eepromAddress + 131);
    ki2 = EEPROM.read(eepromAddress + 132);
    kd2 = EEPROM.read(eepromAddress + 133);

  }
  if (currentProfile == 3)
  {
    //new stuff {
    for (int i=0; i<9; i+1) {
      rampRateStep[i] = EEPROM.read(eepromAddress + i + 60)/20;
      i++;
    }
    //}
    profileSteps = EEPROM.read(eepromAddress + 58);
    Setpoint2 = EEPROM.read(eepromAddress + 59);
    //new stuff {
    for (int i=0; i<9; i+1) {
      dwellTimerStep[i] = EEPROM.read(eepromAddress + i + 69)*5;
      i++;
    }
    //}
    //new stuff {
    for (int i=0; i<9; i+1) {
      temperatureStep[i] = EEPROM.read(eepromAddress + i + 78);
      i++;
    }
    //}
    kp1 = EEPROM.read(eepromAddress + 134);
    ki1 = EEPROM.read(eepromAddress + 135);
    kd1 = EEPROM.read(eepromAddress + 136);
    kp2 = EEPROM.read(eepromAddress + 137);
    ki2 = EEPROM.read(eepromAddress + 138);
    kd2 = EEPROM.read(eepromAddress + 139);

  }
  if (currentProfile == 4)
  {
    //new stuff {
    for (int i=0; i<9; i+1) {
      rampRateStep[i] = EEPROM.read(eepromAddress + i + 89)/20;
      i++;
    }
    //}
    profileSteps = EEPROM.read(eepromAddress + 87);
    Setpoint2 = EEPROM.read(eepromAddress + 88);
    //new stuff {
    for (int i=0; i<9; i+1) {
      dwellTimerStep[i] = EEPROM.read(eepromAddress + i + 98)*5;
      i++;
    }
    //}
    //new stuff {
    for (int i=0; i<9; i+1) {
      temperatureStep[i] = EEPROM.read(eepromAddress + i + 107);
      i++;
    }
    //}
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
    } 
    else {
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

    //Update whole screen only once
    if (updateScreen) {
      //setup idle screen
      lcd.clear();
      lcd.setCursor(8, 0);
      lcd.print("IDLE");
      lcd.setCursor(1, 1);
      lcd.print("PTN:");
      //lcd.print(currentProfile);
      // lcd.setCursor(13, 1);
      lcd.print("      STEP:1 ");
      //lcd.print(currentStep);
      //  lcd.print(1);
      lcd.setCursor(0, 2);
      lcd.print(" TH  SV:");
      //lcd.setCursor(5, 2);
      //lcd.print("SV:");
      // lcd.print(temperatureStep1);
      //lcd.print(" ");    
      lcd.setCursor(13, 2);
      lcd.print("PV:");  
      lcd.setCursor(0, 3);
      lcd.print(" BH  SV:");
      //lcd.setCursor(5, 3);
      //lcd.print("SV:");
      // lcd.print(sv2);
      lcd.setCursor(13, 3);
      lcd.print("PV:");
      updateScreen = false;
    }
    lcd.setCursor(5, 1);
    lcd.print(currentProfile);
    lcd.print(" ");
    lcd.setCursor(8, 2);
    lcd.print(temperatureStep[0]);
    lcd.print(" ");
    lcd.setCursor(8, 3);
    lcd.print(sv2);
    lcd.print(" ");
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
      updateScreen = true;
    }


    if (okSwitchState == HIGH)
    {
      updateScreen = true;
      reflowStatus = REFLOW_STATUS_ON;
      reflowState = REFLOW_STATE_STEP_RAMP;

    }

    break;
  case REFLOW_STATE_MENU_STEPS:
    if (updateScreen) {
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
      updateScreen = false;
    }
    lcd.setCursor(17, 2);
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
      //lcd.setCursor(2, 2);
      //lcd.print("                  ");

      updateScreen = true;
      reflowState = REFLOW_STATE_MENU_BOTTOM_HEAT;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      updateScreen = true;
      reflowState = REFLOW_STATE_IDLE;
    }

    break;

  case REFLOW_STATE_MENU_BOTTOM_HEAT:

    if (updateScreen) {
      //lcd.clear();
      // lcd.setCursor(3, 0);
      //lcd.print("Profile ");
      // lcd.print(currentProfile);
      // lcd.print(" Edit");
      lcd.setCursor(2, 2);
      lcd.print("Bottom Heat:      ");
      updateScreen = false;
    }
    lcd.setCursor(14, 2);
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
      //lcd.setCursor(2, 2);
      //lcd.print("                  ");
      updateScreen = true;
      //reflowState = REFLOW_STATE_MENU_STEP_1_RAMP;
      reflowState = REFLOW_STATE_MENU_STEP_RAMP;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      //lcd.clear();
      updateScreen = true;
      reflowState = REFLOW_STATE_IDLE;
    }

    break;

    //new stuff "case REFLOW_STATE_MENU_STEP_RAMP"*************************************

  case REFLOW_STATE_MENU_STEP_RAMP:

    if (updateScreen) {
      //lcd.setCursor(3, 0);
      //lcd.print("Profile ");
      //lcd.print(currentProfile);
      //lcd.print(" Edit *");
      lcd.setCursor(2, 2);
      lcd.print("Step ");
      lcd.print(editStep + 1);
      lcd.print(" Ramp:      ");
      updateScreen = false;
    }
    //lcd.setCursor(7, 2);
    //lcd.print(editStep + 1);
    lcd.setCursor(14, 2);
    lcd.print(rampRateStep[editStep]);
    if (upSwitchState == HIGH)
    {
      rampRateStep[editStep] = rampRateStep[editStep] + .25;
      delay(25);
      if (rampRateStep[editStep] >= 9)
      {
        rampRateStep[editStep] = 9;
      }
    }
    if (downSwitchState == HIGH)
    {
      rampRateStep[editStep] = rampRateStep[editStep] - .25;
      delay(25);
      if (rampRateStep[editStep] <= .25)
      {
        rampRateStep[editStep] = .25;
      }

    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      updateScreen = true;
      if (editStep + 1 == profileSteps){
        editStep = 0;
        reflowState = REFLOW_STATE_MENU_STEP_TARGET;
      } 
      else {
        editStep++;
      }
    }
    if (cancelSwitchState == HIGH)
    {
      updateScreen = true;
      editStep = 0; 
      delay(25);
      lcd.clear();
      reflowState = REFLOW_STATE_IDLE;
    }

    break;

    //new stuff "case REFLOW_STATE_MENU_STEP_TARGET"*************************************

  case REFLOW_STATE_MENU_STEP_TARGET:
    if (updateScreen) {
      lcd.setCursor(2, 2);
      lcd.print("Step ");
      lcd.print(editStep + 1);
      lcd.print(" Target: ");
      lcd.print(temperatureStep[editStep]);
      lcd.print(" ");
      updateScreen = false;
    }
    //lcd.clear();
    //lcd.setCursor(3, 0);
    //lcd.print("Profile ");
    //lcd.print(currentProfile);
    //lcd.print(" Edit");
    //lcd.setCursor(7, 2);
    //lcd.print(editStep + 1);
    lcd.print(" ");
    lcd.setCursor(16, 2);
    lcd.print(temperatureStep[editStep]);
    lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      temperatureStep[editStep] = temperatureStep[editStep] + 2;
      delay(25);
      if (temperatureStep[editStep] >= 250)
      {
        temperatureStep[editStep] = 250;
      }
    }
    if (downSwitchState == HIGH)
    {

      temperatureStep[editStep] = temperatureStep[editStep] - 2;
      delay(25);
      if (temperatureStep[editStep] <= 0)
      {
        temperatureStep[editStep] = 0;
      }
      if (temperatureStep[editStep] <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (temperatureStep[editStep] <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }

    if (okSwitchState == HIGH)
    {
      delay(25);
      updateScreen = true;
      if (editStep + 1 == profileSteps){
        editStep = 0;
        reflowState = REFLOW_STATE_MENU_STEP_DWELL;
      } 
      else {
        editStep++;
      }
    }  
    if (cancelSwitchState == HIGH)
    {
      updateScreen = true;
      delay(25);
      editStep = 0;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

    //new stuff "case REFLOW_STATE_MENU_STEP_DWELL"*************************************
  case REFLOW_STATE_MENU_STEP_DWELL:

    if (updateScreen) {
      lcd.setCursor(2, 2);
      lcd.print("Step ");
      lcd.print(editStep + 1);
      lcd.print(" Dwell: ");
      lcd.print(dwellTimerStep[editStep]);
      lcd.print(" ");
    }
    //lcd.setCursor(7, 2);
    //lcd.print(editStep + 1);
    lcd.print(" ");
    lcd.setCursor(15, 2);
    lcd.print(dwellTimerStep[editStep]);
    lcd.print(" ");
    //lcd.clear();
    //lcd.setCursor(3, 0);
    //lcd.print("Profile ");
    //lcd.print(currentProfile);
    //lcd.print(" Edit");
    //lcd.setCursor(2, 2);
    // lcd.print("Step 1 Dwell: ");
    //lcd.print(dwellTimerStep1);
    //lcd.print(" ");
    if (upSwitchState == HIGH)
    {
      dwellTimerStep[editStep] = dwellTimerStep[editStep] + 5;
      delay(25);
      if (dwellTimerStep[editStep] >= 1000)
      {
        dwellTimerStep[editStep] = 1000;
      }
    }
    if (downSwitchState == HIGH)
    {

      dwellTimerStep[editStep] = dwellTimerStep[editStep] - 5;
      delay(25);
      if (dwellTimerStep[editStep] <= 0)
      {
        dwellTimerStep[editStep] = 0;
      }
      if (dwellTimerStep[editStep] <= 99)
      {
        lcd.setCursor(18, 2);
        lcd.print(" ");
      }    
      if (dwellTimerStep[editStep] <= 9)
      {
        lcd.setCursor(17, 2);
        lcd.print(" ");
      }    
    }
    if (okSwitchState == HIGH)
    {
      delay(25);
      updateScreen = true;
      if (editStep + 1 == profileSteps){

        editStep = 0;
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
      } 
      else {
        editStep++;
      }
    }  
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      updateScreen = true;
      editStep = 0;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_MENU_BOTTOM_P:
    if (updateScreen){
      lcd.setCursor(0, 2);
      lcd.print("                  ");
      lcd.setCursor(3, 2);
      lcd.print("Bottom Heater:");
      lcd.setCursor(8, 3);
      lcd.print("P=");
      lcd.print(kp2);
      updateScreen = false;
    }
    lcd.setCursor(10, 3);
    lcd.print(kp2);
    lcd.print("  ");
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
      updateScreen = true;
      reflowState = REFLOW_STATE_MENU_BOTTOM_I;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      updateScreen = true;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_BOTTOM_I:
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
      reflowState = REFLOW_STATE_MENU_BOTTOM_D;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;

  case REFLOW_STATE_MENU_BOTTOM_D:
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
      reflowState = REFLOW_STATE_MENU_TOP_P;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;

  case REFLOW_STATE_MENU_TOP_P:
    if (updateScreen){
      lcd.setCursor(0, 2);
      lcd.print("                  ");
      lcd.setCursor(5, 2);
      lcd.print("Top Heater:");
      lcd.setCursor(8, 3);
      lcd.print("P=");
      lcd.print(kp1);
      updateScreen = false;
    }
       
    lcd.setCursor(10, 3);
    lcd.print(kp1);
    lcd.print("  ");
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
      updateScreen = true;
      reflowState = REFLOW_STATE_MENU_TOP_I;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      updateScreen = true;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
  case REFLOW_STATE_MENU_TOP_I:
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
      reflowState = REFLOW_STATE_MENU_TOP_D;
    }
    if (cancelSwitchState == HIGH)
    {
      delay(25);
      reflowState = REFLOW_STATE_IDLE;
    }  
    break;

  case REFLOW_STATE_MENU_TOP_D:
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


      if (currentProfile == 1)
      {

        EEPROM.write(eepromAddress, profileSteps);
        EEPROM.write((eepromAddress) + 1, Setpoint2);
        int i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 2, rampRateStep[i] * 20);
          i++;
        }

        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 11, dwellTimerStep[i]/5);
          i++;
        }

        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 20, temperatureStep[i]);
          i++;
        }

        EEPROM.write((eepromAddress) + 122, kp1);
        EEPROM.write((eepromAddress) + 123, ki1);
        EEPROM.write((eepromAddress) + 124, kd1);
        EEPROM.write((eepromAddress) + 125, kp2);
        EEPROM.write((eepromAddress) + 126, ki2);
        EEPROM.write((eepromAddress) + 127, kd2);    
      }
      if (currentProfile == 2)
      {
        int i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 31, rampRateStep[i] * 20);
          i++;
        }
        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 40, dwellTimerStep[i]/5);
          i++;
        }
        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 49, temperatureStep[i]);
          i++;
        }

        EEPROM.write(eepromAddress + 29, profileSteps);
        EEPROM.write((eepromAddress) + 30, Setpoint2);

        EEPROM.write((eepromAddress) + 128, kp1);
        EEPROM.write((eepromAddress) + 129, ki1);
        EEPROM.write((eepromAddress) + 130, kd1);
        EEPROM.write((eepromAddress) + 131, kp2);
        EEPROM.write((eepromAddress) + 132, ki2);
        EEPROM.write((eepromAddress) + 133, kd2);    

      }
      if (currentProfile == 3)
      {
        int i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 60, rampRateStep[i] * 20);
          i++;
        }
        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 69, dwellTimerStep[i]/5);
          i++;
        }
        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 78, temperatureStep[i]);
          i++;
        }

        EEPROM.write(eepromAddress + 58, profileSteps);
        EEPROM.write((eepromAddress) + 59, Setpoint2);

        EEPROM.write((eepromAddress) + 134, kp1);
        EEPROM.write((eepromAddress) + 135, ki1);
        EEPROM.write((eepromAddress) + 136, kd1);
        EEPROM.write((eepromAddress) + 137, kp2);
        EEPROM.write((eepromAddress) + 138, ki2);
        EEPROM.write((eepromAddress) + 139, kd2);    

      }
      if (currentProfile == 4)
      {
        int i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 89, rampRateStep[i] * 20);
          i++;
        }
        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 98, dwellTimerStep[i]);
          i++;
        }
        i=0;
        for (i=0 ; i<9 ; i+1){
          EEPROM.write((eepromAddress) + i + 107, temperatureStep[i]);
          i++;
        }

        EEPROM.write(eepromAddress + 87, profileSteps);
        EEPROM.write((eepromAddress) + 88, Setpoint2);


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

    //new stuff "case REFLOW_STATE_STEP_RAMP"*************************************
  case REFLOW_STATE_STEP_RAMP:
    //currentStep = 1;
    if (updateScreen){
      lcd.setCursor(8, 0);
      lcd.print("RUN ");
      updateScreen = false;
    }
    startTemp = tc1;
    lcd.setCursor(16, 1);
    lcd.print(currentStep);
    lcd.setCursor(8, 3);
    lcd.print(sv2);
    //ramp rate counter
    if(currentMillis - previousMillis > 1000 / rampRateStep[currentStep-1]) {//seconds counter
      previousMillis = currentMillis;
      counter = counter + 1;
      setpointRamp = startTemp + counter;
      lcd.setCursor(8, 2);
      lcd.print("   ");
      lcd.setCursor(8, 2);
      lcd.print(setpointRamp);
      lcd.print("  ");
      Setpoint1 = setpointRamp;
    }

    if (setpointRamp >= temperatureStep[currentStep-1]) {
      lcd.setCursor(8,2);
      lcd.print(temperatureStep[currentStep-1]);
      reflowState = REFLOW_STATE_STEP;
      updateScreen = true;
    }
    if (cancelSwitchState == HIGH)
    {
      currentStep = 1;
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
      updateScreen = true;
    }
    break;

    //new stuff "case REFLOW_STATE_STEP"*************************************
  case REFLOW_STATE_STEP:
    Setpoint1 = temperatureStep[currentStep-1];    
    if (Input1 >= temperatureStep[currentStep-1])
    {
      counter = 0;
      reflowState = REFLOW_STATE_STEP_DWELL;
    }
    if (cancelSwitchState == HIGH)
    {
      updateScreen = true;
      currentStep = 1;
      counter = 0;
      setpointRamp = 0;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

    //new stuff "case REFLOW_STATE_STEP_DWELL"*************************************
  case REFLOW_STATE_STEP_DWELL:
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
      counter = counter + 1;
    }
    if (counter == dwellTimerStep[currentStep-1]) {
      counter = 0;
      setpointRamp = 0;
      if (profileSteps == 1) {
        reflowState = REFLOW_STATE_COMPLETE;
      }
      else {
        currentStep++;
        reflowState = REFLOW_STATE_STEP_RAMP;
      }
    }
    if (cancelSwitchState == HIGH)
    {
      updateScreen = true;
      currentStep = 1;
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



















