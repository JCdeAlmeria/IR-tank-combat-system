#include <Arduino.h>
/*
  The IR combat system includes:
  - An identical programmed arduino nano board in every vehicle, managing combat functions.
  - Another arduino nano board to control the vehicle, with outputs which are inputs for the combat arduino.
  - The control or vehicle arduino is
  - A infrared LED (TSAL6100, 940nm), directly controlled by the combat arduino, but with the shot initiated by the control arduino.
  - 4x infrared sensors (tsop4838), covering 360º.
  - Several mosfets allowing current to different vehicle systems, allowed by the combat arduino.
  - Several ouputs to voluntarily indicate vehicle status (outputs for LEDs, buzzers, etc.).

  INPUTS and OUTPUTS of the arduino board:
    Combat INPUTS:
    - 4x IR sensors

    Control INPUTS (commands received from the control arduino):
    - Command shot
    - Command repair the vehicle

    Combat OUTPUTS (after receiving the command from the control arduino, effectively shoot if legal):
    - To shoot (allows current by the IR LED)

    Status OUTPUTS (typically for different color LEDs):
    - Recharging: time when is forbidden to shoot again after shooting
    - Impact received
    - Invulnerability: Invulnerability time after receiving an impact
    - Repairing

    - These outputs can be ignored or used as inputs for the control arduino to do complex actions, like simulated recoil, etc.

    Failure OUTPUTS (connected to mosfets and allowing or denying current to vehicle systems):
    - Motors failure: a percentage of the total voltage; proportional
    - Turret failure: preventing turret movements, but not shooting
    - Total failure: battery energy cut to vehicle (but not to combat arduino)
*/

//Pin definitions for arduino nano
//A0 & A1 free pins
#define pinSensorFront A2
#define pinSensorBack A3
#define pinSensorLeft A6
#define pinSensorRight A7
//A4 & A5 free pins for a future I2C with LCD or another systems

//2 & 3 free pins
#define pinCommandShoot 4 //An analogic pin could be used
#define pinShoot 5 //PWM
#define pinRecharging 6 //PWM

#define pinCommandRepair 7 //An analogic pin could be used
#define pinImpact 8
#define pinInvulnerable 9 //PWM
#define pinRepairing 10 //PWM

#define pinFailureMotors 11 //PWM
#define pinFailureTurret 12
#define pinFailureTotal 13

//Global states of the combat system
bool impactedFront=false;
bool impactedBack=false;
bool impactedLeft=false;
bool impactedRight=false;

bool invulnerable=false;
bool recharging=false;

bool allowedRepair=false;
bool repairing=false;

//Asynchronous multitasking parameters
unsigned long currentMillis = 0;
unsigned long startingMillisOfInvulnerability = 0;
unsigned long startingMillisOfShot = 0;

//Adjusting sensors parameters
#define thresholdImpact 270 //Being 0 saturated sensor and 1023 darkness

//Game parameters
byte remainingImpacts = 4;
#define timeOfInvulnerability 2000 //ms
#define durationOfShot 200 //ms
#define timeBetweenShots 200 //ms

//Debugging
void debug(); //With platformIO each function must be declared before calling it
#include <Wire.h>
//#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR 0x27
LiquidCrystal_I2C lcd(I2C_ADDR,2,1,0,4,5,6,7);
int minimumSignalDetected=1023;
#define timeOfMinimumSignalDetected 2000
unsigned long startingMillisOfSignalDetected = 0;

void setup() {
  //Initiate pins by their type
  pinMode(pinSensorFront,INPUT);
  pinMode(pinSensorBack,INPUT);
  pinMode(pinSensorLeft,INPUT);
  pinMode(pinSensorRight,INPUT);
  pinMode(pinCommandShoot,INPUT);
  pinMode(pinShoot,OUTPUT);
  pinMode(pinRecharging,OUTPUT);
  pinMode(pinCommandRepair,INPUT);
  pinMode(pinImpact,OUTPUT);
  pinMode(pinInvulnerable,OUTPUT);
  pinMode(pinRepairing,OUTPUT);
  pinMode(pinFailureMotors,OUTPUT);
  pinMode(pinFailureTurret,OUTPUT);
  pinMode(pinFailureTotal,OUTPUT);
  digitalWrite(pinFailureMotors,LOW);
  digitalWrite(pinFailureTurret,LOW);
  digitalWrite(pinFailureTotal,LOW);
  //Debugging
  Serial.begin(9600);
  lcd.begin(16,2);
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Signal: ");
  for(byte n=0; n<5; n++){
    digitalWrite(pinInvulnerable,HIGH);
    delay(200);
    digitalWrite(pinInvulnerable,LOW);
    delay(200);
  }
}

void senseImpact(){
  int signalFront = analogRead(pinSensorFront);
  // int signalBack = analogRead(pinSensorBack);
  // int signalRight = analogRead(pinSensorRight);
  // int signalLeft = analogRead(pinSensorLeft);
  if (signalFront<thresholdImpact){
    impactedFront=true;
  }
  //TODO
}

void shoot(){
  digitalWrite(pinShoot, HIGH);
  recharging=true;
  startingMillisOfShot=currentMillis;
}

void repair(){
  //TODO
}

void updateStates(){
  if (remainingImpacts==0){
    digitalWrite(pinFailureTotal,HIGH);
  }
  //TODO: Remaining effects of impacts: FailureMotors and FailureTurret

  if (currentMillis-startingMillisOfShot>=durationOfShot){
    digitalWrite(pinShoot, LOW);
  }
  if (recharging&&(currentMillis-startingMillisOfShot>=timeBetweenShots+durationOfShot)){
    recharging=false;
  }
  if (invulnerable&&(currentMillis-startingMillisOfInvulnerability>=timeOfInvulnerability)){
    invulnerable=false;
  }
}

void manageIndicators(){
  if (recharging){
    digitalWrite(pinRecharging,HIGH);
  } else {
    digitalWrite(pinRecharging,LOW);
  }

  if (invulnerable){
    digitalWrite(pinInvulnerable,HIGH);
  } else {
    digitalWrite(pinInvulnerable,LOW);
  }
  //TODO: To do some of the indicators blink with a raising frequency

  //TODO: Impact signal for a defined time, slowing motors
}

void loop() {
  currentMillis = millis();

  //Update impacted areas
  if (!invulnerable){
    senseImpact();
  }

  if (impactedFront||impactedBack||impactedLeft||impactedRight){
    //TODO: different effects depending on the impacted side
    //Actions produced by an impact
    remainingImpacts = remainingImpacts - 1;
    invulnerable=true;
    startingMillisOfInvulnerability=currentMillis; //Start the time counter for Invulnerability
    impactedFront=false;
  }

  updateStates(); //Control states and times of shooting, etc to deactivate

  //Shot
  if(!recharging&&digitalRead(pinCommandShoot)==HIGH){
    shoot();
  }

  //Reparation
  if (allowedRepair&&digitalRead(pinCommandRepair)==HIGH){
    repair();
  }

  //Manage of state indicators LEDs
  manageIndicators();

  //Comment call after a stable version
  debug();
}

void debug(){
  int signalFront = analogRead(pinSensorFront);
  Serial.print(invulnerable);
  Serial.print("   ");
  Serial.print(signalFront);
  Serial.println();
  lcd.setCursor(8,0);
  lcd.print("    ");
  lcd.setCursor(8,0);
  lcd.print(signalFront);
  if (signalFront<minimumSignalDetected){
    minimumSignalDetected=signalFront;
    lcd.setCursor(8,1);
    lcd.print("    ");
    lcd.setCursor(8,1);
    lcd.print(minimumSignalDetected);
    startingMillisOfSignalDetected=currentMillis;
  }
  if (currentMillis-startingMillisOfSignalDetected>=timeOfMinimumSignalDetected){
    minimumSignalDetected=1023;
  }
}
