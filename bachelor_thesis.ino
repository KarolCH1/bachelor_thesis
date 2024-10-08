#include <TFT.h>
#include "Mux.h"
#include <SPI.h>
#include "LCD.h"
#include "DEV_Config.h"
#include <Arduino.h>
#include <Stepper.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <ezButton.h>
#include <math.h>

using namespace admux;

//Przycisk bezpieczeństwa
#define emerg A0

//Silniki krokowe (zapewne niepotrzebne)
#define pul2 32
#define dir2 33
#define dir1 34
#define pul1 35
#define dir3 36
#define pul3 37
#define pul4 38
#define dir4 39
#define dir5 40
#define pul5 41
#define pul6 42
#define dir6 43

//Czujniki krańcowe
#define sw_j1_front A4
#define sw_j6 A5
#define sw_j5_down A6
#define sw_j5_up A7
#define sw_j4_cw A8
#define sw_j4_acw A9
#define sw_j3_back A10
#define sw_j3_front A11
#define sw_j2_back A12
#define sw_j2_front A13
#define sw_j1_back A14
#define sw_grip A15



//Kontroler + ledy
#define muxPin 44
#define S0 45
#define S1 46
#define S2 47
#define S3 48
#define pd1 49
#define pd2 50

#define red 11
#define green 12
#define blue 13

//Home position
int home_pos[6] = { -16950, 9005, -3688, -11880, -10267, -2259 };  //Perpendicular to the ground  -10087, down -218230
int stepper5_downPos = -5418 * 2;

//Cube positions
double p1[3] = { 28.86, 0.00, 17.65 };
double p2[3] = { 33.86, 0.00, 17.65 };
double p3[3] = { 38.86, 0.00, 17.65 };
double p4[3] = { 34.00, -7.16, 17.65 };

double p5[3] = { 22.90, 0.00, 3.92 };  //dot
double p6[3] = { 29.8, 11.23, 16.3};

double pkos[3] = {27.08, 0.00, 16.12 };

double pv1[3] = {27.24, -20.85, 39.03};
double pv2[3] = {22.32, 31.96, 18.98};
double pv3[3] = {22.32, 14.18, 39.03};
double pv4[3] = {27.24, -15.85, 18.98};
double pv5[3] = {36.23, 0.00, 10.29};
double pv6[3] = {9.31, 0.00, 55.67};



//Stepper
AccelStepper stepper6(1, pul6, dir6);
AccelStepper stepper5(1, pul5, dir5);
AccelStepper stepper4(1, pul4, dir4);
AccelStepper stepper3(1, pul3, dir3);
AccelStepper stepper2(1, pul2, dir2);
AccelStepper stepper1(1, pul1, dir1);

//==============================================================
//Motors parameters
//==============================================================

//Steps per revolution
double stepper1_spr = 400.0 * 50.0 * (40.0 / 12.0);
double stepper2_spr = 800.0 * 50.0;
double stepper3_spr = 800.0 * 50.0;
double stepper4_spr = 1600.0 * (19.0 + (38.0 / 187.0));
double stepper5_spr = 3200.0 * (13.0 + (212.0 / 289.0));
double stepper6_spr = 3200;

double stepper5_dpr = 360.0 / stepper5_spr;

int stepCalPos1 = 0, stepCalPos2 = 0, stepCalPos3 = 0, stepToGo4 = 0, stepCalPos5 = 0, stepToGo6 = 0;


int speed = 100;  //Speed in percents
int max_speed_j1 = 800, max_speed_j2 = 150, max_speed_j3 = 150, max_speed_j4 = 400, max_speed_j5 = 500, max_speed_j6 = 60;
int init_home6 = 0, init_home5 = 0, init_home4 = 0, init_home3 = 0, init_home2 = 0, init_home1 = 0;
int gripper_pos = 70;
int motor_pos_inc = 10;
int servo_item_size = 60;
int servo_item_sizeMM = 0;
double posChangeX = 0;
double posChangeY = 0;
double posChangeZ = 0;
double posX = 0, posY = 0, posZ = 0;
double j = 0.01;

double DegreesToSteps(double stepper_spr, double degrees) {
  return (round(degrees * stepper_spr / 360.0));
}

double StepsToDegrees(double stepper_spr, double steps) {
  return (steps * 360 / stepper_spr);
}

int servToMM(int pos) {
  return -pos / 3 + 40;
}

//==============================================================
//Help variables
//==============================================================
bool init_disp_auto = false, init_disp_teach = false;
bool gripper_status = false;
bool mode_changed = false;
bool speed_changed = false;
bool motors_changed = false;
bool gripper_changed = false;
bool speed_toggle = false;
bool gripper_toggle = false;
bool save_pos = false;
bool homing_toggle = false;
bool inverse_kin = true;
bool calib_toggle = false; //Indicates if calibration procedure was completed

bool calibration = true;
bool is_calibrated = false;
bool calib_text = false;
bool j1_calib = false;
bool j2_calib = false;
bool j3_calib = false;
bool j4_calib = false;
bool j5_calib = false;
bool j6_calib = false;
bool is_home = false;

bool inPos1 = false, inPos2 = false, inPos3 = false;
bool inPos4 = false, inPos5 = false, inPos6 = false;


bool j1p = false, j1m = false;
bool j2p = false, j2m = false;
bool j3p = false, j3m = false;
bool j4p = false, j4m = false;
bool j5p = false, j5m = false;
bool j6p = false, j6m = false;

bool motors = false;
bool changingGripper = false;
bool changingGripper_toggle = false;

//==============================================================
//Positions
//==============================================================
int pos_count = 0;
double saved_position[10][6];

double xpos, ypos, zpos;
double L1 = 17.188, L2 = 21.836, L3 = 19.215;
double x_home = 19.215, y_home = 0.0, z_home = 39.024;  //Położenie początkowe w mm
double Th1Desired = 0.0, Th2Desired = 0.0, Th3Desired = 0.0, Th5Desired = 0.0;
double Th1 = 0.0, Th2 = 90.0, Th3 = 90.0, Th5 = 180.0;
double Th1_rad = 0.0, Th2_rad = 0.0, Th3_rad = 0.0;
double Th2_1 = 0.0;
double Th2_2 = 0.0;
double Th3_1 = 0.0;
double Th3_2 = 0.0;
int Rot4 = 0, Rot5 = 0, Rot6 = 0;  //

//==============================================================
//Funkcja poruszajaca od punktu do punktu
//==============================================================
int signX = 0, signY = 0, signZ = 0;
double r = 0, n = 0, incX = 0, incY = 0, incZ = 0;
double incRot4 = 0, incRot6 = 0;
double xtemp = 0, ytemp = 0, ztemp = 0;
bool inPosX = false, inPosY = false, inPosZ = false;
bool inPosRot4 = false, inPosRot6 = false;
int loopInc = 0;

int speedJ1 = 0, speedJ2 = 0, speedJ3 = 0, speedJ4 = 0, speedJ5 = 0, speedJ6 = 0;
int acceleration = 1;

//Funkcje obliczające kąty i położenie
double radToDeg(double arc_in_rad) {
  return arc_in_rad * (180.0 / M_PI);
}

double degToRad(double arc_in_deg) {
  return arc_in_deg / (180.0 / M_PI);
}

double TH1(double x, double y) {
  return radToDeg(atan2(y, x));
}

double TH2(double x, double y, double z) {
  Th2_1 = acos((L2 * L2 - L3 * L3 + (z - L1) * (z - L1) + y * y + x * x) / (2.0 * L2 * sqrt((z - L1) * (z - L1) + y * y + x * x)));
  Th2_2 = atan2(z - L1, sqrt(y * y + x * x));
  return radToDeg(Th2_1 + Th2_2);
}

double TH3(double x, double y, double z) {
  return radToDeg(acos((L2 * L2 + L3 * L3 - x * x - y * y - (z - L1) * (z - L1)) / (2.0 * L2 * L3)));
}

double TH5(double Th2_, double Th3_) {
  return 180 - Th2_ - Th3_;
}

int sgn(double x) {
  if (x > 0.0) return 1;
  else if (x < 0.0) return -1;
  else return 0;
}

void updatePosition() {
     Th1 = stepper1.currentPosition() * 360 / stepper1_spr;
     Th2 = stepper2.currentPosition() * 360 / stepper2_spr + 90;
     Th3 = stepper3.currentPosition() * 360 / stepper3_spr + 90;     
     Th1_rad = degToRad(Th1);
     Th2_rad = degToRad(Th2);
     Th3_rad = degToRad(Th3);
     xpos = -cos(Th1_rad) * (L3 * cos(Th2_rad + Th3_rad) + L2 * cos(Th2_rad));
     ypos = -sin(Th1_rad) * (L3 * cos(Th2_rad + Th3_rad) + L2 * cos(Th2_rad));
     zpos = L1 + L3 * sin(Th2_rad + Th3_rad) + L2 * sin(Th2_rad);
}

void moveController(double x, double y, double z, double r5) {
  inPos1 = false;
  inPos2 = false;
  inPos3 = false;
  inPos4 = false;
  inPos5 = false;
  inPos6 = false;
  Th1Desired = TH1(x, y);
  Th2Desired = TH2(x, y, z);  //+ Th2_0;
  Th3Desired = TH3(x, y, z);  // + Th3_0;
  Th5Desired = TH5(Th2Desired, Th3Desired) + StepsToDegrees(stepper5_spr, r5);

  stepCalPos1 = DegreesToSteps(stepper1_spr, Th1Desired);
  stepCalPos2 = -DegreesToSteps(stepper2_spr, Th2Desired - 90);
  stepCalPos3 = -DegreesToSteps(stepper3_spr, Th3Desired - 90);
  stepCalPos5 = DegreesToSteps(stepper5_spr, Th5Desired);

  stepper1.moveTo(stepCalPos1);
  stepper2.moveTo(stepCalPos2);
  stepper3.moveTo(stepCalPos3);
  stepper5.moveTo(stepCalPos5);

  stepper1.setSpeed(2 * max_speed_j1);
  stepper2.setSpeed(2 * max_speed_j2);
  stepper3.setSpeed(2 * max_speed_j3);
  stepper5.setSpeed(2 * max_speed_j5);

  //Serial.print(stepCalPos1); Serial.print(" | "); Serial.print(stepCalPos2); Serial.print(" | "); Serial.println(stepCalPos3);
  updatePosition();


  while (!inPos1 || !inPos2 || !inPos3 || !inPos5) {
    if (stepper1.currentPosition() != stepCalPos1) stepper1.runSpeedToPosition();
    else inPos1 = true;
    if (stepper2.currentPosition() != stepCalPos2) stepper2.runSpeedToPosition();
    else inPos2 = true;
    if (stepper3.currentPosition() != stepCalPos3) stepper3.runSpeedToPosition();
    else inPos3 = true;
    if (stepper5.currentPosition() != stepCalPos5) stepper5.runSpeedToPosition();
    else inPos5 = true;
  }
}

void moveToPosInc(double x, double y, double z, double r5, double inc) {
  inPos1 = false;
  inPos2 = false;
  inPos3 = false;
  inPos5 = false;

  Th1_rad = degToRad(stepper1.currentPosition() * 360 / stepper1_spr);
  Th2_rad = degToRad(stepper2.currentPosition() * 360 / stepper2_spr + 90);
  Th3_rad = degToRad(stepper3.currentPosition() * 360 / stepper3_spr + 90);
  xpos = -cos(Th1_rad) * (L3 * cos(Th2_rad + Th3_rad) + L2 * cos(Th2_rad));
  ypos = -sin(Th1_rad) * (L3 * cos(Th2_rad + Th3_rad) + L2 * cos(Th2_rad));
  zpos = L1 + L3 * sin(Th2_rad + Th3_rad) + L2 * sin(Th2_rad);

  signX = sgn(x - xpos), signY = sgn(y - ypos), signZ = sgn(z - zpos);
  r = sqrt(sq(x) + sq(y) + sq(z));
  n = r / inc;
  incX = abs((x - xpos) / n);
  incY = abs((y - ypos) / n);
  incZ = abs((z - zpos) / n);
  xtemp = xpos;
  ytemp = ypos;
  ztemp = zpos;
  inPosX = false;
  inPosY = false;
  inPosZ = false;

  loopInc = round(n) + 1;

  speedJ1 = max_speed_j1/8;
  speedJ2 = max_speed_j2/8;
  speedJ3 = max_speed_j3/8;
  speedJ5 = max_speed_j5;

  int accelHelp = 0;
  int loopIncMax = loopInc;
  int decelerationJ1 = 0, decelerationJ2 = 0, decelerationJ3 = 0;
  int accelerationJ1 = 0, accelerationJ2 = 0, accelerationJ3 = 0;
  accelerationJ1 = 15;
  accelerationJ2 = 100;
  accelerationJ3 = 100;

  while (loopInc > 0 && digitalRead(pd1)) {
    //Inkrementacja pozycji o zmienną inc
    if (incX < abs(x - xtemp)) xtemp += incX * signX;
    else xtemp = x;

    if (incY < abs(y - ytemp)) ytemp += incY * signY;
    else ytemp = y;

    if (incZ < abs(z - ztemp)) ztemp += incZ * signZ;
    else ztemp = z;

    Th1Desired = TH1(xtemp, ytemp);
    Th2Desired = TH2(xtemp, ytemp, ztemp);  //+ Th2_0;
    Th3Desired = TH3(xtemp, ytemp, ztemp);  // + Th3_0;
    Th5Desired = TH5(Th2Desired, Th3Desired) + r5;

    stepCalPos1 = DegreesToSteps(stepper1_spr, Th1Desired);
    stepCalPos2 = -DegreesToSteps(stepper2_spr, Th2Desired - 90);
    stepCalPos3 = -DegreesToSteps(stepper3_spr, Th3Desired - 90);
    stepCalPos5 = DegreesToSteps(stepper5_spr, Th5Desired);

    stepper1.moveTo(stepCalPos1);
    stepper2.moveTo(stepCalPos2);
    stepper3.moveTo(stepCalPos3);
    stepper5.moveTo(stepCalPos5);

    if (loopInc > loopIncMax * 5 / 10) {
      if (speedJ1 < 1.5 * max_speed_j1) speedJ1 += accelerationJ1;
      if (speedJ2 < 6 * max_speed_j2) speedJ2 += accelerationJ2;
      if (speedJ3 < 6 * max_speed_j3) speedJ3 += accelerationJ3;
    }
    else if (loopInc + 1 > loopIncMax * 5 / 10) {
      decelerationJ1 = (speedJ1 - max_speed_j1/8)/(loopIncMax * 3 / 10);
      decelerationJ2 = (speedJ2 - max_speed_j2/8)/(loopIncMax * 3 / 10);
      decelerationJ3 = (speedJ3 - max_speed_j3/8)/(loopIncMax * 3 / 10);
    }
    else if (loopInc < loopIncMax * 3.25 / 10) {
      if (speedJ1 > max_speed_j1 / 8) speedJ1 -= decelerationJ1;
      if (speedJ2 > max_speed_j2 / 8) speedJ2 -= decelerationJ2;
      if (speedJ3 > max_speed_j3 / 8) speedJ3 -= decelerationJ3;
    }


    stepper1.setSpeed(speedJ1);
    stepper2.setSpeed(speedJ2);
    stepper3.setSpeed(speedJ3);
    stepper5.setSpeed(speedJ5);

    while (!inPos1 || !inPos2 || !inPos3 || !inPos5 && digitalRead(pd1)) {
      if (stepper1.currentPosition() != stepCalPos1) stepper1.runSpeedToPosition();
      else inPos1 = true;
      if (stepper2.currentPosition() != stepCalPos2) stepper2.runSpeedToPosition();
      else inPos2 = true;
      if (stepper3.currentPosition() != stepCalPos3) stepper3.runSpeedToPosition();
      else inPos3 = true;
      if (stepper5.currentPosition() != stepCalPos5) stepper5.runSpeedToPosition();
      else inPos5 = true;
    }
    inPos1 = inPos2 = inPos3 = inPos5 = false;
    loopInc--;
  }
}

//==============================================================
//Graphics parameters
//==============================================================
int marginup = 30;
int sip = 90;       //Status info place
int sbl = 15;       //Space between lines
int col = 76;       //Space between columns
int sbw = 25;       //space between words
bool anim = false;  //animation speed in milliseconds

//==============================================================
//Other
//==============================================================
unsigned long but13TimeBeg, but13TimeEnd;
unsigned long but12TimeBeg, but12TimeEnd;

byte data;

Servo gripper_servo;
//Mux
Mux mux(Pin(muxPin, INPUT, PinType::Digital), Pinset(S0, S1, S2, S3));


void setup() {
  pinMode(A15, INPUT);
  pinMode(A10, INPUT);
  //Ledy
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  digitalWrite(blue, HIGH);
  digitalWrite(red, HIGH);
  digitalWrite(green, HIGH);


  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_DC, OUTPUT);
  pinMode(pd1, INPUT);
  pinMode(pd2, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();
  LCD.LCD_Init(U2D_R2L);
  LCD.LCD_Clear(BLACK);
  Serial.begin(9600);
  stepper1.setMaxSpeed(100000);
  stepper2.setMaxSpeed(100000);
  stepper3.setMaxSpeed(100000);
  stepper4.setMaxSpeed(100000);
  stepper5.setMaxSpeed(100000);
  stepper6.setMaxSpeed(10000);
  stepper1.setAcceleration(1000);
  stepper2.setAcceleration(1000);
  stepper3.setAcceleration(1000);
  stepper4.setAcceleration(1000);
  stepper5.setAcceleration(1000);
  stepper6.setAcceleration(1000);
  gripper_servo.attach(6);
  stepperSetSpeed();
}

void loop() {
  if (!calibration) {
    calibration_and_home();
  } else {
    if (!init_disp_auto && !init_disp_teach) {
      if (digitalRead(pd2)) initialize_display_teach();
      if (!digitalRead(pd2)) initialize_display_auto();
    }
    //Dźwignia wyłączająca wszystkie sygnały wysyłane do robota
    if (motors_changed && digitalRead(pd1)) {
      LCD.LCD_DisplayString(sip, marginup, "ON ", &Font12, BLACK, GREEN);
      LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "None ", &Font12, BLACK, GREEN);
      motors_changed = false;

    } else if (!motors_changed && !digitalRead(pd1)) {
      LCD.LCD_DisplayString(sip, marginup, "OFF ", &Font12, BLACK, GREEN);
      LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "ESTOP", &Font12, BLACK, YELLOW);
      motors_changed = true;
    }
    //==============================================================
    //Teach mode
    //==============================================================
    if (digitalRead(pd1) && digitalRead(pd2)) {
      if (!init_disp_teach) initialize_display_teach();
      teachMode();
    }
    //==============================================================
    // Auto mode
    //==============================================================
    if (digitalRead(pd1) && !digitalRead(pd2)) {
      if (!init_disp_auto) initialize_display_auto();
      autoMode();
    }
  }
}

void stepperSetSpeed() {
  max_speed_j1 = 80 * speed;  //Prędkość od 200 do 2900kroków/s
  max_speed_j2 = 30 * speed - 150;
  max_speed_j3 = 45 * speed - 300;
  max_speed_j4 = 60 * speed - 300;
  max_speed_j5 = 80 * speed;
  max_speed_j6 = 6 * speed;
  stepper1.setSpeed(max_speed_j1);
  stepper2.setSpeed(max_speed_j2);
  stepper3.setSpeed(max_speed_j3);
  stepper4.setSpeed(max_speed_j4);
  stepper5.setSpeed(max_speed_j5);
  stepper6.setSpeed(max_speed_j6);
}

void home() {
  inPos1 = false;
  inPos2 = false;
  inPos3 = false;
  inPos4 = false;
  inPos5 = false;
  inPos6 = false;
  posChangeX = 0;
  posChangeY = 0;
  posChangeZ = 0;
  Rot5 = 0;
  int s1 = speed;
  speed = 100;
  stepperSetSpeed();

  moveToPosInc(x_home, y_home, z_home, 0, 0.05);
  stepper4.moveTo(0);
  stepper6.moveTo(0);
  stepper4.runToPosition();
  stepper6.runToPosition();
  speed = s1;
  stepperSetSpeed();
  updatePosition();
}

void calibration_and_home() {
  updatePosition();
  Rot5 = 0;
  posChangeX = posChangeY = posChangeZ = 0.0;
  is_calibrated = false;
  j1_calib = j2_calib = j3_calib = j4_calib = j5_calib = j6_calib = false;
  
  if(!calib_text) {
    LCD.LCD_Clear(BLACK);
    LCD.LCD_DisplayString(20, 45, "Press any", &Font16, BLACK, GBLUE);
    LCD.LCD_DisplayString(30, 65, " button", &Font16, BLACK, GBLUE);
    calib_text = true;    
  }
 


  for (byte i = 0; i < mux.channelCount(); i++) {
    data = mux.read(i);
    if (!data) {
      LCD.LCD_Clear(BLACK);
      LCD.LCD_DisplayString(5, 40, "Calibration...", &Font16, BLACK, WHITE);
      stepper1.setSpeed(40000);
      stepper2.setSpeed(-15000);
      stepper3.setSpeed(10000);
      stepper4.setSpeed(10000);
      stepper5.setSpeed(10000);
      stepper6.setSpeed(5000);
      calibration = true;
      while (!is_calibrated) {
        //Calibration process
        if (analogRead(sw_j6) < 900) {
          stepper6.runSpeed();
        } else j6_calib = true;
        if (analogRead(sw_j5_up) < 900) {
          stepper5.runSpeed();
        } else j5_calib = true;
        if (analogRead(sw_j4_acw) < 900) {
          stepper4.runSpeed();
        } else j4_calib = true;
        if (analogRead(sw_j3_front) < 900) {
          stepper3.runSpeed();
        } else j3_calib = true;
        if (analogRead(sw_j2_back) < 900) {
          stepper2.runSpeed();
        } else j2_calib = true;
        if (analogRead(sw_j1_front) < 900) {
          stepper1.runSpeed();
        } else j1_calib = true;

        ///////////////////////////////////////
        if (j6_calib && j5_calib && j4_calib && j3_calib && j2_calib && j1_calib) is_calibrated = true;
      }

      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);
      stepper5.setCurrentPosition(0);
      stepper6.setCurrentPosition(0);

      stepper1.setSpeed(40000);
      stepper2.setSpeed(15000);
      stepper3.setSpeed(10000);
      stepper4.setSpeed(10000);
      stepper5.setSpeed(10000);
      stepper6.setSpeed(5000);

      LCD.LCD_DisplayString(5, 40, "Homing...     ", &Font16, BLACK, WHITE);

      while (stepper1.run() || stepper2.run() || stepper3.run() || stepper4.run() || stepper5.run() || stepper6.run()) {
        stepper1.moveTo(home_pos[0]);
        stepper2.moveTo(home_pos[1]);
        stepper3.moveTo(home_pos[2]);
        stepper4.moveTo(home_pos[3]);
        stepper5.moveTo(home_pos[4]);
        stepper6.moveTo(home_pos[5]);
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
        stepper5.run();
        stepper6.run();
      }

      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);
      stepper5.setCurrentPosition(0);
      stepper6.setCurrentPosition(0);
    }
  }
  stepperSetSpeed();
  updatePosition();
}

void initialize_display_teach() {
  init_disp_teach = true;
  init_disp_auto = false;
  LCD.LCD_Clear(BLACK);
  //Lines
  for (int i = 0; i <= 128; i++) {
    LCD.LCD_DrawPoint(1, i, WHITE, DOT_PIXEL_1X1, DOT_FILL_AROUND);
  }
  for (int i = 0; i <= 128; i++) {
    LCD.LCD_DrawPoint(160, i, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);
  }

  LCD.LCD_DrawLine(160, 128, 160, 0, WHITE, LINE_SOLID, DOT_PIXEL_2X2);
  LCD.LCD_DrawLine(0, 2, 160, 2, WHITE, LINE_SOLID, DOT_PIXEL_1X1);

  //Permanently displayed text
  LCD.LCD_DisplayString(5, 5, "Teach Mode", &Font12, BLACK, CYAN);
  LCD.LCD_DrawLine(0, 22, 160, 22, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
  LCD.LCD_DrawLine(0, 25, 160, 25, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
  LCD.LCD_DrawLine(0, 128, 160, 128, WHITE, LINE_SOLID, DOT_PIXEL_1X1);

  LCD.LCD_DisplayString(5, marginup, "Motors:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(4, marginup + sbl, "Speed:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 2 * sbl, "Saved:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 3 * sbl, "InvKin:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 4 * sbl, "ObjectSize:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 5 * sbl, "Alerts:", &Font12, BLACK, RED);

  LCD.LCD_DisplayString(sip, marginup + 2 * sbl, "0", &Font12, BLACK, GREEN);
  LCD.LCD_DisplayString(sip, marginup + sbl, "   %", &Font12, BLACK, GREEN);
  LCD.LCD_DisplayNum(sip, marginup + sbl, speed, &Font12, BLACK, GREEN);
  LCD.LCD_DisplayString(sip, marginup + 4 * sbl, "   mm", &Font12, BLACK, GREEN);
  servo_item_sizeMM = servToMM(servo_item_size);
  if (servo_item_sizeMM == 0) LCD.LCD_DisplayString(sip, marginup + 4 * sbl, "0 ", &Font12, BLACK, GREEN);
  else LCD.LCD_DisplayNum(sip, marginup + 4 * sbl, servo_item_sizeMM, &Font12, BLACK, GREEN);

  if (!changingGripper) LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "None", &Font12, BLACK, GREEN);
  else LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "Grip+-", &Font12, BLACK, YELLOW);





  if (digitalRead(pd1)) LCD.LCD_DisplayString(sip, marginup, "ON ", &Font12, BLACK, GREEN);
  else LCD.LCD_DisplayString(sip, marginup, "OFF ", &Font12, BLACK, GREEN);

  if (inverse_kin) LCD.LCD_DisplayString(sip, marginup + 3 * sbl, "Yes", &Font12, BLACK, GREEN);
  else LCD.LCD_DisplayString(sip, marginup + 3 * sbl, "No ", &Font12, BLACK, GREEN);
}

void initialize_display_auto() {
  init_disp_teach = false;
  init_disp_auto = true;
  LCD.LCD_Clear(BLACK);
  //Lines
  for (int i = 0; i <= 128; i++) {
    LCD.LCD_DrawPoint(1, i, WHITE, DOT_PIXEL_1X1, DOT_FILL_AROUND);
  }
  for (int i = 0; i <= 128; i++) {
    LCD.LCD_DrawPoint(160, i, WHITE, DOT_PIXEL_2X2, DOT_FILL_AROUND);
  }

  LCD.LCD_DrawLine(160, 128, 160, 0, WHITE, LINE_SOLID, DOT_PIXEL_2X2);
  LCD.LCD_DrawLine(0, 2, 160, 2, WHITE, LINE_SOLID, DOT_PIXEL_1X1);

  //Permanently displayed text
  LCD.LCD_DisplayString(5, 5, "Auto Mode", &Font12, BLACK, CYAN);
  LCD.LCD_DrawLine(0, 22, 160, 22, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
  LCD.LCD_DrawLine(0, 25, 160, 25, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
  LCD.LCD_DrawLine(0, 128, 160, 128, WHITE, LINE_SOLID, DOT_PIXEL_1X1);


  LCD.LCD_DisplayString(5, marginup, "Motors:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(4, marginup + sbl, "Speed:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 2 * sbl, "Programs:", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 3 * sbl, "Vel -> J1+", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(sip - 10, marginup + 3 * sbl, "Tow0 -> J1-", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 4 * sbl, "Tow -> J2+", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(sip - 10, marginup + 4 * sbl, "InvT -> J2-", &Font12, BLACK, WHITE);
  LCD.LCD_DisplayString(5, marginup + 5 * sbl, "Alerts:", &Font12, BLACK, RED);

  LCD.LCD_DisplayString(sip, marginup + sbl, "   %", &Font12, BLACK, GREEN);
  LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "None", &Font12, BLACK, GREEN);
  LCD.LCD_DisplayNum(sip, marginup + sbl, speed, &Font12, BLACK, GREEN);


  if (digitalRead(pd1)) LCD.LCD_DisplayString(sip, marginup, "ON ", &Font12, BLACK, GREEN);
  else LCD.LCD_DisplayString(sip, marginup, "OFF ", &Font12, BLACK, GREEN);
}

void autoMode() {
  for (byte i = 0; i < mux.channelCount(); i++) {
    data = mux.read(i);
    //Program 1
    if (i == 0 && !data) {
      velTest();
    }
    //Program 2
    if (i == 1 && !data) {
      delay(300); home(); delay(300); gripper_servo.write(0);
      moveToPosInc(pkos[0], pkos[1], pkos[2], -90, 0.1); delay(1000);
      delay(300); home();
    }
    if (i == 2 && !data) {
      tower();
    }
    if (i == 3 && !data) {
      invTower();
    }
    //Program 3

    //Program 4

    //Przycisk odpowiedzialny za ustalanie prędkości
    if (i == 13 && !data && !speed_toggle) {
      speed += 10;
      if (speed > 100) {
        LCD.LCD_DisplayString(sip, marginup + sbl, "10 ", &Font12, BLACK, GREEN);
        speed = 10;
      }
      speed_toggle = true;
      stepperSetSpeed();
      LCD.LCD_DisplayNum(sip, marginup + sbl, speed, &Font12, BLACK, GREEN);
    } else if (i == 13 && data) {
      speed_toggle = false;
    }

    //Przycisk odpowiedzialny za kalibracje robota
    if (i == 14 && !data && !calib_toggle) {
      calib_toggle = true;
      calibration = false;
      calib_text = false;
      while (!calibration) calibration_and_home();
      initialize_display_auto();
    } else if (i == 15 && data) {
      calib_toggle = false;
    }

    //Przycisk odpowiedzialny za powrót robota do pozycji początkowej
    if (i == 15 && !data && !homing_toggle) {
      homing_toggle = true;
      home();
    } else if (i == 15 && data) {
      homing_toggle = false;
    }
  }
}

void teachMode() {
  if (!init_disp_teach) initialize_display_teach();
  //Multiplekserowanie przycisków
  for (byte i = 0; i < mux.channelCount(); i++) {
    data = mux.read(i) /* Reads from channel i (returns HIGH or LOW) */;

    //==============================================================
    //Joint 1 lub X
    //==============================================================

    if (inverse_kin) {
      if (i == 0 && !data && analogRead(sw_j2_front) < 900 && analogRead(sw_j5_down) < 900 && Th3 > 15) {
        posChangeX += 0.02;
        moveController(x_home + posChangeX, y_home + posChangeY, z_home + posChangeZ, Rot5);
      }

      if (i == 1 && !data && analogRead(sw_j2_back) < 900 && analogRead(sw_j5_down) < 900 && analogRead(sw_j3_front) < 900 && xpos > 0) {
        posChangeX -= 0.02;
        moveController(x_home + posChangeX, y_home + posChangeY, z_home + posChangeZ, Rot5);
      }
    } else {
      if (i == 0 && !data && !j1p) {
        stepper1.setSpeed(max_speed_j1);
        j1p = true;
      } else if (i == 0 && data) j1p = false;

      if (i == 1 && !data && !j1m) {
        stepper1.setSpeed(-max_speed_j1);
        j1m = true;
      } else if (i == 1 && data) j1m = false;

      if (j1p && analogRead(sw_j1_front) < 900) stepper1.runSpeed();
      if (j1m && analogRead(sw_j1_back) < 900) stepper1.runSpeed();
    }



    //==============================================================
    //Joint 2 lub Y
    //==============================================================

    if (inverse_kin) {
      if (i == 2 && !data && analogRead(sw_j5_down) < 900) {
        posChangeY += 0.02;
        moveController(x_home + posChangeX, y_home + posChangeY, z_home + posChangeZ, Rot5);
      }

      if (i == 3 && !data && analogRead(sw_j5_down) < 900) {
        posChangeY -= 0.02;
        moveController(x_home + posChangeX, y_home + posChangeY, z_home + posChangeZ, Rot5);
      }
    } else {
      if (i == 2 && !data && !j2p) {
        stepper2.setSpeed(max_speed_j2);
        j2p = true;
      } else if (i == 2 && data) j2p = false;

      if (i == 3 && !data && !j2m) {
        stepper2.setSpeed(-max_speed_j2);
        j2m = true;
      } else if (i == 3 && data) j2m = false;

      if (j2p && analogRead(sw_j2_front) < 900) stepper2.runSpeed();
      if (j2m && analogRead(sw_j2_back) < 900) stepper2.runSpeed();
    }

    //==============================================================
    //Joint 3 lub Z
    //==============================================================

    if (inverse_kin) {
      if (i == 4 && !data && analogRead(sw_j5_down) < 900 && Th3 > 15) {
        posChangeZ += 0.02;
        moveController(x_home + posChangeX, y_home + posChangeY, z_home + posChangeZ, Rot5);
      }

      if (i == 5 && !data && analogRead(sw_j5_up) < 900 && analogRead(sw_j3_front) < 900)   {
        posChangeZ -= 0.02;
        moveController(x_home + posChangeX, y_home + posChangeY, z_home + posChangeZ, Rot5);
      }
    } else {
      if (i == 4 && !data && !j3p) {
        stepper3.setSpeed(max_speed_j3);
        j3p = true;
      } else if (i == 4 && data) j3p = false;

      if (i == 5 && !data && !j3m) {
        stepper3.setSpeed(-max_speed_j3);
        j3m = true;
      } else if (i == 5 && data) j3m = false;

      if (j3p && analogRead(sw_j3_front) < 900) stepper3.runSpeed();
      if (j3m && analogRead(sw_j3_back) < 900) stepper3.runSpeed();
    }


    //==============================================================
    //Joint 4
    //==============================================================

    if (!inverse_kin) {
      if (i == 6 && !data && !j4p) j4p = true;
      else if (i == 6 && data) j4p = false;

      if (i == 7 && !data && !j4m) j4m = true;
      else if (i == 7 && data) j4m = false;

      if (j4p && analogRead(sw_j4_acw) < 900) {
        stepper4.setSpeed(max_speed_j4);
        stepper4.runSpeed();
      }
      if (j4m && analogRead(sw_j4_cw) < 900) {
        stepper4.setSpeed(-max_speed_j4);
        stepper4.runSpeed();
      }
    }
    else {
      if (i == 6 && !data) {
        moveToPosInc(xpos, ypos, zpos, -90, 0.1);
        Rot5 = DegreesToSteps(stepper5_spr,-90);
      }
      if (i == 7 && !data) {
        moveToPosInc(xpos, ypos, zpos, 0, 0.1);
        Rot5 = DegreesToSteps(stepper5_spr,-90);
      }
    }

    //==============================================================
    //Joint 5
    //==============================================================

    if (i == 8 && !data && !j5p) j5p = true;
    else if (i == 8 && data) j5p = false;

    if (i == 9 && !data && !j5m) j5m = true;
    else if (i == 9 && data) j5m = false;

    if (j5p && analogRead(sw_j5_up) < 900) {
      stepper5.setSpeed(max_speed_j5);
      stepper5.runSpeed();
      Rot5++;
      delayMicroseconds(300);
    }
    if (j5m && analogRead(sw_j5_down) < 900) {
      stepper5.setSpeed(-max_speed_j5);
      stepper5.runSpeed();
      Rot5--;
      delayMicroseconds(300);
    }
    //==============================================================
    //Joint 6
    //==============================================================

    if (!inverse_kin) {
      if (i == 10 && !data && !j6p) {
        stepper6.setSpeed(max_speed_j6);
        j6p = true;
      } else if (i == 10 && data) j6p = false;

      if (i == 11 && !data && !j6m) {
        stepper6.setSpeed(-max_speed_j6);
        j6m = true;
      } else if (i == 11 && data) j6m = false;

      if (j6p && analogRead(sw_j6) < 900) stepper6.runSpeed();
      if (j6m && stepper6.currentPosition() > -1200) stepper6.runSpeed();
    }


    //==============================================================
    //==============================================================

    //Przycisk odpowiedzialny za ustalanie prędkości
    if (i == 13 && !data && !speed_toggle) {
      //==============================================================
      //Zmienna czasu żeby uruchomić inną funkcje
      but13TimeBeg = millis();
      ///==============================================================
      speed += 10;
      if (speed > 100) {
        LCD.LCD_DisplayString(sip, marginup + sbl, "10 ", &Font12, BLACK, GREEN);
        speed = 10;
      }
      speed_toggle = true;
      stepperSetSpeed();
      LCD.LCD_DisplayNum(sip, marginup + sbl, speed, &Font12, BLACK, GREEN);
    } else if (i == 13 && data) {
      speed_toggle = false;
    }

    //Kombinacja przycisków zmieniająca kinematyke robota
    if (i == 13 && !data) {
      but13TimeEnd = millis();
      if (but13TimeEnd - but13TimeBeg >= 2000.0) {
        LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "ToHome", &Font12, BLACK, YELLOW);
        delay(300);
        home();
        inverse_kin = !inverse_kin;
        if (inverse_kin) LCD.LCD_DisplayString(sip, marginup + 3 * sbl, "Yes", &Font12, BLACK, GREEN);
        else LCD.LCD_DisplayString(sip, marginup + 3 * sbl, "No ", &Font12, BLACK, GREEN);
        LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "none  ", &Font12, BLACK, GREEN);
        delay(300);
        but13TimeEnd = but13TimeBeg = 0.0;
      }
    }



    //Przycisk odpowiedzialny za zamykanie i otwieranie chwytaka
    if (i == 12 && !data && !gripper_toggle) {
      //==============================================================
      //Zmienna czasu żeby uruchomić inną funkcje
      but12TimeBeg = millis();
      ///==============================================================
      if (!changingGripper) {
        gripper_toggle = true;
        if (analogRead(sw_grip) > 900) {
          gripper_servo.write(servo_item_size);
          gripper_status = true;
        }
        if (analogRead(sw_grip) < 900) {
          gripper_servo.write(0);
          gripper_status = false;
        }
      } else {
        gripper_toggle = true;
        servo_item_size += 6;
        if (servo_item_size > 120) servo_item_size = 0;
        servo_item_sizeMM = servToMM(servo_item_size);
        if (servo_item_sizeMM == 0) LCD.LCD_DisplayString(sip, marginup + 4 * sbl, "0 ", &Font12, BLACK, GREEN);
        else {
          LCD.LCD_DisplayString(sip, marginup + 4 * sbl, "  ", &Font12, BLACK, GREEN);
          LCD.LCD_DisplayNum(sip, marginup + 4 * sbl, servo_item_sizeMM, &Font12, BLACK, GREEN);
        }
      }
    } else if (i == 12 && data) {
      gripper_toggle = false;
      changingGripper_toggle = false;
    }
    //Kombinacja przycisków zmieniająca ustawienie chwytaka
    if (i == 12 && !data) {
      but12TimeEnd = millis();
      if (but12TimeEnd - but12TimeBeg >= 2000.0 && !changingGripper_toggle) {
        changingGripper_toggle = true;
        changingGripper = !changingGripper;
        Serial.println(changingGripper);
        if (changingGripper) LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "Grip+-", &Font12, BLACK, YELLOW);
        else LCD.LCD_DisplayString(sip, marginup + 5 * sbl, "None  ", &Font12, BLACK, GREEN);
        but12TimeEnd = but12TimeBeg = 0.0;
      }
    }

    //Przycisk odpowiedzialny za zapisywanie pozycji

    if (i == 14 && !data && !save_pos) {
      save_pos = true;
      pos_count++;
      if (pos_count > 10) pos_count = 1;

      //Pozycja
      Th1_rad = degToRad(stepper1.currentPosition() * 360 / stepper1_spr);
      Th2_rad = degToRad(stepper2.currentPosition() * 360 / stepper2_spr + 90);
      Th3_rad = degToRad(stepper3.currentPosition() * 360 / stepper3_spr + 90);
      saved_position[pos_count - 1][0] = -cos(Th1_rad) * (L3 * cos(Th2_rad + Th3_rad) + L2 * cos(Th2_rad));
      saved_position[pos_count - 1][1] = -sin(Th1_rad) * (L3 * cos(Th2_rad + Th3_rad) + L2 * cos(Th2_rad));
      saved_position[pos_count - 1][2] = L1 + L3 * sin(Th2_rad + Th3_rad) + L2 * sin(Th2_rad);
      //saved_position[pos_count-1][3] = StepsToDegrees(stepper4_spr,stepper4.currentPosition());
      saved_position[pos_count - 1][3] = StepsToDegrees(stepper5_spr, Rot5);
      //saved_position[pos_count-1][5] = StepsToDegrees(stepper6_spr,stepper6.currentPosition());

      Serial.print("New saved position: ");
      Serial.print(pos_count);
      Serial.print(" || ");
      Serial.print("X: ");
      Serial.print(saved_position[pos_count - 1][0]);
      Serial.print(" Y: ");
      Serial.print(saved_position[pos_count - 1][1]);
      Serial.print(" Z: ");
      Serial.println(saved_position[pos_count - 1][2]);
      Serial.print("Rot5 ");
      Serial.print(saved_position[pos_count - 1][3]);

      Serial.println("Rotacje");
      Serial.print("1: ");
      Serial.print(stepper1.currentPosition());
      Serial.print("  2: ");
      Serial.print(stepper2.currentPosition());
      Serial.print("3: ");
      Serial.print(stepper3.currentPosition());
      Serial.print("  4: ");
      Serial.print(stepper4.currentPosition());
      Serial.print("5: ");
      Serial.print(stepper5.currentPosition());
      Serial.print("  6: ");
      Serial.print(stepper6.currentPosition());
    } else if (i == 14 && data) {
      save_pos = false;
    }

    //Przycisk odpowiedzialny za powrót robota do pozycji początkowej
    if (i == 15 && !data && !homing_toggle) {
      homing_toggle = true;
      home();
    } else if (i == 15 && data) {
      homing_toggle = false;
    }
  }
}

void velTest() {
  j = speed/900.0;
  delay(300); home(); delay(300);
  moveToPosInc(pv1[0], pv1[1], pv1[2], 0, j/2);
  moveToPosInc(pv2[0], pv2[1], pv2[2], 0, j/4);
  moveToPosInc(pv3[0], pv3[1], pv3[2], 0, j);
  moveToPosInc(pv6[0], pv6[1], pv6[2], 0, j);
  moveToPosInc(pv5[0], pv5[1], pv5[2], 0, j/1.5);
  home(); delay(300);
}

void tower() {
  j = speed/1000.0;
  delay(300); home(); delay(300); gripper_servo.write(0);
  //1 kostka
  moveToPosInc(pkos[0], pkos[1], pkos[2] + 5.0, -90, j); 
  moveToPosInc(pkos[0], pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0], pkos[1], pkos[2] + 5.0, -90, j);

  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 3.0, -90, j); 
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 0.1, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 3.0, -90, j);

  //2 kostka
  moveToPosInc(pkos[0] + 3.0, pkos[1], pkos[2] + 5.0, -90, j); 
  moveToPosInc(pkos[0] + 3.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1], pkos[2] + 5.0, -90, j);

  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 2.0, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 5.0, -90, j);

  //3 kostka
  moveToPosInc(pkos[0] + 6.0, pkos[1], pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 6.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 6.0, pkos[1], pkos[2] + 5.0, -90, j);

  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 7.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 3.7, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 7.0, -90, j);

  //4 kostka
  moveToPosInc(pkos[0] + 9.0, pkos[1], pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 9.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 9.0, pkos[1], pkos[2] + 5.0, -90, j);

  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 9.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 5.5, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 9.0, -90, j);
  
  //5 kostka
  moveToPosInc(pkos[0] + 12.0, pkos[1], pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 12.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 12.0, pkos[1], pkos[2] + 5.0, -90, j);

  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 11.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 7.3, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 11.0, -90, j);


  delay(300); home(); delay(300);
}

void invTower() {
  j = speed/1000.0;
  delay(300); home(); delay(300); gripper_servo.write(0);
  //5 kostka
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 11.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 7.3, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 11.0, -90, j);

  moveToPosInc(pkos[0] + 12.0, pkos[1], pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 12.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 12.0, pkos[1], pkos[2] + 5.0, -90, j);

  //4 kostka
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 9.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 5.5, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 9.0, -90, j);

  moveToPosInc(pkos[0] + 9.0, pkos[1], pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 9.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 9.0, pkos[1], pkos[2] + 5.0, -90, j);


  //3 kostka
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 7.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 3.7, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 7.0, -90, j);

  moveToPosInc(pkos[0] + 6.0, pkos[1], pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 6.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 6.0, pkos[1], pkos[2] + 5.0, -90, j);

  //2 kostka
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 5.0, -90, j);
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 1.9, -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 5.0, -90, j);

  moveToPosInc(pkos[0] + 3.0, pkos[1], pkos[2] + 5.0, -90, j); 
  moveToPosInc(pkos[0] + 3.0, pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1], pkos[2] + 5.0, -90, j);

  //1 kostka
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 3.0, -90, j); 
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(75); delay(700);}
  moveToPosInc(pkos[0] + 3.0, pkos[1] + 7.0, pkos[2] + 3.0, -90, j);

  moveToPosInc(pkos[0], pkos[1], pkos[2] + 5.0, -90, j); 
  moveToPosInc(pkos[0], pkos[1], pkos[2], -90, j);
  if(digitalRead(pd1)) {delay(300); gripper_servo.write(0); delay(700);}
  moveToPosInc(pkos[0], pkos[1], pkos[2] + 5.0, -90, j);

  delay(300); home(); delay(300);
}
