#include <RelayRace.h>
#include <BRCClient.h>

#include <HMC5883L.h>

#include <NewPing.h>
#include <Wire.h>

#include <SPI.h>
#include <RFID.h>

#define MAX_DISTANCE 35

/* If you are using UNO, uncomment the next line. */
// #define UNO
/* If you are using MEGA and want to use HardwareSerial,
   umcomment the next 2 lines. */
#define USE_HARDWARE_SERIAL
#define HW_SERIAL Serial1

#ifdef UNO
#define UART_RX 3
#define UART_TX 2
#else
#define UART_RX 10
#define UART_TX 2
#endif

BRCClient brcClient(&HW_SERIAL);

#define AP_SSID    "HMKRL"
#define AP_PASSWD  "hatsune39"
#define TCP_IP     "192.168.43.1"

#define TCP_PORT   5000
#define MY_COMM_ID 0x37

RelayRace race(&brcClient, MY_COMM_ID);

/* The pin mapping of SPI */
#ifdef UNO
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCLK 13
#else
#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52
#endif

// SPI_SS pin can be chosen by yourself
// becasue we use SPI in master mode.
#define SPI_SS 53
#define MFRC522_RSTPD 47

RFID rfid(SPI_SS, MFRC522_RSTPD);

//set up motor pin
const int LeftMotorF = 2;
const int LeftMotorB = 3;
const int RightMotorF = 4;
const int RightMotorB = 5;

//set up sonic sansors' pins
const int sonar_trigger_F = 24;
const int sonar_echo_F = 26;
const int sonar_trigger_L = 23;
const int sonar_echo_L = 22;
const int sonar_trigger_R = 28;
const int sonar_echo_R = 30;

//set up sonic sansors
NewPing sonarF(sonar_trigger_F, sonar_echo_F, MAX_DISTANCE);
NewPing sonarL(sonar_trigger_L, sonar_echo_L, MAX_DISTANCE);
NewPing sonarR(sonar_trigger_R, sonar_echo_R, MAX_DISTANCE);

//set up electronic compass
HMC5883L compass;
float curDegree;

//set up variable to store distance
int disL, disR, disF;
//set up variables
int lw, rw, count;

//for RFID
static uint8_t status;
static uint16_t card_type;
static uint8_t sn[4], snBytes;

int countT;

void setup()  {
  // put your setup code here, to run once:
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000L, MSBFIRST, SPI_MODE3));
  rfid.begin();

  Serial.begin(9600);
  //while (!Serial);

  pinMode(LeftMotorF, OUTPUT);
  pinMode(LeftMotorB, OUTPUT);
  pinMode(RightMotorF, OUTPUT);
  pinMode(RightMotorB, OUTPUT);
  Wire.begin();
  compass = HMC5883L();
  while (!compass.begin()) delay(500);

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);

  compass.setOffset(11, -385);
  delay(1000);
  curDegree = getDegree();
  Serial.println(curDegree);
  lw = 100;
  rw = 116;
  
  brcClient.begin(9600);
  brcClient.beginBRCClient(AP_SSID, AP_PASSWD, TCP_IP, TCP_PORT);

  while (!race.registerID()) {
    delay(1000);
  }
  race.waitLaunchSignal();
//  race.askRFID(sn,true);
//  while(1);

  countT = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  // for dead
  if(race.dead()) {
    Motorstop(20);
    while(1);
  }
  
  int error = getError();
  if (error > 15) rectify();
  Serial.println(getDegree());

  detectDis();

  if (disF && disF < 10) {
    if (disL == 0 && disR) {
      turn(1);
      Motorstop(100);
      forward(lw, rw);
      delay(370);
    } else if (disR == 0 && disL) {
      turn(-1);
      Motorstop(100);
      forward(lw, rw);
      delay(370);
    } else {
      Motorstop(50);
      delay(100);

      if ((status = rfid.findTag(&card_type)) == STATUS_OK) {
        if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) {
          rfid.piccHalt();
          if (race.askRFID(sn)) {
            while (1);
          } else {
            turn(1);
            delay(30);
            turn(1);
          }
        }
      } else {
        backward(lw, rw);
        delay(30);
      }
    }
  } else {
    if (disR == 0) {
      if(countT == 1 || countT == 2) {
        forward(lw,rw);
        delay(370);
        countT++;
      }else{
        delay(100);
        turn(-1);
        Motorstop(50);
        forward(lw, rw);
        delay(370);
        countT++;
      }
    } else {
      if(disL == 0 && countT == 3) {
        forward(lw,rw);
        delay(370);
        countT++;
      }else if(disL == 0 && countT == 4) {
        delay(100);
        turn(1);
        forward(lw,rw);
        delay(370);
        countT++;
      }else{
        forward(lw, rw);
        delay(50);
      }
      //Motorstop(20);
    }
  }
  delay(30);
}
void Motorstop(int time) {
  analogWrite(LeftMotorF, 0);
  analogWrite(LeftMotorB, 0);
  analogWrite(RightMotorF, 0);
  analogWrite(RightMotorB, 0);

  delay(time);
}

void forward(int lw, int rw) {
  analogWrite(LeftMotorF, lw);
  analogWrite(LeftMotorB, 0);
  analogWrite(RightMotorF, rw);
  analogWrite(RightMotorB, 0);
  lw = 100;
  rw = 116;  
  if(disL < disR) lw += 10;
  else if(disL > disR)  rw += 10;
}

void turnleft(int lw, int rw) {
  analogWrite(LeftMotorF, 0);
  analogWrite(LeftMotorB, lw);
  analogWrite(RightMotorF, rw);
  analogWrite(RightMotorB, 0);
}

void turnright(int lw, int rw) {
  analogWrite(LeftMotorF, lw);
  analogWrite(LeftMotorB, 0);
  analogWrite(RightMotorF, 0);
  analogWrite(RightMotorB, rw);
}

void backward(int lw, int rw) {
  analogWrite(LeftMotorF, 0);
  analogWrite(LeftMotorB, lw);
  analogWrite(RightMotorF, 0);
  analogWrite(RightMotorB, rw);
}

//to detect distance
void detectDis() {
  disF = sonarF.ping_cm();
  disL = sonarL.ping_cm();
  disR = sonarR.ping_cm();
  Serial.print(disF);
  Serial.print(" ");
  Serial.print(disL);
  Serial.print(" ");
  Serial.println(disR);
}

float getDegree() {
  Vector scaled = compass.readNormalize();
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  return heading * 180 / M_PI;
}

void turn(int dir) {
  //left = 1, right = -1
  float finalDegree;
  if(dir > 0) {
    finalDegree = curDegree + 90;
  }else {
    finalDegree = curDegree - 88;
  }

  float temp = getDegree();
  if (finalDegree < 0) finalDegree += 360;
  if (finalDegree > 360) finalDegree -= 360;

  while (abs(temp - finalDegree) >= 5) {
    temp = getDegree();
    Serial.println(abs(temp - finalDegree));
    if (dir < 0) {
      turnright(90, 90);
    } else {
      turnleft(90, 90);
    }
    delay(50);
    Motorstop(70);
  }
  Motorstop(100);
  if(dir>0) curDegree += 90;
  else curDegree -= 90;
  if (curDegree < 0) curDegree += 360;
  if (curDegree >= 360) curDegree -= 360;
  delay(100);
  rectify();
  lw = 100;
  rw = 116;
}

int getError() {
  int error = abs(getDegree() - curDegree);
  if (error > 180) error = abs(error - 360);

  return error;
}

void rectify() {
  //left = 1, right = -1
  int temp = getDegree() - curDegree;
  int dir;

  if (temp < -180) dir = -1;
  else if (temp > 180) dir = 1;
  else if (temp > 0) dir = -1;
  else dir = 1;

  temp = getDegree();
  while (1) {
    temp = getDegree();
    if (dir < 0) {
      turnright(lw, rw);
    } else {
      turnleft(lw, rw);
    }
    delay(50);
    Motorstop(70);
    if (abs(temp - curDegree) <= 5 || abs(temp - curDegree) >= 355) break;
  }
}

