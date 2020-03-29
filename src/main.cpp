#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <AutoPID.h>


/* Digital Pins */
int steamPin = 1;       // Steam mode input
int boilerCmdPin = 9;   // Boiler heater SSR (PWM)
int CS = 10;            // MAX31865 SPI CS
int SDI = 11;           // MAX31865 SPI SDI
int SDO = 12;           // MAX31865 SPI SDO
int CLK = 13;           // MAX31865 SPI CLK

/* Analog Pins */
// N/A

/* Global Variables */
// Inputs
float boilerTemp;   // Boiler water temperature (DegF)
float resLevel;     // Reservoir water level (% Full)
// Outputs
float boilerCmd;    // Boiler heater SSR command (0-100%)
// MAX31865 (Boiler RTD board)
Adafruit_MAX31865 boilerRtd = Adafruit_MAX31865(CS, SDI, SDO, CLK);     // Configures SPI (CS, SDI, SDO, CLK)

void setup() {
  Serial.begin(115200);
  boilerRtd.begin(MAX31865_3WIRE);  //MAX31865 initialization
}


void loop() {
  /* Inputs/Reads */
  boilerTempInp();

  /* Processing/Calculations */
  // boilerTempPid();

  /* Outputs/Writes */
  int boilerCmdRaw = boilerCmd / 100 * 255;   // Scaling from 0-100% to 0-255 for PWM
  analogWrite(boilerCmdPin, boilerCmdRaw);

}


void boilerTempInp() {
  // Boiler temperature read
  int rNominal = 100;                                                     // 0 DegC resistance of RTD (100 for PT100)
  int rRef = 430;                                                         // Resistance of Rref resistor on board (430 for PT100 board)
  boilerTemp = boilerRtd.temperature(rNominal, rRef) * 1.8 + 32;          // Temperature measurement, converted to DegF

  //Boiler temperature fault check
  byte fault = boilerRtd.readFault();                                     // Check MAX board for fault codes
  if (fault) {                                                            // If fault is present, print fault code and description of fault
    Serial.print("RTD Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    boilerRtd.clearFault();
  }
  Serial.println();
}


void boilerTempPid() {

}