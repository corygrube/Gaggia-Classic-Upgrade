#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>


/* Pins */
int pin_SteamMode = 1;      // Steam mode input pin
int pin_BoilerCmd = 9;      // Boiler heater SSR pin (PWM)
int pin_CS = 10;            // MAX31865 SPI CS pin
int pin_SDI = 11;           // MAX31865 SPI SDI pin
int pin_SDO = 12;           // MAX31865 SPI SDO pin
int pin_CLK = 13;           // MAX31865 SPI CLK pin


/* Global Variables */
// Inputs
float g_BoilerTemp;   // Boiler water temperature (DegF)

// Outputs
float g_BoilerCmd;    // Boiler heater SSR command (0-100%)

// Adafruit MAX31865 (RTD sensor board)
Adafruit_MAX31865 g_BoilerRtd = Adafruit_MAX31865(pin_CS, pin_SDI, pin_SDO, pin_CLK);     // Boiler RTD MAX31865 instance


/* Function Declarations */
void boilerTempInp();
void boilerTempPID();


/* Setup */
void setup() {
  Serial.begin(115200);
  g_BoilerRtd.begin(MAX31865_3WIRE);  //RTD initialization
}


/* Main Loop */
void loop() {
  /* Inputs/Reads */
  boilerTempInp();

  /* Processing/Calculations */
  // boilerTempPid();

  /* Outputs/Writes */
  int boilerCmdRaw = g_BoilerCmd / 100 * 255;   // Scaling from 0-100% to 0-255 for PWM
  analogWrite(pin_BoilerCmd, pin_BoilerCmd);
}


/* Function Definitions */
void boilerTempInp() {
  // Boiler temperature read
  int rNominal = 100;                                                     // 0 DegC resistance of RTD (100 for PT100)
  int rRef = 430;                                                         // Resistance of Rref resistor on board (430 for PT100 board)
  g_BoilerTemp = g_BoilerRtd.temperature(rNominal, rRef) * 1.8 + 32;      // Temperature measurement, converted to DegF

  //Boiler temperature fault check
  byte fault = g_BoilerRtd.readFault();                                   // Check MAX board for fault codes
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
    g_BoilerRtd.clearFault();
  }
  Serial.println();
}


void boilerTempPid() {

}