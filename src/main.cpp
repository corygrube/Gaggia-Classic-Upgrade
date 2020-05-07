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
double g_BoilerTemp;                     // Boiler water temperature (DegF)
double g_BoilerCmd;                      // Boiler heater SSR PWM command (0-255)
double g_BoilerSp;                       // Boiler water temperature setpoint (DegF)
bool g_BoilerTempFault;                  // Boiler water temperature fault (1=Fault)
bool g_SteamMode;                        // Machine Steam Mode (0=espresso mode, 1=steam mode)
int g_SteamModeRawPrev;                  // value of Steam Mode pin on previous scan (1=steam mode)
unsigned long g_SteamModeDbPrev;         // Previous time Steam Mode pin was toggled (ms)
unsigned long g_SteamModeDbCfg = 50;     // Configured debounce time for toggle (ms)

// Adafruit MAX31865 instance (RTD sensor board) (CS, SDI, SDO, CLK)
Adafruit_MAX31865 g_BoilerRtd = Adafruit_MAX31865(pin_CS, pin_SDI, pin_SDO, pin_CLK);     // Boiler RTD MAX31865

// PID Instance (Input, Output, Setpoint, kP, kI, kD, POn Direction)
PID g_BoilerPid = PID(&g_BoilerTemp, &g_BoilerCmd, &g_BoilerSp, 1, 1, 0, P_ON_M, DIRECT); // Boiler Temp PID

/* Function Declarations */
void boilerTempInp(), steamModeInp(), boilerTempControl();


/* Setup */
void setup() {
  Serial.begin(9600);
  g_BoilerRtd.begin(MAX31865_3WIRE);  //RTD initialization
  
  // Pin configurations
  pinMode(pin_SteamMode, INPUT);
  pinMode(pin_BoilerCmd, OUTPUT);
}


/* Main Loop */
void loop() {
  // Inputs/Reads
  boilerTempInp();
  steamModeInp();
  

  // Processing/Calculations
  boilerTempControl();

  // Monitoring
  int boilerCmdPct = g_BoilerCmd / 255 * 100;   // Scaling from PWM 0-255 to 0-100% for Diagnostics

  // Outputs/Writes
  analogWrite(pin_BoilerCmd, g_BoilerCmd);
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
    g_BoilerTempFault = 1;                                                // Set global temp fault if one exists
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
  else g_BoilerTempFault = 0;                                             // Clear global temp fault if none exists
  Serial.println();
}


void steamModeInp() {
// read the state of the switch into a local variable:
  int steamModeRaw = digitalRead(pin_SteamMode);
  
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (steamModeRaw != g_SteamModeRawPrev) {
    // reset the debouncing timer
    g_SteamModeDbPrev = millis();
  }

    if ((millis() - g_SteamModeDbPrev) > g_SteamModeDbCfg) {
    // whatever the steamModeRaw is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the mode has changed:
      if (steamModeRaw != g_SteamMode) {
        // set the debounced steam mode to the raw value
        g_SteamMode = steamModeRaw;
      }
  }

    // save the steamModeRaw. Next time through the loop, it'll be the g_SteamModeRawPrev:
  g_SteamModeRawPrev = steamModeRaw;

}

void boilerTempControl() {
// if (fdsa) 


g_BoilerPid.Compute();
}