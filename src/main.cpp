#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>


/* Pins */
const int pin_SteamMode = 2;      // Steam mode input pin
const int pin_LightCmd = 3;       // Indicator light SSR pin (PWM)
const int pin_BoilerCmd = 9;      // Boiler heater SSR pin (PWM)
const int pin_CS = 10;            // MAX31865 SPI CS pin
const int pin_SDI = 11;           // MAX31865 SPI SDI pin
const int pin_SDO = 12;           // MAX31865 SPI SDO pin
const int pin_CLK = 13;           // MAX31865 SPI CLK pin


/* Global Variables */
double g_BoilerTemp;                     // Boiler water temperature (DegF)
bool g_BoilerTempFault;                  // Boiler water temperature fault (1=Fault)

double g_BoilerCmd = 0;                  // Boiler heater SSR PWM command (0-255)
double g_BoilerSp = 0;                   // Boiler water temperature setpoint (DegF)
double g_BoilerSpEsp = 205;              // Boiler water temperature setpoint, espresso (DegF)
double g_BoilerSpSteam = 280;            // Boiler water temperature setpoint, steam (DegF)
double g_BoilerKpEsp = 2;                // Boiler water temperature PID P gain, espresso
double g_BoilerKiEsp = 1;                // Boiler water temperature PID I gain, espresso
double g_BoilerKdEsp = 0;                // Boiler water temperature PID D gain, espresso
double g_BoilerKpSteam = 2;              // Boiler water temperature PID P gain, steam
double g_BoilerKiSteam = 1;              // Boiler water temperature PID I gain, steam
double g_BoilerKdSteam = 0;              // Boiler water temperature PID D gain, steam

bool g_SteamMode;                        // Machine Steam Mode (0=espresso mode, 1=steam mode)
bool g_SteamModeOSR;                     // Machine Steam Mode Rising Oneshot
bool g_SteamModeOSF;                     // Machine Steam Mode Falling Oneshot
int g_SteamModeRawPrev;                  // value of Steam Mode pin on previous scan (1=steam mode)
unsigned long g_SteamModeDbPrev;         // Previous time Steam Mode pin was toggled (ms)
unsigned long g_SteamModeDbCfg = 50;     // Configured debounce time for toggle (ms)

int g_LightCmd = 0;                      // Indicator light SSR PWM command (0-255)
int g_LightMode = 0;                     // Indicator light mode (0=Fault, 1=Heating, 2=Ready, 3=Too hot)

// Adafruit MAX31865 instance (RTD sensor board) (CS, SDI, SDO, CLK)
Adafruit_MAX31865 g_BoilerRtd = Adafruit_MAX31865(pin_CS, pin_SDI, pin_SDO, pin_CLK);     // Boiler RTD MAX31865

// PID Instance (Input, Output, Setpoint, kP, kI, kD, POn Direction)
PID g_BoilerPid = PID(&g_BoilerTemp, &g_BoilerCmd, &g_BoilerSp, 1, 1, 0, P_ON_M, DIRECT); // Boiler Temp PID


/* setup()*********************************************************************
 * Enables Serial.
 * Initializes boiler RTD instance (starts SPI I think).
 * Sets IO pin modes.
 ******************************************************************************/
void setup() {
  Serial.begin(9600);
  g_BoilerRtd.begin(MAX31865_3WIRE);  //RTD initialization
  
  // Pin configurations
  pinMode(pin_SteamMode, INPUT);
  pinMode(pin_LightCmd, OUTPUT);
  pinMode(pin_BoilerCmd, OUTPUT);
}


/* timing()*********************************************************************
 * Tracks time for miscellaneous oneshots.
 * Loops 1000ms timer for machine light logic (see lightControl() ).
 ******************************************************************************/
void timing() {
  
}


/* boilerTempInp()********************************************************************
 * Configures and reads RTD for boiler temperature. 
 * Fault checks RTD sensor, prints faults to serial, and sets global bit accordingly.
 *************************************************************************************/
void boilerTempInp() {
  // Variables needed for RTD read
  int rNominal = 100;  // 0 DegC resistance of RTD (100 for PT100)
  int rRef = 430;      // Resistance of Rref resistor on board (430 for PT100 board)
  g_BoilerTemp = g_BoilerRtd.temperature(rNominal, rRef) * 1.8 + 32;  // Temperature measurement, converted to DegF

  //Boiler temperature fault check
  byte fault = g_BoilerRtd.readFault();

  // If fault is present, determine exact fault and set global bit
  if (fault) {
    // Set global temp fault
    g_BoilerTempFault = 1;

    // Print fault codes
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
  
  Serial.println();
  }

  // Clear global temp fault if none exists
  else g_BoilerTempFault = 0;
}


/* steamModeInp()*************************************************************
 * Debounce function to read the state of the steam mode switch.
 * Sets miscellaneous oneshot bits for use elsewhere in program.
 ******************************************************************************/
void steamModeInp() {
  // Reset Steam Mode oneshot bits
  g_SteamModeOSR = 0;
  g_SteamModeOSF = 0;
  
  // Read the state of the switch into a local variable:
  int steamModeRaw = digitalRead(pin_SteamMode);
  
  // Check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (steamModeRaw != g_SteamModeRawPrev) {
    // Mark the current time as the the most recent switch change.
    g_SteamModeDbPrev = millis();
  }

  // Check if steamModeRaw is different than g_SteamMode.
  // If so, check to see if debounce timer setpoint has been exceeded, meaning
  // that switch has been in the current state for more ms than the Debounce SP
  if (steamModeRaw != g_SteamMode) {
    if ((millis() - g_SteamModeDbPrev) > g_SteamModeDbCfg) {
      g_SteamMode = steamModeRaw;
      
      // Set Oneshot rising/falling bits based on how the debounced value changed
      if (g_SteamMode == 1) {
        g_SteamModeOSR = 1;
      }
      else {
        g_SteamModeOSF = 1;
      }
    }
  }

  // Save the steamModeRaw as g_SteamModeRawPrev for next scan. 
  // This will be used to determine when the switch changes state.
  g_SteamModeRawPrev = steamModeRaw;
}


/* boilerTempControl()***************************************************
 * Sets PID Temp SP based on boiler mode oneshot.
 * Sets PID mode based on overall system health (fault/no fault).
 * Executes PID.
 **************************************************************************/
void boilerTempControl() {
  // If a fault exists, place PID into manual (0) and set output to 0
  if (g_BoilerTempFault) {
    g_BoilerPid.SetMode(0);
    g_BoilerCmd = 0;
  }
  
  // If there are no faults, run normal PID logic.
  else  {
    // Place PID into Auto (1)
    g_BoilerPid.SetMode(1);

    // Steam Mode rising oneshot. Use steam SP/tuning params.
    if (g_SteamModeOSR) {
      g_BoilerSp = g_BoilerSpSteam;
      g_BoilerPid.SetTunings(g_BoilerKpSteam, g_BoilerKiSteam, g_BoilerKdSteam);
    }
      
    // Steam Mode falling oneshot. Use espresso SP/tuning params.
    if (g_SteamModeOSF) {
      g_BoilerSp = g_BoilerSpEsp;
      g_BoilerPid.SetTunings(g_BoilerKpEsp, g_BoilerKiEsp, g_BoilerKdEsp);
    }
  }

// PID compute logic - handles timing of PID execution.
// Always called, even when in manual.
g_BoilerPid.Compute();
}


/* lightControl()**************************************************************
 * Controls machine indicator light depending on machine status (light mode). 
 * 0=Fault - triple blink, pause
 * 1=Heating - fade on, fade off
 * 2=Ready - Steady on
 * 3=Too hot - sawtooth (fade on, cut off)
 * Else=Fault
 ******************************************************************************/
void lightControl() {
  // 1=Heating
  if (g_LightMode == 1) {
    
  }

  // 2=Ready
  else if (g_LightMode == 2) {
    g_LightCmd = 255;
  }

  // 3=Too hot
  else if (g_LightMode == 3) {
    
  }

  // 0=Fault (or other Mo)
  else {
    
  }
}


/* loop()**********************************************************************
 * Sequencing function. Sequences read, computation, and write functions.
 * [TBD] may trigger serial prints/handle serial inputs for diagnostics.
 ******************************************************************************/
void loop() {
  // Utilities
  timing();
  
  // Inputs/Reads
  boilerTempInp();
  steamModeInp();
  
  // Processing/Calculations
  
  boilerTempControl();
  lightControl();

  // Monitoring
  int boilerCmdPct = g_BoilerCmd / 255 * 100;   // Scaling from PWM 0-255 to 0-100% for Diagnostics
  int lightCmdPct = g_LightCmd / 255 * 100;     // Scaling from PWM 0-255 to 0-100% for Diagnostics

  // Outputs/Writes
  analogWrite(pin_BoilerCmd, g_BoilerCmd);
  analogWrite(pin_LightCmd, g_LightCmd);
}