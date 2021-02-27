/*
>>>>>>>>>>>>>>>>>>  TO DO  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
SD card
Mapping van fan bFanLevel naar fFanLevel

*/
/* =============================================================================
IntelliFan V5.0.1
uP controlled fan for Bathroom and separate Toilet room.
This version senses occupation of rooms by checking that light is on:
- Bathroom: System senses Ligh-On and controles fanspeed according to
  humidty measured and calculated dewpoint(= temperature). Fan continues after Light-Out
  for as long as measured temperature is lower than the dewpoint temperature calculated

- Toilet: System senses light on and starts fan after X minutes.
  Fan continues after light out for Y minutes
  X and Y are to be set at compile time.
- Switch on unit allows to set unit modes to full/off/auto

Other ways of sensing occupation can easily be adopted: <bSense> and
<tSense> must be <true, or HIGH> to start fan running sequence.

This version is aimed at the Teensy 3.0/3.1/3.2 Arduino-like uP.
This version is for use with DHT11/22 humidity/temperature sensors.
This version uses a fan that is DC controlled fan. Harware translates ouput of the
DAC value to the requred DC control value of the fan.

*** VARIABLE NAMING CONVENTION ***

Variable names are written in lower case: variable
Constructed variable names use upper case for readability: myVariable
All non FSM function- and void names start with an underscore: _bool2Text(), _gech2HumTemp(),
consequently all variable names within functions/voids strat with underscore: _variable, _myVariable

*** DEBUGGING AND LOGGING WITH <nbrLogBits> WORD ***
In each state, only once after entering the state, first the assigned bit in the BitBool word is set (1)
In each state, BEFORE the YASM <FSM.next(state) command is stated, the assigned bit in the BitBool word
is reset (0).


Connect sensor(s) DHT11/22 to PCB V1.2 via JP1;
____________________________________
|  ch1 Signal  |  Pin JP1          |
|--------------|-------------------|
|   +5V        |    1              |
|   Data       |    2 (OneWire)    |
|   GND        |    3              |
|______________|___________________|
|  ch2 Signal  |  Pin JP1          |
|--------------|-------------------|
|   +5V        |    6              |
|   Data       |    7 (OneWire)    |
|   GND        |    8              |
|______________|___________________|

(C) Hans Derks, TenM, 2016, 2017, 2018, 2019, 2020, 2021
==================================================================================================*/
// *** BEGIN Includes ***
#include <SoftwareSerial.h> // library for serial communication
#include <yasm.h>           // FSM library to controll Finite Sate Machines
#include <TimeLib.h>        // time library to covert <millis> to hour:min:sec
#include <BitBool.h>        // bits manipulation library
#include <dhtnew.h>         // library for the DHT11/22 sensors
#include <TimedPID.h>       // PID library
#include <SD.h>             // SD card library
#include <SPI.h>            // SPI bus library
// *** END Includes ***

// *************************** Declarations **************************************
// all timing constants in mSecs (= system timing unit)
// to set seconds multiply <seconds> by 1000
// to set minutes multiply <minute(s) by 60000
/*
#define bFanWait 180000         // <b> time to wait before fan starts 30 min
#define tFanWait 120000         // <b> time to wait before fan starts 2 min
#define tFanLagTime 300000      // time of continued fan action after <t>
                                // occupation goes false (5 min)
#define bFanLagTime 1200000     // time of continued fan action after <b>
                                // sensor error (20 min)
#define fanStartPeriod 1000     // time fan start sequence (1 sec)
#define bFanTimeError 2700000   // time error flag after 45 mins
#define tFanTimeError 1200000   // time error flag after 20 mins
#define bRegretTime 60000       // time to stop fan running after <bFanWait> period expires
#define tRegretTime 60000       // time to stop fan running after <tFanWait> period expires
*/

// test values %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define bFanWait 1800         // <b> time to wait before fan starts 30 min
#define tFanWait 1200         // <b> time to wait before fan starts 2 min
#define tFanLagTime 3000      // time of continued fan action after <t>
                              // occupation goes false (5 min)
#define bFanLagTime 12000     // time of continued fan action after <b>
                              // sensor error (20 min)
#define fanStartPeriod 100    // time fan start sequence (1 sec)
#define bFanTimeError 2700000 // time error flag after 45 mins
#define tFanTimeError 120000  // time error flag after 20 mins
#define bRegretTime 60000     // time to stop fan running after <bFanWait> period expires
#define tRegretTime 60000     // time to stop fan running after <tFanWait> period expires

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Errorhandling
#define sensorErrorValue -999     // value sensors signal error with
#define sensorNotConnected 1      // no sensor
#define ch1MaxErrorCount 3        // afther <maxErrorCount> errors system goes into fallback mode
#define ch2MaxErrorCount 3        // afther <maxErrorCount> errors system goes into fallback mode
#define SensorErrorCountLimit 256 // avoid overspill of integer

// Define pins: ouside world
#define ledPin 13            // pin with on-board LED
#define ch1SensorPin 9       // pin for <b> humidity/temperature sensor
#define ch2SensorPin 10      // pin for <t> humidity/temperature sensor
#define bSensePin 5          // pin for b light sensor
#define tSensePin 6          // pin for t light sensor
#define fullSwitchPin 8      // pin to swich full continuesly mode pin
#define autoSwitchPin 7      // pin to swich auto mode pin
#define testSimulationPin 22 // pin to set user manupilation of values, sensing etc at rumtime
#define fanPin 3             // PWM pin for fan control
#define spiClockPin 14       // SPI pin for clock
#define spiMosiPin 11        // SPI data output
#define spiMisoPin 12        // SPI data intput
#define spiChipSelectPin 4   // SPI chip select

// Define run constants
#define bErrorFanLevel 100  // fan level when error in humidity sensors, in %
#define tDefaultFanLevel 50 // 50% fan speed equals 5V after hardware 
                            // amplification
#define bMinFanLevel 10     // minimum FanLevel to ensure correct humidity 
                            // measurement on occupation <b>
#define fanStartLevel 100   // 100% fan speed equals 10V after hardware 
                            // amplification
#define fanFullLevel 100    // level for full continues fan mode
#define slowFlash 1000      // LED flash no activities (wait state)
#define mediumFlash 500     // LED flash active state
#define fastFlash 250       // LED flash errornetwork share entries examples state
#define flashOnOffRatio 10  // sets on-off ratio of flashing onboard LED
#define bDefaultFanLevel 10 // 10% fan speed equals 1V output of hardware
#define bHumStart 30        // value for <bHum> to start fan
#define bHumStop 60         // value for <bHum> to stop fan
#define pwmMaxInput 135     // stores maximum input value for 
                            // PWM to obtain 10V output of    
                            // opamp (gain opamp is determined by hardware)
#define sensMax 120         // upper bound of allowable sensor value
#define sensMin 10          // lower bound of allowable sensor value
#define humMax 100          // of course, 100% means mosture is forming
#define nbrLogBits 23       // positions in logBits
#define tempNumSamples 10   // # of temperature samples to average
// PID defines
#define maxLvlPID 100 // upper bound of PID output (100%)
#define minLvlPID 10  // lower bound

// *** Declare and initialyze variables ***
/*
// longs
unsigned long runInTime = 10000;        // time on startup to settle all sensors
unsigned long previousHumMillis = 0;    // last time update for timed humidity task
unsigned long currentHumMillis = 0;     // stores the start time for timed humidity task
*/

// bytes
byte sysStartUp = true;      // used to sart system in loop only once
byte fanRunFlag = false;     // true if fan is running (nounts for <t>
byte ch1SensorError = false; // signals error in sensor channel 1
byte ch2SensorError = false; // signals error in sensor channel 2
byte switchFull = false;     // signals switch position 'continued full speed'
byte switchAuto = false;     // signals switch position 'auto mode'
byte bSense = false;         // sigals active <b> sensor
byte tSense = false;         // sigals active <t> sensor
byte sensorError = false;    // signals all sensors down to stages in FSM
byte bitChanged = false;     // signals one of the logbits changed
// following options are set or reset by placing jumper on PCB accordingly
byte testSimulation = false; // software emulation of sensors via serial comm by setting jumper on
                                 // PCB or bij setting pin D21 to HIGH during tests

// ints
// all timing values in mSecs (= system timing unit)
// to set seconds multiply <seconds> by 1000
// to set minutes multiply <minute(s) by 60000
unsigned int fFanLevel = 0;           // stores control value for fan (0-100%)
unsigned int tFanLevel = 0;           // stores run level for <t>
unsigned int bFanLevel = 0;           // stores calculated run leven for <b>
unsigned int activeFlash = slowFlash; // stores current LED flash rate
unsigned int ch1SensorErrorCount = 0; // stores the number of error counts for <ch1>
unsigned int ch2SensorErrorCount = 0; // stores the number of error counts for <ch2>
unsigned int fileNameCounter = 0;     // uses to creat unique filename for SD card logging

// flotes
float ch1Temp = 0;     // holds measured temperature
float ch1Hum = 0;      // holds measured humidity
float ch2Temp = 0;     // holds measured temperature
float ch2Hum = 0;      // holds measured humidity
float ch1LastHum = 0;  // holds last valid value for <ch1> humidity
float ch1LastTemp = 0; // holds last valid value for <ch1> teperature
float ch2LastHum = 0;  // holds last valid value for <ch2> humidity
float ch2LastTemp = 0; // holds last valid value for <ch2> teperature
float testHum = 0;

// floats for the PID algorithm
// Define the PID controller proportional, integral and derivative gains
float Kp = 10.0;
float Ki = 0.15;
float Kd = 5.0;

// doubles
// ......

// Strings
String dataString = String(); //Declare Sting() object for logging debug data

// chars
// ......

// initialize DHT22 sensor(s)
DHTNEW ch1Sensor(ch1SensorPin);
DHTNEW ch2Sensor(ch2SensorPin);

// initialize PID controller
TimedPID pid(Kp, Ki, Kd);

// initialize Finite State Machines (FSM)
YASM bFSM;   // <b> Finite State Machine (FSM)
YASM tFSM;   // <t> Finite State Machine (FSM)
YASM fFSM;   // <f> (fan control) Finite State Machine (FSM)
YASM ledFSM; // <led> (onboard LED) flash FSM

// create a BitBool variable (<nbrLogBits> bits word) that stores FSM states for logging
/*  logcodes table:
1	swFull
2	swOff
3	bSense
4	bTrue
5	bWait
6	bSetLvl
7	bLag
8	bRegretErr
9	bSenseErr
10	bOverTimeErr
11	tSense
12	tTrue
13	tWait
14	tSetLvl
15	tFalse
16	tLag
17	tOverTimeErr
18	fWaitLvl
19	fRunIn
20	fLvlCompare
21	fLvlZero
22  ch1SensorError
23  ch2SensorError
*/

// *** start instances of <BitBool> constructs
BitBool<nbrLogBits> logBits = {};    // initialize all bits to zero
BitBool<nbrLogBits> oldLogBits = {}; // used to detect changes in <logBits>

void setup()
{                        //############## SETUP ##########################################
    // *** start serial communication for logging purposes
    Serial.begin(57600); 
    delay(100);          // let serial com settle

    // *** start SPI communication for SD card, set SPI pins
    SPI.setSCK(spiClockPin);
    SPI.setMISO(spiMisoPin);
    SPI.setMOSI(spiMosiPin);
    SPI.begin();
    delay(100); // let SPI com settle
    
    // *** Configure digital & analog pins
    pinMode(ledPin, OUTPUT);
    pinMode(bSensePin, INPUT_PULLUP);
    pinMode(tSensePin, INPUT_PULLUP);
    pinMode(fullSwitchPin, INPUT_PULLUP);
    pinMode(autoSwitchPin, INPUT_PULLUP);
    pinMode(testSimulationPin, INPUT_PULLUP); // jumper on PCB on pin D22and GND deactivates user input
    pinMode(spiChipSelectPin, OUTPUT);         
    
    // set pins in initial states
    digitalWrite(ledPin, HIGH); // <led> to on state
    analogWrite(fanPin, 0);     // fan off

    // *** set options for DHT11/22 sensor
    ch1Sensor.setWaitForReading(true);
    ch2Sensor.setWaitForReading(true);

    // *** set output range options for PID machine
    pid.setCmdRange(minLvlPID, maxLvlPID);

    // *** set initial state of finite state machines
    bFSM.next(bTrue);    // production state machine for <b>, start in <bTrue>
    tFSM.next(tTrue);    // state machine for <t>, start in <tTrue>
    fFSM.next(fWaitLvl); // state machine for <f>, (fan) start in <fWailtLvl>
    ledFSM.next(ledOff); // state machine for <led>, start in <ledOff>

/*
    // *** check SD card rdy
    if (SD.begin(SdChipSelect)) 
    {
        if (SD.exists(nameFile))
        {
            SD.open(nameFile, FILE_READ);
            SD.nameFile.read;
        }
    
    }
*/
    
    delay(5000); // let hum/temp sensors settle after power up

    

    // *** start led flash in waiting mode
    activeFlash = slowFlash;

} // ################# END OF SETUP ############################################

//######################## BEGIN LOOP ##########################################
void loop()
{ 
    // *** startup, one time actions
    if (sysStartUp)
    {
        _sysStartUp(); // all one time actions that could not get done in setup void
    }

    // *** check unit switch status and set mode to full/off/auto
    _doSwitchStatus();
    
    // get status of <bSensePin> and tSensePin>
    _getLightsStatus();

    // read sensors ,can take up to 2 secs between readings
    _getSensorValues(ch1Hum, ch2Hum);
    
    if (testSimulation) // user input via serial, sets manually input values, etc
    {                    
        _getUserInput(); // get input for sensor values in debug mode
        ch1Hum = ch2Hum = testHum; // replace measured humidity with manual test input
    }
    
    // run Finite State Machines (FSMs)
    fFSM.run();   // state machine for <f(=fan)>, run FSM
    bFSM.run();   // state machine for <b>, FSM
    tFSM.run();   // state machine for <t>, run FSM
    ledFSM.run(); // state machine for flashing onboard LED

    // finally log debug information to serial and/or SD card if
    //    -> one of the debug states has changed
    //    -> serial comm established succesfully
    //    -> SD card present and rdy
    if (Serial)  //|| SDcardRdy)***********************************
    {
        _logDebugInfo();
    }
    
    /*
    // *** create unique filename, open file
    Serial.print("--------------------------------------------------------------------> ");
    Serial.println(String(random(100000)) + ".txt");
    //File dataFile = SD.open((String(random(100000)) + ".txt" ), FILE_WRITE);
    */

    // *** print debug information if one of the states has changed, write to serial port
    
} //########### END LOOP #######################################################

// utility voids for <b> FSM ***************************************************
// state bTrue
// wait untill bSense goes true, word pos. 6
void bTrue()
{
    if (bFSM.isFirstRun())
    { // only on first call
        // write bit to logging word (22 bits word)
        logBits[4] = true;
    }
    if (bSense)
    { // <b> sensor is active
        // write bit to logging word (22 bits)word
        logBits[4] = false;
        bFSM.next(bWait); // to next state
    }
}

// state bWait -----------------------------------------------------------------
// wait before more action for <bFanWait>, word pos. 7
void bWait()
{
    if (bFSM.isFirstRun())
    {   // only on first call
        // write bit to logging word (22 bits)word
        logBits[5] = true;
    }
    // just wait...
    if (bFSM.elapsed(bFanWait))
    {
        bFSM.next(bSetLvl); // transit to next state
        logBits[5] = false;
    }
}

//state bSetLvl ----------------------------------------------------------------
// set fan level according to logic and declares levels, word pos. 8
void bSetLvl()
{
    // *** handle <bSense> goes false, sensor errors, set <bHum> value
    if (!bSense)
    { // <bSense> false
        logBits[6] = false;
        bFSM.next(bLag);
    }
    else if (bFSM.elapsed(bFanTimeError))
    { // handle over time error, stop
        logBits[6] = false;
        bFSM.next(bOverTimeError);
    }
    else if (!(bFSM.timeOnState() < bRegretTime) && !bSense)
    { // dSense goes false during regret time
        logBits[6] = false;
        bFSM.next(bRegretError);
    }
    else
    {
        _bGetFanLevel();
        Serial.print("_bGetLevel gives as result ");
        Serial.println(bFanLevel);
        if (ch1SensorError && ch2SensorError)
        { // both sensors off line, go to default sceme
            logBits[6] = false;
            bFSM.next(allSensorError);
        }

    }
}
//state bLag -------------------------------------------------------------------
// state when light goes off after fan period
void bLag()
{
    if (bFSM.isFirstRun())
    { // only on first call
        logBits[7] = true;
    }
    if (bFanLevel <= bHumStop)
    {
        bFanLevel = 0;
        logBits[7] = false;
        bFSM.next(bTrue); // start over again, back to beginning
    }
    if ((sensorError) && (bFSM.elapsed(bFanLagTime)))
    {                  // wait till lag time expires -> sensor error
        bFanLevel = 0; //  stop fan
        logBits[7] = false;
        bFSM.next(bTrue); // start over again, back to beginning
    }
    //?????????????????????????????????? HIER AFHANDELING ALS VOCHTIGHEID NOG TE HOOG IS
}
//state bRegretError ---------------------------------------------------------
// trigger error state when light goes off during <bRegretTime>
void bRegretError()
{
    if (bFSM.isFirstRun())
    { // only on first call
        logBits[8] = true;
    }
    bFanLevel = 0;
    logBits[8] = false;
    bFSM.next(bTrue);
}
// state bSensorError -------------------------------------------------------------
// handle situation where sensor error is detected, word pos. 9
void allSensorError()
{
    if (bFSM.isFirstRun())
    { // only on first call
        logBits[9] = true;
    }
    bFanLevel = bErrorFanLevel; // set <b> FanLevel to Error value
    if (!bSense)
    { // wait till light goes out
        logBits[9] = false;
        bFSM.next(bLag);
    }
}
//state bOverTimeError ---------------------------------------------------------
// trigger error state when ligt is left on by mistatke, word pos. 22
void bOverTimeError()
{
    if (bFSM.isFirstRun())
    { // only on first call

        logBits[10] = true;
    }
    bFanLevel = 0;           // set <b> FanLevel to zero
    activeFlash = fastFlash; // flash LED at high speed
    if (!bSense)
    { // wait for <b> sensor to go inactive again
        logBits[10] = false;
        bFSM.next(bTrue); // start over again, back to beginning
    }
}
//end <b> FSM utility functions ************************************************

// *** utility voids for  <t> Finite State Machine *****************************
// state tTrue, wait for tSense to go true, word pos. 13
void tTrue()
{

    if (tFSM.isFirstRun())
    { // only on first call
        logBits[12] = true;
    }

    if (tSense)
    { // <t> sensor is active
        logBits[12] = false;
        tFSM.next(tWait); // to next state
    }
}
// state tWait -----------------------------------------------------------------
// wait for delay in start is expired, word pos. 12
void tWait()
{
    if (tFSM.isFirstRun())
    { // only on first call
        logBits[13] = true;
    }
    // wait during <t> start wait time, make shure that <t> sensor did not go
    // inactive during wait
    if (tFSM.elapsed(tFanWait))
    {
        if (tSense)
        { // <t> sensor did not go inactive during wait
            logBits[13] = false;
            tFSM.next(tSetLvl); // transit to next state
        }
        else
        {
            logBits[13] = false;
            tFSM.next(tTrue); // start over again, back to beginning
        }
    }
}

// state tSetLvl ----------------------------------------------------------------
// set fan level according to logic and pre-defined levels, word pos. 13
void tSetLvl()
{
    if (tFSM.isFirstRun())
    { // only on first call
        logBits[14] = true;
    }
    // map default tFanLevel (0 -100) to PWM scale
    tFanLevel = tDefaultFanLevel;
    logBits[14] = false;
    tFSM.next(tFalse);
}

// state tFalse ----------------------------------------------------------------
// wait for tSense to go false, log word pos. 14
void tFalse()
{
    if (tFSM.isFirstRun())
    { // only on first call
        logBits[15] = true;
    }
    if (tFSM.elapsed(tFanTimeError))
    {
        logBits[15] = false;
        tFSM.next(tOverTimeError); // too long error, shutdown and transit
    }
    if (!tSense)
    { // wait <t> sensor go inactive
        if (!(tFSM.timeOnState() < tRegretTime))
        { //premature end of cycle?
            logBits[15] = false;
            tFanLevel = 0;
            tFSM.next(tTrue);
        }
        logBits[15] = false;
        tFSM.next(tLag); // transition to tLag state
    }
}

// state tLag -------------------------------------------------------------------
// keep fan running during lag period, word pos. 15
void tLag()
{
    if (tFSM.isFirstRun())
    { // only on first call
        logBits[16] = true;
    }
    if (tFSM.elapsed(tFanLagTime))
    { // lag time expired?
        tFanLevel = 0;
        logBits[16] = false;
        tFSM.next(tTrue); // start over again, back to beginning
    }
}

// state tOverTimeError ---------------------------------------------------------
// trigger error state when ligt is left on by mistatke, word pos. 16
void tOverTimeError()
{
    if (tFSM.isFirstRun())
    { // only on first call
        logBits[17] = true;
    }
    activeFlash = fastFlash; // flash LED high speed
    tFanLevel = 0;           // set <t> FanLevel to zero
    analogWrite(fanPin, 0);
    if (!tSense)
    { // wait for <t> sensor to go high again
        logBits[17] = false;
        activeFlash = slowFlash;
        tFSM.next(tTrue); // start over again, back to beginning
    }
}
// end <t> FSM utility functions ************************************************

// utility voids for FSM <f> ---------------------------------------------------
// state fWaitLvl, set fan level depending on <b> and <t> levels, word pos. 17
void fWaitLvl()
{
    if (fFSM.isFirstRun())
    { // only on first call
        logBits[18] = true;
    }
    if ((bFanLevel > 0) || (tFanLevel > 0))
    {
        if (fanRunFlag)
        {
            logBits[18] = false;
            fFSM.next(fLvlComp); // jump que to fLvlComp
        }
        else
        {
            logBits[18] = false;
            fFSM.next(fRunIn); // start fan with run in
        }
    }
}

// state fRunIn -----------------------------------------------------------------
// set start level to avoid stalling fan with short pulse, word pos. 18
void fRunIn()
{
    if (fFSM.isFirstRun())
    { // only on first call
        logBits[19] = true;
    }
    fanRunFlag = true;
    fFanLevel = fanStartLevel;
    if (fFSM.elapsed(fanStartPeriod))
    {
        logBits[19] = false;
        fFSM.next(fLvlComp); // jump to fLvlCompare
    }
}

// state fLvlComp --------------------------------------------------------------
// compare levels of <b> and <t> and set fan level, word pos. 19
void fLvlComp()
{
    if (fFSM.isFirstRun())
    { // only on first call
        logBits[20] = true;
    }
    fFanLevel = max(bFanLevel, tFanLevel); // set level to the bigger of the two
    analogWrite(fanPin, _perct2PWM(fFanLevel));
    if (fFanLevel == 0)
    {
        logBits[20] = false;
        fFSM.next(fLvlZero); // jump to fLvlZero
    }
}

// state fLvlZero----------------------------------------------------------------
// set fan level to zero, word pos. 20
void fLvlZero()
{
    if (fFSM.isFirstRun())
    { // only on first call
        logBits[21] = true;
    }
    fFanLevel = 0;
    analogWrite(fanPin, _perct2PWM(fFanLevel));
    fanRunFlag = false;
    logBits[21] = false;
    fFSM.next(fWaitLvl); // jump to begin state
}
//end <f> FSM utility functions ************************************************

// utility voids for FSM <LED> -------------------------------------------------
// state ledOn -----------------------------------------------------------------
void ledOn()
{
    digitalWrite(ledPin, HIGH); // <led> on state
    // <activeFlash> "off" state (= on time / on-off ratio) elapsed?
    if (ledFSM.elapsed((activeFlash / flashOnOffRatio)))
    {
        ledFSM.next(ledOff); // to <activeFlash> "off" state
    }
}

// state ledOff ----------------------------------------------------------------
void ledOff()
{
    digitalWrite(ledPin, LOW); // <led> off state
    if (ledFSM.elapsed(activeFlash))
    {                       // <activeFlash> "on" state elapsed?
        ledFSM.next(ledOn); // to <activeFlash> "on" state
    }
}

//end <led> FSM utility functions **********************************************

//===================== VOIDS & FUNCTIONS IntelliFan =======================

// void _sysStartUp, check and start up sensors
void _sysStartUp() 
{
    // insert here startup processes that need to run one time only and cannot run in <setup>
    // *** detect option for user test input -> set/remove jumper on PCB
    if (digitalRead(testSimulationPin) == HIGH)
    {
        testSimulation = true;
        testHum = 10;
        Serial.println("Simualation Mode is SET");
    }
    else
    {
        testSimulation = false;
        Serial.println("Simualation Mode is NOT SET");
    }
    /*
    // open file on SD card and get name of last file written debug info to
    fileNameCounter = 
    
    
    
    
    */
}

//void _doSwitchStatus, check unit switch status, set flags
void _doSwitchStatus()
{
    while (digitalRead(fullSwitchPin) == LOW)
    {                                                  // switch in full fan position
        analogWrite(fanPin, _perct2PWM(fanFullLevel)); // fan at full continues level
        digitalWrite(ledPin, HIGH);                    //  LED continually
        // set logging bits in <logBits>
        logBits[1] = true; // switch full
        logBits[2] = false;
    }
    // if switch not in full nor auto position, fan = off
    while ((digitalRead(fullSwitchPin) == HIGH) && (digitalRead(autoSwitchPin) == HIGH))
    {
        analogWrite(fanPin, 0);    // fan off
        digitalWrite(ledPin, LOW); // signal OFF status
        // set logging bits in <logBits>
        logBits[1] = false;
        logBits[2] = true; // switch off
    }
}
// *** get status of <bSensePin> and tSensePin>, set flags <b> and <t>
void _getLightsStatus()
{
    if (digitalRead(bSensePin) == HIGH)
    {
        bSense = false;
    }
    else
    {
        bSense = true;
    }
    if (digitalRead(tSensePin) == HIGH)
    {
        tSense = false;
    }
    else
    {
        tSense = true;
    }
    
    // set logging bits in <logBits>
    logBits[3] = bSense;
    logBits[11] = tSense;
}

// *** void _getSensorValues , check sensors, get values and set flags.
// error codes get replaces with last known correct value
void _getSensorValues(float _ch1Hum, float _ch2Hum)
{
    float _ch1LastHum = _ch1Hum; // store old value in case of senseor error
    float _ch2LastHum = _ch2Hum;
    ch1Sensor.read();
    ch2Sensor.read();
    _ch1Hum = ch1Sensor.getHumidity();
    if ((_ch1Hum == sensorErrorValue) || (_ch1Hum <= sensorNotConnected))
    {
        _ch1Hum = _ch1LastHum; // restore old value in case of sensor error
        if (ch1SensorErrorCount < ch1MaxErrorCount)
        { // avoid overspill of error counter
            ch1SensorErrorCount++;
        }
        else
        {
            ch1SensorError = true; // set error flag for channel 1
            ch1SensorErrorCount = 0;
            ch1Hum = 0; // reset old humidity value
                        // Serial.println("Sensor 1 offline");
        }
    }
    else
    {
        ch1Hum = _ch1Hum;
        ch1SensorError = false; // reset error flag for channel 1
    }
    // *** all the same for sensor channel 2
    _ch2Hum = ch2Sensor.getHumidity();
    if ((_ch2Hum == sensorErrorValue) || (_ch2Hum <= sensorNotConnected))
    {
        _ch2Hum = _ch2LastHum;
        if (ch2SensorErrorCount < ch2MaxErrorCount)
        {
            ch2SensorErrorCount++;
        }
        else
        {
            ch2SensorError = true;
            ch2SensorErrorCount = 0;
            ch2Hum = 0;
            // Serial.println("Sensor 2 offline");
        }
    }
    else
    {
        ch2Hum = _ch2Hum;
        ch2SensorError = false;
    }
    logBits[22] = ch1SensorError; // set logbits if sensor error
    logBits[23] = ch2SensorError;
}

// *** void _bGetFanLevel, returns the command value from the PID machine
// try first both sensors, then ch1, then ch2, otherwise <bDefaultFanLevel>
void _bGetFanLevel()
{
    Serial.println("Now in _bFanGetLevel");
    if ((!ch1SensorError) && (!ch2SensorError))
    {
        if (max(ch1Hum, ch2Hum) >= bHumStart)
        {
            bFanLevel = pid.getCmdAutoStep(bHumStop, max(ch1Hum, ch2Hum));
        }
    }
    else if (!ch1SensorError)
    {
        Serial.println(ch1Hum);
        if (ch1Hum >= bHumStart)
        {
            bFanLevel = pid.getCmdAutoStep(bHumStop, ch1Hum);
        }
    }
    else if (!ch2SensorError)
    {
        Serial.println(ch2Hum);
        if (ch2Hum >= bHumStart)
        {
            bFanLevel = pid.getCmdAutoStep(bHumStop, ch2Hum);
        }
    }
    else
    {
        bFanLevel = bDefaultFanLevel;
    }
}
// *** function _perct2PWM, translates fan level percentage to PWM input
int _perct2PWM(int _level)
{
    _level = map(_level, 0, 100, 0, pwmMaxInput); //translate percentage (max 100) to PID value (max. <PWMtoPercnt>)
    return _level;
}

// *** function _bool2text, translate boolean to printable text
String _bool2text(byte _bool)
{
    String _text;
    if (_bool)
    {
        _text = "true";
    }
    else
    {
        _text = "false";
    }
    return _text;
}

// *** void _printLogging() prints logging info to serial over USB and/or to SD card
void _logDebugInfo()
{
   // _bitsChanged(); // check if there are changes in status bits
    if (_bitsChanged())
    {
        if (Serial)
        {
            _buildLogString(); // create log string
            Serial.println(dataString);
        }
        
        
    /*    
        if (SDcardRdy)
        {
            _logDebugInfoToSD();
        }
    */
    }
    
}

// *** logbits changed since last run of loop?
byte _bitsChanged()
{
    int _i = 0;
    byte _r = false;
    for (_i = 1; _i <= nbrLogBits; _i++)
    { 
        if (logBits[_i] != oldLogBits[_i])
        {
            oldLogBits[_i] = logBits[_i];
            _r = true;
        }
    }
    return _r;
}
// *** // create log string
void _buildLogString() 
{
    int _i = 0;
    dataString = "";    // start with empty string
    // add timestamp
    dataString += ((String(hour()) + ":" + String(minute()) + ":" + String(second()) + ","));
    for (_i = 1; _i <= nbrLogBits; _i++)
    {
        dataString += (logBits[_i]); 
        dataString += ","; // and pad with comma for CSV format
    }
    // and add with humidity and levels
    dataString += (String(ch1Hum, 2) + "," + String(ch2Hum, 2) + "," + bFanLevel + "," + tFanLevel + "," + fFanLevel);
}

// *** void _getUserInput, gets user input in debug state via serial comm
void _getUserInput()
{
    int _serChar;
    if (Serial.available())
    {
        _serChar = char(Serial.read());
        switch (_serChar)
        {
        case 'b':
            bSense = !bSense;
            Serial.print("B sense set to: ");
            Serial.println(_bool2text(bSense));
            break;
        case 't':
            tSense = !tSense;
            Serial.print("T sense set to: ");
            Serial.println(_bool2text(tSense));
            break;
        case 'u':
            testHum += 2;
            if (testHum > 100)
            {
                testHum = 100;
            }
            Serial.print("testHum plus 2%: ");
            Serial.println(testHum);
            break;
        case 'U':
            testHum += 10;
            if (testHum > 100)
            {
                testHum = 100;
            }
            Serial.print("testHum plus 10%: ");
            Serial.println(testHum);
            break;
        case 'd':
            testHum -= 2;
            if (testHum < 1)
            {
                testHum = 0;
            }
            Serial.print("testHum minus 2%: ");
            Serial.println(testHum);
            break;
        case 'D':
            testHum -= 10;
            if (testHum < 1)
            {
                testHum = 0;
            }
            Serial.print("testHum minus 10%: ");
            Serial.println(testHum);
            break;
        default:
            Serial.flush();
            Serial.println("Not a valid option: b,t,u/U,d/D");
            Serial.print("testHum currently: ");
            Serial.println(testHum);
            break;
        }
    }
}

// ########################## END OF FILE #############################################
