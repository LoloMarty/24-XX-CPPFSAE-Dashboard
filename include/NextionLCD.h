#include <FlexCAN_T4.h>

#define pgBtnPin A17
bool pgBtnBool = false;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

enum BSPD {Standby, Trig, TRIP};            // BSPD statuses
enum Screen {Config1, Config2, DragMode, Params, BSPD_Trig, BSPD_Trip, Shift, SlowDown};  //  Screen Mode

long unsigned int timeHolder;               // used to measure the amount of elapsed time since isSilentTime's value has changed
long unsigned int showTimeHolder;
long unsigned int silenceTime = 50;         // time between the inversion of isSilentTime's value
bool isSilentTime = false;                  // locks out normal CAN message processing, so the one-shot GearP CAN message can be listened for

Screen currScreen;                          // holds the last screen changed before BSDP Trip, Trig, or Shift screen changes
bool irregScreen;                           // a bool flag set when BSPD Trip, Trig, or Shift screen changes occur, used for resetting to last screen

double currBatt;                            // battery voltage                      paramCode 0
//char currBSPDState;                       // status of the BSPD                   paramCode 1      0=Stdby; 1=Trig; 2=Trip
int currECT;                                // current engine coolant temp          paramCode 2
int currFrontBP;                            // front brake bias pressure            paramCode 3
int currRearBP;                             // rear brake bias pressure             paramCode 4
int currFuelPSR;                            // fuel pressure                        paramCode 5
int currGearP;                              // gear position                        paramCode 6
double currLamb;                            // engine lambda (AFR)                  paramCode 7
int currMAP;                                // manifold airpressure                 paramCode 8
int currOilPSR;                             // oil pressure                         paramCode 9
int currRPM;                                // RPM                                  paramCode 10
double currThrtl;                           // throttle pedal %                     paramCode 11
long unsigned int currTimerDel;             // master timer time delta              paramCode 12     
char currTimerDelPic;                       // LCD imageID for master timer delta   paramCode 13     0=NegDelt; 1=PosDelt
int maxWSpd;                                // maximum all-drive wheelspeed         paramCode 14
long unsigned int timer_R[] = {0, 0, 0};    // all 3 timers in master timer         paramCode 15-17

char WARN_ECTO;                             // status of engine coolant temp warn   paramCode 18     0=noWarn; 1=warnOn
char WARN_FPRSR;                            // status of fuel pressure warn         paramCode 19     0=noWarn; 1=warnOn
char WARN_OTEMP;                            // status of oil temperature warn       paramCode 20     0=noWarn; 1=warnOn
char WARN_OPRSR;                            // status of oil pressure warn          paramCode 21     0=noWarn; 1=warnOn         

/* Newly Added Params*/
int currOilTemp;                            // engine oil temperature               paramCode 22

void canSniff(const CAN_message_t &msg);
void CANmsgRecieve(const CAN_message_t &msg);
void chngScrn(int scrnCode);               
void chngParamVal(int paramCode, double val);
void chngParamVal(int paramCode, int val);
void endCommand();
void pageBtnPressed();
void returnToLastNormScrn();
void chngScrnSlowDown();

String convMSec_to_TForm(long unsigned int val);
long unsigned int getTime();
