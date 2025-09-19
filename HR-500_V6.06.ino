/*
Hardrock 500 Firmware

*/

#include <stdint.h>
#include <HR500.h>
#include <HR500X.h>
#include <HR500_pins.h>
#include "HR500V1.h"
#include <SPI.h>
#include <FreqCount.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <Wire.h>
#include <Chrono.h>
#include <EwmaT.h>
#include <Watchdog.h>


const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

XPT2046_Touchscreen ts1(TP1_CS);
XPT2046_Touchscreen ts2(TP2_CS);

long M_CORR = 100;                            // Meter correction factor (%)

byte FAN_SP = 0;                              // Cooling fan speed 0 to 2 for four levels
char ATU_STAT;
bool ATTN_P = 0;                              // Flag for presence of attenuator 1/0 present / not
byte ATTN_ST = 0;                             // Flag for attenuator state 1/0 on/off
volatile byte BAND = 10;                      // Current band selection
byte OBAND = 0;                               // Old band selection
byte NBAND = 0;                               // Test variable for band switching.
bool frq_chkd = 0;                            // Indicates drive frequency has been measured 
byte TMP_UNIT = 1;                            // C/F 1/0  temperature selection
byte ANTSEL[11] = {1,1,1,1,1,1,1,1,1,1,1};    // Array of antenna per band selections default ANT 1
byte ATUB[11] = {0,0,0,0,0,0,0,0,0,0,0};      // Array of ATU button states per band default OFF
byte ACC_Baud;                                // ACC serial baude rate
byte USB_Baud;                                // USB baud rate
int8_t menu_choice = 0;                       // variable for which item in menu is currently displayed
bool ATU_P = 0;                               // Flag for detecting presence of ATU
byte ATU = 0;                                 // Flag for ATU in/out
byte MeterSel = 0;                            // 1 - FWD; 2 - RFL; 3 - DRV; 4 - VDD; 5 - IDD
byte OMeterSel;                               // Saves meter selection
unsigned int temp_utp, temp_dtp;              // Fan hysteresis points upper and lower
byte menuSEL = 0;                             // </> menu item scrolling (0) / menu choice (1)                            
byte Bias_Meter = 0;                          // Flag to display bias current reading in menu
int MAX_CUR = 20;                             // Maximum power supply current 20 Amps
byte XCVR = 0;                                // XCVR number from enumerated array
bool trip = 0;                                // to latch tripped status
bool do_swr = 0;                              // flag for updating vswr display
bool a_chg = 0;                               // flag that alert level has changed
bool fault = 0;                               // flag indicated trip has been latched.
int OVSWR = 0;                                // Saves VSWR value from last measurement

volatile byte MODE = 0;                       // 0 - OFF, 1 - STBY
byte oMODE;                                   // Stores previous mode for mode change detect
bool TX = 0;                                  // flag for TX/RX state (1/0)
bool PTT = 0;                                 // Logic state for PTT
byte SCREEN = 0;                              // variable for which screen level active on right TFT
volatile byte SR_DATA = 0;                    // variable for relay control
volatile byte SR_SAVE= 0;                     // variable for saving relay control byte

volatile byte Theme = 0;                      // 0 - NORMAL, 1 - NIGHT

volatile unsigned int f_tot = 0;              // Smoothed forward power value
volatile unsigned int r_tot = 0;              // Smoothed reflected power value
volatile unsigned int d_tot = 0;              // Smoothed drive power value

unsigned int f_pw = 0;                        // Calculated forward power
unsigned int fwd_pk = 0;                      // Peak forward power
unsigned int r_pw = 0;                        // Calculated reflected power
unsigned int d_pw = 0;                        // Calculated drive power
long dc_cur = 0;                              // DC current reading
long dc_vol = 0;                              // DC voltage reading
long MX_PWR = 0;                              // 24 bit max power fault threshold value
int MXP_USB = 0;                              // Max power threshold upper byte
int MXP_MSB = 0;                              // Max power threshold mid byte
int MXP_LSB = 0;                              // Max power threshold lower byte


long  t_tmp;                                  // Variable for temperature analog read
long t_smth;                                  // Smoothed temperature
unsigned int t_ave;                           // Smoothed temperature converted from long to unsigned int

volatile byte flagDIS = 0;                    // Number of Timer 1 overflows before port change INT re-enabled

byte I_alert = 0, V_alert = 0, F_alert = 0, R_alert = 0, D_alert = 0;       // current alert flags
byte OI_alert = 0, OV_alert = 0, OF_alert = 0, OR_alert = 0, OD_alert = 0;  // old alert flags

byte ADCvinMSB, ADCvinLSB, curSenseMSB, curSenseLSB;  // for reading ADC in LTC2945 wide range I2C power monitor
unsigned int ADCvin, ADCcur;

unsigned int uartPtr = 0, uartPtr2 = 0;       // Serial string pointers
unsigned int uartMsgs = 0, uartMsgs2 = 0, readStart = 0, readStart2 = 0;
char rxbuff[128];                             // 128 byte circular Buffer for storing rx data
char workingString[128];
char rxbuff2[128];                            // 128 byte circular Buffer for storing rx data
char workingString2[128];
char RL_TXT[] = {"    "};                     // Initial swr display text
char ORL_TXT[] = {" ..."};

char ATU_buff[40], ATU_cmd[8], ATU_cbuf[32], ATU_ver[8];
bool TUNING = 0;

uint16_t acolor;                              // Antenna selection colour  Grey = invalid
uint16_t r_col;                               // Alert colour
uint16_t vswr_col;                            // VSWR display colour according to alert level

bool DC_DATA = 0;                             // Flag indicates DC power & efficiency displayed
char DC_TXT[4];                               // Text for DC power
char ODC_TXT[4];                              // Old text for DC power
char EF_TXT[4];                               // Text for DC efficiency
char OEF_TXT[4];                              // Old text for DC efficiency

char TEMPbuff[16];                            // Initialize buffer 
int t_count = 0;                              // Initialize loop counters so that 
int a_count = 1;                              // "a"and "t" loops execute alternately ;^{>

unsigned int F_bar = 15, OF_bar = 15;         // Initialize bar graph bottom of scale
unsigned int F_pk;                            // Peak hold variable
bool Hold = false;                            // Peak hold 'active' flag

int t_disp = 0, t_read;                       // Temperature registers
int otemp = -99;
int s_disp = 19;                              // SWR will be displayed on first round

char Bias_buff[] = {"          "};
int oBcurr = -1;
byte FTband;
bool VPTT = 0;                                // Virtual PTT flag (when amp is offline)
bool TB = 0;                                      

uint16_t  BG_col;                             // Background colour
uint16_t  FG_col;                             // Foreground colour
uint16_t  DK_BTN;                             // Button highlight
uint16_t  LT_BTN;                             // Button highlight
uint16_t  ACT_TXT;                            // Active text colour
uint16_t  PSV_TXT;                            // Passive text colour
uint16_t  CHG_TXT;                            // Changing item text colour
uint16_t  MTR_TXT;                            // Meter text colour
uint16_t  MTR_MRK;                            // Meter mark colour
uint16_t  MTR_SCL;                            // Meter scale background
uint16_t  MTR_BKG;                            // Background of meter bar
uint16_t  MTR_BAR;                            // Background of meter bar
uint16_t  MTR_PK;                             // Peak indicator of meter bar
uint16_t  A1_TXT;                             // Antenna 1 text colour (per band)
uint16_t  A2_TXT;                             // Antenna 2 text colour (per band)
uint16_t  SPLASH;                             // Splash screen display colour
uint16_t  GOOD_LED;                           // Alert 1 colour
uint16_t  WARN_LED;                           // Alert 2 colour
uint16_t  ALRM_LED;                           // Alert 3 colour
uint16_t  IDLE_LED;                           // Status LED colour when amp is off
uint16_t  COR_TXT;                            // Carrier detect colour


Chrono Peak_hold;                             // Timebase for bargraph peak hold function
Chrono drv_wait;                              // Timebase for drive power bar update
Chrono swr_wait;                              // Timebase for SWR display
Chrono PTT_wait;                              // Timebase to debounce PTT input
Chrono TR_wait;                               // Timebase for carrier detect T/R switching
Chrono TX_time;                               // Measures time in TX state

EwmaT <long> fwd_pwr(1,2);                    // Exponential filter for forward power
EwmaT <long> rfl_pwr(1,2);                    // Exponential filter for reflected power
EwmaT <long> drv_pwr(1,2);                    // Exponential filter for drive power
EwmaT <long> swr_val(8,10);                   // Exponential filter for vswr value
EwmaT <long> temperature(6,100);              // Exponential filter for temperature

Watchdog watchdog;                            // Watchdog timer


//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
// Program setup section

void setup()
{
  int c;
  
  SETUP_FAN1 SETUP_FAN2 SetFanSpeed(0);                           // Initialize fan speed low
  SETUP_RELAY_CS  RELAY_CS_HIGH                                   // Run macros to setup program parameters
  SETUP_LCD1_CS  LCD1_CS_HIGH
  SETUP_LCD2_CS  LCD2_CS_HIGH
  SETUP_SD1_CS  SD1_CS_HIGH
  SETUP_SD2_CS  SD2_CS_HIGH
  SETUP_TP1_CS  TP1_CS_HIGH
  SETUP_TP2_CS  TP2_CS_HIGH
  SETUP_LCD_BL  analogWrite(LCD_BL, 0);                           // start with backlight off
  SETUP_BYP_RLY RF_BYPASS
  SETUP_F_COUNT
  SETUP_ANT_RLY  
  SEL_ANT1
  SETUP_COR
  SETUP_PTT
  SETUP_TRIP_RFL
  SETUP_RESET
  RESET_PULSE
  SETUP_BIAS
  BIAS_OFF
  SETUP_TTL_PU
  S_POL_SETUP
  SETUP_ATU_TUNE
  ATU_TUNE_HIGH
  SETUP_ATTN_INST
  SETUP_ATTN_ON
  ATTN_ON_LOW
  
  pinMode (RST_OUT, OUTPUT);
  digitalWrite (RST_OUT, HIGH);

  

  Theme = EEPROM.read(eetheme);                                   // Recall stored Theme setting
  if (Theme < 0 || Theme > 1) Theme = 0;
  if (Theme == 0){
    snprintf(item_disp[mTheme], IDSZ, "     NORMAL     ");
  }
  if (Theme == 1) {
    snprintf(item_disp[mTheme], IDSZ, "     NIGHT      ");
  }

  BAND = EEPROM.read(eeband);                                     // Recall stored band
  if (BAND > 10) BAND = 10;
  OBAND = BAND;
  SetBand();
  
  ATU = EEPROM.read(eeatub);                                      // Recall stored ATU state
  if (ATU < 0 || ATU > 1) ATU = 0;

  for (byte i = 1 ; i < 11 ; i++){
    ANTSEL[i] = EEPROM.read(eeantsel+i);                          // Recall saved per-band-antenna selections
    if (ANTSEL[i] == 1) {SEL_ANT1;}
    else if(ANTSEL[i] == 2) {SEL_ANT2;}
    else{
      SEL_ANT1;
      ANTSEL[i] = 1;                                              // If not previously set then default to ANT1
      EEPROM.write(eeantsel+i,1); 
    }
    ATUB[i] = EEPROM.read(eeatub+i);
    if (ATUB[i] == 1) ATU = 1;                                    // Turn ATU on/off per band as stored
    else if (ATUB[i] == 0) ATU = 0;                               
    else {
      ATUB[i] = 0;                                                // otherwise default to off per band and store
      EEPROM.write(eeatub+i,0);
    }
  }

  ACC_Baud = EEPROM.read(eeaccbaud);                              // Recall saved accessory serial baud rate
  if (ACC_Baud < 0 || ACC_Baud > 5) ACC_Baud = 3;
  Set_Ser2(ACC_Baud);

  USB_Baud = EEPROM.read(eeusbbaud);                              // Recall saved USB serial baud rate
  if (USB_Baud < 0 || USB_Baud > 5) USB_Baud = 5;
  Set_Ser(USB_Baud);

  TMP_UNIT = EEPROM.read(eecelsius);                              // Recall saved temperature unit setting
  if (TMP_UNIT != 0) TMP_UNIT = 1;
  snprintf(item_disp[mCELSIUS], IDSZ, 
    "       &%c       ", TMP_UNIT ? 'C' : 'F');

  MeterSel = EEPROM.read(eemetsel);                              // Recall saved last meter selection
  if (MeterSel < 1 || MeterSel > 5) MeterSel = 1;
  OMeterSel = MeterSel;

  XCVR = EEPROM.read(eexcvr);                                     // Recall saved transceiver selection
  if (XCVR < 0 || XCVR > xcvr_max) XCVR = 0;
  strcpy (item_disp[mXCVR] ,xcvr_disp[XCVR]);
  SET_XCVR(XCVR);

  byte MCAL = EEPROM.read(eemcal);                                // Recall saved meter calibration
  if (MCAL < 75 || MCAL > 125) MCAL = 100;
  M_CORR = long(MCAL);
  snprintf(item_disp[mMCAL], IDSZ, 
    "      %3u       ", M_CORR);

  if (ATTN_INST_READ == LOW) {
    ATTN_P = 1;
    ATTN_ST = EEPROM.read(eeattn);                                // Recall attenuator selection if present
    if (ATTN_ST > 1) ATTN_ST = 0;
    if (ATTN_ST == 1){
      ATTN_ON_HIGH;
      item_disp[mATTN] = (char*)" ATTENUATOR IN  ";
    }
    else { 
      ATTN_ON_LOW;
      item_disp[mATTN] = (char*)" ATTENUATOR OUT ";
    }
  }
  else { 
    ATTN_P = 0;
    ATTN_ST = 0;
    item_disp[mATTN] = (char*)" NO ATTENUATOR  ";
  } 

  analogReference(EXTERNAL);                                    // External reference voltage enabled for A/D conversion.
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_32;

  
  SPI.begin();                                                  // Setup SPI port
  Wire.begin();
  Wire.setClock(400000);                                        // Set SPI clock speed 400KHz (maximum for LTC2945)
  
  MX_PWR = MAX_PWR * 8000L;                                      // 5mV/bit and 25mA/bit so watts x 8000
  MXP_USB = ((MX_PWR & 0xFF0000) >> 16);                         // convert to three bytes
  MXP_MSB = ((MX_PWR & 0x00FF00) >> 8);
  MXP_LSB = (MX_PWR & 0x0000FF);
  mxp_set();                                                    // Program max power threshold in LTC2945
  trip_clear();                                                 // Intitialize LTC2945 power monitor
  Set_Theme();                                                  // Choose display colours according to chosen theme

  Tft.LCD_SEL = 0;                                              // Initialize TFT LCD screens with chosen background colour
  Tft.lcd_init(BG_col);
  Tft.LCD_SEL = 1;
  Tft.lcd_init(BG_col);
  analogWrite(LCD_BL, 255);                                     // Turn on backlight
  
  Tft.LCD_SEL = 0;                                              // Do splash screen
  Tft.drawString((uint8_t*)"  WHEN  QRP  ", 40, 75,  3, SPLASH);
  Tft.drawString((uint8_t*)"IS NOT ENOUGH?", 40, 120,  3, SPLASH);
  delay(1000);
  Tft.LCD_SEL = 1;
  Tft.drawString((uint8_t*)"   VDMOS    ", 40, 75,  3, SPLASH);
  Tft.drawString((uint8_t*)"HF AMPLIFIER", 40, 120, 3, SPLASH); 
  delay(1000);
 
  Tft.LCD_SEL = 0;                                              // Erase splash screen
  Tft.lcd_init(BG_col);
  Tft.LCD_SEL = 1;
  Tft.lcd_init(BG_col);

 Serial3.begin(19200);                                          // ATU detect and setup
  Serial3.println(" ");
  strcpy(ATU_cmd, "*I");
  strcpy(ATU_ver, "---");
  if (ATU_exch() > 10){
    strncpy(ATU_cbuf, ATU_buff, 9);
    if (strcmp(ATU_cbuf, "HR500 ATU") == 0) {
      ATU_P = 1;
      strcpy(ATU_cmd, "*V");
      c = ATU_exch();
      strncpy(ATU_ver, ATU_buff, c - 1);     
    }

  }


//ATU_P = 1; // FAKE atu PRESENT FOR TEST
  
  MODE = 0;                                                       // Always begin with amplifier OFF

  DrawMeter();                                                      // Draw initial screens
  DrawHome();
  
  Tft.LCD_SEL = 0;
  Tft.lcd_fill_rect(20, 82, 25, 10, GOOD_LED);                      // Fill status LED's with green
  Tft.lcd_fill_rect(84, 82, 25, 10, GOOD_LED);
  if (ATTN_ST == 0) Tft.lcd_fill_rect(148, 82, 25, 10, GOOD_LED);
  Tft.lcd_fill_rect(212, 82, 25, 10, GOOD_LED);
  Tft.lcd_fill_rect(276, 82, 25, 10, GOOD_LED);

  while(ts1.touched());                                             // Wait till screens are not touched
  while(ts2.touched());

  DC_DATA = 0;                                                      // Initialise flag and display strings for DC power
  strcpy(DC_TXT, "   ");
  strcpy(ODC_TXT, "   ");
  strcpy(EF_TXT, "   ");
  strcpy(OEF_TXT, "   ");
  fwd_pk = 0;

  Tft.LCD_SEL = 0; 
  Tft.lcd_draw_v_line(F_bar, 46, 22, MTR_BAR);                      // Draw initial meter zero bar
  Tft.lcd_draw_v_line(F_bar+1, 46, 22, MTR_BAR);
  Tft.lcd_draw_v_line(F_bar+2, 46, 22, MTR_BAR);
  Tft.lcd_draw_v_line(F_bar+3, 46, 22, MTR_BAR);

  TR_wait.restart();                                                // Start program Chronos
  PTT_wait.restart();
  swr_wait.restart();
  drv_wait.restart();
  Peak_hold.restart();
  TX_time.restart();
                                                         
  Timer1.initialize(50000); 
  Timer1.attachInterrupt(SleepingDog);
  interrupts();     
 

  watchdog.enable(Watchdog::TIMEOUT_120MS);                         // Watchdog timeout 120 milliseconds

  #ifdef TOPBAND
    TB = true;
  #endif
  
}

//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS




//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
void SleepingDog(void){

    watchdog.reset();                                                           // Reset watchdog timer once each loop 
}
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII



//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
// Main program loop

void loop()
{

  #ifdef DEVELOPMENT
    PORTE = PORTE ^ B00100000;                                                      // Toggle scope trigger to measure main loop timing
  #endif                                                                            // trig on Digital out 3 (TRIP_FWD on schematic)

  if(!TX) {
    DC_DATA = 0;                                                                    // Re-initialize DC power data display
    fwd_pk = 0;
  }
  if(!digitalRead(COR_DET)) {                                                       // Keep drive timer zeroed when no RF drive present.
    drv_wait.restart();
  }
  
  RF_Sense();                                                                       // Read RF power values
  
  if (MODE == 1 && SCREEN == 0 && BAND != 0 && TUNING != 1 && !trip){               // Amp ON . Must be ready for transmit
    if ((PTT_wait.hasPassed (Debounce)) && (digitalRead(PTT_DET))) {                // Debounce and switch to TX
      if(!TX) {
        Switch_to_TX();                                                
        TX_time.restart();                                                          // TX timer runs only during transmit
      }
      PTT_wait.restart();                                                           // Reset debounce and idle timers while TX active
      TR_wait.restart();    
    }
    if ((PTT_wait.hasPassed (Debounce)) && !(digitalRead(PTT_DET))                  // Debounce and switch to RX
    && !(digitalRead(COR_DET)) && TR_wait.hasPassed(TR_dly) ) {
      if(TX) {
        Switch_to_RX();
        TX_time.stop();                                                             // TX timer runs only during transmit
      }
      PTT_wait.restart();                                                           // Reset debounce and T/R timers in RX
    }
    if(!(digitalRead(PTT_DET)) && digitalRead(COR_DET)){                            // Do T/R switching without PTT via RF sensing
      if(!TX) {
        Switch_to_TX();
        TX_time.restart();                                                              // TX timer runs only during transmit
      }
    }
    if(digitalRead(COR_DET)) {                                                      // So long as RF is present 
      TR_wait.restart();                                                            // keep resetting T/R and Idle timers
    }
  }


  if (MODE == 0 && SCREEN == 0 && BAND != 0 && TUNING != 1 && !trip){               // AMP OFF. Just update status LED
    if ((PTT_wait.hasPassed (Debounce)) && (digitalRead(PTT_DET))) {                // according to PTT line
      if (!VPTT) {
        VPTT = true;
        DrawTxPanel(ALRM_LED);                                                      // Show PTT present
      }
      PTT_wait.restart();
      TR_wait.restart();    
    }
    if ((PTT_wait.hasPassed (Debounce)) && !(digitalRead(PTT_DET))                  // PTT and RF off
    && !(digitalRead(COR_DET)) && TR_wait.hasPassed(TR_dly) ) {
      if (VPTT) {
        VPTT = false;
        DrawTxPanel(IDLE_LED);                                                      // Show PTT not present
      }
      PTT_wait.restart();                                                           // Reset debounce and T/R timers in RX
    }
    if(!(digitalRead(PTT_DET)) && digitalRead(COR_DET)){                            // Do RF sensing
      if (!VPTT) {
        VPTT = true;
        DrawTxPanel(COR_TXT);                                                       // Show PTT present
      }
    }
    if(digitalRead(COR_DET)) {                                                      // So long as RF is present 
      TR_wait.restart();                                                            // keep resetting T/R and Idle timers
    }
  }

  if (MODE == 0 && (digitalRead(COR_DET)) && !frq_chkd) ReadFreq();                  // Auto band setting only when amp off
  if (MODE == 0 && !(digitalRead(COR_DET))) frq_chkd = false;                        // Prepare for next auto band setting
  

  if((analogRead(INPUT_RF) > 3) && !trip){
    int VSWR = Read_Power(vswr);                                                    // Check current VSWR
      if (VSWR < 15) vswr_col = GOOD_LED;                                           // VSWR color green at 1:1
      if (VSWR >= 18) vswr_col = WARN_LED;                                          // Above 1.8:1 swr display yellow
      if (VSWR >= 25) vswr_col = ALRM_LED;                                          // Above 2.5:1 swr display red      
    if (VSWR >= 10 && (VSWR > (OVSWR + 1) || VSWR < (OVSWR - 1))) {
      snprintf(RL_TXT, sizeof(RL_TXT), "%2d.%d", VSWR/10, VSWR % 10);
      OVSWR = VSWR;
    }
  
    if (strcmp(ORL_TXT, RL_TXT) != 0 && !do_swr) {                                  // If swr changing and not already erased
      Tft.LCD_SEL = 0;
//      Tft.lcd_fill_rect(66, 199, 76, 26, BG_col);                                   // Erase stale SWR display (slow)
      Tft.drawString((uint8_t*)ORL_TXT, 70, 203, 2, BG_col);                        // Erase stale SWR display
      swr_wait.restart();
      do_swr = true;
    }
    
    if (swr_wait.hasPassed(100) && do_swr) {                                        // Wait for tft display to erase
      Tft.LCD_SEL = 0;
      Tft.drawString((uint8_t*)RL_TXT, 70, 203, 2, vswr_col);                       // then display SWR value once only in color
      strcpy(ORL_TXT, RL_TXT);
      do_swr = false;
    }
  }
  
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// These trip setting routines run once every (a_count) main process loops
  
  if (++a_count == 2){
    unsigned int f_yel = 600, f_red = 660;
    a_count = 0;

    if(BAND == 10 && !TB){
      f_yel = 200;
      f_red = 250;
    }
    
    if (f_tot > f_yel && F_alert == 1) {F_alert = 2;}
    if (f_tot > f_red) {F_alert = 3; trip_set();}
    
    if (F_alert != OF_alert){
      OF_alert = F_alert;
      r_col = GOOD_LED;
      if (F_alert == 2) r_col = WARN_LED;
      if (F_alert == 3) r_col = ALRM_LED;
      Tft.LCD_SEL = 0;
      Tft.lcd_fill_rect(20, 82, 25, 10, r_col);                               // Fill FWD status LED with alert status colour
    }

    if (r_tot > 300 && R_alert == 1) {R_alert = 2;}
    if (r_tot > 500) {
      R_alert = 3; 
      vswr_col = RED; 
      trip_set();
    }
    
    if (R_alert != OR_alert){
      OR_alert = R_alert;
      unsigned int r_col = GOOD_LED;
      if (R_alert == 2) r_col = WARN_LED;
      if (R_alert == 3) r_col = ALRM_LED;
      Tft.LCD_SEL = 0;
      Tft.lcd_fill_rect(84, 82, 25, 10, r_col);                               // Fill RFL status LED with alert status colour
    }
    
    if (d_tot > 900 && D_alert == 1) {D_alert = 2;;}                           // Drive levels vs alert conditions
    if (d_tot > 1100) {D_alert = 3;;}
    if (ATTN_ST == 1) D_alert = 0;
    if (!TX) D_alert = 0;
    
    if (D_alert != OD_alert){
      OD_alert = D_alert;
      unsigned int r_col = GOOD_LED;
      if (ATTN_ST == 1) r_col = DGRAY;
      if (D_alert == 2) r_col = WARN_LED;
      if (D_alert == 3) {
        r_col = ALRM_LED;
        if (TX) trip_set();
      }
      Tft.LCD_SEL = 0;
      Tft.lcd_fill_rect(148, 82, 25, 10, r_col);                              // Fill DRV status LED with alert status colour
    }

    dc_vol = Read_Voltage();
    V_alert = 1;
    if (dc_vol > 2500) {V_alert = 3; trip_set();}                             // Trip amp if VDD rises above 62v
    if (dc_vol < 2200) V_alert = 2;                                           // Indicate low VDD yel = VDD < 55v
    if (dc_vol < 2000) V_alert = 3;                                           // red = VDD < 50v
    
     
    if (V_alert != OV_alert){
      OV_alert = V_alert;
      unsigned int r_col = GOOD_LED;
      if (V_alert == 2) r_col = WARN_LED;
      if (V_alert == 3) r_col = ALRM_LED;
      Tft.LCD_SEL = 0;
      Tft.lcd_fill_rect(212, 82, 25, 10, r_col);                              // Fill VDD status LED with alert status colour
    }

    dc_cur = Read_Current();
    int MC1 = 180 * MAX_CUR;
    int MC2 = 200 * MAX_CUR;
    if (dc_cur > MC1 && I_alert == 1) I_alert = 2;
    if (dc_cur > MC2) {I_alert = 3; trip_set();}
      
    if (I_alert != OI_alert){
      OI_alert = I_alert;
      unsigned int r_col = GOOD_LED;
      if (I_alert == 2) r_col = WARN_LED;
      if (I_alert == 3) r_col = ALRM_LED;
      Tft.LCD_SEL = 0;
      Tft.lcd_fill_rect(276, 82, 25, 10, r_col);                            // Fill IDD status LED with alert status colour
    }
      
    if (t_ave > temp_utp) SetFanSpeed(++FAN_SP);                            // Adjust fan speed according to temperature
    else if(t_ave < temp_dtp) SetFanSpeed(--FAN_SP);
  }
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
// These routines run once every (t_count) main process loops
   
  if (++t_count == 2){                                                        // Scale and format measured data for display
    t_count = 0;                                                              // and draw bar graph according to which
    
    f_pw = Read_Power(fwd_p);
    if(f_pw > fwd_pk) fwd_pk = f_pw;                                          // Track peak forward power
                                                                              // meter select button is currently selected
    if (MeterSel == 1) {                                                      
      F_bar = map(f_pw, 0, 500, 15, 296);                                     // Bar graph runs 19 to 300. Last 4 pixels for peak indicator
    }
    else if (MeterSel == 2) {                                                 // Reflected power
      r_pw = Read_Power(rfl_p);
      F_bar = map(r_pw, 0, 500, 15, 296);
    }
    else if (MeterSel == 3) {                                                 // Drive power
      d_pw = Read_Power(drv_p);
      F_bar = map(d_pw, 0, 100, 15, 296);
    }
    else if (MeterSel == 4) {                                                 // DC Voltage
      F_bar = map(dc_vol, 0, 2400, 15, 296);
    }
    else {                                                                    // DC Current
      F_bar = map(dc_cur, 0, 4000, 15, 296);
    }

    
    Tft.LCD_SEL = 0;
    F_bar = constrain(F_bar, 15, 296);                                        // Do not overrun bounds of bargraph display

    if(MODE == 1 || frq_chkd || MeterSel > 3){                                // Limit false drive pwr peak during frq measurement
      if (F_bar > OF_bar) {                                                   // When bar is increasing
        Tft.lcd_draw_v_line(OF_bar, 46, 22, MTR_BKG);
        Tft.lcd_draw_v_line(OF_bar+1, 46, 22, MTR_BKG);                       // Erase old bar
        Tft.lcd_draw_v_line(OF_bar+2, 46, 22, MTR_BKG);
        Tft.lcd_draw_v_line(OF_bar+3, 46, 22, MTR_BKG);
        Tft.lcd_draw_v_line(F_bar, 46, 22, MTR_BAR);                          // Draw new bar
        Tft.lcd_draw_v_line(F_bar+1, 46, 22, MTR_BAR);
        Tft.lcd_draw_v_line(F_bar+2, 46, 22, MTR_BAR);
        Tft.lcd_draw_v_line(F_bar+3, 46, 22, MTR_BAR);
        OF_bar = F_bar;
          
          if(Hold == false || F_bar > F_pk){                                  // Add a peak indicator in red
            Tft.lcd_draw_v_line(F_pk+4, 46, 22, MTR_BKG);                     // Erase old peak
            Tft.lcd_draw_v_line(F_pk+5, 46, 22, MTR_BKG);
            Tft.lcd_draw_v_line(F_pk+6, 46, 22, MTR_BKG);
            Tft.lcd_draw_v_line(F_pk+7, 46, 22, MTR_BKG);
            Tft.lcd_draw_v_line(F_bar+4, 46, 22, MTR_PK);                     // Draw new peak
            Tft.lcd_draw_v_line(F_bar+5, 46, 22, MTR_PK);
            Tft.lcd_draw_v_line(F_bar+6, 46, 22, MTR_PK);
            Tft.lcd_draw_v_line(F_bar+7, 46, 22, MTR_PK);
            F_pk = F_bar;
            Peak_hold.restart();
          }
      }
    }
    if (F_bar == OF_bar){                                                     // Expire peak if bar steady
      Hold = true;
    }
      
    if (F_bar < OF_bar){                                                      // Redraw bargraph if decreasing
      Hold = true;                                                            // Hold the peak indication
      Tft.lcd_draw_v_line(OF_bar, 46, 22, MTR_BKG);
      Tft.lcd_draw_v_line(OF_bar+1, 46, 22, MTR_BKG);                         // Erase old bar
      Tft.lcd_draw_v_line(OF_bar+2, 46, 22, MTR_BKG);
      Tft.lcd_draw_v_line(OF_bar+3, 46, 22, MTR_BKG);
      Tft.lcd_draw_v_line(F_bar, 46, 22, MTR_BAR);                            // Draw new bar
      Tft.lcd_draw_v_line(F_bar+1, 46, 22, MTR_BAR);
      Tft.lcd_draw_v_line(F_bar+2, 46, 22, MTR_BAR);
      Tft.lcd_draw_v_line(F_bar+3, 46, 22, MTR_BAR);
      OF_bar = F_bar;
    }                                                                         // -- not an ideal solution but helps.

    if (Hold && Peak_hold.hasPassed(5000)){                                   // Erase peak indicator after expiry time
      Hold = false;        
      Tft.lcd_draw_v_line(F_pk+4, 46, 22, MTR_BKG);                           // Erase stale peak indicator
      Tft.lcd_draw_v_line(F_pk+5, 46, 22, MTR_BKG);
      Tft.lcd_draw_v_line(F_pk+6, 46, 22, MTR_BKG);
      Tft.lcd_draw_v_line(F_pk+7, 46, 22, MTR_BKG);
    }

    if (Bias_Meter == 1){
      int Bcurr = Read_Current() * 5;
      if (Bcurr != oBcurr){
        oBcurr = Bcurr;
        Tft.LCD_SEL = 1;
        Tft.drawString((uint8_t*)Bias_buff, 65, 80, 2, BG_col);               // Erase stale display
        snprintf(Bias_buff, sizeof(Bias_buff), "   %4d mA", Bcurr);
        Tft.drawString((uint8_t*)Bias_buff, 65, 80, 2, ACT_TXT);
      }
    }


    if (t_disp++ == 200){                                                     // Slow temperature update to once per 200 loops
      t_disp = 0;                                                             // One ADC LSB = 0.2 deg C
      t_tmp = Ana_Read(14);
      t_tmp = map(t_tmp, 0, 1023, 5, 2000);                                   // Scale analog voltage reading
      t_smth = temperature.filter(t_tmp);
      t_ave = (unsigned) (t_smth * 1);                                        // convert long to unsigned
      
      unsigned int t_color = GOOD_LED;                                        // set temperature display colour according to alert level
      if (t_ave > 500) t_color = WARN_LED;
      if (t_ave> 650) t_color = ALRM_LED;
      if (t_ave> 700 && TX) trip_set();
      
      t_read = (TMP_UNIT ? t_ave : ((t_ave * 9) / 5) + 320);
      t_read /= 10;
      if (t_read != otemp){
        otemp = t_read;
        Tft.LCD_SEL = 0;
        Tft.drawString((uint8_t*)TEMPbuff, 237, 203, 2, BG_col);              // Erase stale display
        snprintf(TEMPbuff, sizeof(TEMPbuff), 
          "%3d&%c", t_read, TMP_UNIT ? 'C' : 'F');
        Tft.drawString((uint8_t*)TEMPbuff, 237, 203, 2, t_color);
      }
    }
    
    if(TX && !DC_DATA){                                                       // Measure DC power and display
      Tft.LCD_SEL = 0;
      Tft.drawString(ODC_TXT, 75, 153, 2, BG_col);                            // Erase stale DC power text
      Tft.drawString(OEF_TXT, 248, 153, 2, BG_col);                           
      if(TX_time.hasPassed(1000)){                                            // Wait for sustained power before display
        long dc_pwr = Read_DCPwr();
        if(dc_pwr > fwd_pk){
          long amp_eff = (fwd_pk * 100) / dc_pwr;
          ltoa(dc_pwr,DC_TXT,10);
          ltoa(amp_eff,EF_TXT,10);
          Tft.drawString(DC_TXT, 75, 153, 2, ACT_TXT);                        // Display DC power        
          Tft.drawString(EF_TXT, 248, 153, 2, ACT_TXT);                       // Display amplifier efficiency
          strcpy(ODC_TXT, DC_TXT);
          strcpy(OEF_TXT, EF_TXT);                                            
          DC_DATA = 1;                                                        // Display DC stats only once per TX cycle
        }
      }
    }
    if (dc_vol < 50){                                                         // On DC power over limit LTC2945 will cut Vdd
      DrawPanel(63, 146, 80, 30);
      Tft.drawString("OVER", 75, 153, 2, RED);
      trip_set();  
    }
  }
// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
  

// These routines run every main process loop


  if (!TX && XCVR == xft817){                                               // Yaesu FT-817 xcvr type band selection
    byte nFT817 = FT817det();
    while (nFT817 != FT817det())nFT817 = FT817det();
    if (nFT817 != 0) FTband = nFT817;
    if (FTband != BAND){
      BAND = FTband;
      SetBand();
    }
  }


  
  if (!TX && XCVR == xxieg){                                                // Xiegu xcvr type band selection
    byte nXieg = Xiegudet();
    while (nXieg != Xiegudet())nXieg = Xiegudet();
    if (nXieg != 0) FTband = nXieg;
    if (FTband != BAND){
      BAND = FTband;
      SetBand();
    }
  }


  
  if (!TX && XCVR == xelad){                                                // Elad xcvr type band selection
    byte nElad = Eladdet();
    while (nElad != Eladdet())nElad = Eladdet();
    if (nElad != 0) FTband = nElad;
    if (FTband != BAND){
      BAND = FTband;
      SetBand();
    }
  }

 
  if (ts1.touched()) {                                                            // Left touch screen touch detected?
    byte ts_key = getTS(1);
      switch (ts_key){
      case 5: case 10: MeterSel = 1; break;                                       // FWD power 
      case 6: case 11: MeterSel = 2; break;                                       // REV power
      case 7: case 12: MeterSel = 3; break;                                       // DRV power
      case 8: case 13: MeterSel = 4; break;                                       // PS Volts
      case 9: case 14: MeterSel = 5; break;                                       // PS Amps
    }
    if (OMeterSel != MeterSel){
      DrawButtonUp(OMeterSel);
      DrawButtonDn(MeterSel);
      OMeterSel = MeterSel;
      EEPROM.write(eemetsel, MeterSel);
    }
    while (ts1.touched());                                                        // Await left touch screen released
  }




  if (ts2.touched()) {                                                            // Right touch screen touch detected?
    byte ts_key = getTS(2);    
    if (SCREEN == 0){                                                             // State machine for default screen (SCREEN 0)
      switch (ts_key){
      case 0: case 1: case 5: case 6:                                             // MODE selection
        if (analogRead(INPUT_RF) == 0){                                           // Drive must be removed to reset a trip condx
          if (++MODE == 2) MODE = 0;
          if (trip == 1) {                                                        // Reset trip condition and displays
            fault = 0;
            trip = 0;
            Tft.LCD_SEL = 0;
            DrawPanel(63, 146, 80, 30);                                           // Initialize DC power display window
            Tft.drawString("W", 118, 153, 2, ACT_TXT);
            DrawPanel(230, 146, 80, 30);                                          // Initialize efficiency display window
            Tft.drawString("%", 280, 153, 2, ACT_TXT);
            F_alert = 1; R_alert = 1; D_alert = 1; V_alert = 1; I_alert = 1;
            OF_alert = 0; OR_alert = 0; OD_alert = 0; OV_alert = 0; OI_alert = 0;
          }
          Switch_to_RX ();
          Tft.LCD_SEL = 0;
          DrawPanel(63, 196, 80, 30);                                             // Erase stale swr display text
          delay(200);                                                             // wait for display           
          do_swr = false;
          DrawMode();
        }
        break;
        
      case 8:                                                                     // BAND UP selection
        if (!TX){
          if (++BAND >= 11) BAND = 1;
          SetBand();
        }
        break;
      case 9:                                                                     // BAND DN selection
        if (!TX){
          if (--BAND == 0) BAND = 10;
          if (BAND == 0xff) BAND = 10;
          SetBand();
        }
        break;
        
      case 10: case 11: case 15: case 16:                                         // ANT selection
        if (!TX){
          if (++ANTSEL[BAND] == 3) ANTSEL[BAND] = 1;
          EEPROM.write(eeantsel+BAND, ANTSEL[BAND]);
          if (ANTSEL[BAND] == 1) SEL_ANT1;
          if (ANTSEL[BAND] == 2) SEL_ANT2;
          DrawAnt();
          Tft.LCD_SEL = 0;
          DrawPanel(63, 196, 80, 30);                                             // Erase stale swr display text
          delay(200);                                                             // wait for display
          strcpy(ORL_TXT, RL_TXT);           
          do_swr = false;
        }
        break;
        
      case 18: case 19:                                                           // ATU on/off selection
        if (ATU_P && !TX && !TUNING){
          if (++ATU == 2) ATU = 0;
          ATUB[BAND] = ATU;                                                       // Update per-band array of ATU button state
          DrawATU();
          EEPROM.write(eeatub+BAND, ATU);                                         // Update array in EEPROM too
        }
        break;
        
      case 12:                                                                    // Menu selection without ATU
        if (!ATU_P && !TX){
          Tft.LCD_SEL = 1;
          Tft.lcd_clear_screen(BG_col);
          DrawMenu();
          SCREEN = 1;
        }
        break;
        
      case 7:                                                                     // Menu selection with ATU
        if (ATU_P && !TX){
          Tft.LCD_SEL = 1;
          Tft.lcd_clear_screen(BG_col);
          DrawMenu();
          SCREEN = 1;
        }
        break;
        
      case 17:                                                                    // Automatch request 
        if (ATU_P && !TUNING){
          Tune_button();  
        }
      }
    }
    
    else{                                                                         // State machine for Menu screen (SCREEN = 1)
      switch (ts_key){
      case 0: case 1:                                                             // Left arrow (Menu selection)
        if (menuSEL == 0){
          Tft.LCD_SEL = 1;
          Tft.drawString((uint8_t*)menu_items[menu_choice], 65, 20, 2, BG_col);   // Erase stale menu and item
          Tft.drawString((uint8_t*)item_disp[menu_choice], 65, 80, 2, BG_col);
          menu_choice--;
          if (menu_choice < 0) menu_choice = menu_max;
         if ((menu_choice == mATTN) && (!ATTN_P)) menu_choice--;                  // Skip ATTN setting if no atten installed
          Tft.drawString((uint8_t*)menu_items[menu_choice], 65, 20, 2, ACT_TXT);  // Draw new menu in white
          Tft.drawString((uint8_t*)item_disp[menu_choice], 65, 80, 2, FG_col);    // Draw new item in grey
        }
        break;

        
      case 3: case 4:                                                             // Right arrow (Menu selection)
        if (menuSEL == 0){
          Tft.LCD_SEL = 1;
          Tft.drawString((uint8_t*)menu_items[menu_choice], 65, 20, 2, BG_col);   // Erase stale menu and item
          Tft.drawString((uint8_t*)item_disp[menu_choice], 65, 80, 2, BG_col);
          menu_choice++;
          if ((menu_choice == mATTN) && (!ATTN_P)) menu_choice++;                 // Skip ATTN setting if no atten installed
          if (menu_choice > menu_max) menu_choice = 0;
          Tft.drawString((uint8_t*)menu_items[menu_choice], 65, 20, 2, ACT_TXT);  // Draw new menu in active colour
          Tft.drawString((uint8_t*)item_disp[menu_choice], 65, 80, 2, FG_col);    // Draw new item in grey
        }
        break;

        
      case 5: case 6:                                                             // Left arrow (Item selection)
        if (menuSEL == 1) menuFunction(menu_choice, 0);
        break;
        
      case 8: case 9:                                                             // Right arrow (Item selection)
        if (menuSEL == 1) menuFunction(menu_choice, 1);
        break;
        
      case 7: case 12:                                                            // Select button
        menuSelect();
        break;
              
      case 17:                                                                    // Exit button
        if (menu_choice == mTheme){                                               
        Set_Theme();                                                              // When changing themes:
        OF_alert = 2; F_alert = 1;                                                // Re-initialize alert LED's
        OR_alert = 2; R_alert = 1;
        OD_alert = 2; D_alert = 1;
        OV_alert = 2; V_alert = 1;
        OI_alert = 2; I_alert = 1;
        otemp = (t_read -1);                                                      // Trigger a temperature display update.       
        DrawMeter();                                                              // Re-draw left screen.
        }
        if (menu_choice == mSETbias){                                             // When done setting amp bias:
          BIAS_OFF                                                                // return amp to running parameters.
          Send_RLY(SR_DATA);
          MODE = oMODE;
          MAX_CUR = 20;
          Bias_Meter = 0;
        }
        Tft.LCD_SEL = 1;                                                          // Re-draw right screen.
        Tft.lcd_clear_screen(BG_col);
        SCREEN = 0;
        DrawHome();
        Tft.LCD_SEL = 0;
        Tft.lcd_reset();
      }  
    }  
    while (ts2.touched());                                                        // Await right touch screen released
  }



  if (ATU_P){                                                                     // When there is an ATU present
  bool atu_bsy = digitalRead(ATU_BUSY);                                           // end the ATU tune sequence when 
  if (TUNING && !atu_bsy) Tune_End();                                             // the ATU says so.
  }


  
  while (Serial2.available()){                                                    // Look for data on ACC serial
     unsigned char cx = Serial2.read();
      rxbuff2[uartPtr2] = cx;
 
      if(rxbuff2[uartPtr2] == 0x3B){
        uartGrabBuffer2();
        findBand(2);
      }
      if(++uartPtr2 > 127) {
        uartPtr2 = 0;
      }
  }



  while (Serial.available()){                                                     // Look for data on USB serial
      rxbuff[uartPtr] = Serial.read();                                            // Storing read data
      if(rxbuff[uartPtr] == 0x3B){
        uartGrabBuffer();
        findBand(1);
      }
      if(++uartPtr > 127) {
        uartPtr = 0;
      }
  }
  if (fault) DrawMode();
}

//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL









// Subroutine section

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Switches state to TX. 
// TX/RXindicator -> RED.

void Switch_to_TX(void){
  PTT = 1; 
  RF_ACTIVE                                                                 // Bypass relay -> ENGAGE
  byte SRSend_RLY = SR_DATA + 0x10;                                         // Set bit 4 of TPIC6B595 shift register AMP_RLY ON
  RELAY_CS_LOW
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(SRSend_RLY);
  SPI.endTransaction();
  RELAY_CS_HIGH    
  BIAS_ON
  TX = 1;
  F_alert = 1; R_alert = 1; D_alert = 1; V_alert = 1; I_alert = 1;            // Initialize alert flags
  OF_alert = 0; OR_alert = 0; OD_alert = 0; OV_alert = 0; OI_alert = 0;
  if(digitalRead(PTT_DET)) DrawTxPanel(ALRM_LED);                             // TX/RX LED to RED on PTT
  else DrawTxPanel(COR_TXT);                                                  // TX/RX LED to carrier detect colour
  RESET_PULSE                                                                 // Reset DC power enable flip flop
  s_disp = 19;                                                                // Get ready to display SWR immediately
  strcpy(ORL_TXT, RL_TXT);
  do_swr = false;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Switches state to RX. 
// TX/RXindicator -> GREEN.
// Draw last VSWR according to vswr in alert level colour
 
void Switch_to_RX (void){
  PTT = 0;
  BIAS_OFF
  TX = 0;
  f_tot = 0; r_tot = 0; d_tot = 0;
  byte SRSend_RLY = SR_DATA;                                                  // Clear bit 4 of TPIC6B595 shift register AMP_RLY OFF
  RELAY_CS_LOW
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(SRSend_RLY);
  SPI.endTransaction();
  RELAY_CS_HIGH    
  RF_BYPASS                                                                   // Bypass relay -> BYPASS
  DrawTxPanel(GOOD_LED);                                                      // TX/RX LED to GREEN
  trip_clear();
  RESET_PULSE                                                                 // Reset power enable flip flop
  Tft.LCD_SEL = 0;
  Tft.drawString((uint8_t*)ORL_TXT, 70, 203, 2, vswr_col);                    // Display last SWR value once only in color
  do_swr = false;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Yaesu FT-817 type band selection

byte FT817det(void){
    int ftv = Ana_Read(FT817_V);
    if (ftv < 98) return 10;
    if (ftv > 100 && ftv < 160) return 9;
    if (ftv > 164 && ftv < 225) return 7;
    if (ftv > 229 && ftv < 285) return 6;
    if (ftv > 289 && ftv < 345) return 5;
    if (ftv > 349 && ftv < 410) return 4;
    if (ftv > 414 && ftv < 475) return 3;
    if (ftv > 479 && ftv < 508) return 2;
    if (ftv > 512 && ftv < 605) return 1;
    return 0;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Elad type band selection

byte Eladdet(void){
    int ftv = Ana_Read(FT817_V);
    if (ftv < 118) return 10;
    if (ftv > 130 && ftv < 200) return 9;
    if (ftv > 212 && ftv < 282) return 7;
    if (ftv > 295 && ftv < 365) return 6;
    if (ftv > 380 && ftv < 450) return 5;
    if (ftv > 462 && ftv < 532) return 4;
    if (ftv > 545 && ftv < 615) return 3;
    if (ftv > 630 && ftv < 700) return 2;
    if (ftv < 780)return 1;
    return 0;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Xiegu type band selection

byte Xiegudet(void){
    int ftv = Ana_Read(FT817_V);
    if (ftv < 136) return 10;
    if (ftv < 191) return 9;
    if (ftv < 246) return 8;
    if (ftv < 300) return 7;
    if (ftv < 355) return 6;
    if (ftv < 410) return 5;
    if (ftv < 465) return 4;
    if (ftv < 520) return 3;
    if (ftv < 574) return 2;
    return 1;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Counts RF input frequency and determines band selection

void ReadFreq(void){
  unsigned long frq = 0;
  int cnt = 0;
  byte same_cnt = 0;
  byte band_read;
  byte last_band = 0;

  SR_SAVE = SR_DATA;                                                        // Save current band selection
  BIAS_OFF
  RF_ACTIVE                                                                 // Bypass relay -> ENGAGE
  byte SRSend_RLY = SR_DATA + 0x10;                                         // Set bit 4 of TPIC6B595 shift register (AMP_RLY OFF: RF to counter)
  Send_RLY(SRSend_RLY);                                                     // This routes RF to the prescaler.
  
  FreqCount.begin(16);                                                      // Setup gate time for frequency counting (milliseconds)
   
  while (cnt < 6 && digitalRead(COR_DET)){                                  // Make sure RF is present for the measurements
    while (!FreqCount.available());                                         // Wait for frequency reading 
    frq = FreqCount.read();
    band_read = 0;
    if (frq > 1750 && frq < 2100) band_read = 10;
    else if (frq > 3000 && frq < 4500) band_read = 9;
    else if (frq > 4900 && frq < 5700) band_read = 8;
    else if (frq > 6800 && frq < 7500) band_read = 7;
    else if (frq > 9800 && frq < 11000) band_read = 6;
    else if (frq > 13500 && frq < 15000) band_read = 5;
    else if (frq > 17000 && frq < 19000) band_read = 4;
    else if (frq > 20500 && frq < 22000) band_read = 3;
    else if (frq > 23500 && frq < 25400) band_read = 2;
    else if(frq > 27500 && frq < 30000) band_read = 1;

    if (band_read == last_band && last_band != 0) {                       // Sanity check: must have two consecutive same readings
      same_cnt++;
    }
    if (same_cnt > 2){
      BAND = last_band;
      cnt = 99;
    }
    last_band = band_read;
    cnt++;
  }
  FreqCount.end();
  RF_BYPASS                                                               // Bypass relay -> BYPASS
  if (BAND != OBAND) {
    SetBand();                                                            // If on new band switch band relays 
  }
  else Send_RLY(SR_SAVE);                                                 // Else return band select relays to previous setting
  frq_chkd = true;                                                   // set flag to prevent PTT/COR freq measurement
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Checks which touch screen has been touched and returns 0 (left) or 1 (right)

byte getTS(byte ts){
  uint16_t x, y;
  uint8_t key;
  TS_Point p;
  
  if (ts == 1) p = ts1.getPoint();
  if (ts == 2) p = ts2.getPoint();
    
  x = map(p.x, 3850, 400, 0, 5);
  y = map(p.y, 350, 3700, 0, 4);
//  z = p.z;
  key = x + 5 * y;
  
  return key;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sends serial data to I/O expander on LPF board to change LPF relay banks

void SetBand(void){
  byte sr_data = 0;
  if (BAND > 10) return;
  if (TX) return;                                                        // Must be in RX with valid band variable.

  switch (BAND){
    case 0:  sr_data = 0x00; Serial3.println("*B0"); break;
    case 1:  sr_data = 0x20; Serial3.println("*B1"); break;
    case 2:  sr_data = 0x20; Serial3.println("*B2"); break;
    case 3:  sr_data = 0x20; Serial3.println("*B3"); break;
    case 4:  sr_data = 0x08; Serial3.println("*B4"); break;
    case 5:  sr_data = 0x08; Serial3.println("*B5"); break;
    case 6:  sr_data = 0x04; Serial3.println("*B6"); break;
    case 7:  sr_data = 0x04; Serial3.println("*B7"); break;
    case 8:  sr_data = 0x02; Serial3.println("*B8"); break;
    case 9:  sr_data = 0x01; Serial3.println("*B9"); break;
    case 10: sr_data = 0x00; Serial3.println("*B10"); break; 
  }

  if (ATU_P && SCREEN == 0){
    ATU = ATUB[BAND];
    Tft.LCD_SEL = 1;
    Tft.lcd_fill_rect(121, 142, 74, 21, BG_col);                              // Erase stale ATU text window when present
    DrawATU();
  }

  SR_DATA = sr_data;

  if (BAND != OBAND){
    Send_RLY(SR_DATA);
    Tft.LCD_SEL = 1;
    DrawBand(OBAND, BG_col);                                                  // Erase stale band text
    OBAND = BAND;
    DrawBand(BAND, acolor);                                                   // Update band text
    EEPROM.write(eeband, BAND);                                               // Update band selection in NVRAM
    DrawAnt();
    delay(50);                                                                // Short delay for relays before continuing
  }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Controls cooling fan speed according to temperature.

void SetFanSpeed(byte FS){
  switch (FS){
    case 0: 
      digitalWrite(FAN1, LOW); 
      digitalWrite(FAN2, LOW);
      temp_utp = 400; temp_dtp = 00; 
      break;
    case 1: 
      digitalWrite(FAN1, HIGH);
      digitalWrite(FAN2, LOW); 
      temp_utp = 500; temp_dtp = 380; 
      break;
    case 2: 
      digitalWrite(FAN1, LOW);
      digitalWrite(FAN2, HIGH); 
      temp_utp = 550; temp_dtp = 420; 
      break;
    case 3: 
      digitalWrite(FAN1, HIGH); 
      digitalWrite(FAN2, HIGH);                
      temp_utp = 2500; temp_dtp = 480; 
  }  
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets port direction and pullup bits and controls serial polarity bit
// depending on XCVR menu selection

void SET_XCVR(byte s_xcvr){
    if (s_xcvr == xhobby || s_xcvr == xkx23) S_POL_REV
    else S_POL_NORM

    if (s_xcvr == xhobby){
      pinMode(TTL_PU, OUTPUT);
      digitalWrite(TTL_PU, HIGH);
    }  
    else {
      digitalWrite(TTL_PU, LOW);
      pinMode(TTL_PU, INPUT);
    }

    if (s_xcvr == xft817 || s_xcvr == xelad || s_xcvr == xxieg){
      item_disp[mACCbaud] = (char*)"  XCVR MODE ON  ";
      Serial2.end();
    }
    else{
      Set_Ser2(ACC_Baud);
    }
    if (s_xcvr == xic705){
      ACC_Baud = 2;
      EEPROM.write(eeaccbaud, ACC_Baud);
      Set_Ser2(ACC_Baud);
    }
    if (s_xcvr == xmodem){
      ACC_Baud = 3;
      EEPROM.write(eeaccbaud, ACC_Baud);
      Set_Ser2(ACC_Baud);
    }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sends required data to serial I/O epander on low pass filter board

void Send_RLY(byte data){
  noInterrupts();
  RELAY_CS_LOW
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  SPI.transfer(data);
  SPI.endTransaction();
  RELAY_CS_HIGH
  interrupts();
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reads required analog input

int Ana_Read(byte Pin){
  noInterrupts();
  int a = analogRead(Pin);
  interrupts();
  return a;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reads RF power inputs
// In fast CW keying sequential analog reads which take time can lead to errors
// in SWR calculations.  This code tries to reduce those errors.

void RF_Sense(void){
  long s_calc;
  long p_fwd;
  long p_rev;
  long p_drv;
  
  if((digitalRead(PTT_DET)) || (digitalRead(COR_DET))){             // Power read only when RF present 
    p_rev =  analogRead(RFL_RF);  
  }
  else p_rev = 0;
  
  if((digitalRead(PTT_DET)) || (digitalRead(COR_DET))){             // Power read only when RF present 
    p_fwd =  analogRead(FWD_RF);  
  }
  else p_fwd = 0;
  
  if (p_fwd && p_rev && p_fwd > p_rev){  
    f_tot = fwd_pwr.filter(p_fwd * M_CORR / 100L);                // Filtered forward analog voltage
    r_tot = rfl_pwr.filter(p_rev * M_CORR / 100L);                // Filtered reflected analog voltage    
  }
  if((digitalRead(PTT_DET)) || (digitalRead(COR_DET))){                                                
    p_drv = analogRead(INPUT_RF);  
    d_tot = drv_pwr.filter(p_drv * long(d_lin[BAND]) / 100L);     // Filterd drive power voltage  
  }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets display theme color values

void Set_Theme(void){
  if(Theme == 0){                             // Normal theme
    BG_col = BLACK;                           // Background colour
    FG_col = LGRAY;                           // Foreground colour
    DK_BTN = DGRAY;                           // Button highlight
    LT_BTN = LGRAY;                           // Button highlight
    ACT_TXT = GREEN;                          // Active text colour
    PSV_TXT = LGRAY;                          // Passive text colour
    CHG_TXT = YELLOW;                         // Changing item text colour
    MTR_TXT = RED;                            // Meter text colour
    MTR_MRK = BLACK;                          // Meter mark colour
    MTR_SCL = WHITE;                          // Meter scale background
    MTR_BKG = BLACK;                          // Background of meter bar
    MTR_BAR = GREEN;                          // Background of meter bar
    MTR_PK = RED;                             // Peak indicator of meter bar
    A1_TXT = GREEN;                           // Antenna 1 text colour (per band)
    A2_TXT = BLUE;                            // Antenna 2 text colour (per band)
    SPLASH = WHITE;                           // Splash screen display colour
    GOOD_LED = GREEN;                         // Alert 1 colour
    WARN_LED = YELLOW;                        // Alert 2 colour
    ALRM_LED = RED;                           // Alert 3 colour
    IDLE_LED = DKGRY;                         // TX/RX status colour when amp off
    COR_TXT = BLUE;                           // Carrier detect colour  
  }
    
  if(Theme == 1){                             // Night theme
    BG_col = BLACK;                           // Background colour
    FG_col = RED;                             // Foreground colour
    DK_BTN = DKRED;                           // Button highlight
    LT_BTN = RED;                             // Button highlight
    ACT_TXT = DKGRN;                          // Active text colour
    PSV_TXT = DKRED;                          // Passive text colour
    CHG_TXT = DKYEL;                          // Changing item text colour
    MTR_TXT = BLACK;                          // Meter text colour
    MTR_MRK = BLACK;                          // Meter mark colour
    MTR_SCL = BRRED;                          // Meter scale background
    MTR_BKG = BLACK;                          // Background of meter bar
    MTR_BAR = GREEN;                          // Background of meter bar
    MTR_PK = RED;                             // Peak indicator of meter bar
    A1_TXT = DKGRN;                           // Antenna 1 text colour (per band)
    A2_TXT = DKBLU;                           // Antenna 2 text colour (per band)
    SPLASH = RED;                             // Splash screen display colour
    GOOD_LED = DKGRN;                         // Alert 1 colour
    WARN_LED = DKYEL;                         // Alert 2 colour
    ALRM_LED = RED;                           // Alert 3 colour    
    IDLE_LED = DKGRY;                         // TX/RX status colour when amp off
    COR_TXT = YELLOW;                         // Carrier detect colur  
  }
  
  if(Theme == 2){                             // Dark theme
    BG_col = BLACK;                           // Background colour
    FG_col = LTGRY;                           // Foreground colour
    DK_BTN = DKGRY;                           // Button highlight
    LT_BTN = LTGRY;                           // Button highlight
    ACT_TXT = DKGRN;                          // Active text colour
    PSV_TXT = DKGRY;                          // Passive text colour
    CHG_TXT = DKYEL;                          // Changing item text colour
    MTR_TXT = BLACK;                          // Meter text colour
    MTR_MRK = BLACK;                          // Meter mark colour
    MTR_SCL = DKGRY;                          // Meter scale background
    MTR_BKG = BLACK;                          // Background of meter bar
    MTR_BAR = GREEN;                          // Background of meter bar
    MTR_PK = RED;                             // Peak indicator of meter bar
    A1_TXT = DKGRN;                           // Antenna 1 text colour (per band)
    A2_TXT = DKBLU;                           // Antenna 2 text colour (per band)
    SPLASH = RED;                             // Splash screen display colour
    GOOD_LED = DKGRN;                         // Alert 1 colour
    WARN_LED = DKYEL;                         // Alert 2 colour
    ALRM_LED = RED;                           // Alert 3 colour 
    IDLE_LED = DKGRY;                         // TX/RX status colour when amp off
    COR_TXT = YELLOW;                         // Carrier detect colur  
  }
  
}

//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
  
