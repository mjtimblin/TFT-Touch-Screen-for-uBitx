#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <Wire.h>
#include <avr/io.h>
#include "userdefs.h"

//--------------------Installable Libraries-----------------------------------------------------------------------
#include <Rotary.h>          // https://github.com/brianlow/Rotary
#include <si5351.h>          // https://github.com/etherkit/Si5351Arduino
#include <Adafruit_GFX.h>    // Core graphics library located at adafuit website  https://github.com/adafruit/Adafruit-GFX-Library
#include <MCUFRIEND_kbv.h>   // https://github.com/prenticedavid/MCUFRIEND_kbv
#include "TouchScreen.h"     // https://github.com/adafruit/Touch-Screen-Library
#include <Bounce2.h>

// ---------------------------- Color definitions -----------------------------
#define BLACK       0x0000      /*   0,   0,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define GREY        0x7BEF      /* 128, 128, 128 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define PINK        0xF81F		  /* 255,   0, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */

#define DEBOUNCE_INTERVAL 25
#define MINPRESSURE 20
#define MAXPRESSURE 1000

void dispPos();
void setup_vfo_screen();
void display_msg(int xposn, String msg);
void init_eprom();
void read_eprom();
void set_vfo();
void set_bfo1();
void display_frequency();
void display_vfo();
void display_band();
void display_step();
void display_sideband();
void display_mem();
void display_bfo1();
void set_band();
void band_incr();
void band_decr();
void step_decr();
void step_incr();
void save_frequency();
void displ_rx();
void vfo_sel();
void ptt_ON();
void ptt_OFF();
void toggle_ptt();
void sideband_chg();
void vfo_to_mem();
void mem_decr();
void mem_incr();
void mem_to_vfo();
void bfo1_decr();
void bfo1_incr();
void bfo2_decr();
void bfo2_incr();
void save();
void bfo2_decr();
void bfo2_incr();
void read_ch();
void set_bfo2();
void write_ch();
void write_vfo_A();
void write_vfo_B();
//void write_vfo_M();
void displ_timeout_button();
void displ_split_button();
void scan_dn();
void scan_dn();
void scan_up();
void update_row5();
void check_CAT();

// most mcufriend shields use these pins and Portrait mode:
// **? can we auto define these pins
uint8_t YP;  // must be an analog pin, use "An" notation!
uint8_t XM;  // must be an analog pin, use "An" notation!
uint8_t YM;  // can be a digital pin
uint8_t XP;  // can be a digital pin

// Touch coordinates determined by one of the sample programs provided with touch screen library
uint16_t TS_LEFT;  // Touch Screen Left edge
uint16_t TS_RT;    // Touch Screen right edge
uint16_t TS_TOP;   // Touch Screen Top edge
uint16_t TS_BOT;   // Touch Screen Bottom edge

Si5351 si5351;
Rotary r = Rotary(ENCODER_A_PIN, ENCODER_B_PIN);
MCUFRIEND_kbv tft;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;

//--------------------------------------------------------------------------------------
boolean txstatus = false;     // Rx = False Tx = True
uint32_t bfo_A, bfo_B, bfo_M;  //The bfo frequency is added to or subtracted from the vfo frequency in the "Loop()"
uint32_t bfo2 = 11993900L;  // Fixed 12 MHz BFO2 Farhans 11996500 this value found by test
uint32_t bfo1, bfo1_USB = 56995000L , bfo1_LSB = 32994000L;   // Initial Values of BFO1 for USB or LSB
uint32_t vfo , vfo_tx; // vfo is displayed freq, vfo_tx is actual vfo on clock2 of Si5351
uint32_t vfo_A = 7050000L, vfo_B = 7130000L, vfo_M = 14000000L ; // temp values // either vfo A or B or mem channel is selected for output at any time
boolean vfo_A_sel = true, vfo_B_sel = false, vfo_M_sel = false; // true for vfo selected
boolean changed_f = 0;      // indicating need for updating display
boolean xch_M = 0; // flag for xchged mem in V > M
uint16_t sideband, sb_A, sb_B, sb_M;
uint16_t LSB = 1, USB = 2;
// display step size and radix
String step_sz_txt[] = {
   "   1Hz ",
   "   10Hz ",
   "  100Hz ",
   "   1kHz ",
   "  10kHz ",
   "  100kHz",
   "   1MHz "
};
uint32_t step_sz[] = {
   1L,
   10L,
   100L,
   1000L,
   10000L,
   100000L,
   1000000L
};
int step_index = 3;
uint32_t radix = 1000L;  //start step size - change to suit

// If the touch on touch screen is producing multiple touch effect increase this delay
uint16_t ts_delay = 80;    // delay between touch button to reduce sensitivity

// Transmit time out in seconds for reducing heating of the finals
uint16_t Tx_timeout = 20; // time in sec upto which Tx works continuosly then goes to Rx
bool Tx_timeout_mode = false;
uint16_t max_timeout = 60; // Max value of timeout in sec  decided by finals and their cooling

// PTT has two modes
// **** Only Toggle PTT in normal mode / in active_ptt mode PTT remains LOW during Tx
bool active_PTT_in = false; // if PTT remains continuously low during QSO make it true else false means "toggle PTT" on active low


uint8_t magic_no = 04; // used for checking the initialization of eprom or to reinitialize change this no.
// If mem not initialized there may be garbled display. In that case simply change this number and reload the prog

//------------EPROM memory related -----------------------------------------------------------
uint16_t max_memory_ch = 100;  // each ch takes 10-4=6 bytes *ver 3.1 shifted bfo1 to eprom
struct allinfo {
  uint32_t s_vfo;
  //uint32_t s_bfo;
  uint16_t s_sb;
} Allinfo;  // complete description of channel saved in mem

// Eprom content sequence: allinfo for vfoA, vfoB, mem1,2,3...
uint16_t address;  // temp address used by fns
allinfo ch_info;    // local copy
unsigned int memCh = 1;  // memory  channel  number (1,2,3...100) now.  Try names..??
int32_t Offset_VFO = 0, Offset_80m = 0, Offset_40m = 0, Offset_30m = 0, Offset_20m = 0, Offset_17m = 0, Offset_15m = 0, Offset_12m = 0, Offset_10m = 0; //in mem at 8-11, 12-15, etc...

int bnd_count = 2, old_band; // bnd_count=2 means 40m
int MAX_BANDS = 9;
// Band Limits and frequencies and their corresponding display on band button
volatile uint32_t F_MIN_T[9] = {100000UL,  3500000UL, 7000000UL, 10100000UL, 14000000UL, 18068000UL, 21000000UL, 24890000UL, 28000000UL};
volatile uint32_t  F_MAX_T[9] = {75000000UL,  3800000UL, 7200000UL, 10150000UL, 14350000UL, 18168000UL, 21450000UL, 24990000UL, 29700000UL};
String  B_NAME_T[] = {"  VFO", "  80m", "  40m", "  30m", "  20m", "  17m", "  15m", "  12m", "  10m" };
volatile uint32_t  VFO_T[9] = {9500000UL, 3670000UL, 7100000UL, 10120000UL, 14200000UL, 18105000UL, 21200000UL, 24925000UL, 28500000UL};
volatile int32_t offsets[9] = { Offset_VFO , Offset_80m , Offset_40m , Offset_30m , Offset_20m , Offset_17m , Offset_15m , Offset_12m , Offset_10m };
// offsets for different bands could be -ve also & to be found by experimentation for each rig
unsigned long F_MIN, F_MAX;
uint16_t SM_min = 100, SM_max = 1023 ; //  depending on input to Smeter input Analog pin 55 (A12) this should be adjsuted (by comparison or by ear)
uint16_t PM_min = 100, PM_max = 1023; // Power output
// -- eprom mem map in Ver 3.1 the beginning of mem used to save info from on screen adjustments. VFO infos after mem addr 100
uint16_t magic_no_address = 0;
uint16_t bfo1_USB_address = magic_no_address + sizeof(magic_no);
uint16_t bfo1_LSB_address = bfo1_USB_address + sizeof(bfo1_USB);
uint16_t bfo2_address = bfo1_LSB_address + sizeof(bfo1_LSB);
uint16_t PTT_type_address = bfo2_address + sizeof(bfo2);   // active_PTT_in (TRUE/FALSE)
uint16_t Tx_timeout_mode_address = PTT_type_address + sizeof(active_PTT_in); // Tx_timeout_mode ON/OFF
uint16_t TxTmO_Time_address = Tx_timeout_mode_address + sizeof(Tx_timeout_mode);  // time for Tx Time out in sec
uint16_t touch_sens_address = TxTmO_Time_address + sizeof(Tx_timeout);  // ts_delay for adjusting touch sensitivity
uint16_t offsets_base_address = touch_sens_address + sizeof(ts_delay); // all offsets for each band start here
uint16_t sm_min_address = offsets_base_address + sizeof(offsets); // min value for s meter
uint16_t sm_max_address = sm_min_address + sizeof(SM_min); // max value for s meter

uint16_t eprom_base_addr = 100;  //offsets_base_address + sizeof(offsets[]) ;   // from here all VFO details would be stored
//----------------------------------------------------------------------------------------------

unsigned long Tx_start_time = 0, rem_time = 0;
int diff;

uint16_t xpos, ypos;  //screen coordinates

uint16_t identifier;        // TFT identifier : can be found from the example programs in the TFT library
int wd;                     //= tft.width();
int ht;                     //= tft.height();
int displ_rotn = 1;         // 0 - normal, 1 - 90deg rotn (landscape), 2 - reverse, 3-rev LS  ** if the touch buttons do not match try changing here

// individual button x,y coordinates, height and width, some common params,
uint16_t buttonht, buttonwd, smwd, smx, smy, smht, firstrowy, vfox, vfoy, vfowd, vfoht;
uint16_t memx, memy, memwd, memht, txrx, txry, txrwd, txrht, frqx, frqy, frqwd, frqht;
uint16_t vfoABMx, vfoABMy, frq2x1, frq2x2, frq2y, bandx, bandy, bandwd, bandht;
uint16_t stpx, stpy, stpwd, stpht, sbx, sby, sbwd, sbht, vmx, vmy, vmwd, vmht, splx, sply, splwd, splht, svx, svy, svwd, svht;
uint16_t prsnx, prsny, prsnwd, prsnht,  nxsnx, nxsny, nxsnwd, nxsnht; // prev & nxt screen arrows
uint16_t f1x, f1y, f1wd, f1ht, f2x, f2y, f2wd, f2ht; // F1 to F4 used for various adjustable parameters on line 5
uint16_t f3x, f3y, f3wd, f3ht, f4x, f4y, f4wd, f4ht;
// bfo1x, bfo1y, bfo1wd, bfo1ht, bfo2x, bfo2y, bfo2wd, bfo2ht, , f3x, f3y, f3wd, f3ht
uint16_t botx, boty, botwd, botht;
uint16_t roundness;  // box edge roundness
uint16_t spacing;      // between buttons on same row
//scan buttons
uint16_t scandnx, scandny, scandnwd, scandnht, scanupx, scanupy, scanupwd, scanupht;
bool in_scan_dn = false, in_scan_up = false; // flag to indicate freq scan

//-------------------- S Meter -------------------------------------------------------------------------
unsigned int Sval, Sens, Ssamp = 0;   // s meter related
unsigned long Savg;
//----------------------CAT relted-------------
long prev_CAT_rd = 0l;  // time when last command was received
byte CAT_buff[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
int CAT_buff_ptr = 0;
byte i;
bool checkingCAT = 0; // when CAT is still being received
bool CAT_ctrl = 0; // Under CAT control or not
bool PTT_by_CAT = false;

///---------- etc
bool PTT_by_touch = false; //flag for PTT from touch screen
bool splitON = 0; // Is split freq operation between VFO A and B on?
int screen_no = 0; // screen_no 0 = main, 1 = PTT Setup , 2 = offsets, 3 = Smeter/O/P Power meter displays
//As per Joes suggestion ver 3.1U onwards
int max_screen = 3; // last screen

Bounce bandUpDebounce = Bounce();
Bounce bandDownDebounce = Bounce();
Bounce memoryUpDebounce = Bounce();
Bounce memoryDownDebounce = Bounce();
Bounce sideBandSelectDebounce = Bounce();
Bounce vfoDebounce = Bounce();
Bounce vfoToMemoryDebounce = Bounce();
Bounce memoryToVfoDebounce = Bounce();
Bounce stepSizeUpDebounce = Bounce();
Bounce stepSizeDownDebounce = Bounce();
Bounce bounce_TxTmOut = Bounce();
Bounce toggleSplitDebounce = Bounce();
Bounce pttDebounce = Bounce();

// Fixed main display  - VFO, Mem, Tx/Rx : VfoDispl, Dn, Freq, Up : Band, Step, SideBand : New Row 4: V><M, SPLIT, SAVE,
// in fourth row the end buttons PrevScrn <,  NextScrn > should be available on Tx for - adjusting Power meter, activating Tx time out
// Variable Fifth Row  differently arranged starting from v 3.1 // adjustable variables / active buttons are white fonts on grey background
// screen 0 : Variable Row 5 :  BFO1, BFO2 : (indicated as B1 and B2)
// screen 1 - PTT setup :   PTT Type, TxTimeOut on/off and its value (Indicated as PT and TO)
// screen 2 - offsets  adjustable offset in Hz for the currently selected band, Touch Sensitivity (Indicated as OF and TS)
// screen 3 - S meter Low (SL) and High (SH) capture buttons and their adjustable values
// screen 4 - Power meter Low (PL) and Hi (PH) capture buttons and their adjustable values
/**************************************/
/* Interrupt service routine for      */
/* encoder frequency change           */
/**************************************/
ISR(PCINT2_vect)
{
  unsigned char result = r.process();
  if (result && !txstatus)      // lock freq during transmit
  {
    if (result == DIR_CW)
      vfo += radix;
    else if (result == DIR_CCW)
      vfo -= radix;
    changed_f = 1;
  }
}
//-------------
void setup()
{
  Serial.begin(38400, SERIAL_8N1);  // 2 stop bits hamlib need - needed for CAT //
  // CAT_get_freq();
  Serial.flush();

  identifier = tft.readID(); // get display ID.

  //#ifdef Sa35_9486
  if ((identifier == 0x9486) || (identifier == 0x7783))   // identify display driver chip (known from mcufriend test progs)
    // spf Ventor's display ST7783,   // spf Ventor's display ST7783,
  {
    TS_LEFT = 50; TS_RT  =  890; TS_TOP = 880; TS_BOT = 100;
    YP = A2; XM = A1; YM = 6;  XP = 7;
  }

  if ((identifier == 0x154) || (identifier == 0x9325))
    // spf robodo display
  {
    TS_LEFT = 950;  TS_RT  = 120;  TS_TOP = 920;  TS_BOT = 120;
    YP = A1;  XM = A2;   YM = 7;   XP = 6;
  }

  if (identifier == 0x2053)
  {
    TS_LEFT = 950 ; TS_RT  = 120; TS_TOP = 120; TS_BOT = 920;
    YP = A1; XM = A2; YM = 7;  XP = 6;
  }

  if (identifier == 0x9320) // Joe's 2.8 DISPLAY blueg5/5elegoo detected as 0x9341 but with settings below
    // Rejimon's 2.4 DISPLAY from Robodo detected as 5408 but works as 9320
  {
    TS_LEFT = 74; TS_RT  = 906; TS_TOP = 117 ; TS_BOT = 923;
    YP = A3; XM = A2; YM = 9; XP = 8;
  }

  if (identifier == 0x9341) // Elegoo 2.8 inch
  {
    TS_LEFT = 921; TS_RT = 110; TS_TOP = 909; TS_BOT = 87;
    YP = A2; XM = A3; YM = 8; XP = 9;
  }

  ts = TouchScreen(XP, YP, XM, YM, 300);
  tft.begin(identifier);        // setup to use driver
  wd = tft.width();
  ht = tft.height();
  tft.setRotation(displ_rotn); // LS

  if (displ_rotn == 1 || displ_rotn == 3) // exchange the width and height
  {
    int temp = wd; wd = ht; ht = temp;
  }
  dispPos();

  Wire.begin();   // for si5351
  // welcome_screen();
  setup_vfo_screen(); // create main VFO screen

  if (EEPROM.read(magic_no_address) != magic_no)
    init_eprom();  // if blank eeprom or changed/new magic_no for reinit

  read_eprom(); // get infos of VFO A & B (and 1st memory channel only??)
  vfo = vfo_A;   // then display and use VFO A
  vfo_A_sel = 1;
  bfo1 = bfo_A;
  sideband = sb_A;



  // Ports Init
  pinMode(TX_RX_PIN, OUTPUT);
  pinMode(CW_TONE_PIN, OUTPUT);
  pinMode(CW_KEY_PIN, OUTPUT);
  pinMode(TX_LPF_A_PIN, OUTPUT);
  pinMode(TX_LPF_B_PIN , OUTPUT);
  pinMode(TX_LPF_C_PIN, OUTPUT);
  digitalWrite(TX_RX_PIN, 0);
  digitalWrite(CW_TONE_PIN, 0);
  digitalWrite(CW_KEY_PIN, 0);
  digitalWrite(TX_LPF_A_PIN, 0);
  digitalWrite(TX_LPF_B_PIN, 0);
  digitalWrite(TX_LPF_C_PIN, 0);

  // Buttons
  bandUpDebounce.attach(BAND_SELECT_UP_PIN, INPUT_PULLUP);
  bandDownDebounce.attach(BAND_SELECT_DOWN_PIN, INPUT_PULLUP);
  memoryUpDebounce.attach(MEMORY_UP_PIN, INPUT_PULLUP);
  memoryDownDebounce.attach(MEMORY_DOWN_PIN, INPUT_PULLUP);
  sideBandSelectDebounce.attach(SELECT_SIDE_BAND_PIN, INPUT_PULLUP);
  vfoDebounce.attach(VFO_PIN, INPUT_PULLUP);
  vfoToMemoryDebounce.attach(VFO_TO_MEMORY_PIN, INPUT_PULLUP);
  memoryToVfoDebounce.attach(MEMORY_TO_VFO_PIN, INPUT_PULLUP);
  stepSizeUpDebounce.attach(STEP_SIZE_UP_PIN, INPUT_PULLUP);
  stepSizeDownDebounce.attach(STEP_SIZE_DOWN_PIN, INPUT_PULLUP);
  toggleSplitDebounce.attach(TOGGLE_SPLIT_MODE_PIN, INPUT_PULLUP);
  pttDebounce.attach(PTT_PIN, INPUT_PULLUP);

  bandUpDebounce.interval(DEBOUNCE_INTERVAL);
  bandDownDebounce.interval(DEBOUNCE_INTERVAL);
  memoryUpDebounce.interval(DEBOUNCE_INTERVAL);
  memoryDownDebounce.interval(DEBOUNCE_INTERVAL);
  sideBandSelectDebounce.interval(DEBOUNCE_INTERVAL);
  vfoDebounce.interval(DEBOUNCE_INTERVAL);
  vfoToMemoryDebounce.interval(DEBOUNCE_INTERVAL);
  memoryToVfoDebounce.interval(DEBOUNCE_INTERVAL);
  stepSizeUpDebounce.interval(DEBOUNCE_INTERVAL);
  stepSizeDownDebounce.interval(DEBOUNCE_INTERVAL);
  toggleSplitDebounce.interval(DEBOUNCE_INTERVAL);
  pttDebounce.interval(DEBOUNCE_INTERVAL);

  // Initialise Si5351
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, si5351correction); // If using a 27Mhz crystal, put in 27000000 instead of 0
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA); // this is 11dBm  // you can set this to 2MA, 4MA, 6MA or 8MA
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA); // be careful though - measure into 50ohms
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_6MA); //

  si5351.set_freq(bfo2 * SI5351_FREQ_MULT, SI5351_CLK0); // 12 MHZ which remains fixed
  set_bfo1();  // adjust bfo1 and vfo_tx
  //si5351.set_freq(((vfo_tx + if_offset)* SI5351_FREQ_MULT), SI5351_CLK2); // 45 to 75 MHz

  PCICR |= (1 << PCIE2);                      // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT21) | (1 << PCINT22);  // MEGA interrupt pins mapped to A14 A13
  sei();                                      // Start interrupts

  set_band();
  display_frequency();
  display_band();
  display_step();
  display_sideband();
  display_mem();
  display_bfo1();
}

void loop()
{
  if (CAT_ctrl) return;   // if in cat control go back

  // Update the display if the frequency changed
  if (changed_f)
  {
    //  set_vfo();
    display_vfo();
    set_bfo1();
    display_frequency();
    set_band();
    display_band();
    display_sideband();
    save_frequency();
    changed_f = 0;
    return;
  }

  if (Tx_timeout_mode)
  {
    rem_time = (millis() - Tx_start_time) / 1000; // remaining time
    if  (txstatus)
    {
      if ( rem_time >= Tx_timeout)   // time over
      {
        digitalWrite(TX_RX_PIN, LOW);    // disable Tx
        displ_rx();
        txstatus = false;
      }
      else
      {
        tft.setCursor(txrx + 45, txry + 5);  // else display the remaining time
        tft.setTextSize(2);
        tft.setTextColor(WHITE, RED);
        diff = (Tx_timeout - rem_time) ;
        if (diff <= 9)
          tft.print(" ");
        tft.print(diff);
      }
    }
  }
  //-----------------------------------
  // External button controls


  //#### Toggle PTT button
  if (!PTT_by_CAT)   // detect button press if not done by CAT - else if PTT is put on by CAT the open PTT
    // was causing immediate PTT_off. PTT_by_CAT is set/reset from CAT PTT controls
  {
    // for uBitx PTT   - touch ptt is always toggle - first touch on second off
    if (active_PTT_in && !PTT_by_touch && bnd_count > 0)     // i.e. Normal type PTT we call active PTT
    {
      if (pttDebounce.fell())     // only if not in Tx already
        ptt_ON();
      else if (txstatus)    // only if in Tx
        ptt_OFF();
    }
    else      // Toggle type PTT
    {
      if (pttDebounce.fell() && bnd_count > 0)
        toggle_ptt();
    }
  }

  bandUpDebounce.update();
  bandDownDebounce.update();
  memoryUpDebounce.update();
  memoryDownDebounce.update();
  sideBandSelectDebounce.update();
  vfoDebounce.update();
  vfoToMemoryDebounce.update();
  memoryToVfoDebounce.update();
  stepSizeUpDebounce.update();
  stepSizeDownDebounce.update();
  bounce_TxTmOut.update();
  toggleSplitDebounce.update();
  pttDebounce.update();

  if (!txstatus)    // only if not in Tx
  {
    if (vfoDebounce.fell())
      vfo_sel();
    if (bandUpDebounce.fell())
      band_incr();
    if (bandDownDebounce.fell())
      band_decr();
    if (stepSizeUpDebounce.fell())
      step_incr();
    if (stepSizeDownDebounce.fell())
      step_decr();
    if (sideBandSelectDebounce.fell())
      sideband_chg();
    if (vfoToMemoryDebounce.fell())
      vfo_to_mem();
    if (memoryToVfoDebounce.fell())
      mem_to_vfo();
    if (memoryUpDebounce.fell())
      mem_incr();
    if (memoryDownDebounce.fell())
      mem_decr();
    if (bounce_TxTmOut.fell())
      displ_timeout_button();
    if (toggleSplitDebounce.fell())
      displ_split_button();
  }
  //----------------------------------------------
  //$$$$  for Touch Screen input control

  tp = ts.getPoint();
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  pinMode(XP, OUTPUT);
  pinMode(YM, OUTPUT);
  delay(ts_delay);     // delay between two touches to reduce sensitivity

  if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE)
  {
    //#if defined (MCUF0x154) || defined(PL0x9341) || defined (VE0x7783) || defined (pl0x2053)
    xpos = map(tp.x, TS_LEFT, TS_RT, 0, tft.width());
    ypos = map(tp.y, TS_TOP, TS_BOT, 0, tft.height());
    //#endif

    /*
      #if defined elegoo923 || defined (IL9325)
        xpos = map(tp.y, TS_LEFT, TS_RT, 0, tft.width());
        ypos = map(tp.x, TS_TOP, TS_BOT, 0, tft.height());
      #endif*/

    // Rx/Tx PTT touch button or PTT_PIN
    if (ypos > firstrowy && ypos < firstrowy + buttonht)  // first row of buttons
    {
      if (xpos > txrx  && xpos < txrx + txrwd - 2  && (bnd_count > 0) ) // toggle between Rx & Tx, TX_RX_PIN (D14) goes Hi on Tx
      {
        toggle_ptt();
        PTT_by_touch = !PTT_by_touch;
        delay(50);
      }
    }

    if (!txstatus)    // only if not in Tx should any button be recognized for change
    {
      if (ypos > firstrowy && ypos < firstrowy + buttonht)  // first row of buttons (orig 5,42)
      {
        // VFO Button:  cycle VFO A/B/M in sequence when VFO button is touched
        if (xpos > vfox && xpos < vfox + vfowd ) // change VFO
          vfo_sel();

        // MEM Ch change Button
        // Left half button decreases channel no
        else if (xpos > memx && xpos < memx + buttonwd - 2 ) // decrease channel
          mem_decr();   // decrease memory channel number

        // right half buttton increases ch no
        else if (xpos > memx + buttonwd + 2 && xpos < memx + 2 * buttonwd ) // increase channel
          mem_incr();   // increase mem ch no
        return;
      }
      // First row end

      // Freq Change Second Row touch  Button
      if (ypos > frqy && ypos < frqy + frqht && !txstatus) //  lock freq change during transmit
      {
        if (xpos > scandnx && xpos < (scandnx + scandnwd))
        {
          in_scan_dn = true;
          scan_dn();
          return;
        }
        if (xpos > frqx && xpos < (frqx + frqwd / 2) - 2 ) // Left half button decreases frq by step size
        {
          in_scan_dn = false;
          vfo = vfo - radix;
          changed_f = 1;
          save_frequency();   // added 7/7/17
          return;
        }

        if (xpos > (frqx + frqwd / 2) + 2 && xpos < frqx + frqwd) // Right half button increases freq by step size
        {
          in_scan_up = false;
          vfo = vfo + radix;
          changed_f = 1;
          save_frequency();   // added 7/7/17
          return;
        }

        if (xpos > scanupx && xpos < (scanupx + scanupwd))
        {
          in_scan_up = true;
          scan_up();
          return;
        }
      }  // Freq Button/ Second Row end

      // Third Row  Band Change Button
      if (ypos > bandy && ypos < bandy + bandht)
      {
        if (xpos > bandx && xpos < (bandx + bandwd / 2) - 2 ) // Left half button decreases band(20,65)
          band_decr();

        else if (xpos > (bandx + bandwd / 2) + 2 && xpos < bandx + bandwd ) // Right half button increases band (67,115)
          band_incr();


        // Third Row Step Size change  Button
        //  left half of step button decreases step size
        else if (xpos > stpx && xpos < (stpx + stpwd / 2) - 2 )
          step_decr();

        //  right half of step button increases step size
        else if (xpos > (stpx + stpwd / 2) + 2 && xpos < stpx + stpwd )
          step_incr();


        /// Third Row side band flip flop between LSB & USB   (others may be added if hardware permits)
        else if (xpos > sbx && xpos < sbx + sbwd )
          sideband_chg();
        return;
      } // Third row end

      // Fourth row Central part no change during Tx (new from ver 3.1)
      if (ypos > vmy && ypos < vmy + vmht) //  fourth row
      {
        // Fourth Row VFO < > Mem switch nothing saved on EEPROM unless SAVE button pressed
        if (xpos > vmx && xpos < (vmx + vmwd / 2) - 2 ) //left half VFO -> MEM
          // currently selected VFO stored on currently selected mem ch (not in EEPROM which is by SAVE button)
          vfo_to_mem();

        else if (xpos > (vmx + vmwd / 2) + 2 && xpos < vmx + vmwd ) //right half VFO <- MEM
          mem_to_vfo();

        else if (xpos > splx && xpos < (splx + splwd)) // Split freq Control
        {
          splitON = !splitON;
          displ_split_button();
        }

        // Fourth Row SAVE Button
        else if (xpos > svx && xpos < svx + svwd ) // Save "current" Vfo/Mem and all other parameters on eeprom
          save();


      } // Fourth Row entral part  end
    } // all above buttons not active during Tx

    // Fixed main display  - VFO, Mem, Tx/Rx : VfoDispl, Dn, Freq, Up : Band, Step, SideBand : New Row 4: V><M, SPLIT, SAVE,
    // in fourth row the end buttons PrevScrn <,  NextScrn > should be available on Tx for adjusting Power meter, activating Tx time out
    // Variable Fifth Row  differently arranged starting from v 3.1 // adjustable variables / active buttons are white fonts on grey background
    // screen 0 : Variable Row 5 :  BFO1, BFO2 : (indicated as B1 and B2)
    // screen 1 - PTT setup :   PTT Type, TxTimeOut on/off and its value (Indicated as PT and TO)
    // screen 2 - offsets  adjustable offset in Hz for the currently selected band, Touch Sensitivity (Indicated as OF and TS)
    // screen 3 - S meter Low (SL) and High (SH) capture buttons and their adjustable values
    // screen 4 - Power meter Low (PL) and Hi (PH) capture buttons and their adjustable values

    // Fourth row  two End buttons <- & -> new from ver 3.1 active even during Tx
    if (ypos > vmy && ypos < vmy + vmht) //  fourth row
    {
      if (xpos > prsnx && xpos < (prsnx + prsnwd) ) // previous screen button "<-" changes 5th row displays
      {
        screen_no --;
        if (screen_no < 0)
          screen_no = max_screen;
        update_row5();
      }
      else if (xpos > nxsnx && xpos < (nxsnx + nxsnwd) ) // next screen button "->" : affects 5th row displays
      {
        screen_no ++;
        if (screen_no > max_screen)
          screen_no = 0 ;
        update_row5();
      }
    }

    // One of the screen for Tx power needs to be active when transmitting therefore we have to handle each screen separately
    if (ypos > f1y && ypos < f1y + f1ht) //  fifth row
    {
      if (screen_no == 0 && !txstatus)     // bfo1 / 2 change only when in Receive mode
      {
        // left half button decreases bfo freq
        if (xpos > f2x && xpos < (f2x + f2wd / 2) - 2 ) // decrease freq
          bfo1_decr();
        // right half button increases bfo freq
        else if (xpos > (f2x + f2wd / 2) + 2 && xpos < f2x + f2wd ) // increase freq  (175,235)
          bfo1_incr();

        if (xpos > f4x && xpos < (f4x + f4wd / 2) - 2 ) // decrease freq
          bfo2_decr();
        // right half button increases bfo freq
        else if (xpos > (f4x + f4wd / 2) + 2 && xpos < f4x + f4wd ) // increase freq  (175,235)
          bfo2_incr();
      }


      else if (screen_no == 1 && !txstatus)  // change allowed in Rx mode only
      {
        if ( xpos > f2x && xpos < (f2x + f2wd)) // touch on button to toggle
          active_PTT_in = ! active_PTT_in;

        else if (xpos > f3x && xpos < f3x + f3wd)    // touch on TO buttom
          Tx_timeout_mode = !Tx_timeout_mode;

        else if (xpos > f4x && xpos < (f4x + f4wd / 2) - 2) // Tx time out button RED / Green, Mode activated, timer starts on PTT
        {
          Tx_timeout = Tx_timeout - 1;   // decrease Time out by 1 sec
          if (Tx_timeout <= 0)
            Tx_timeout = 1; // min 1 sec
        }

        else if ((  xpos > (f4x + f4wd / 2) + 2 ) && xpos < f4x + f4wd) // increase Time out
        {
          Tx_timeout = Tx_timeout + 1;
          if (Tx_timeout > max_timeout)  // max timeout defined in used defs
            Tx_timeout = max_timeout;
        }
      }


      else if (screen_no == 2 && !txstatus)  // only change in Rx mode
      {
        if (xpos > f2x && xpos < (f2x + f2wd / 2) - 2 ) // decrease offset for currently displayed band
        {
          offsets[bnd_count] = offsets[bnd_count] - radix;
          set_vfo();
        }
        else if (xpos > (f2x + f2wd / 2) + 2 && xpos < f2x + f2wd ) // increase offset for currently displayed band
        {
          offsets[bnd_count] = offsets[bnd_count] + radix;
          set_vfo();
        }

        else if (xpos > f4x && xpos < (f4x + f4wd / 2) - 2) // ts_delay decrement
        {
          ts_delay--;   // decrease delay Time
          if (ts_delay < 0)
            ts_delay = 80;   // typical val 80 ms
        }

        else if ((xpos > (f4x + f4wd / 2) + 2 ) && xpos < f4x + f4wd) // increase Time out
        {
          ts_delay++;  // no upper limit
        }
      }

      else if (screen_no == 3)    // should change even during Tx
      {
        if (xpos > f1x && xpos < f1x + f1wd)    // touch on Min  button Capture current SM value as min value
          if ( !txstatus)
            SM_min = analogRead(S_METER_PIN);

        if (xpos > f2x && xpos < (f2x + f2wd / 2) - 2 ) // decrease min value
        {
          if ( !txstatus)
            SM_min =   SM_min - radix;
          else
            PM_min = PM_min - radix;
        }
        else if (xpos > (f2x + f2wd / 2) + 2 && xpos < f2x + f2wd ) // increase min value
        {
          if ( !txstatus)
            SM_min =   SM_min + radix;
          else
            PM_min = PM_min + radix;
        }

        if (xpos > f3x && xpos < f3x + f3wd)    // touch on Min  button Capture current SM value as min value
          if ( !txstatus)
            SM_max = analogRead(S_METER_PIN);

        else if (xpos > f4x && xpos < (f4x + f4wd / 2) - 2) // decrease max value
        {
          if ( !txstatus)
            SM_max =   SM_max - radix;
          else
            PM_max = PM_max - radix;
        }
        else if ((xpos > (f4x + f4wd / 2) + 2 ) && xpos < f4x + f4wd) // increase Time out
        {
          if ( !txstatus)
            SM_max =   SM_max + radix;
          else
            PM_max = PM_max + radix;
        }
      }
    }

    update_row5();
  }



  //=====================
  //$$$$ S Meter display Take average and display after Ssamp no of samples

  Ssamp++; // sample no
  if (!txstatus)
    Sval = analogRead(S_METER_PIN);   // read s-meter

  Savg = (Savg + Sval );
  if (Ssamp >= SM_speed)     // calc & display every few samples (SM_speed 2-10)
  {
    Savg = Savg / Ssamp;
    {
      Sens = map(Savg, SM_min, SM_max, 0, wd - 70); // play with SM_FullScale as per input to S_METER_PIN 70 for ver displ
      tft.fillRect(botx + 2, boty + 3, botwd - 8, botht - 5, BLACK);

      if (!txstatus)
        tft.fillRect(botx + 2, boty + 3, Sens - 4, botht - 5, GREEN);  // display S-meter
      else
        tft.fillRect(botx + 2, boty + 3, Sens - 4, botht - 5, PINK);  // display Power
    }
    Ssamp = 0;
    Savg = 0;
    tft.setTextColor(WHITE, BLUE);  // ver info
  }
  check_CAT();
}    // end of loop()


void vfo_sel()   // select vfo A/B/M when VFO button pressed
{
  if (vfo_M_sel)
  {
    vfo_M = vfo;
    vfo_selA();
  }
  else if (vfo_B_sel)
  {
    vfo_B = vfo; // save current value of vfo for use later
    if (!xch_M)
      read_ch();   // get data from memory channel
    else
      xch_M = 0;

    vfo_A_sel = false;
    vfo_B_sel = false;
    vfo_M_sel = true;  // select
    vfo = vfo_M;    // restore values
    // bfo1 = bfo_M;
    sideband = sb_M;
    if (sideband == USB)
      bfo1 = bfo1_USB;  // = bfo_M;
    else
      bfo1 = bfo1_LSB; // = bfo_M;
  }
  else if (vfo_A_sel)
  {
    vfo_A = vfo;
    vfo_selB();
  }
  set_vfo();
  display_vfo();
  display_frequency();
  save_frequency();
  // display_bfo1();
  set_bfo1();
  display_sideband();
  set_band();                        // 2 new lines 24/8/17 Joe
  display_band();
  update_row5();
}

void vfo_selA()
{
  vfo_A_sel = true;
  vfo_B_sel = false;
  vfo_M_sel = false;
  vfo = vfo_A;
  //bfo1 = bfo_A;
  sideband = sb_A;
  if (sideband == USB)
    bfo1 = bfo1_USB; //= bfo_A;
  else
    bfo1 = bfo1_LSB; //= bfo_A;
}

void vfo_selB()
{
  vfo_A_sel = false;
  vfo_B_sel = true;
  vfo_M_sel = false;
  vfo = vfo_B;
  // bfo1 = bfo_B;
  sideband = sb_B;
  if (sideband == USB)
    bfo1 = bfo1_USB ; //= bfo_B;
  else
    bfo1 = bfo1_LSB; // = bfo_B;
}

void mem_decr()     // decrement mem ch no
{
  old_band = bnd_count;
  memCh = memCh - 1;
  if (memCh <= 0)
    memCh = max_memory_ch;
  display_mem();
  changed_f = 1;
}

void mem_incr()     // increment mem ch no
{
  old_band = bnd_count;
  memCh = memCh + 1;
  if (memCh > max_memory_ch)
    memCh = 1;
  display_mem();
  changed_f = 1;
}


void ptt_ON()
{
  if (txstatus == false)
  {
    if (splitON)
    {
      vfo_selB(); // in Split mode vfo B is Tx vfo
      changed_f = 1;
    }
    txstatus = true;
    set_TX_filters();
    digitalWrite(TX_RX_PIN, HIGH);
    displ_tx();
    update_row5();
  }
}

void ptt_OFF()
{
  // if (txstatus == true)
  {
    if (splitON)
    {
      vfo_selA(); // in Split mode vfo A is Rx vfo
      changed_f = 1;
    }
    txstatus = false;
    reset_TX_filters();
    digitalWrite(TX_RX_PIN, LOW);
    displ_rx();
    update_row5();
  }
}

void toggle_ptt()      // toggle the PTT_output pin TX_RX_PIN, either by touch button or PTT_PIN pin activated
{
  txstatus = !txstatus;
  if (Tx_timeout_mode)
    Tx_start_time = millis();
  if (txstatus)   // Tx mode
  {
    if (splitON)
    {
      vfo_selB(); // in Split mode vfo B is Tx vfo
      changed_f = 1;
    }
    set_TX_filters();
    digitalWrite(TX_RX_PIN, HIGH);
    displ_tx();
  }
  else
  {
    if (splitON)
    {
      vfo_selA(); // in Split mode vfo A is Rx vfo
      changed_f = 1;
    }
    reset_TX_filters();
    digitalWrite(TX_RX_PIN, LOW);
    displ_rx();
  }

  update_row5();
  Ssamp = 0;  // reset the s-meter / power meter
  Savg = 0;
}

void set_TX_filters()
{
  // Serial.println(vfo);
  if ( vfo > 21000000L)
  {
    digitalWrite(TX_LPF_A_PIN, 0);
    digitalWrite(TX_LPF_B_PIN, 0);
    digitalWrite(TX_LPF_C_PIN, 0);
  }
  else if (vfo >= 14000000L)
  {
    digitalWrite(TX_LPF_A_PIN, 1);
    digitalWrite(TX_LPF_B_PIN, 0);
    digitalWrite(TX_LPF_C_PIN, 0);
  }
  else if (vfo >= 7000000L)
  {
    digitalWrite(TX_LPF_A_PIN, 1);
    digitalWrite(TX_LPF_B_PIN, 1);
    digitalWrite(TX_LPF_C_PIN, 0);
  }
  else
  {
    digitalWrite(TX_LPF_A_PIN, 1);
    digitalWrite(TX_LPF_B_PIN, 1);
    digitalWrite(TX_LPF_C_PIN, 1);
  }
}

void reset_TX_filters()
{
  digitalWrite(TX_LPF_A_PIN, 0);
  digitalWrite(TX_LPF_B_PIN, 0);
  digitalWrite(TX_LPF_C_PIN, 0);
}

void scan_up()
{
  while (in_scan_up)  // will be taken care in loop
  {
    vfo = vfo + radix;
    if (vfo >= F_MAX_T[bnd_count]) in_scan_up = false; //stop at band edge
    display_frequency();
    set_bfo1();
    if (check_touch())  // any touch to stop scan?
      break;
    Serial.write(0);
    delay(200);
    CAT_get_freq();  // update CAT freq if connected
  }
  CAT_get_freq();  // update CAT freq if connected
}

void scan_dn()
{
  while (in_scan_dn)
  {
    vfo = vfo - radix;
    if (vfo <= F_MIN_T[bnd_count]) in_scan_dn = false; //stop at band edge
    display_frequency();
    set_bfo1();
    if (check_touch())
      break;
    Serial.write(0);
    delay(200);
    CAT_get_freq();  // update CAT freq if connected
  }
  CAT_get_freq();  // update CAT freq if connected
}


void band_decr()      //decrement band count
{
  old_band = bnd_count;
  bnd_count = bnd_count - 1;
  if (bnd_count < 0)
    bnd_count = 8;
  change_band();
  set_band();
  adjust_sideband();
  save_frequency();
  update_row5();
}

void band_incr()       // increment band count
{
  old_band = bnd_count;
  bnd_count = bnd_count + 1;
  if (bnd_count > 8)
    bnd_count = 0;
  change_band();
  set_band();
  adjust_sideband();
  save_frequency();
  update_row5();
}

void step_decr()      // decrement step size 1M to 1Hz
{
  step_index = step_index - 1;
  if (step_index < 0)
    step_index = 6;
  radix = step_sz[step_index];
  display_step();
}

void step_incr()      // incremet step size 1Hz to 1M
{
  step_index = step_index + 1;
  if (step_index > 6)
    step_index = 0;
  display_step();
  radix = step_sz[step_index];
}

void sideband_chg()     // change sidebands between USB/LSB and may be more in future
{
  if (sideband == LSB)
  {
    sideband = USB;
    bfo1 = bfo1_USB;
  }
  else
  {
    sideband = LSB;
    bfo1 = bfo1_LSB;
  }
  display_sideband();
  if (screen_no == 0)
    display_bfo1();
  save_frequency();
  set_bfo1();
}

void adjust_sideband()
{
  if (vfo > 10000000)
  {
    sideband = USB;
    bfo1 = bfo1_USB;
  }
  else
  {
    sideband = LSB;
    bfo1 = bfo1_LSB;
  }
  display_sideband();
  if (screen_no == 0)
    display_bfo1();
  save_frequency();
  set_bfo1();
}

void vfo_to_mem()       // transfer current vfo to M
{
  if (vfo_A_sel)     // when vfo A is selected its content transferred to current memory
  {
    vfo_M = vfo_A;
    bfo_M = bfo_A;
  }
  else if (vfo_B_sel)   // B -> MemCh
  {
    vfo_M = vfo_B;
    bfo_M = bfo_B;
  }
  xch_M = 1;
  display_frequency2();
}

void mem_to_vfo()
{
  if ( vfo_A_sel)   // when vfo A is working Mem goes to A and changes vfo freq
  {
    vfo_A = vfo_M;
    vfo = vfo_A;
    bfo_A = bfo_M;
    bfo1 = bfo_A;
  }
  else if ( vfo_B_sel)  // when vfo B is working Mem goes to B and changes vfo freq
  {
    vfo_B = vfo_M;
    vfo = vfo_B;
    bfo_B = bfo_M;
    bfo1 = bfo_B;
  }
  changed_f = 1;
}

void bfo1_decr()       // decrement bfo freq
{
  if (sideband == LSB)
  {
    bfo1_LSB = bfo1_LSB - radix;
    bfo1 = bfo1_LSB;
  }
  else
  {
    bfo1_USB = bfo1_USB - radix;
    bfo1 = bfo1_USB;
  }
  set_bfo1();
  if (screen_no == 0)
    display_bfo1();
  save_frequency();

  //  set_vfo();
}

void bfo1_incr()
{
  if (sideband == LSB)
  {
    bfo1_LSB = bfo1_LSB + radix;
    bfo1 = bfo1_LSB;
  }
  else
  {
    bfo1_USB = bfo1_USB + radix;
    bfo1 = bfo1_USB;
  }

  set_bfo1();
  if (screen_no == 0)
    display_bfo1();
  save_frequency();

  // set_vfo();
}

void bfo2_decr()       // decrement bfo2 freq
{
  bfo2 = bfo2 - radix;
  set_bfo2();
  display_bfo2();
  save_frequency();

  //  set_vfo();
}

void bfo2_incr()
{
  bfo2 = bfo2 + radix;
  set_bfo2();
  display_bfo2();
  save_frequency();

  // set_vfo();
}

void save()       // save on EEPROM
{
  // Change message during save on Button
  tft.drawRoundRect(svx, svy, svwd, svht, roundness, WHITE); // Save button outline
  tft.fillRoundRect(svx + 2, svy + 2, svwd - 4, svht - 4, roundness - 4, GREEN); //Save
  tft.setTextSize(2);
  tft.setTextColor(YELLOW);
  tft.setCursor(svx + 8, svy + 10);
  tft.print("SVNG");

  EEPROM_writeAnything (bfo1_USB_address, bfo1_USB);  // initial values to be stored
  EEPROM_writeAnything (bfo1_LSB_address, bfo1_LSB);
  EEPROM_writeAnything (bfo2_address, bfo2);
  EEPROM_writeAnything (PTT_type_address, active_PTT_in);
  EEPROM_writeAnything (Tx_timeout_mode_address, Tx_timeout_mode);
  EEPROM_writeAnything (TxTmO_Time_address, Tx_timeout);
  EEPROM_writeAnything (touch_sens_address, ts_delay);
  EEPROM_writeAnything (offsets_base_address, offsets);  // for loop may be reqd
  EEPROM_writeAnything (sm_min_address, SM_min);
  EEPROM_writeAnything (sm_max_address, SM_max);


  if (vfo_M_sel)
    write_ch();
  else if (vfo_A_sel)
    write_vfo_A();
  else
    write_vfo_B();

  //  delay(200);  // test
  // Reset Save button
  tft.drawRoundRect(svx, svy, svwd, svht, roundness, MAGENTA); // Save button outline
  tft.fillRoundRect(svx + 2, svy + 2, svwd - 4, svht - 4, roundness - 4, RED); //Save
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(svx + 10, svy + 10);
  tft.print("SAVE");
}

bool check_touch()    // check if touched on screen
{
  tp = ts.getPoint();
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  pinMode(XP, OUTPUT);
  pinMode(YM, OUTPUT);
  delay(ts_delay);     // delay between two touches to reduce sensitivity

  if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE)
    return true;
  else
    return false;
}

void update_display()
{
  save_frequency();
  display_frequency();
  set_band();
  display_band();
  display_sideband();
}

void set_vfo()
{
  si5351.set_freq(((vfo_tx + offsets[bnd_count])* SI5351_FREQ_MULT), SI5351_CLK2); // 45 to 75 MHz. Individual band offsets introduced 2.9cU
}


void save_frequency()    // for temporarily saving in variables not in EEPROM
{
  if (vfo_M_sel)
  {
    vfo_M = vfo;
    bfo_M = bfo1;
    sb_M = sideband;
  }
  else if (vfo_A_sel)
  {
    vfo_A = vfo;
    bfo_A = bfo1;
    sb_A = sideband;
  }
  else
  {
    vfo_B = vfo;
    bfo_B = bfo1;
    sb_B = sideband;
  }
}


void set_bfo1()   // if sb changes readjust bfo and vfo
{
  if (sideband == LSB)
  {

    bfo1 = bfo1_LSB;
    vfo_tx = bfo1 + bfo2 + vfo;  // vfo is the displayed freq
  }
  else
  {
    bfo1 = bfo1_USB;
    vfo_tx =  bfo1 - bfo2 + vfo;
  }

  // si5351.set_freq(bfo2, SI5351_CLK0); // 12 MHZ
  si5351.set_freq(bfo1 * SI5351_FREQ_MULT, SI5351_CLK1);  // 33 MHz for USB , 57 MHz for LSB
  si5351.set_freq(((vfo_tx + offsets[bnd_count])* SI5351_FREQ_MULT), SI5351_CLK2); // 45 to 75 MHz   //indiv band offsets 2.9cU
  //  if (CAT_ctrl)
  //  CAT_get_freq();
  //    update_CAT();
}

void set_bfo2()
{
  si5351.set_freq(bfo2 * SI5351_FREQ_MULT, SI5351_CLK0);  // 12 MHZ
  set_bfo1();  // for setting up other clocks
}

void setup_vfo_screen() // sets up main screen for VFO etc display
{
  tft.fillScreen(BLACK); // setup blank screen LIGHTGREY
  // tft.fillRoundRect(0, 0, 320, 121, BLUE); // top segment
  tft.drawRect(0, 0, wd, ht, WHITE);  // outer full border
  tft.setTextSize(2);
  tft.setTextColor(WHITE);  //

  /* tft.drawRoundRect(smx, smy, smwd, smht, RED); // vert S meter
    tft.fillRoundRect(smx + 3, smy + 3, smwd - 4, smht - 5, YELLOW);
    tft.setTextSize(2);
    tft.setTextColor(RED);
    tft.setCursor(smx + 5, smy + smht + 5); // print below s meter
    tft.println("S");
  */
  tft.drawRoundRect(vfox, vfoy, vfowd, vfoht, roundness, RED);  // VFO A/B box outline
  tft.fillRoundRect(vfox + 2, vfoy + 2, vfowd - 4, vfoht - 4, roundness - 4, GREEN); //VFO A/B box
  tft.setCursor(vfox + 10, vfoy + 5);
  tft.setTextSize(3);
  tft.setTextColor(RED);
  tft.print("VFO");

  tft.drawRoundRect(memx, memy, memwd, memht, roundness, RED);  // Mem box outline
  tft.fillRoundRect(memx + 2, memy + 2, memwd - 4, memht - 4, roundness - 4, GREY); //Mem box
  tft.setCursor(memx + 20, memy + 5);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.print("MEM ");
  display_mem();

  tft.drawRoundRect(txrx, txry, txrwd, txrht, roundness, RED);  // TxRx box outline
  tft.fillRoundRect(txrx + 2, txry + 2, txrwd - 4, txrht - 4, roundness - 4, GREEN); //TxRx box
  tft.setCursor(txrx + 10, txry + 5);
  tft.setTextSize(3);
  tft.setTextColor(BLUE);
  tft.print("Rx");

  tft.drawRect(scandnx, scandny, scandnwd, scandnht, GREEN);  // Scan Down button
  tft.fillRect(scandnx + 2, scandny + 2, scandnwd - 2, scandnht - 2, BLUE);
  tft.setCursor(scandnx + 3, scandny + 20);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.print("D");

  tft.drawRoundRect(frqx, frqy, frqwd, frqht, roundness, WHITE);  // freq box outline
  // tft.fillRoundRect(frqx+2, frqy+2, frqwd-4, frqht-4, roundness-4, ORANGE);   //freq box

  tft.drawRect(scanupx, scanupy, scanupwd, scanupht, GREEN); // Scanup button
  tft.fillRect(scanupx + 2, scanupy + 2, scanupwd - 2, scanupht - 2, BLUE);
  tft.setCursor(scanupx + 3, scanupy + 5);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.print("U");

  tft.fillRoundRect(bandx, bandy, bandwd, bandht, roundness, WHITE); //band button outline
  tft.fillRoundRect(bandx + 2, bandy + 2, bandwd - 4, bandht - 4, roundness - 4, GREY); //band

  tft.fillRoundRect(stpx, stpy, stpwd, stpht, roundness, WHITE); // step button outline
  tft.fillRoundRect(stpx + 2, stpy + 2, stpwd - 4, stpht - 4, roundness - 4, GREY); //step

  tft.fillRoundRect(sbx, sby, sbwd, sbht, roundness, WHITE); // sideband button outline
  tft.fillRoundRect(sbx + 2, sby + 2, sbwd - 4, sbht - 4, roundness - 4, GREY); //sideband

  // from v 3.1 onwards line 4 mofdified as <, V<>M, Split, Save, >
  // Previous screen button
  tft.drawRoundRect(prsnx, prsny, prsnwd, prsnht, roundness, RED); //bfo button outline
  tft.fillRoundRect(prsnx + 2, prsny + 2, prsnwd - 4, prsnht - 4, roundness - 4, GREY); //bfo
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(prsnx + 2, prsny + 10);
  tft.print("<-");

  tft.drawRoundRect(vmx, vmy, vmwd, vmht, roundness, RED); //  VFO <> MEM  button outline
  tft.fillRoundRect(vmx + 2, vmy + 2, vmwd - 4, vmht - 4, roundness - 4, GREY); //  V/M
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(vmx + 15, vmy + 10);
  tft.print("V> <M");

  tft.drawRoundRect(splx, sply, splwd, splht, roundness, GREEN); // SPLIT button outline
  tft.fillRoundRect(splx + 2, sply + 2, splwd - 4, splht - 4, roundness - 4, PURPLE);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(splx + 10, sply + 10);
  tft.print("SPLIT");

  tft.drawRoundRect(svx, svy, svwd, svht, roundness, MAGENTA); // Save button outline
  tft.fillRoundRect(svx + 2, svy + 2, svwd - 4, svht - 4, roundness - 4, RED); //Save
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(svx + 15, svy + 10);
  tft.print("SAVE");

  // Next screen button
  tft.drawRoundRect(nxsnx, nxsny, nxsnwd, nxsnht, roundness, RED); //bfo button outline
  tft.fillRoundRect(nxsnx + 2, nxsny + 2, nxsnwd - 4, nxsnht - 4, roundness - 4, GREY); //<
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(nxsnx + 2, nxsny + 10);
  tft.print("->");

  // new arrangement in v3.1 for line 5   // 5th row of adjustable buttons

  tft.drawRoundRect(f1x, f1y, f1wd, f1ht, roundness, WHITE); // F1 button outline TxTimeOut
  tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, BLUE); //F1
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(f1x + 5, f1y + 10);
  tft.print("B1");

  tft.drawRoundRect(f2x, f2y, f2wd, f2ht, roundness, RED); // F2 button outline long for bfo2
  tft.fillRoundRect(f2x + 2, f2y + 2, f2wd - 4, f2ht - 4, roundness - 4, GREY);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(f2x + 5, f2y + 10);
  tft.print(bfo1);

  tft.drawRoundRect(f3x, f3y, f3wd, f3ht, roundness, WHITE); // F4 button outline
  tft.fillRoundRect(f3x + 2, f3y + 2, f3wd - 4, f3ht - 4, roundness - 4, BLUE); //F4
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(f3x + 5, f3y + 10);
  tft.print("B2");

  tft.drawRoundRect(f4x, f4y, f4wd, f4ht, roundness, GREEN); // F4 button outline
  tft.fillRoundRect(f4x + 2, f4y + 2, f4wd - 4, f4ht - 4, roundness - 4, GREY); //F4
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(f4x + 5, f4y + 10);
  tft.print(bfo2);


  // bottom line
  tft.drawRect(botx, boty, botwd, botht, WHITE);  // surrounding RECT
  tft.fillRect(botx + 2, boty + 2, botwd - 4, botht - 4, BLACK); // bot strip
}  // end of setup_vfo_screen()
//---------------------------


// EEPROM related
void init_eprom()      // write some info on EEPROM when initially loaded or when magic number changes
{
  uint16_t i;
  EEPROM.write(magic_no_address, magic_no);  // check byte may be same as ver no at 0 address
  display_mem_msg("InitEPrm");
  // write various parameters in the memory beginning
  EEPROM_writeAnything (bfo1_USB_address, bfo1_USB);  // initial values to be stored
  EEPROM_writeAnything (bfo1_LSB_address, bfo1_LSB);
  EEPROM_writeAnything (bfo2_address, bfo2);
  EEPROM_writeAnything (PTT_type_address, active_PTT_in);
  EEPROM_writeAnything (Tx_timeout_mode_address, Tx_timeout_mode);
  EEPROM_writeAnything (TxTmO_Time_address, Tx_timeout);
  EEPROM_writeAnything (touch_sens_address, ts_delay);
  EEPROM_writeAnything (offsets_base_address, offsets);  //
  EEPROM_writeAnything (sm_min_address, SM_min);
  EEPROM_writeAnything (sm_max_address, SM_max);


  // VFO's
  ch_info = {vfo_A, LSB};
  address = eprom_base_addr + 1 + sizeof(ch_info) * 1;  // initial infos for VFO A
  EEPROM_writeAnything(address, ch_info);

  //  ch_info={vfo_B, bfo1_usb, USB};  //or
  ch_info.s_vfo = vfo_B;   // initial values of VFO B
  ch_info.s_sb = 2 ;
  address = eprom_base_addr + 1 + sizeof(ch_info) * 2;  // initial infos for VFO B
  EEPROM_writeAnything(address, ch_info);

  // Now store next 13 channels

  for ( i = 1; i <= MAX_BANDS; i++)  // starting from 1st entry in table of freq
  {
    ch_info.s_vfo = VFO_T[i];
    if (VFO_T[i] < 10000000)
    {
      ch_info.s_sb = LSB;
    }
    else
    {
      ch_info.s_sb = USB;
    }
    address = eprom_base_addr + 1 + sizeof(ch_info) * (i + 2); // first byte for magic no and first 2 infos for VFO A & B
    EEPROM_writeAnything(address, ch_info);
  }

  vfo = 7100000;
  for ( i = MAX_BANDS + 1; i <= 24; i++) // starting from
  {
    ch_info.s_vfo = vfo;
    ch_info.s_sb = LSB;
    address = eprom_base_addr + 1 + sizeof(ch_info) * (i + 2); // first byte for magic no and first 2 infos for VFO A & B
    EEPROM_writeAnything(address, ch_info);
  }
  vfo = 14000000;
  for ( i = 25; i <= 50; i++)  // starting from 160m
  {
    ch_info.s_vfo = vfo;
    ch_info.s_sb = USB;
    address = eprom_base_addr + 1 + sizeof(ch_info) * (i + 2); // first byte for magic no and first 2 infos for VFO A & B
    EEPROM_writeAnything(address, ch_info);
  }
  vfo = 14200000;
  for (int i = 51; i <= max_memory_ch; i++)  // starting from 160m
  {
    ch_info.s_vfo = vfo;
    ch_info.s_sb = USB;
    address = eprom_base_addr + 1 + sizeof(ch_info) * (i + 2); // first byte for magic no and first 2 infos for VFO A & B
    EEPROM_writeAnything(address, ch_info);
  }
  //display_msg(60, "           ");
  // clear area
  tft.drawRoundRect(memx, memy, memwd, memht, roundness, RED);  // Mem box outline
  tft.fillRoundRect(memx + 2, memy + 2, memwd - 4, memht - 4, roundness - 4, GREY); //Mem box
  tft.setCursor(memx + 20, memy + 5);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.print("MEM ");
  display_mem();
}

void read_eprom()     // should be called at powerup to retrieve stored values for vfos A and B
{
  //display_msg(60, "Read EEPROM");
  display_mem_msg("RdEPr");
  // read various parameters in the memory beginning
  EEPROM_readAnything (bfo1_USB_address, bfo1_USB);  // stored values
  EEPROM_readAnything (bfo1_LSB_address, bfo1_LSB);
  EEPROM_readAnything (bfo2_address, bfo2);
  EEPROM_readAnything (PTT_type_address, active_PTT_in);
  EEPROM_readAnything (Tx_timeout_mode_address, Tx_timeout_mode);
  EEPROM_readAnything (TxTmO_Time_address, Tx_timeout);
  EEPROM_readAnything (touch_sens_address, ts_delay);
  EEPROM_readAnything (offsets_base_address, offsets);  // for loop may be reqd
  EEPROM_readAnything (sm_min_address, SM_min);
  EEPROM_readAnything (sm_max_address, SM_max);


  address = eprom_base_addr + 1 + sizeof(ch_info) * 1;  // first infos for VFO A
  EEPROM_readAnything(address, ch_info);
  vfo_A = ch_info.s_vfo ;   // initial values of VFO A
  sb_A = ch_info.s_sb;

  address = eprom_base_addr + 1 + sizeof(ch_info) * 2;  // second infos for VFO B
  EEPROM_readAnything(address, ch_info);
  vfo_B = ch_info.s_vfo ;   // initial values of VFO B
  sb_B = ch_info.s_sb;

  memCh = 1;
  read_ch();   // for 1st mem channel also
  //display_msg(60, "           ");
  // clear area
  tft.drawRoundRect(memx, memy, memwd, memht, roundness, RED);  // Mem box outline
  tft.fillRoundRect(memx + 2, memy + 2, memwd - 4, memht - 4, roundness - 4, GREY); //Mem box
  tft.setCursor(memx + 20, memy + 5);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.print("MEM ");
  display_mem();

}

void read_ch()    // read channel info from eeprom when ever mem ch no changed
{
  address = eprom_base_addr + 1 + sizeof(ch_info) * (memCh + 2); // info for mem channel displayed
  EEPROM_readAnything(address, ch_info);
  vfo_M = ch_info.s_vfo ;
  sb_M = ch_info.s_sb;
}

void write_ch()   // write memory channel into eeprom
{
  ch_info = {vfo_M, sb_M};
  address = eprom_base_addr + 1 + sizeof(ch_info) * (memCh + 2); // initial infos for VFO A
  EEPROM_writeAnything(address, ch_info);
}

void write_vfo_A()
{
  ch_info = {vfo_A,  sb_A};
  address = eprom_base_addr + 1 + sizeof(ch_info) * 1;  // initial infos for VFO A
  EEPROM_writeAnything(address, ch_info);

}

void write_vfo_B()
{
  ch_info = {vfo_B, sb_B};
  address = eprom_base_addr + 1 + sizeof(ch_info) * 2;  // initial infos for VFO B
  EEPROM_writeAnything(address, ch_info);
}

void display_mem()
{
  tft.setCursor(memx + 75, memy + 5); //(185, 12);
  tft.setTextSize(3);
  tft.setTextColor(GREEN, GREY);
  if (memCh < 10)
    tft.print("0");
  tft.print(memCh);
  if (memCh < 100)
    tft.print(" ");
  if (!xch_M)
    read_ch();
  else
    xch_M = 0;

  if (vfo_M_sel)
  {
    vfo = vfo_M;
    bfo1 = bfo_M;
    display_bfo1();
    set_bfo1();
    sideband = sb_M;
    display_sideband();
    display_frequency();
  }
  else
    display_frequency2();
}

void display_mem_msg(String msg)
{
  tft.setCursor(memx + 10, memy + 5); //(185, 12);
  tft.setTextSize(3);
  tft.setTextColor(RED, GREY);
  tft.print(msg);
}

void displ_rx()
{
  tft.drawRoundRect(txrx, txry, txrwd, txrht, roundness, RED);  // TxRx box outline
  tft.fillRoundRect(txrx + 2, txry + 2, txrwd - 4, txrht - 4, roundness - 4, GREEN); //TxRx box
  tft.setCursor(txrx + 10, txry + 5);
  tft.setTextSize(3);
  tft.setTextColor(BLUE);
  tft.print("Rx");
}
void displ_tx()
{
  tft.drawRoundRect(txrx, txry, txrwd, txrht, roundness, RED);  // TxRx box outline
  tft.fillRoundRect(txrx + 2, txry + 2, txrwd - 4, txrht - 4, roundness - 4, RED); //TxRx box
  tft.setCursor(txrx + 10, txry + 5);
  tft.setTextSize(3);
  tft.setTextColor(BLUE);
  tft.print("Tx");
}

void display_vfo()
{
  tft.setCursor(vfoABMx, vfoABMy);    //(25, 50);
  tft.setTextSize(4);
  tft.setTextColor(WHITE, BLACK);
  old_band = bnd_count;

  if (vfo_M_sel)
    tft.print("M");  // Mem   ....
  else if (vfo_A_sel)
    tft.print("A");  // VFO A or B  ....
  else
    tft.print("B");  // VFO A or B  ....

  display_frequency2(); // 2nd line of display only when vfos changed
  set_band();  // select and display band according to frequency displayed
  display_band();
}


void display_frequency()
{
  tft.setTextSize(4);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(frqx + 2, frqy + 8); //(70, 50);
  if (vfo < 10000000)
    tft.print(" ");
  tft.print(vfo / 1000.0, 3);
}

void display_frequency2()
{
  //other 2 vfo's displayed below
  tft.setTextSize(2);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(frq2x1, frq2y);   //(25, 93);

  if (vfo_A_sel)
  {
    if (splitON)
      tft.setTextColor(RED, BLACK); //else use WHITE / BLACK
    tft.print("B ");
    tft.print(vfo_B / 1000.0, 3);
    tft.print(" ");
    tft.setCursor(frq2x2, frq2y);   // (170, 93);
    tft.setTextColor(WHITE, BLACK);
    tft.print("M ");
    tft.print(vfo_M / 1000.0, 3);
    tft.print(" "); // takes care of previous leftover digit
  }
  if (vfo_B_sel)
  {
    if (splitON)
      tft.setTextColor(GREEN, BLACK); //else use WHITE / BLACK
    tft.print("A ");
    tft.print(vfo_A / 1000.0, 3);
    tft.print(" ");
    tft.setCursor( frq2x2, frq2y);  //(170, 93);
    tft.setTextColor(WHITE, BLACK);
    tft.print("M ");
    tft.print(vfo_M / 1000.0, 3);
    tft.print(" ");
  }
  if (vfo_M_sel)
  {
    if (splitON)
      tft.setTextColor(GREEN, BLACK);// else use WHITE / BLACK
    tft.print("A ");
    tft.print(vfo_A / 1000.0, 3);
    tft.print(" ");
    tft.setCursor(frq2x2, frq2y);  //(170, 93);
    if (splitON)
      tft.setTextColor(RED, BLACK); //else use WHITE / BLACK
    tft.print("B ");
    tft.print(vfo_B / 1000.0, 3);
    tft.print(" ");
    tft.setTextColor(WHITE, BLACK);
  }
} // end of display_frequency2()

void set_band()       // from frequecy determine band and activate corresponding relay TBD
{
  for (int i = MAX_BANDS; i >= 0; i--)
  {
    if ((vfo >= F_MIN_T[i]) && (vfo <= F_MAX_T[i]))
    {
      bnd_count = i ;
      break;
    }
  }
  //  digitalWrite(band_cntrl[old_band], LOW);   // deactivate old band relay
  //  digitalWrite(band_cntrl[bnd_count], HIGH); // activate new selected band
}

void display_band()
{
  tft.setCursor(bandx + 2, bandy + 10); //22, 125);
  tft.setTextSize(2);
  tft.setTextColor(WHITE, GREY);
  //  changed_f = 1;           // ???? why here
  tft.print(B_NAME_T[bnd_count]);
}  // end of Display-band()

void change_band() {
  display_band();
  F_MIN = F_MIN_T[bnd_count];
  F_MAX = F_MAX_T[bnd_count];
  vfo = VFO_T[bnd_count];
  //  set_band();
  changed_f = 1;
}  // end of change_band()

// Displays the frequency change step
void display_step()
{
   tft.setCursor(stpx + 3, stpy + 10); // (117, 125);
   tft.setTextSize(2);
   tft.setTextColor(WHITE, GREY);
   tft.print(step_sz_txt[step_index]);
}

void display_sideband() {
  tft.setCursor(sbx + 18, sby + 10); //(261, 125);
  tft.setTextSize(2);
  tft.setTextColor(WHITE, GREY);
  if (sideband == LSB)
  {
    tft.print("LSB");
  }
  else if (sideband == USB)
  {
    tft.print("USB");
  }
}

void update_row5()
{
  if (screen_no == 0)    // bfo1 and bfo2 in f2 and f4 buttons
  {
    // display bfo1 and bfo2
    tft.setTextSize(2);
    tft.setTextColor(WHITE, BLUE);
    tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, BLUE); //F1
    tft.setCursor(f1x + 5, f1y + 10);
    tft.print("B1");
    tft.fillRoundRect(f2x + 2, f2y + 2, f2wd - 4, f2ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    display_bfo1();

    tft.fillRoundRect(f3x + 2, f3y + 2, f3wd - 4, f3ht - 4, roundness - 4, BLUE);
    tft.setTextColor(WHITE, BLUE);
    tft.setCursor(f3x + 5, f3y + 10);
    tft.print("B2");
    tft.fillRoundRect(f4x + 2, f4y + 2, f4wd - 4, f4ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    display_bfo2();
  }
  else if (screen_no == 1)
  {
    // display PTT Type and Tx Time out
    tft.setTextSize(2);
    tft.setTextColor(WHITE, BLUE);
    tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, BLUE); //F1
    tft.setCursor(f1x + 5, f1y + 10);
    tft.print("PT");
    tft.setCursor(f2x + 15, f2y + 10);
    tft.fillRoundRect(f2x + 2, f2y + 2, f2wd - 4, f2ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    if (!active_PTT_in)
      tft.print("Toggle");
    else
      tft.print("Normal");

    if ( Tx_timeout_mode)
      tft.setTextColor(RED, GREY); // Activated then RED Fonts
    else
      tft.setTextColor(GREEN, GREY);

    tft.fillRoundRect(f3x + 2, f3y + 2, f3wd - 4, f3ht - 4, roundness - 4, GREY);
    tft.setCursor(f3x + 5, f3y + 10);
    tft.print("TO");
    tft.setCursor(f4x + 40, f4y + 10);
    tft.fillRoundRect(f4x + 2, f4y + 2, f4wd - 4, f4ht - 4, roundness - 4, GREY);
    // tft.setTextColor(RED, GREY);
    tft.print(Tx_timeout);
  }
  else if (screen_no == 2)   // band offset
  {
    tft.setTextSize(2);
    tft.setTextColor(WHITE, BLUE);
    tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, BLUE); //F1
    tft.setCursor(f1x + 5, f1y + 8);
    tft.print("OF");
    tft.setCursor(f2x + 25, f2y + 10);
    tft.fillRoundRect(f2x + 2, f2y + 2, f2wd - 4, f2ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    tft.print(offsets[bnd_count]);

    tft.setCursor(f3x + 5, f3y + 10);
    tft.fillRoundRect(f3x + 2, f3y + 2, f3wd - 4, f3ht - 4, roundness - 4, BLUE);
    tft.setTextColor(WHITE, BLUE);
    tft.print("TS"); // Touch Sensitivity
    tft.setCursor(f4x + 40, f4y + 10);
    tft.fillRoundRect(f4x + 2, f4y + 2, f4wd - 4, f4ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    tft.print(ts_delay); // delay time between two touches
  }

  else if (screen_no == 3 && !txstatus)   // Smeter if Rx & Power meter value
  {
    tft.setTextSize(2);
    tft.setTextColor(WHITE, GREY);
    tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, GREY); //F1
    tft.setCursor(f1x + 5, f1y + 8);
    tft.print("SL"); // Min value of S meter  (S meter Low)
    tft.setCursor(f2x + 25, f2y + 10);
    tft.fillRoundRect(f2x + 2, f2y + 2, f2wd - 4, f2ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    tft.print(SM_min); // Value for displaying Full scale from A12, Adjustable


    tft.setCursor(f3x + 5, f3y + 10);
    tft.fillRoundRect(f3x + 2, f3y + 2, f3wd - 4, f3ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    tft.print("SH"); // Max value of S Meter (S meter High)
    tft.setCursor(f4x + 40, f4y + 10);
    tft.fillRoundRect(f4x + 2, f4y + 2, f4wd - 4, f4ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    tft.print(SM_max); // S9 value ~65% of Full scale, Not adjustable
  }
  else if(screen_no == 3 and txstatus)  // if TX on Power meter
  {
    tft.setTextSize(2);
    tft.setTextColor(PINK, GREY);
    tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, GREY); //F1
    tft.setCursor(f1x + 5, f1y + 8);
    tft.print("PL"); // Min value of Power meter  (P meter Low)
    tft.setCursor(f2x + 25, f2y + 10);
    tft.fillRoundRect(f2x + 2, f2y + 2, f2wd - 4, f2ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    tft.print(PM_min); // Value for displaying Full scale from A11, Adjustable


    tft.setCursor(f3x + 5, f3y + 10);
    tft.fillRoundRect(f3x + 2, f3y + 2, f3wd - 4, f3ht - 4, roundness - 4, GREY);
    tft.setTextColor(PINK, GREY);
    tft.print("PH"); // Max value of S Meter (P meter High)
    tft.setCursor(f4x + 40, f4y + 10);
    tft.fillRoundRect(f4x + 2, f4y + 2, f4wd - 4, f4ht - 4, roundness - 4, GREY);
    tft.setTextColor(WHITE, GREY);
    tft.print(PM_max); // P meter Max value
  }

}

void display_bfo1()    // bfo1 ver 3.1 at 5th row
{
  tft.setTextSize(2);
  tft.setTextColor(WHITE, GREY);
  tft.setCursor(f2x + 5, f2y + 10);
  if (bfo1 < 10000000)
    tft.print(" ");
  tft.print(bfo1);
}

void display_bfo2()
{
  tft.setTextSize(2);
  tft.setTextColor(WHITE, GREY);
  tft.setCursor(f4x + 5, f4y + 10);
  if (bfo2 < 10000000)
    tft.print(" ");
  tft.print(bfo2);
}

void display_msg(int xposn, String msg)
{ tft.setTextSize(2); // may setup some soft buttons here
  tft.setCursor(xposn, boty);
  tft.setTextColor(WHITE, BLUE);
  tft.println(msg);
}
void debug_msg(int xposn, int msg)
{ tft.setTextSize(2); // may setp some soft buttons here
  tft.setCursor(xposn, boty);    //223);
  tft.setTextColor(WHITE, BLUE);
  tft.write(msg);
}

void displ_split_button()
{
  if (!splitON)
  {
    tft.drawRoundRect(splx, sply, splwd, splht, roundness, GREEN); // F3 button outline
    tft.fillRoundRect(splx + 2, sply + 2, splwd - 4, splht - 4, roundness - 4, PURPLE); //Split
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(splx + 10, sply + 10);
    tft.print("SPLIT");
    //  display_vfo();
    display_frequency2();
  }
  else   // under Split mode control
  {
    tft.drawRoundRect(splx, sply, splwd, splht, roundness, GREEN); // F3 button outline
    tft.fillRoundRect(splx + 2, sply + 2, splwd - 4, splht - 4, roundness - 4, YELLOW); //Split
    tft.setTextSize(2);
    tft.setTextColor(RED);
    tft.setCursor(splx + 10, sply + 10);
    tft.print("SPLIT");
    //  vfo_selA();
    //   display_vfo();
    display_frequency2();
  }
}

void displ_timeout_button()
{
  if (Tx_timeout_mode)   // button red
  {
    tft.drawRoundRect(f1x, f1y, f1wd, f1ht, roundness, WHITE); // TxTmO button outline TxTimeOut
    tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, RED); //F1
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(f1x + 5, f1y + 8);
    tft.print("TxTmO");
    Tx_start_time = 0;    // timer acually starts by PTT in rx_tx_ptt() function
  }
  else     // button green when not in Tx timeout mode
  {
    tft.drawRoundRect(f1x, f1y, f1wd, f1ht, roundness, WHITE); // TxTmO button outline TxTimeOut
    tft.fillRoundRect(f1x + 2, f1y + 2, f1wd - 4, f1ht - 4, roundness - 4, GREEN); //F1
    tft.setTextSize(2);
    tft.setTextColor(BLUE);
    tft.setCursor(f1x + 5, f1y + 8);
    tft.print("TxTmO");
  }
}

void dispPos()
{
  // all these coordinates and sizes need to be scaled for different size of displays automagically
  //  should determine these values by querying tft and touch - maybe in setup
  // 320 wd X 240  ht display

  roundness = 4;   // orig 14
  spacing = 3;
  buttonht = ht / 7; // 37;   general height of buttons 26
  buttonwd = ((wd - 3 * spacing) / 4); // max 4 buttons 

 
  // First row of buttons
  firstrowy = 3;   //ht/48 
  // VFO button related
  vfox =  spacing;   //wd /16 ;  // 20; top of screen
  vfoy = firstrowy;    //ht/48; // 5;
  vfowd = buttonwd ;  // wd/4.2; //  75;
  vfoht = buttonht;

  // MEM button related
  memx = vfox + vfowd + spacing;   //110; 5 px space
  memy = firstrowy;    //5;
  memwd = 2 * buttonwd;    //135; reduce spacing 5
  memht = buttonht;    //37;

  // Tx Rx box
  txrx = memx + memwd + spacing; // 260;
  txry = firstrowy;    //5;
  txrwd = buttonwd;    //50;
  txrht = buttonht;    //37;

    // Scan Down area
  scandnx = 35;
  scandny = firstrowy + buttonht + 5;
  scandnwd = 20;
  scandnht = 1.3 * buttonht;

  // frequeny box  Second Row
  frqx = scandnx + scandnwd + 5;
  frqy = firstrowy + buttonht + 5; // 45;
  frqwd = 3.0 * buttonwd;   //  3.0 was 3.5
  frqht = 1.3 * buttonht ;    //40;

  // Scan Up area
  scanupx = frqx + frqwd + 5;
  scanupy = scandny;
  scanupwd = scandnwd;
  scanupht = 1.3 * buttonht;

  vfoABMx = spacing;  // 25 where A/B or M is displayed
  vfoABMy = frqy + 8;

  frq2x1 = vfoABMx + spacing; // next line to display other freqs
  frq2x2 = vfoABMx + wd / 2 - 5; // x pos for 2nd freq display
  frq2y = vfoABMy + frqht + 1;   // second display of VFO /mem

  // band button  Third Row  : below freq2 display row
  bandx = spacing;    //20;
  bandy = frq2y + 20;     // 113; 2 for freq2 displ
  bandwd = 1.4 * buttonwd;    //89;
  bandht = buttonht;  //37;

  //step button
  stpx = bandx + bandwd + spacing;  // 114;
  stpy = bandy;     //113
  stpwd = 1.6 *  buttonwd  ;   //124;
  stpht = buttonht;   //37;
 
  // sideband button
  sbx = stpx + stpwd + spacing ;   //243;
  sby = bandy;         //113;
  sbwd =  buttonwd;  //69;
  sbht = buttonht;   //37;


  // modified Line 4:  V><M, SPLIT, SAVE : v 3.1 onwards
  
   // Previous screen Just display "<"
  prsnx = spacing;
  prsny = bandy + buttonht + 2;
  prsnwd = 30;
  prsnht = buttonht;

  // vfo < > mem button   Fourth Row
 
  vmx = prsnx + prsnwd + spacing;
  vmy = bandy + bandht + 2;
  vmwd =  buttonwd * 1.2;     // was 1.4 times
  vmht = buttonht;

  // SPLIT button
  splx = vmx + vmwd + spacing;
  sply = vmy;
  splwd = buttonwd;
  splht = buttonht;

  // save button
  svx = splx + splwd + spacing;
  svy = sply;
  svwd =  buttonwd;
  svht = buttonht;

   // Next screen button ">"
  nxsnx = svx + svwd + spacing;
  nxsny = svy;
  nxsnwd = 30;
  nxsnht = buttonht;


  // New line 5 :PrevScrn <, BFO1, BFO2 , NextScrn >:  v 3.1 onwards
 
   // F1 button 2 char msg button
  f1x = spacing;
  f1y = prsny + buttonht +2;
  f1wd = 0.5 * buttonwd;
  f1ht = buttonht; 

  // F2 button for bfo1 / PTT Type   etc variable params
  f2x = f1x + f1wd + spacing;
  f2y = f1y;
  f2wd = 1.5 * buttonwd; 
  f2ht = buttonht; 

  // F3 button for 2 char msg
  f3x = f2x + f2wd + spacing;
  f3y = f1y; // 192,
  f3wd = 0.5 * buttonwd;
  f3ht = buttonht; 

  //F4 button for bfo2 / Tx Timeout etc
  f4x = f3x + f3wd + spacing;
  f4y = f1y;        
  f4wd = 1.5 * buttonwd; 
  f4ht = buttonht; 

  // BOT MESSAGE STRIP /S Meter
  botx =  5;
  boty = f3y + f3ht; 
  botwd = wd - botx - 70; 
  botht = ht - (f3y + f3ht + 1); // 20
}

void check_CAT()
//void serialEvent()
{
  while (Serial.available())
  {
    CAT_buff[CAT_buff_ptr] = Serial.read();
    CAT_buff_ptr++;
    if (CAT_buff_ptr == 5)
    {
      CAT_buff_ptr = 0;
      CAT_ctrl = 1;
      exec_CAT(CAT_buff);
    }
  }
}

// The next 4 functions are needed to implement the CAT protocol, which
// uses 4-bit BCD formatting.
//
byte setHighNibble(byte b, byte v) {
  // Clear the high nibble
  b &= 0x0f;
  // Set the high nibble
  return b | ((v & 0x0f) << 4);
}

byte setLowNibble(byte b, byte v) {
  // Clear the low nibble
  b &= 0xf0;
  // Set the low nibble
  return b | (v & 0x0f);
}

byte getHighNibble(byte b) {
  return (b >> 4) & 0x0f;
}

byte getLowNibble(byte b) {
  return b & 0x0f;
}

// Takes a number and produces the requested number of decimal digits, starting
// from the least significant digit.
//
void getDecimalDigits(unsigned long number, byte* result, int digits) {
  for (int i = 0; i < digits; i++) {
    // "Mask off" (in a decimal sense) the LSD and return it
    result[i] = number % 10;
    // "Shift right" (in a decimal sense)
    number /= 10;
  }
}

// Takes a frequency and writes it into the CAT command buffer in BCD form.
//
void writeFreq(unsigned long freq, byte* cmd) {
  // Convert the frequency to a set of decimal digits. We are taking 9 digits
  // so that we can get up to 999 MHz. But the protocol doesn't care about the
  // LSD (1's place), so we ignore that digit.
  byte digits[9];
  getDecimalDigits(freq, digits, 9);
  // Start from the LSB and get each nibble
  cmd[3] = setLowNibble(cmd[3], digits[1]);
  cmd[3] = setHighNibble(cmd[3], digits[2]);
  cmd[2] = setLowNibble(cmd[2], digits[3]);
  cmd[2] = setHighNibble(cmd[2], digits[4]);
  cmd[1] = setLowNibble(cmd[1], digits[5]);
  cmd[1] = setHighNibble(cmd[1], digits[6]);
  cmd[0] = setLowNibble(cmd[0], digits[7]);
  cmd[0] = setHighNibble(cmd[0], digits[8]);
}

// This function takes a frquency that is encoded using 4 bytes of BCD
// representation and turns it into an long measured in Hz.
//
// [12][34][56][78] = 123.45678? Mhz
//
unsigned long readFreq(byte* cmd) {
  // Pull off each of the digits
  byte d7 = getHighNibble(cmd[0]);
  byte d6 = getLowNibble(cmd[0]);
  byte d5 = getHighNibble(cmd[1]);
  byte d4 = getLowNibble(cmd[1]);
  byte d3 = getHighNibble(cmd[2]);
  byte d2 = getLowNibble(cmd[2]);
  byte d1 = getHighNibble(cmd[3]);
  byte d0 = getLowNibble(cmd[3]);
  return
    (unsigned long)d7 * 100000000L +
    (unsigned long)d6 * 10000000L +
    (unsigned long)d5 * 1000000L +
    (unsigned long)d4 * 100000L +
    (unsigned long)d3 * 10000L +
    (unsigned long)d2 * 1000L +
    (unsigned long)d1 * 100L +
    (unsigned long)d0 * 10L;
}

void update_CAT()
{
  //CAT_get_freq();
 // CAT_set_mode();
}

void CAT_set_freq()   // first four bytes in buffer are freq in compressed bcd
{
  // This function sets  the VFO frequency
  Serial.write(0); // ACK
  vfo = readFreq(CAT_buff);
  set_vfo();
  // changed_f = 1; //update display gives prob with wsjtx reading current freq time out
  display_frequency();
  set_bfo1();
  set_band();
  display_band();
  CAT_ctrl = 0;
}

void CAT_SetSplit()
{
  Serial.write(0); // ACK
  CAT_ctrl = 0;

}

void CAT_get_freq()
{
  writeFreq(vfo, CAT_buff);
  if (sideband == LSB)
    CAT_buff[4] = 0x00;
  else
    CAT_buff[4] = 0x01;

  for (i = 0; i < 5; i++)
  {
    Serial.write(CAT_buff[i]);
  }

//  Serial.write(0);
  CAT_ctrl = 0;
}

void CAT_ptt_on()
{
  PTT_by_CAT=true;
  ptt_ON();
  Serial.write(0);
  CAT_ctrl = 0;
}

void CAT_ptt_off()
{
  PTT_by_CAT=false;
  ptt_OFF();
  Serial.write(0);
  CAT_ctrl = 0;
}

void CAT_set_mode()
{
  if (CAT_buff[0] == 00)
    sideband = LSB ;
  else
    sideband = USB;

  set_bfo1();
  display_sideband();
  Serial.write(0);
  CAT_ctrl = 0;
}

void CAT_toggle_VFO()   // only between VFO A & B
{
  //Serial.write(0x00);    // just send 1 bytes ACK
  if (vfo_A_sel)
    vfo_selB();
  else
    vfo_selA();
  // CAT_get_freq();
  Serial.write(0x00);    // just send 1 bytes ACK
  CAT_ctrl = 0;
}

void CAT_Eeprom_read()
{
  Serial.write(0x10);    // Mem 64 = 10 means 38400 baud
  Serial.write(0x00);    // Mem 65 = 00
  CAT_ctrl = 0;

  //  Serial.write(0x10);  // cat rate 38400
}

void CAT_Tx_status()
{
  Serial.write(0x88);    // just send a dummy byte
  CAT_ctrl = 0;

}

void exec_CAT(byte* cmd)
{
  switch (cmd[4])
  {
    case 0x01 :   //Set Frequency
      CAT_set_freq();
      break;

    case 0x02 : //Split On
    case 0x82:  //Split Off
      CAT_SetSplit();
      break;

    case 0x03 :   //Read Frequency and mode
      CAT_get_freq();   // retreive freq & mode
      break;

    case 0x07 :   //Set Operating  Mode
      CAT_set_mode();
      // modes 00 - LSB, 01 - USB, 02 - CW, 03 - CWR, 04 - AM, 08 - FM, 0A - DIG, 0C - PKT
      break;

    case 0x08 : //Set PTT_ON
      CAT_ptt_on();
      break;

    case 0x88:  //Set PTT Off
      CAT_ptt_off();
      break;

    case 0x81:  //Toggle VFO
      CAT_toggle_VFO(); // between A & B
      break;

    case 0xDB:  //Read uBITX EEPROM Data
      Serial.write(0x00);    // just send a dummy byte
      break;

    case 0xBB:  //Read FT-817 EEPROM Data  (for comfortable)
      CAT_Eeprom_read();
      break;

    case 0xDC:  //Write uBITX EEPROM Data
      Serial.write(0x00);    // just send a dummy byte
      break;

    case 0xBC:  //Write FT-817 EEPROM Data  (for comfirtable)
      Serial.write(0x00);    // just send a dummy byte
      break;

    case 0xE7 :       //Read RX Status
      Serial.write(0x00);    // just send a dummy byte
      break;

    case 0xF7:      //Read TX Status
      CAT_Tx_status();
      break;
    default:
      /*
        char buff[16];
        sprintf(buff, "DEFAULT : %x", CAT_BUFF[4]);
        printLine2(buff);
      */
      //     Serial.write(0x00);
    //  Serial.write(0x00);
      CAT_ctrl = 0;
      //   Serial.flush();

      break;
  } //end of switch
  //  CAT_ctrl = 0;
  checkingCAT = 0;
}
