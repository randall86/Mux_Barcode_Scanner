// Mux Barcode Scanner System
// Rev 2.0 (17/05/2018)
// - Maxtrax
#include <DTIOI2CtoParallelConverter.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <USBKeyboard.h>

//#define DEBUG

#define THUMB_SW_BCD8 A3
#define THUMB_SW_BCD4 A2
#define THUMB_SW_BCD2 A1
#define THUMB_SW_BCD1 A0

#define MUX_A2 7
#define MUX_A1 6
#define MUX_A0 5
#define MUX_EN 4
#define MUX_RX 3 //MUX_RX - Transmit Data to MUX
#define MUX_TX 2 //MUX_TX - Receive Data from MUX

#define FIXTURE_SW 9
#define FIXTURE_DET PIN1_7
#define START_DELAY_MSEC 3000 //delay before starting the first scanning - 3 seconds
#define SCANNER_DELAY_MSEC 500 //how long to block for reading the scanner data via cmd
#define DEBOUNCE_FREQ_MSEC 200 //stable time before switch state changed
#define TIMER_FREQ_MSEC 50 //read the switch every 50ms

#define BSCANNER_MAX_NUM 6
#define BSCANNER_BIT_MASK ((1 << BSCANNER_MAX_NUM) - 1)

#define GET_BIT(val, shift) ((val >> (shift & BSCANNER_BIT_MASK)) & 1)
#define NOP __asm__("nop\n\t") //"nop" executes in one machine cycle (at 16 MHz) yielding a 62.5 ns delay

#define MAX_BARCODE_DATA 64
#define CODEXML_HDR_LEN 7

const byte SOH = 0x01;
const byte RS = 0x1E;

typedef struct _bscanner_param_t
{
  int wait;
  byte mux_addr;
  byte en_pin;
  byte delim;
  byte do_decode;
  char *cmd;
}bscanner_param_t;

bscanner_param_t g_bscanner[BSCANNER_MAX_NUM] =
{
  {SCANNER_DELAY_MSEC*6, 0x0, PIN0_0, KEY_TAB, 0, "P%C43\r"}, //tri-scanning
  {SCANNER_DELAY_MSEC, 0x1, PIN0_1, KEY_ENTER, 1, "$%03\r"},
  {SCANNER_DELAY_MSEC, 0x2, PIN0_2, KEY_ENTER, 1, "$%03\r"},
  {SCANNER_DELAY_MSEC, 0x3, PIN0_3, KEY_TAB, 1, "$%03\r"},
  {SCANNER_DELAY_MSEC, 0x4, PIN0_4, KEY_ENTER, 1, "$%03\r"},
  {SCANNER_DELAY_MSEC, 0x5, PIN0_5, KEY_ENTER, 1, "$%03\r"}
};

//PCA9539 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter g_ioExpandr(0x74);

//SoftwareSerial object for the Mux port
SoftwareSerial g_muxSerial(MUX_TX, MUX_RX); // RX, TX

//SimpleTimer object
SimpleTimer g_timer;

byte g_bscannerNum = 0;
byte g_debouncedState = 0;
bool g_isFixtureDetected = false;
bool g_isFixtureRemoved = false;

byte g_barcode_reply[MAX_BARCODE_DATA] = {0};

//send Keyboard command to cycle to the subsequent input
void sendBScannerDelim(byte bscanner_index)
{
  //print delimiter to indicate end of data
  Keyboard.sendKeyStroke(g_bscanner[bscanner_index].delim);
}

void sendBScannerData(byte start_index, byte bscanner_index, byte * raw_data, byte len)
{
  for(byte index = start_index; index < len; index++)
  {
    char data_byte[2] = {raw_data[index], 0};
    Keyboard.print(data_byte);
  }  
}

void decodeBScannerData(byte bscanner_index, byte * raw_data, byte len)
{
  //received data is at least 7 bytes
  if(len > CODEXML_HDR_LEN)
  {
    byte start_index = 0;

    //decode the header(CodeXML-start) to find the start of data
    for(byte raw_index = CODEXML_HDR_LEN; raw_index < len; raw_index++)
    {
      if( (SOH == raw_data[raw_index - 7]) &&
          ('X' == raw_data[raw_index - 6]) &&
          (RS == raw_data[raw_index - 5]) &&
          ('a' == raw_data[raw_index - 4]) &&
          ('p' == raw_data[raw_index - 3]) &&
          ('/' == raw_data[raw_index - 2]) &&
          ('d' == raw_data[raw_index - 1]) )
      {
        start_index = raw_index + 1; //increment to discard the space
        break;
      }
    }

    //send the decoded data
    if(0 != start_index)
    {
      sendBScannerData(start_index, bscanner_index, raw_data, len);
    }
  }
}

//trigger the barcode scanning via command
void scanBScannerCmd(byte bscanner_index)
{
  bool data_available = false;
  byte count = 0;
  memset(g_barcode_reply, 0, MAX_BARCODE_DATA);
  
  g_muxSerial.flush();

  g_muxSerial.print(g_bscanner[bscanner_index].cmd); //trigger the scanning command
  delay(g_bscanner[bscanner_index].wait); //delay for the command reply

  while(g_muxSerial.available())
  {
    data_available = true;
    
    //read and store the raw data for decoding
    g_barcode_reply[count++] = g_muxSerial.read();
    
    //check if raw data exceeds buffer size
    if(count >= MAX_BARCODE_DATA)
    {
      break;
    }
  }
  
  if(data_available)
  {
#ifdef DEBUG
    for(byte i = 0; i < count; i++)
    {
      char data_byte[2] = {g_barcode_reply[i], 0};
      Keyboard.print(data_byte);
    }
    Keyboard.print("\n");
#else
    if(g_bscanner[bscanner_index].do_decode)
    {
      decodeBScannerData(bscanner_index, g_barcode_reply, count);
    }
    else //bypass decoding, send data directly
    {
      sendBScannerData(1, bscanner_index, g_barcode_reply, count);
    }
#endif
  }
}

//trigger the barcode scanning via the external trigger pin
void scanBScannerTrig(byte bscanner_index)
{
  g_muxSerial.flush();
  
  g_ioExpandr.digitalWrite0(g_bscanner[bscanner_index].en_pin, LOW);
  NOP; //60ns delay
  g_ioExpandr.digitalWrite0(g_bscanner[bscanner_index].en_pin, HIGH);

  //get the scanned data
  if(g_muxSerial.available())
  {
    while(g_muxSerial.available())
    {
      char data_byte[2] = {g_muxSerial.read(), 0};
      Keyboard.print(data_byte);
    }
  }
}

void selectBScanner(byte bscanner_index)
{
  //disable the mux
  digitalWrite(MUX_EN, LOW);
  
  NOP; //60ns delay
  
  //update the mux address lines
  digitalWrite(MUX_A0, GET_BIT(g_bscanner[bscanner_index].mux_addr, 0));
  digitalWrite(MUX_A1, GET_BIT(g_bscanner[bscanner_index].mux_addr, 1));
  digitalWrite(MUX_A2, GET_BIT(g_bscanner[bscanner_index].mux_addr, 2));
  
  NOP; //60ns delay
  
  //enable the mux with the updated address
  digitalWrite(MUX_EN, HIGH);
}

//returns true if state changed
bool debounceSW(byte *state)
{
  static uint8_t count = DEBOUNCE_FREQ_MSEC/TIMER_FREQ_MSEC;
  bool state_changed = false;
  
  //read the fixture switch state
  byte raw_state = digitalRead(FIXTURE_SW);
  *state = g_debouncedState;
  
  if(raw_state == g_debouncedState)
  {
    //set the timer which allows a change from current state.
    count = DEBOUNCE_FREQ_MSEC/TIMER_FREQ_MSEC;
  }
  else
  {
    //state has changed - wait for new state to become stable.
    if (--count == 0)
    {
      // Timer expired - accept the change.
      g_debouncedState = raw_state;
      state_changed = true;
      *state = g_debouncedState;
      
      // And reset the timer.
      count = DEBOUNCE_FREQ_MSEC/TIMER_FREQ_MSEC;
    }
  }
  
  return state_changed;
}

void debounceSWRoutine()
{
  byte sw_state = 0;
  
  //if fixture switch state changed, update the state
  if(debounceSW(&sw_state))
  {
    if(sw_state)
    {
      g_isFixtureDetected = true;
    }
    else
    {
      g_isFixtureRemoved = true;
    }
  }
}

byte getBScannerNum()
{
    byte mask = 0x01;
    byte bit00 = (mask & !digitalRead(THUMB_SW_BCD1));
    byte bit01 = (mask & !digitalRead(THUMB_SW_BCD2));
    byte bit02 = (mask & !digitalRead(THUMB_SW_BCD4));
    byte bit03 = (mask & !digitalRead(THUMB_SW_BCD8));

    return ((bit03 << 3) | (bit02 << 2) | (bit01 << 1) | bit00);
}

void setup()
{
  Wire.begin(); //need to start the Wire for I2C devices to function
  
  // define pin modes for tx, rx
  pinMode(MUX_RX, OUTPUT);
  pinMode(MUX_TX, INPUT);
  
  //set the data rate for the SoftwareSerial port
  g_muxSerial.begin(9600);

  //init the barcode scanner enable pins and LED as output
  g_ioExpandr.portMode0(ALLOUTPUT);
  g_ioExpandr.digitalWritePort0(0xFF);
  g_ioExpandr.portMode1(ALLOUTPUT);
  g_ioExpandr.digitalWritePort1(0xFF);

  //init the MUX enable pins
  pinMode(MUX_EN, OUTPUT);
  digitalWrite(MUX_EN, LOW);
  
  //init the MUX address pins
  pinMode(MUX_A0, OUTPUT);
  digitalWrite(MUX_A0, LOW);
  pinMode(MUX_A1, OUTPUT);
  digitalWrite(MUX_A1, LOW);
  pinMode(MUX_A2, OUTPUT);
  digitalWrite(MUX_A2, LOW);

  //init the thumbwheel sw as input pin
  pinMode(THUMB_SW_BCD1, INPUT);
  pinMode(THUMB_SW_BCD2, INPUT);
  pinMode(THUMB_SW_BCD4, INPUT);
  pinMode(THUMB_SW_BCD8, INPUT);

  //init the sw detection input pin
  pinMode(FIXTURE_SW, INPUT);

  //get the number of barcode scanners used
  g_bscannerNum = getBScannerNum(); 
  if(g_bscannerNum > BSCANNER_MAX_NUM)
  {
    g_bscannerNum = BSCANNER_MAX_NUM; //if exceed max, set to max
  }

  //initialize the debounce timer
  g_timer.setInterval(TIMER_FREQ_MSEC, debounceSWRoutine);

  Keyboard.init(); //init the USB Keyboard HID driver
}

void loop()
{
  // this is where the "polling" occurs
  g_timer.run();

  //perform scanning if the fixture is in place
  if(g_isFixtureDetected)
  {
    //set the detection LED to green
    g_ioExpandr.digitalWrite1(FIXTURE_DET, LOW);

    delay(START_DELAY_MSEC);

    //loop through all the barcode scanners
    for(byte index = 0; index < g_bscannerNum; index++)
    {
      //select the barcode scanner mux addr
      selectBScanner(index);

      //perform the scanning and send the data
      //scanBScannerTrig(index);
      scanBScannerCmd(index);

      //trigger sequence to the next input
      sendBScannerDelim(index);
    }
    
    g_isFixtureDetected = false;
  }

  //reset the LEDs when the fixture is removed
  if(g_isFixtureRemoved)
  {
    //reset the barcode LED and detection LED to red
    g_ioExpandr.digitalWritePort1(0xFF);

    g_isFixtureRemoved = false;
  }
}

