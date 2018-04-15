#include <DTIOI2CtoParallelConverter.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

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
#define FIXTURE_LED 8

#define SCANNER_DELAY_MSEC 500 //how long to block for reading the scanner data via cmd
#define DEBOUNCE_FREQ_MSEC 200 //stable time before switch state changed
#define TIMER_FREQ_MSEC 50 //read the switch every 50ms

#define BSCANNER_MAX_NUM 8
#define BSCANNER_BIT_MASK ((1 << BSCANNER_MAX_NUM) - 1)

#define GET_BIT(val, shift) ((val >> (shift & BSCANNER_BIT_MASK)) & 1)
#define NOP __asm__("nop\n\t") //"nop" executes in one machine cycle (at 16 MHz) yielding a 62.5 ns delay

#define MAX_BARCODE_DATA 64
#define CODEXML_HDR_LEN 7

#define SEC_TO_MSEC 1000
#define BYTE_READ_MSEC 100

#define BSCANNER_LIMIT 6 //TODO: demo board max barcode scanner

const int DELAY_ADDR = 0x00; //store the delay value at the first EEPROM byte
const byte DEFAULT_DELAY = 3; //default delay is 3 sec
const byte SOH = 0x01;
const byte RS = 0x1E;

typedef struct _bscanner_param_t
{
  byte mux_addr;
  byte en_pin;
  byte led_pin;
}bscanner_param_t;

bscanner_param_t g_bscanner[BSCANNER_MAX_NUM] =
{
  {0x0, PIN0_0, PIN1_0},
  {0x1, PIN0_1, PIN1_1},
  {0x2, PIN0_2, PIN1_2},
  {0x3, PIN0_3, PIN1_3},
  {0x4, PIN0_4, PIN1_4},
  {0x5, PIN0_5, PIN1_5},
  {0x6, PIN0_6, PIN1_6},
  {0x7, PIN0_7, PIN1_7}
};

//PCA9539 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter g_ioExpandr(0x74);

// As compared to what the Arduino's UART is expecting, RS232 
// devices use inverted voltage levels to represent 0s and 1s.
//SoftwareSerial g_muxSerial(MUX_TX, MUX_RX, true); // RX, TX, true - invert the data
SoftwareSerial g_muxSerial(MUX_TX, MUX_RX); // RX, TX

//SimpleTimer object
SimpleTimer g_timer;

byte g_delay = 0;
byte g_bscannerNum = 0;
byte g_debouncedState = 0;
bool g_isFixtureDetected = false;
bool g_isFixtureRemoved = false;

byte g_barcode_reply[MAX_BARCODE_DATA] = {0};

//process the scanning results
void processScanStatus(void)
{
  bool isStatus = false;
  byte scan_status = Serial.read();

  if(scan_status == '#') //check if first byte is '#' then possibly its change delay cmd
  {
    //read the 2nd byte for the cmd value
    delay(BYTE_READ_MSEC);
    byte cmd_data = Serial.read();
    if(cmd_data != -1)
    {
      //convert the ascii to dec
      if(cmd_data >= '1' && cmd_data <= '9')
      {
        g_delay = cmd_data - '0';
        EEPROM.write(DELAY_ADDR, g_delay); //save to eeprom
      }
    }
    else //if 2nd byte is no data then it is status
    {
      isStatus = true;
    }
  }
  else
  {
    isStatus = true;
  }

  if(isStatus)
  {
    //loop through all the barcode scanners and toggle the LED
    for(byte index = 0; index < BSCANNER_MAX_NUM-1; index++) //TODO: minus 1 as temporary the last relay is used for detection
    {
      g_ioExpandr.digitalWrite1(g_bscanner[index].led_pin, !GET_BIT(scan_status, index));
    }
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
      //print the barcode scanner index
      Serial.print(bscanner_index);
      Serial.print(":");
    
      for(byte index = start_index; index < len; index++)
      {
        Serial.write(raw_data[index]);
      }

      //print LF to indicate end of data
      Serial.println();
    }
  }
}

//trigger the barcode scanning via command
void scanBScannerCmd(byte bscanner_index)
{
  bool data_available = false;
  byte count = 0;
  
  g_muxSerial.flush();

  if(bscanner_index == 0)
  {
    g_muxSerial.print("P%C43\r"); //trigger the tri-scanning command
    delay(SCANNER_DELAY_MSEC*6); //delay 3 sec for the command reply
  }
  else
  {
    g_muxSerial.print("$%03\r"); //trigger the scanning command
    delay(SCANNER_DELAY_MSEC); //delay for the command reply
  }
  
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
    Serial.flush();
#ifdef DEBUG
    for(byte i = 0; i < count; i++)
    {
      Serial.write(g_barcode_reply[i]);
    }
    Serial.println();
#else
    //decode and send the data
    decodeBScannerData(bscanner_index, g_barcode_reply, count);
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

  Serial.flush();

  //print the barcode scanner index
  Serial.print(bscanner_index);
  Serial.print(":");

  //get the scanned data
  if(g_muxSerial.available())
  {
    while(g_muxSerial.available())
    {
      Serial.write(g_muxSerial.read());
    }
  }

  //print LF to indicate end of data
  Serial.println();
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
  
  if (raw_state == g_debouncedState)
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
  Serial.begin(115200);
  
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

  //init the fixture LED output pin and sw detection input pin
  pinMode(FIXTURE_LED, OUTPUT);
  digitalWrite(FIXTURE_LED, LOW);
  pinMode(FIXTURE_SW, INPUT);

  //get the number of barcode scanners used
  g_bscannerNum = getBScannerNum(); 
  if(g_bscannerNum > BSCANNER_LIMIT)
  {
    g_bscannerNum = BSCANNER_LIMIT; //if exceed max, set to max
  }

  //get the delay stored in the EEPROM
  g_delay = EEPROM.read(DELAY_ADDR);

  //if delay value is invalid set to default and write back to EEPROM
  if( (g_delay < 1) && (g_delay > 9) )
  {
    g_delay = DEFAULT_DELAY;
    EEPROM.write(DELAY_ADDR, g_delay); //save to eeprom
  }

  //initialize the debounce timer
  g_timer.setInterval(TIMER_FREQ_MSEC, debounceSWRoutine);
}

void loop()
{
  // this is where the "polling" occurs
  g_timer.run();

  //perform scanning if the fixture is in place
  if(g_isFixtureDetected)
  {
    //toggle the fixture detected LED to green
    digitalWrite(FIXTURE_LED, HIGH);

    //TODO: temporary set as the actual FIXTURE_LED is not connected
    g_ioExpandr.digitalWrite1(PIN1_7, LOW);

    delay(g_delay*SEC_TO_MSEC);
    
    //loop through all the barcode scanners
    for(byte index = 0; index < g_bscannerNum; index++)
    {
      //select the barcode scanner mux addr
      selectBScanner(index);

      //perform the scanning and send the data
      //scanBScannerTrig(index);
      scanBScannerCmd(index);
    }

    g_isFixtureDetected = false;
  }

  //reset the LEDs when the fixture is removed
  if(g_isFixtureRemoved)
  {
    //toggle the LED to red when fixture is removed
    digitalWrite(FIXTURE_LED, LOW);

    //reset the barcode LED to red
    g_ioExpandr.digitalWritePort1(0xFF);

    g_isFixtureRemoved = false;
  }

  if(Serial.available())
  {
    processScanStatus();
  }
}

