#include <DTIOI2CtoParallelConverter.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

#define MUX_A2 7
#define MUX_A1 6
#define MUX_A0 5
#define MUX_EN 4
#define MUX_RX 3
#define MUX_TX 2

#define FIXTURE_SW 9
#define FIXTURE_LED 8

#define SCANNER_DELAY_MSEC 50 //how long to block for reading the scanner data
#define DEBOUNCE_FREQ_MSEC 200 //stable time before switch state changed
#define TIMER_FREQ_MSEC 50 //read the switch every 50ms

#define BSCANNER_MAX_NUM 8
#define BSCANNER_BIT_MASK ((1 << BSCANNER_MAX_NUM) - 1)

#define GET_BIT(val, shift) ((val >> (shift & BSCANNER_BIT_MASK)) & 1)
#define NOP __asm__("nop\n\t") //"nop" executes in one machine cycle (at 16 MHz) yielding a 62.5 ns delay

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

SoftwareSerial g_muxSerial(MUX_RX, MUX_TX); // RX, TX

//SimpleTimer object
SimpleTimer g_timer;

byte g_debouncedState = 0;
bool g_isFixtureDetected = false;
bool g_isFixtureRemoved = false;

void selectBScanner(byte index)
{
  //disable the mux
  digitalWrite(MUX_EN, LOW);
  
  NOP; //60ns delay
  
  //update the mux address lines
  digitalWrite(MUX_A0, GET_BIT(g_bscanner[index].mux_addr, 0));
  digitalWrite(MUX_A1, GET_BIT(g_bscanner[index].mux_addr, 1));
  digitalWrite(MUX_A2, GET_BIT(g_bscanner[index].mux_addr, 2));
  
  NOP; //60ns delay
  
  //enable the mux with the updated address
  digitalWrite(MUX_EN, HIGH);
}

void scanBScanner(byte index)
{
  //enable barcode scanning
  g_ioExpandr.digitalWrite0(g_bscanner[index].en_pin, LOW);

  //print the barcode scanner index
  Serial.print(index);
  Serial.print(":");
  
  //get the scanned data
  while(g_muxSerial.available())
  {
    Serial.write(g_muxSerial.read());
    delay(SCANNER_DELAY_MSEC); //delay for complete data
  }

  //print LF to indicate end of data
  Serial.println();
  
  //disable barcode scanning
  g_ioExpandr.digitalWrite0(g_bscanner[index].en_pin, HIGH);
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

void setup()
{
  Wire.begin(); //need to start the Wire for I2C devices to function
  Serial.begin(115200);
  
  // define pin modes for tx, rx
  pinMode(MUX_RX, INPUT);
  pinMode(MUX_TX, OUTPUT);
  
  //set the data rate for the SoftwareSerial port
  g_muxSerial.begin(115200);

  //init the barcode scanner enable pins and LED as output
  g_ioExpandr.portMode0(ALLOUTPUT);
  g_ioExpandr.digitalWritePort0(0xFF);
  g_ioExpandr.portMode1(ALLOUTPUT);
  g_ioExpandr.digitalWritePort1(0x00);

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

  //init the fixture LED output pin and sw detection input pin
  pinMode(FIXTURE_LED, OUTPUT);
  digitalWrite(FIXTURE_LED, LOW);
  pinMode(FIXTURE_SW, INPUT);

  //initialize the debounce timer
  g_timer.setInterval(TIMER_FREQ_MSEC, debounceSWRoutine);
}

void loop()
{
  //perform scanning if the fixture is in place
  if(g_isFixtureDetected)
  {
    //toggle the fixture detected LED to green
    digitalWrite(FIXTURE_LED, HIGH);
    
    //loop through all the barcode scanners
    for(int index = 0; index < BSCANNER_MAX_NUM; index++)
    {
      selectBScanner(index); //select the barcode scanner mux addr
      scanBScanner(index); //perform the scanning and send the data
    }

    g_isFixtureDetected = false;
  }

  //reset the LEDs when the fixture is removed
  if(g_isFixtureRemoved)
  {
    //toggle the LED to red when fixture is removed
    digitalWrite(FIXTURE_LED, LOW);

    //reset the barcode LED to red
    g_ioExpandr.digitalWritePort1(0x00);

    g_isFixtureRemoved = false;
  }

  //process the scanning results
  if(Serial.available())
  {
    byte result = Serial.read();
    
    //loop through all the barcode scanners and toggle the LED
    for(int index = 0; index < BSCANNER_MAX_NUM; index++)
    {
      g_ioExpandr.digitalWrite1(g_bscanner[index].led_pin, GET_BIT(result, index));
    }
  }
}

