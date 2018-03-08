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

//expander pins
#define BSCANNER_EN1 PIN0_0
#define BSCANNER_EN2 PIN0_1
#define BSCANNER_EN3 PIN0_2
#define BSCANNER_EN4 PIN0_3
#define BSCANNER_EN5 PIN0_4
#define BSCANNER_EN6 PIN0_5
#define BSCANNER_EN7 PIN0_6
#define BSCANNER_EN8 PIN0_7

#define BSCANNER_LED1 PIN1_0
#define BSCANNER_LED2 PIN1_1
#define BSCANNER_LED3 PIN1_2
#define BSCANNER_LED4 PIN1_3
#define BSCANNER_LED5 PIN1_4
#define BSCANNER_LED6 PIN1_5
#define BSCANNER_LED7 PIN1_6
#define BSCANNER_LED8 PIN1_7

#define DEBOUNCE_FREQ_MSEC 200 //stable time before switch state changed
#define TIMER_FREQ_MSEC 50 //read the switch every 50ms

DTIOI2CtoParallelConverter g_ioExpandr(0x74);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 0)

SoftwareSerial g_muxSerial(MUX_RX, MUX_TX); // RX, TX

//SimpleTimer object
SimpleTimer g_timer;

byte g_debouncedState = 0;

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
        }
        else
        {
        }
    }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(); //need to start the Wire for I2C devices to function
  
  //set the data rate for the SoftwareSerial port
  g_muxSerial.begin(9600);

  //init the barcode scanner enable pins and LED as output
  g_ioExpandr.portMode0(ALLOUTPUT);
  g_ioExpandr.digitalWritePort0(0xFF);
  g_ioExpandr.portMode1(ALLOUTPUT);
  g_ioExpandr.digitalWritePort1(0xFF);

  //init the MUX enable pins and address pins as output
  pinMode(MUX_EN, OUTPUT);
  digitalWrite(MUX_EN, LOW);
  pinMode(MUX_A0, OUTPUT);
  pinMode(MUX_A1, OUTPUT);
  pinMode(MUX_A2, OUTPUT);
  digitalWrite(MUX_A0, LOW);
  digitalWrite(MUX_A1, LOW);
  digitalWrite(MUX_A2, LOW);

  //init the fixture LED output pin and sw detection input pin
  pinMode(FIXTURE_LED, OUTPUT);
  digitalWrite(FIXTURE_LED, HIGH);
  pinMode(FIXTURE_SW, INPUT);

  //initialize the debounce timer
  g_timer.setInterval(TIMER_FREQ_MSEC, debounceSWRoutine);
}

void loop()
{
  if(g_muxSerial.available())
  {
    char c = g_muxSerial.read();
  }
}

