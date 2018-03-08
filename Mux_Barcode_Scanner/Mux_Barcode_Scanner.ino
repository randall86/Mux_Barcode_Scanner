#include <DTIOI2CtoParallelConverter.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

#define MUX_RX 10
#define MUX_TX 11

#define FIXTURE_SW_PIN 6

//expander pins
#define SCANNER_LED_1 PIN0_0
#define SCANNER_LED_2 PIN0_1
#define SCANNER_LED_3 PIN0_2
#define SCANNER_LED_4 PIN0_3
#define SCANNER_LED_5 PIN0_4
#define SCANNER_LED_6 PIN0_5
#define SCANNER_LED_7 PIN0_6
#define SCANNER_LED_8 PIN0_7

#define SCANNER_EN_1 PIN1_0
#define SCANNER_EN_2 PIN1_1
#define SCANNER_EN_3 PIN1_2
#define SCANNER_EN_4 PIN1_3
#define SCANNER_EN_5 PIN1_4
#define SCANNER_EN_6 PIN1_5
#define SCANNER_EN_7 PIN1_6
#define SCANNER_EN_8 PIN1_7

#define DEBOUNCE_FREQ_MSEC 200 //stable time before switch state changed
#define TIMER_FREQ_MSEC 50 //read the switch every 50ms

DTIOI2CtoParallelConverter g_ioExpandr(0x74);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 0)

SoftwareSerial barcode_serial(MUX_RX, MUX_TX); // RX, TX

//SimpleTimer object
SimpleTimer timer;

byte g_debouncedState = 0;

//returns true if state changed
bool debounceSW(byte *state)
{
    static uint8_t count = DEBOUNCE_FREQ_MSEC/TIMER_FREQ_MSEC;
    bool state_changed = false;

    //read the fixture switch state
    byte raw_state = digitalRead(FIXTURE_SW_PIN);
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
  
  // set the data rate for the SoftwareSerial port
  barcode_serial.begin(9600);



  //initialize the debounce timer
  timer.setInterval(TIMER_FREQ_MSEC, debounceSWRoutine);
}

void loop()
{
  if(barcode_serial.available())
  {
    char c = barcode_serial.read();
  }
}

