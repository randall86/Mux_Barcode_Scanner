#ifndef __USBKeyboard_h__
#define __USBKeyboard_h__

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "hid_keys.h"

#define LED_NUMLOCK    (1 << 0)
#define LED_CAPSLOCK   (1 << 1)
#define LED_SCROLLLOCK (1 << 2)

#define SHIFT 0x80
const uint8_t hid_ascii_map[128] =
{
	0x00,             // NUL
	0x00,             // SOH
	0x00,             // STX
	0x00,             // ETX
	0x00,             // EOT
	0x00,             // ENQ
	0x00,             // ACK  
	0x00,             // BEL
	KEY_BACKSPACE,			// BS	Backspace
	KEY_TAB,			// TAB	Tab
	KEY_ENTER,			// LF	Enter
	0x00,             // VT 
	0x00,             // FF 
	0x00,             // CR 
	0x00,             // SO 
	0x00,             // SI 
	0x00,             // DEL
	0x00,             // DC1
	0x00,             // DC2
	0x00,             // DC3
	0x00,             // DC4
	0x00,             // NAK
	0x00,             // SYN
	0x00,             // ETB
	0x00,             // CAN
	0x00,             // EM 
	0x00,             // SUB
	0x00,             // ESC
	0x00,             // FS 
	0x00,             // GS 
	0x00,             // RS 
	0x00,             // US 
	KEY_SPACE,		   //  ' '
	KEY_1|SHIFT,	   // !
	KEY_QUOTE|SHIFT,	   // "
	KEY_3|SHIFT,    // #
	KEY_4|SHIFT,    // $
	KEY_5|SHIFT,    // %
	KEY_7|SHIFT,    // &
	KEY_QUOTE,          // '
	KEY_9|SHIFT,    // (
	KEY_0|SHIFT,    // )
	KEY_8|SHIFT,    // *
	KEY_EQUALS|SHIFT,    // +
	KEY_COMMA,          // ,
	KEY_MINUS,          // -
	KEY_PERIOD,          // .
	KEY_SLASH,          // /
	KEY_0,          // 0
	KEY_1,          // 1
	KEY_2,          // 2
	KEY_3,          // 3
	KEY_4,          // 4
	KEY_5,          // 5
	KEY_6,          // 6
	KEY_7,          // 7
	KEY_8,          // 8
	KEY_9,          // 9
	KEY_SEMICOLON|SHIFT,      // :
	KEY_SEMICOLON,          // ;
	KEY_COMMA|SHIFT,      // <
	KEY_EQUALS,          // =
	KEY_PERIOD|SHIFT,      // >
	KEY_SLASH|SHIFT,      // ?
	KEY_2|SHIFT,      // @
	KEY_A|SHIFT,      // A
	KEY_B|SHIFT,      // B
	KEY_C|SHIFT,      // C
	KEY_D|SHIFT,      // D
	KEY_E|SHIFT,      // E
	KEY_F|SHIFT,      // F
	KEY_G|SHIFT,      // G
	KEY_H|SHIFT,      // H
	KEY_I|SHIFT,      // I
	KEY_J|SHIFT,      // J
	KEY_K|SHIFT,      // K
	KEY_L|SHIFT,      // L
	KEY_M|SHIFT,      // M
	KEY_N|SHIFT,      // N
	KEY_O|SHIFT,      // O
	KEY_P|SHIFT,      // P
	KEY_Q|SHIFT,      // Q
	KEY_R|SHIFT,      // R
	KEY_S|SHIFT,      // S
	KEY_T|SHIFT,      // T
	KEY_U|SHIFT,      // U
	KEY_V|SHIFT,      // V
	KEY_W|SHIFT,      // W
	KEY_X|SHIFT,      // X
	KEY_Y|SHIFT,      // Y
	KEY_Z|SHIFT,      // Z
	KEY_LBRACKET,          // [
	KEY_BACKSLASH,          // bslash
	KEY_RBRACKET,          // ]
	KEY_6|SHIFT,    // ^
	KEY_MINUS|SHIFT,    // _
	KEY_TILDE,          // `
	KEY_A,          // a
	KEY_B,          // b
	KEY_C,          // c
	KEY_D,          // d
	KEY_E,          // e
	KEY_F,          // f
	KEY_G,          // g
	KEY_H,          // h
	KEY_I,          // i
	KEY_J,          // j
	KEY_K,          // k
	KEY_L,          // l
	KEY_M,          // m
	KEY_N,          // n
	KEY_O,          // o
	KEY_P,          // p
	KEY_Q,          // q
	KEY_R,          // r
	KEY_S,          // s
	KEY_T,          // t
	KEY_U,          // u
	KEY_V,          // v
	KEY_W,          // w
	KEY_X,          // x
	KEY_Y,          // y
	KEY_Z,          // z
	KEY_LBRACKET|SHIFT,    // {
	KEY_BACKSLASH|SHIFT,    // |
	KEY_RBRACKET|SHIFT,    // }
	KEY_TILDE|SHIFT,    // ~
	0x00				// DEL
};

class USBKeyboard {
 public:
  void init () {
    // We will talk to atmega8u2 using 9600 bps
    Serial.begin(9600);
  }
    
    
  void sendKeyStroke(byte keyStroke) {
    sendKeyStroke(keyStroke, 0);
  }

  void sendKeyStroke(byte keyStroke, byte modifiers) {
    uint8_t keyNone[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };    
  
    Serial.write(modifiers);     // Modifier Keys
    Serial.write(0);          // Reserved
    Serial.write(keyStroke);     // Keycode 1
    Serial.write(0);          // Keycode 2  
    Serial.write(0);          // Keycode 3
    Serial.write(0);          // Keycode 4
    Serial.write(0);          // Keycode 5
    Serial.write(0);          // Keycode 6  
  
    Serial.write(keyNone, 8);    // Release Key   
  }
  
  uint8_t readLedStatus() {
    uint8_t keyNone[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  
    uint8_t ledStatus;
    
    Serial.write(keyNone, 8);    // Release Key
    while (Serial.available() > 0) {
      ledStatus = Serial.read();
    }
    
    return ledStatus;
  }
  
  void print(char *chp)
  {
    uint8_t buf[8] = { 0 };	/* Keyboard report buffer */
    
    while (*chp) {
	  uint8_t hid_char = hid_ascii_map[*chp];
	  
	  if (hid_char & SHIFT)
	  {
	    buf[0] = MOD_SHIFT_LEFT;	/* Caps */
        buf[2] = (hid_char & ~(SHIFT));
	  }
	  else
	  {
	    buf[2] = hid_char;
	  }
	  
      Serial.write(buf, 8);	// Send keystroke
      buf[0] = 0;
      buf[2] = 0;
      Serial.write(buf, 8);	// Release key
      chp++;
    }
  }    
};

USBKeyboard Keyboard;

#endif // __USBKeyboard_h__
