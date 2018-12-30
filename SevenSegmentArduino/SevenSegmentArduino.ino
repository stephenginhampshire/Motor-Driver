#include <SoftwareSerial.h>
/*
	Drive 7 segment display on Telescope, showing ANGLE and then STATUS alternately
*/
// Definitions ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define ALT_display_DIN 11        // MAX7219 DATA pin		D11 green
#define ALT_display_CS  10        // MAX7219 CS/LOAD pin	D10 white
#define ALT_display_CLK 12        // MAX7219 CLK pin		D12 yellow
#define AZM_display_DIN 8         // MAX7219 DATA pin		D8 green
#define AZM_display_CS  7         // MAX7219 CS/LOAD pin	D7 white
#define AZM_display_CLK 9         // MAX7219 CLK pin		D9 yellow
#define Motor_RXpin  3
#define Motor_TXpin  2
#define Motor_baud   38400
#define STX 0x02
#define ETX 0x03
#define true 0x01
#define false 0x00
enum {
  ALT_display_REG_DECODE    = 0x09,
  ALT_display_REG_INTENSITY = 0x0A,
  ALT_display_REG_SCANLIMIT = 0x0B,
  ALT_display_REG_SHUTDOWN  = 0x0C,
  ALT_display_REG_DISPTEST  = 0x0F,
  AZM_display_REG_DECODE	  = 0x09,
  AZM_display_REG_INTENSITY = 0x0A,
  AZM_display_REG_SCANLIMIT = 0x0B,
  AZM_display_REG_SHUTDOWN  = 0x0C,
  AZM_display_REG_DISPTEST  = 0x0F
};
enum {OFF = 0, ON = 1 };
const byte DP = 0b10000000;
char outptr = 0;
char inptr = 0;
char inbuffer[0xFF];
char string_ptr = 0;
unsigned long displaytime = 0;
// Variables ---------------------------------------------------------------------------
typedef struct {
  unsigned char header;     // [0] STX
  double ALT_angle;			// [1 - 4] Altitude Bearing
  double AZM_angle;			// [5 - 8] Azimuth Bearing
  unsigned char footer;     // [9] ETX
} motormessage;
char length_of_message = 12;
union sg_message_from_driver_union {
  motormessage motor_message;
  unsigned char sg_chars[12];
};
union sg_message_from_driver_union message_from_Motor;
bool incoming_message_available = false;
char ALT_angle[10];
char AZM_angle[10];
bool displayflag = false;
char chkcrc = 0;
SoftwareSerial MotorPort(Motor_RXpin, Motor_TXpin);
void resetDisplay(void);
void displayAngle(String);
void displayStatus(char);
void checkserial(void);
//-- setup -----------------------------------------------------------------------------
void setup() {
  Serial.begin(38400);
  while (!Serial) {
    ;  // Wait for serial port to connect. Needed for Leonardo only.
  }
  MotorPort.begin(Motor_baud);
  MotorPort.flush();
  ALT_angle[9] = NULL;  //make the char array into a string by inserting a string terminator
  AZM_angle[9] = NULL;
  pinMode (LED_BUILTIN, OUTPUT);
  pinMode(ALT_display_DIN, OUTPUT);
  pinMode(ALT_display_CS, OUTPUT);
  pinMode(ALT_display_CLK, OUTPUT);
  pinMode(AZM_display_DIN, OUTPUT);
  pinMode(AZM_display_CS, OUTPUT);
  pinMode(AZM_display_CLK, OUTPUT);
  digitalWrite(ALT_display_CS, HIGH);
  digitalWrite(AZM_display_CS, HIGH);
  resetDisplay();                   // reset the MAX7219 display
  String Test_message = "123.45678";
  displayALT_angle(Test_message);
  displayAZM_angle(Test_message);
}
//--MainLoop----------------------------------------------------------------------------
void loop() {
  MotorPort.listen();
  while (MotorPort.available() > 0) {                             // add the received character to the buffer
    inbuffer[inptr++] = (unsigned char) MotorPort.read();         // and increment character count
  }
  checkserial();													// check for a message from Hub (Angle & Status)
}
//--EndofMainLoop--------------------------------------------------------------------------------------------------------------------------
void checkserial(void) {
  while ((outptr != inptr)) {	// check serial buffer for data
    char thisbyte = inbuffer[outptr++];                         // take a character from the input buffer and increment pointer
    if ((thisbyte == (char)STX) && (string_ptr == 0)) {			// look for the STX, but only if the output string is empty
      message_from_Motor.sg_chars[string_ptr++] = STX;        // string_ptr = 0, store the STX and increment the string pointer
      //      Serial.print(STX,DEC);
      //      Serial.print(",");
      digitalWrite(LED_BUILTIN, LOW);
    }
    else {
      if (thisbyte == (char)ETX) { // character was not an STX check for ETX
        message_from_Motor.sg_chars[string_ptr++] = ETX;	// string_ptr == 6, save the ETX and increment the string pointer
        if (string_ptr > 9) {         // does it mean end of packet ie we just saved the ETX at 6!
          string_ptr = 0;                                 // zero the string pointer
          inptr = 0;
          outptr = 0;
          digitalWrite(LED_BUILTIN, HIGH);
          //                   Serial.print(ETX,DEC);
          //                   Serial.print(" Message Received ");
          //          if ((message_from_Motor.motor_message.Angle >= (double)0.00) && (message_from_Motor.motor_message.Angle < (double)360.00)) {
          dtostrf(message_from_Motor.motor_message.ALT_angle, 9, 5, ALT_angle);     // dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
          dtostrf(message_from_Motor.motor_message.AZM_angle, 9, 5, AZM_angle);
          displayALT_angle(ALT_angle);
          displayAZM_angle(AZM_angle);
          //                     Serial.print("Decoded As ");
          //                     Serial.println(angle);
          //         }
        }
      }
      else {
        message_from_Motor.sg_chars[string_ptr++] = thisbyte;           // Not a valid STX or a valid ETX so save it and increment string pointer
        //               Serial.print(thisbyte,DEC);
        //               Serial.print(",");
      }
    }
  }
}
void set_register_ALT(byte reg, byte value) {   // ... write a value into a max7219 register See MAX7219 Datasheet, Table 1, page 6
  digitalWrite(ALT_display_CS, LOW);
  shiftOut(ALT_display_DIN, ALT_display_CLK, MSBFIRST, reg);
  shiftOut(ALT_display_DIN, ALT_display_CLK, MSBFIRST, value);
  digitalWrite(ALT_display_CS, HIGH);
}
void set_register_AZM(byte reg, byte value) {   // ... write a value into a max7219 register See MAX7219 Datasheet, Table 1, page 6
  digitalWrite(AZM_display_CS, LOW);
  shiftOut(AZM_display_DIN, AZM_display_CLK, MSBFIRST, reg);
  shiftOut(AZM_display_DIN, AZM_display_CLK, MSBFIRST, value);
  digitalWrite(AZM_display_CS, HIGH);
}
void displayALT_angle(String angleString) { // ... display the Altitude ANGLE on the 7-segment display
  if (angleString.charAt(0) == 0x20) angleString.setCharAt(0, 0x30);
  if (angleString.charAt(1) == 0x20) angleString.setCharAt(1, 0x30);
  if (angleString.charAt(2) == 0x20) angleString.setCharAt(2, 0x30);
  set_register_ALT(ALT_display_REG_SHUTDOWN, OFF);  // turn off display
  set_register_ALT(ALT_display_REG_SCANLIMIT, 7);   // scan limit 8 digits
  set_register_ALT(ALT_display_REG_DECODE, 0b11111111); // decode all digits
  set_register_ALT(1, angleString.charAt(8));
  set_register_ALT(2, angleString.charAt(7));
  set_register_ALT(3, angleString.charAt(6));
  set_register_ALT(4, angleString.charAt(5));
  set_register_ALT(5, angleString.charAt(4));
  set_register_ALT(6, angleString.charAt(2) | 0x80);	// plus decimal point
  set_register_ALT(7, angleString.charAt(1));
  set_register_ALT(8, angleString.charAt(0));
  set_register_ALT(ALT_display_REG_SHUTDOWN, ON);   // Turn on display
}
void displayAZM_angle(String angleString) { // ... display the Azimuth ANGLE on the 7-segment display
  if (angleString.charAt(0) == 0x20) angleString.setCharAt(0, 0x30);
  if (angleString.charAt(1) == 0x20) angleString.setCharAt(1, 0x30);
  if (angleString.charAt(2) == 0x20) angleString.setCharAt(2, 0x30);
  set_register_AZM(AZM_display_REG_SHUTDOWN, OFF);  // turn off display
  set_register_AZM(AZM_display_REG_SCANLIMIT, 7);   // scan limit 8 digits
  set_register_AZM(AZM_display_REG_DECODE, 0b11111111); // decode all digits
  set_register_AZM(1, angleString.charAt(8));
  set_register_AZM(2, angleString.charAt(7));
  set_register_AZM(3, angleString.charAt(6));
  set_register_AZM(4, angleString.charAt(5));
  set_register_AZM(5, angleString.charAt(4));
  set_register_AZM(6, angleString.charAt(2) | 0x80);	// plus decimal point
  set_register_AZM(7, angleString.charAt(1));
  set_register_AZM(8, angleString.charAt(0));
  set_register_AZM(AZM_display_REG_SHUTDOWN, ON);   // Turn on display
}
void resetDisplay() {
  set_register_ALT(ALT_display_REG_SHUTDOWN, OFF);   // turn off display
  set_register_ALT(ALT_display_REG_DISPTEST, OFF);   // turn off test mode
  set_register_ALT(ALT_display_REG_INTENSITY, 0x0D); // display intensity
  set_register_AZM(AZM_display_REG_SHUTDOWN, OFF);   // turn off display
  set_register_AZM(AZM_display_REG_DISPTEST, OFF);   // turn off test mode
  set_register_AZM(AZM_display_REG_INTENSITY, 0x0D); // display intensity
}

/* Creating Alternate Characters
  segment DP  =				D7
		A = top level		D6
		B = top right		D5
		C = bottom right	D4
		D = bottom level	D3
		E = bottom left		D2
		F = top left		D1
		G = middle level	D0

  const byte C = 0b01001110;
  const byte F = 0b01000111;
  set_register(0x01, C);
  or
  set_register(0x01, F);
*/
