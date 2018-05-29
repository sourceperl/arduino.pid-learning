/*
 *  PID test platform on Arduino UNO
 *  Temperature Control platform :
 *  - Heater module is a "PTC ceramic 5v 50°C" command by PWM and a MOSFET (on velleman board VMA411)
 *  - Sensor is a thermistor NTC-MF52 3950 (on velleman board VMA320)
 *
 *  Cyclic output with process value, output (0/100%), set point for arduino serial plotter
 *
 *  Send commands via serial monitor :
 *  - "MAN" for set PID to manual mode, "AUTO" for automatic mode, "SP 34.2" for fix setpoint at 34.2
 *  - "KP 2.55" to set kp at 2.55, "KI 2" ti set ki at 2.0, "KD 0.2" to set kd at 0.2
 *  - "SAVE" write currents params (SP, kp, ki and kd) to EEPROM
 *
 *  This code is licensed under the MIT license : http://opensource.org/licenses/MIT
 */

#include <EEPROM.h>
// from https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>
// from https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include <LiquidCrystal_I2C.h>

// some const
// I/O
#define ONE_WIRE_BUS        2
// thermistance
#define THERM_INPUT         A0
#define THERM_R0            10E3
#define THERM_T0            25
#define THERM_B_COEF        3950
#define THERM_R_PULL        10E3
// heater output
#define OUT_PWM             3
// LCD display
#define LCD_LINE_SIZE       20
#define MAX_CMD_SIZE        64
// EEPROM storage
#define EEPROM_MAGIC_NB     0xAA55
#define EEPROM_AD_MAGIG_NB  0
#define EEPROM_AD_PID_P     sizeof(uint16_t)
#define EEPROM_AD_PID_SP    sizeof(uint16_t) + sizeof(PidParams)

// some struct
// define struct with defaults values
struct PidParams {
  double kp = 6.0;
  double ki = 0.75;
  double kd = 0.25;
};

// some vars
// serial cmd
int inByte = 0;
String s_cmd = "";
String s_arg = "";
// LCD: address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);
// PID
PidParams pid_p;
double pid_sp = 32.0;
double pid_pv = 0.0;
double pid_out = 0.0;
PID myPID(&pid_pv, &pid_out, &pid_sp, 0, 0, 0, DIRECT);

// print msg on line nb on LCD panel
// pad the line with space char
void lcd_line(byte line, String msg) {
  // limit size of message
  msg.remove(LCD_LINE_SIZE);
  // set pos at begin of a line
  lcd.setCursor(0, line);
  lcd.print(msg);
  // pad with char
  for (byte i = 0; i < LCD_LINE_SIZE - msg.length(); i++)
    lcd.write(' ');
}

void setup() {
  // init serial
  Serial.begin(9600);
  //Serial.setTimeout(5);
  Serial.println(F("system start"));
  // init IO
  pinMode(THERM_INPUT, INPUT);
  // init LCD
  lcd.init();
  lcd.backlight();
  // read EEPROM backup value only if EEPROM have a first init (= magic number is set)
  uint16_t eeprom_magic;
  EEPROM.get(EEPROM_AD_MAGIG_NB, eeprom_magic);
  if (eeprom_magic == EEPROM_MAGIC_NB) {
    EEPROM.get(EEPROM_AD_PID_P, pid_p);
    EEPROM.get(EEPROM_AD_PID_SP, pid_sp);
  }
  // init PID
  myPID.SetOutputLimits(0.0, 100.0);
  myPID.SetMode(AUTOMATIC);
}


void loop() {
  // check command
  while (Serial.available() > 0) {
    // receive loop
    while (true) {
      inByte = Serial.read();
      // no more data
      if (inByte == -1)
        break;
      // add data to s_cmd
      s_cmd += (char) inByte;
      // limit size to MAX_CMD_SIZE
      if (s_cmd.length() > MAX_CMD_SIZE)
        s_cmd.remove(0, s_cmd.length() - MAX_CMD_SIZE);
      // pause receive loop if \n occur
      if (inByte == '\n')
        break;
    }
    // skip command not ended with "\r\n"
    if (! s_cmd.endsWith("\r\n"))
      break;
    // remove trailling \r\n, force case
    s_cmd.trim();
    s_cmd.toUpperCase();
    // check for command argument (cmd [space char] [arg])
    int index_space  = s_cmd.indexOf(" ");
    if (index_space != -1)
      s_arg = s_cmd.substring(index_space);
    // check command
    if (s_cmd.startsWith("AUTO")) {
      Serial.println(F("PID set to auto mode"));
      myPID.SetMode(AUTOMATIC);
    }
    else if (s_cmd.startsWith("MAN")) {
      Serial.println(F("PID set to manual mode"));
      myPID.SetMode(MANUAL);
    }
    else if (s_cmd.startsWith("OUT")) {
      pid_out = s_arg.toFloat();
      Serial.print(F("PID out set at "));
      Serial.println(pid_out);
    }
    else if (s_cmd.startsWith("SP")) {
      pid_sp = s_arg.toFloat();
      Serial.print(F("PID SetPoint set at "));
      Serial.println(pid_sp);
    }
    else if (s_cmd.startsWith("KP")) {
      pid_p.kp = s_arg.toFloat();
      Serial.print(F("PID kp set at "));
      Serial.println(pid_p.kp);
    }
    else if (s_cmd.startsWith("KI")) {
      pid_p.ki = s_arg.toFloat();
      Serial.print(F("PID ki set at "));
      Serial.println(pid_p.ki);
    }
    else if (s_cmd.startsWith("KD")) {
      pid_p.kd = s_arg.toFloat();
      Serial.print(F("PID kd set at "));
      Serial.println(pid_p.kd);
    }
    else if (s_cmd.startsWith("SAVE")) {
      // store magic number, PID params and setpoint
      EEPROM.put(EEPROM_AD_MAGIG_NB, (uint16_t) EEPROM_MAGIC_NB);
      EEPROM.put(EEPROM_AD_PID_P, pid_p);
      EEPROM.put(EEPROM_AD_PID_SP, pid_sp);
      Serial.println(F("Write params (SP, kp, ki and kd) to EEPROM"));
    }
    // reset for next one
    s_cmd = "";
    s_arg = "";
  }

  // read Thermistor
  float therm_r = THERM_R_PULL / (1023 / (float) analogRead(THERM_INPUT) - 1);
  float steinhart = therm_r / THERM_R0;   // (R/Ro)
  steinhart = log(steinhart);             // ln(R/Ro)
  steinhart /= THERM_B_COEF;              // 1/B * ln(R/Ro)
  steinhart += 1.0 / (THERM_T0 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;            // invert
  steinhart -= 273.15;                    // convert to C
  pid_pv = steinhart;

  // update PID and output
  myPID.SetTunings(pid_p.kp, pid_p.ki, pid_p.kd);
  myPID.Compute();
  analogWrite(OUT_PWM, map(pid_out, 0.0, 100.0, 0, 255));

  // refresh LCD display
  String pid_mode = (myPID.GetMode() == AUTOMATIC) ? "AUT" : "MAN";
  lcd_line(0, String("SP  " + String(pid_sp) + " C"));
  lcd_line(1, String("PV  " + String(pid_pv) + " C"));
  lcd_line(2, String("OUT " + String(pid_out) + " %"));
  lcd_line(3, String("PID " + String(pid_p.kp, 1) + "/" + String(pid_p.ki, 1) + "/" + String(pid_p.kd, 1) + " " + pid_mode));

  // print for arduino serial plotter
  Serial.println(String(pid_pv) + "," + String(pid_out) + "," + String(pid_sp));

  // wait before next loop
  delay(1000);
}
