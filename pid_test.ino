/* 
 *  PID test platform on Arduino UNO
 *  Temperature Control platform :
 *  - Heater module is a "PTC ceramic 5v 50°C" command by PWM and a MOSFET (on velleman board VMA411)
 *  - Sensor is a ds18b20.
 *  
 *  Cyclic output with process value, output (0/100%), set point for arduino serial plotter
 *  
 *  Send commands via serial monitor : 
 *  - "M" for set PID to manual mode, "A" for automatic mode, "S34" for fix setpoint at 34.0
 *  - "P2.55" to set kp at 2.55, "I2" ti set ki at 2.0, "D0.2" to set kd at 0.2 
 *  - "W" write currents params (SP, kp, ki and kd) to EEPROM
 *  
 *  This code is licensed under the MIT license : http://opensource.org/licenses/MIT
 */

#include <EEPROM.h>
// from https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>
// from https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include <LiquidCrystal_I2C.h>
// from https://www.pjrc.com/teensy/td_libs_OneWire.html
#include <OneWire.h>
// from https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <DallasTemperature.h>

// some const
// I/O
#define ONE_WIRE_BUS        2
#define OUT_PWM             3
#define LCD_LINE_SIZE       20
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
// LCD: address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);
// ds18b20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
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
  Serial.println("system start");
  // init ds18b20
  sensors.begin();
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
    // read the incoming byte:
    switch (Serial.read()) {
      case 'A':
      case 'a':
        Serial.println("PID set to auto mode");
        myPID.SetMode(AUTOMATIC);
        break;
      case 'M':
      case 'm':
        Serial.println("PID set to manual mode");
        myPID.SetMode(MANUAL);
        break;
      case 'O':
      case 'o':
        pid_out = Serial.parseFloat();
        Serial.println("PID out set at " + String(pid_out));
        break;
      case 'P':
      case 'p':
        pid_p.kp = Serial.parseFloat();
        Serial.println("PID kp set at " + String(pid_p.kp));
        break;
      case 'I':
      case 'i':
        pid_p.ki = Serial.parseFloat();
        Serial.println("PID ki set at " + String(pid_p.ki));
        break;
      case 'D':
      case 'd':
        pid_p.kd = Serial.parseFloat();
        Serial.println("PID kd set at " + String(pid_p.kd));
        break;
      case 'S':
      case 's':
        pid_sp = Serial.parseFloat();
        Serial.println("PID SetPoint set at " + String(pid_sp));
        break;
      case 'W':
      case 'w':
        Serial.println("Write params (SP, kp, ki and kd) to EEPROM");
        // store magic number, PID params and setpoint
        EEPROM.put(EEPROM_AD_MAGIG_NB, (uint16_t) EEPROM_MAGIC_NB);
        EEPROM.put(EEPROM_AD_PID_P, pid_p);
        EEPROM.put(EEPROM_AD_PID_SP, pid_sp);
        break;
    }
  }
  
  // read ds18b20
  sensors.requestTemperatures();
  pid_pv = sensors.getTempCByIndex(0);

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
}
