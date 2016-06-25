
/**

  Three (optionally two) sensors temp monitor

  +-----------------------------------------------+
  |                                               |
  |  NOKIA 5110 breakout                          |
  |                                               |
  |                                               |
  |                                               |
  |                                               |
  |                                               |
  |   RST  CE   DC  DIN   CLK   VCC  LED   GND    |
  |    +   +    +   +     +           +           |
  |    |   |    |   |     |           |           |
  +-----------------------------------------------+
     |   |    |   |     |           |
     |   |    |   |     |           |
     |   |    |   |     |           |
     |   |    |   |     |           |
     |   |    |   |     |           |            (parasitic power - 2 wires)
  +--------------------------------------+       +------------+
  |   |   |    |   |     |           |   |       |     DS18B20|
  |   +   +    +   +     +           +   |  +----+            +----+
  |  D8  D7   D6  D5    D4        D9(PWM)|  |    |            |    |
  |                                      |  |    +------------+    |
  |                                      |  |                      |
  |    ARDUINO PRO MINI clone    12      |  |    +------------+    |    GND
  |                               +--+------+    |     DS18B20|    +-----+
  |                                  |   |  +----+            +----+
  |                                  |   |  |    |            |    |
  |                                 +++  |  |    +------------+    |
  |                            4.7K | |  |  |                      |
  |                                 | |  |  |    +------------+    |
  |                                 +++  |  |    |     DS18B20|    |
  |                                  |   |  +----+            +----+
  |                                  +   |       |            |
  |                                 VCC  |       +------------+
  +--------------------------------------+

  made with http://asciiflow.com/

 * */

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <OneWire.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library

OneWire  ds(12);  // on pin 10 (a 4.7K resistor is necessary)
//Adafruit_PCD8544 display = Adafruit_PCD8544(8, 7, 6,4, 5);
Adafruit_PCD8544 display = Adafruit_PCD8544(4, 5, 6, 7, 8);


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
#define DELTAY 2


#define NUM 3
#define NOTEMP -999

int ids[NUM];
int vals[NUM];
int ii;
void initIDX() {
  for (ii = 0; ii < NUM; ii++) {
    ids[ii] = -1;
    vals[ii] = 0;
  }
}
int idxOF(int id) {
  for (ii = 0; ii < NUM; ii++) {
    if (ids[ii] == id) {
      return ii;
    }
  }
  return -1;
}


int nextIDX() {
  for (ii = 0; ii < NUM; ii++) {
    if (ids[ii] == -1) {
      return ii;
    }
  }
  return 0;
}


int valOF(int id) {
  int idx = idxOF(id);
  if (idx >= 0)
    return vals[idxOF(id)];
  return NOTEMP;
}

void setVal(int id, int val) {
  if (idxOF(id) == -1)  {
    ids[nextIDX()] = id;
  }
  vals[idxOF(id)] = val;
}



void setup(void) {
  Serial.begin(9600);
  initIDX();
  initLCD();
  analogWrite(9, 150);
}

//int  t0 = 123;
//int  t1 = 456;
//int  t2 = 789;

void initLCD() {
  display.begin();
  display.setContrast(25);
  delay(200);
  display.clearDisplay();
  delay(100);
  writeTemp();

}
void writeTemp() {
  int  t0 = -10;
  int  t1 = -20;
  int  t2 = 789;

  t0 = valOF(168);
  t1 = valOF(118);
  t2 = valOF(111);

  int x = 0;


  String tt0(t0);
  if (t0 == NOTEMP)   tt0 = "---";
  for (x = 0; x < tt0.length() - 3; x++) {
    tt0 = " " + tt0;
  }
  String tt1(t1);
  if (t1 == NOTEMP)   tt1 = "---";
  for (x = 0; x < tt1.length() - 3; x++) {
    tt1 = " " + tt1;
  }
#if NUM == 3
  String tt2(t2);
  if (t2 == NOTEMP)   tt2 = "---";
  for (x = 0; x < tt2.length() - 3; x++) {
    tt2 = " " + tt2;
  }
#endif



  display.clearDisplay();
  display.setRotation(0);
  display.drawLine(0, display.height() / 2, display.width(), display.height() / 2, BLACK);


  display.setTextSize(3);
  display.setCursor(5, 0);
  //  if (t0 > -1 && t0 < 100) {
  //    display.print(" ");
  //  }
  display.print(tt0);
  display.setTextSize(2);
  display.drawCircle(display.width() - 20, 11, 3, BLACK);
  display.drawCircle(display.width() - 20, 11, 2, BLACK);
  display.setCursor(display.width() - 14, 7);
  display.print("C");

#if NUM == 2
  display.setTextSize(3);
  display.setCursor(5, display.height() / 2 + 3);
  display.print(tt1);
  display.setTextSize(2);
  display.drawCircle(display.width() - 20,  display.height() / 2 +  14, 3, BLACK);
  display.drawCircle(display.width() - 20, display.height() / 2 +  14, 2, BLACK);
  display.setCursor(display.width() - 14, display.height() / 2 + 10);
  display.print("C");

#else if NUM == 3


  display.drawLine(display.width() / 2 - 1, display.height() / 2, display.width() / 2 - 1, display.height(), BLACK);
  display.setTextSize(2);
  display.setCursor(0, display.height() / 2 + 6);
  display.print(tt1);
  display.setCursor(display.width() / 2 + 5, display.height() / 2 + 6);
  display.print(tt2);

#endif
  display.display();
}


void loop(void) {

  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  if ( !ds.search(addr)) {
    //    Serial.println("No more addresses.");
    //    Serial.println();
    ds.reset_search();
    return;
  }

  //  Serial.print("ROM =");
  //  for( i = 0; i < 8; i++) {
  //    Serial.write(' ');
  //    Serial.print(addr[i], HEX);
  //  }
  int id = addr[7];


  if (OneWire::crc8(addr, 7) != addr[7]) {
    //    Serial.println("CRC is not valid!");
    return;
  }
  //  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //      Serial.println("Device is not a DS18x20 family device.");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(750);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  //  Serial.print("  Data = ");
  //  Serial.print(present, HEX);
  //  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //    Serial.print(data[i], HEX);
    //    Serial.print(" ");
  }

  int16_t raw = (data[1] << 8) | data[0];

  if (raw != -1) {
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    setVal(id, (int)celsius);
    writeTemp();

    Serial.print(id);
    Serial.print(" ");
    Serial.println(celsius);
  }
  else {
    setVal(id, NOTEMP);
    writeTemp();
  }

}
