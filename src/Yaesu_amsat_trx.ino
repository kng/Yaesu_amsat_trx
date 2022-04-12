/* 
 * Teensy 3.2
 * Arduino 1.8.13 or PlatformIO
 * Author: SA2KNG
 * Display: SSD1351 oled 128x128 or 128x96
 * Control: two Yaesu FT-8x7 tranceivers in full duplex
 * GPS: generic nmea-0183
 * 
 * Supported modes:
 * - Doppler control: increase in frequency on the master results in decrease on slave, multiplied regarding to frequency
 * - Linear control: soon (tm)
 * 
 * To be implemented:
 * - Satellite configuration from SDcard, satellites.txt (norad, qrg, mode)
 * - GPS for position and time, used in the doppler calculations
 * - TLE from nasabare.txt with up to date tle's
 */

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 96    // 96 or 128
#define MOSI_PIN 11         // hardware spi
#define MISO_PIN 12         // only for sd card
#define SCLK_PIN 13         // hardware spi
#define CS_PIN   2
#define RST_PIN  3
#define DC_PIN   4
#define SDCS_PIN 5

#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <Adafruit_BusIO_Register.h>
#include <SdFat.h>
#include <SPI.h>

//const int8_t DISABLE_CHIP_SELECT = -1;
//#define SPI_SPEED SD_SCK_MHZ(4)
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);
SdFat sd;

char write_freq[5] = {0,0,0,0,0x01};  // write frequency
char read_freq[5] = {0,0,0,0,0x03};   // read frequency and mode
char read_rxs[5] = {0,0,0,0,0xE7};    // read receiver status
char read_txs[5] = {0,0,0,0,0xF7};    // read transmitter status

HardwareSerial& r1 = Serial1;  // Radio 1, RX 0, TX 1
HardwareSerial& r2 = Serial2;  // Radio 2, RX 9, TX 10
char r1_read[10], r2_read[10];  // RX buffer from radio
byte r1_req, r2_req;            // RX bytes requested
long r1_freq, r1_oldfreq, r1_lockfreq, r2_freq, r2_oldfreq, r2_lockfreq;
byte r1_lock, r2_lock;  // locking mode, selecting master/slave
unsigned long r1_time, r2_time, poll_time;   // millis on last command, last poll
const int intervals[2] = {1000,100};    // slow/fast polling interval
unsigned int poll_interval, timeout = 50;  // in ms
float vfo_factor;   // factor between the vfo's at locking


unsigned long from_bcd_be(char* c){
    unsigned long result(0);
    byte l=4;
    while (l--) {
        result = result * 100 + (*c >> 4) * 10 + (*c & 15);
        ++c;
    }
    return result;
}


char *to_bcd_be(char bcd_data[], unsigned long freq, unsigned char bcd_len){
    int i;
    unsigned char a;

    if (bcd_len&1) {
        bcd_data[bcd_len/2] &= 0x0f;
        bcd_data[bcd_len/2] |= (freq%10)<<4;
        freq /= 10;
    }
    for (i=(bcd_len/2)-1; i >= 0; i--) {
        a = freq%10;
        freq /= 10;
        a |= (freq%10)<<4;
        freq /= 10;
        bcd_data[i] = a;
    }
    return bcd_data;
}


void initDisplay(){
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(1);
  tft.print("Radio 1: RX");
  tft.setCursor(0,SCREEN_HEIGHT/2);
  tft.print("Radio 2: TX");
}

void updateDisplay(){
  tft.setCursor(0,16);
  tft.setTextSize(2);
  char qrg[12];
  sprintf(qrg, "%03li.%03li.%02li", r1_freq/100000, r1_freq/100 % 1000, r1_freq % 100);
  tft.print(qrg);
  //tft.print((float)r1_freq/100);

  tft.setCursor(0,SCREEN_HEIGHT/2 + 16);
  tft.setTextSize(2);
  sprintf(qrg, "%03li.%03li.%02li", r2_freq/100000, r2_freq/100 % 1000, r2_freq % 100);
  tft.print(qrg);
  //tft.print((float)r2_freq/100);
}

void setup() {
  Serial.begin(115200);   // USB Serial monitor
  r1.begin(9600);
  r2.begin(9600);

  while (!Serial) {
    // wait for Arduino Serial Monitor to be ready
    // remove for release
    SysCall::yield();
  }
  tft.begin();
  tft.setRotation(2);     // rotate display 180 deg if necessary
  
  if (!sd.begin(SDCS_PIN)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("SD OK!");
  Serial.println(sd.card()->cardSize());
  
  initDisplay();          // print all static info
  updateDisplay();        // print values

  Serial.println("Yaesu amsat trx with dual FT-817/818/857");
  Serial.println("By: Daniel SA2KNG");
  poll_interval = intervals[0];
  poll_time = millis();
}

void loop() {
  if(Serial.available()){     // USB serial as manager, taking single byte commands for simplicity
    char c = Serial.read();
    if(c=='1'){
      Serial.println("read_freq 1");
      r1.write(read_freq, sizeof(read_freq));
      r1_time = millis();
      r1_req=5;
    }else if(c=='2'){
      Serial.println("read_freq 2");
      r2.write(read_freq, sizeof(read_freq));
      r2_time = millis();
      r2_req=5;

    }else if(c=='r'){
      Serial.println("read_rxs 1");
      r1.write(read_rxs, sizeof(read_rxs));
      r1_time = millis();
      r1_req=1;
    }else if(c=='t'){
      Serial.println("read_txs 1");
      r1.write(read_txs, sizeof(read_txs));
      r1_time = millis();
      r1_req=1;

    }else if(c=='p'){   // start polling
      poll_time = millis();
    }else if(c=='s'){   // stop polling
      poll_time = 0;

    }else if(c=='l'){ // lock vfo's
      poll_interval = intervals[1];
      r1_lock = 1;    // master
      r2_lock = 0;    // slave
    }else if(c=='L'){ // lock vfo's
      poll_interval = intervals[1];
      r1_lock = 0;    // master
      r2_lock = 1;    // slave
    }else if(c=='u'){ // unlock vfo's
      poll_interval = intervals[0];
      r1_lock = 0;    // independent
      r2_lock = 0;
    }
  }

  // auto polling of radios
  if(poll_time != 0 && millis() - poll_time >= poll_interval){
    poll_time = r1_time = r2_time = millis();
    //Serial.println("polling radios");
    r1.write(read_freq, sizeof(read_freq));
    r1_req=5;
    r2.write(read_freq, sizeof(read_freq));
    r2_req=5;
  }

  // parse responses from radio 1
  if(r1_req > 0 && r1.available() == r1_req){           // does the response match the request ?
    for(int i=0;i<r1_req;i++) r1_read[i] = r1.read();
    if(r1_req == 5){                                    // frequency response
      r1_freq = from_bcd_be(r1_read);                   // convert bcd array to long
      //Serial.print("r1_freq: ");
      //Serial.println(r1_freq);
    }else{                                              // single byte response
      Serial.println(r1_read[0], HEX);
    }
    r1_req=0;                                           // clear request length
    updateDisplay();
  }else if(r1_req == 0 && r1.available()){              // flush unrequested data
    r1.read();
  }
  if(r1_req > 0 && millis() > r1_time + timeout){       // request timeout
    r1_freq = 1;
    r1_req = 0;
    updateDisplay();
  }

  // parse responses from radio 2
  if(r2_req > 0 && r2.available() == r2_req){           // does the response match the request ?
    for(int i=0;i<r2_req;i++) r2_read[i] = r2.read();
    if(r2_req == 5){                                    // frequency response
      r2_freq = from_bcd_be(r2_read);                   // convert bcd array to long
      //Serial.print("r2_freq: ");
      //Serial.println(r2_freq);
    }else{                                              // single byte response
      Serial.println(r2_read[0], HEX);
    }
    r2_req=0;                                           // clear request length
    updateDisplay();
  }else if(r2_req == 0 && r2.available()){              // flush unrequested
    r2.read();
  }
  if(r2_req > 0 && millis() > r2_time + timeout){       // request timeout
    r2_freq = 1;
    r2_req = 0;
    updateDisplay();
  }

  if(r1_lock == 1){
    r1_lockfreq = r1_oldfreq = r1_freq;   // store the locked frequencies
    r2_lockfreq = r2_freq;
    r1_lock = 2;
    r2_lock = 0;
    vfo_factor = (float) r2_freq / r1_freq;
    Serial.println("Locking with radio 1 master");
    Serial.print("vfo_factor: ");
    Serial.println(vfo_factor);
  }else if(r1_lock == 2){
    if(r1_freq != r1_oldfreq){
      r2_freq = r2_lockfreq - vfo_factor * (r1_freq - r1_lockfreq);
      r1_oldfreq = r1_freq;
      Serial.print("r1_freq: ");
      Serial.println(r1_freq);
      Serial.print("r2_freq: ");
      Serial.println(r2_freq);
      to_bcd_be(write_freq, r2_freq, 8);
      r2.write(write_freq, sizeof(write_freq));
      updateDisplay();
    }
  }else{
    r1_lock = 0;
  }

  if(r2_lock == 1){
    r2_lockfreq = r2_oldfreq = r2_freq;   // store the locked frequencies
    r1_lockfreq = r1_freq;
    r2_lock = 2;
    r1_lock = 0;
    vfo_factor = (float) r1_freq / r2_freq;
    Serial.println("Locking with radio 2 master");
    Serial.print("vfo_factor: ");
    Serial.println(vfo_factor);
  }else if(r2_lock == 2){
    if(r2_freq != r2_oldfreq){
      r1_freq = r1_lockfreq - vfo_factor * (r2_freq - r2_lockfreq);
      r2_oldfreq = r2_freq;
      Serial.print("r1_freq: ");
      Serial.println(r1_freq);
      Serial.print("r2_freq: ");
      Serial.println(r2_freq);
      to_bcd_be(write_freq, r1_freq, 8);
      r2.write(write_freq, sizeof(write_freq));
      updateDisplay();
    }
  }else{
    r2_lock = 0;
  }

}

