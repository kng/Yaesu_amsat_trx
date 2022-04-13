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
#include <Adafruit_GPS.h>
#include <SdFat.h>
#include <SPI.h>
#include <Sgp4.h>

//const int8_t DISABLE_CHIP_SELECT = -1;
//#define SPI_SPEED SD_SCK_MHZ(4)
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);
SdFat sd;
HardwareSerial& r1 = Serial1;  // Radio 1, RX 0, TX 1
HardwareSerial& r2 = Serial2;  // Radio 2, RX 9, TX 10
HardwareSerial& gpsport = Serial3;  // GPS receiver: RX 7, TX 8
Adafruit_GPS GPS(&gpsport);
Sgp4 sat;

char write_freq[5] = {0,0,0,0,0x01};  // write frequency
char read_freq[5] = {0,0,0,0,0x03};   // read frequency and mode
char read_rxs[5] = {0,0,0,0,0xE7};    // read receiver status
char read_txs[5] = {0,0,0,0,0xF7};    // read transmitter status

char r1_read[10], r2_read[10];  // RX buffer from radio
byte r1_req, r2_req;            // RX bytes requested
long r1_freq, r1_oldfreq, r1_lockfreq, r2_freq, r2_oldfreq, r2_lockfreq;
byte r1_lock, r2_lock;  // locking mode, selecting master/slave
unsigned long r1_time, r2_time, poll_time, sat_time;   // millis on last command, last poll
const int intervals[2] = {1000,100};    // slow/fast polling interval
unsigned int poll_interval, timeout = 50;  // in ms
float vfo_factor;   // factor between the vfo's at locking
double jdt, dist_old, sat_speed; // julian date, old sat distance
byte gps_oldsecond, disp_mode, disp_oldmode = 255;
long downlink = 14596000, uplink = 43514000; // deciherz, same as frequency resolution


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


void updateDisplay(){
  if(disp_oldmode != disp_mode){
    disp_oldmode = disp_mode;
    tft.fillScreen(BLACK);
    tft.setTextColor(WHITE, BLACK);
  }
  if(disp_mode==0){
    tft.setCursor(0,0);
    tft.setTextSize(1);
    tft.print("Radio 1: RX");
    tft.setCursor(0,SCREEN_HEIGHT/2);
    tft.print("Radio 2: TX");
    tft.setCursor(0,16);
    tft.setTextSize(2);
    char qrg[12];
    sprintf(qrg, "%03li.%03li.%02li", r1_freq/100000, r1_freq/100 % 1000, r1_freq % 100);
    if(r1_freq == 1) tft.print("Timeout.  ");
    else tft.print(qrg);
    tft.setCursor(0,SCREEN_HEIGHT/2 + 16);
    tft.setTextSize(2);
    sprintf(qrg, "%03li.%03li.%02li", r2_freq/100000, r2_freq/100 % 1000, r2_freq % 100);
    if(r2_freq == 1) tft.print("Timeout.  ");
    else tft.print(qrg);

  }else if(disp_mode==1){
    tft.setCursor(0,0);
    tft.setTextSize(1);
    tft.print("Satellite: ");
    tft.print(sat.satName);

    tft.setCursor(0, 16);
    tft.print("Az:");
    tft.setCursor(40, 16);
    tft.print("El:");
    tft.setCursor(80, 16);
    tft.print("km/s:");

    tft.setCursor(0,32);
    tft.print(sat.satAz, 0);
    tft.setCursor(40,32);
    tft.print(sat.satEl, 1);
    tft.setCursor(80,32);
    tft.print(sat_speed, 2);

    tft.setCursor(0,48);
    tft.print("UL: ");
    tft.print(uplink);
    tft.print("  ");
    tft.println((double) sat_speed / 29979 * uplink, 0);
    tft.print("DL: ");
    tft.print(downlink);
    tft.print(" ");
    tft.println((double) sat_speed / 29979 * downlink, 0);

  }else{
    disp_mode=0;
  }
}

void setup() {
  Serial.begin(115200);   // USB Serial monitor
  r1.begin(9600);
  r2.begin(9600);

  poll_time = millis();
  while (!Serial) {                   // remove for release!
    SysCall::yield();
    if(millis() - poll_time >= 2000)  // wait 2s for serial monitor to be ready
      break;
  }

  GPS.begin(9600);        // make sure your module settings matches this, common 4800 or 9600

  tft.begin();
  tft.setRotation(2);     // rotate display 180 deg if necessary
  
  if (!sd.begin(SDCS_PIN)) {
    Serial.println("failed!");
    //return;
  }
  Serial.print("SD card size: ");
  Serial.println(sd.card()->cardSize());
  
  updateDisplay();        // draw display

  Serial.println("Yaesu amsat trx with dual FT-817/818/857");
  Serial.println("By: Daniel SA2KNG");
  poll_interval = intervals[0];
  poll_time = millis();

  char satname[] = "AO-73";   // TLE date 2022-04-13
  char tle_line1[] = "1 39444U 13066AE  22097.48225341  .00003217  00000-0  39123-3 0  9993";
  char tle_line2[] = "2 39444  97.6190  74.6247 0055298 265.9560  93.5332 14.83068053451003";
  //char satname[] = "AO-07";
  //char tle_line1[] = "1 07530U 74089B   22097.87728311 -.00000028  00000-0  11420-3 0  9992";
  //char tle_line2[] = "2 07530 101.8995  77.5396 0012201 175.5837 252.4210 12.53653929168869";
  sat.init(satname,tle_line1,tle_line2);     //initialize satellite parameters
  Serial.print("Sat init: ");
  Serial.println(satname);
}

void loop() {   // non blocking loop, no delays or blocking calls

  while(GPS.available()){       // read available gps data
    GPS.read();
  }
  if (GPS.newNMEAreceived()) {  // parse nmea
    GPS.parse(GPS.lastNMEA());
  }
  if(GPS.fix && GPS.seconds != gps_oldsecond){
    gps_oldsecond = GPS.seconds;
    sat.site(GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude);
    jday((int)2000 + GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, 0, false, jdt);
    sat.findsat(jdt);
    dist_old = sat.satDist;
    jdt += (double) 10 / 86400;   // calculate speed over positions separated by 10s
    sat.findsat(jdt);
    sat_speed = (dist_old - sat.satDist) / 10;

    Serial.print("Sat Az: ");
    Serial.print(sat.satAz);
    Serial.print(", El: ");
    Serial.print(sat.satEl);
    Serial.print(", Dist: ");
    Serial.print(sat.satDist, 4);
    Serial.print(", Speed: ");
    Serial.println(sat_speed);
    if(disp_mode==1) updateDisplay();
  }

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

    }else if(c=='3'){   // show radio
      disp_mode=0;
    }else if(c=='4'){   // show satellite
      disp_mode=1;

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

    }else if(c=='9'){   // start polling
      poll_time = millis();
    }else if(c=='0'){   // stop polling
      poll_time = 0;

    }else if(c=='l'){ // lock vfo's
      poll_interval = intervals[1];
      r1_lock = 1;    // master
      r2_lock = 0;    // slave
    }else if(c=='L'){ // lock vfo's
      poll_interval = intervals[1];
      r1_lock = 0;    // slave
      r2_lock = 1;    // master
    }else if(c=='u'){ // unlock vfo's
      poll_interval = intervals[0];
      poll_time = millis();
      r1_lock = 0;    // independent
      r2_lock = 0;
    }else if(c=='d'){ // doppler controlled, release with 'u'
      poll_time = 0;
      r1_lock = 3;
      r2_lock = 3;

    }else if(c=='g'){ // show gps
      Serial.print("Time: ");
      if (GPS.hour < 10) Serial.print('0');
      Serial.print(GPS.hour, DEC); Serial.print(':');
      if (GPS.minute < 10) Serial.print('0');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      if (GPS.seconds < 10) Serial.print('0');
      Serial.println(GPS.seconds, DEC);
      Serial.print("Latitude: ");
      Serial.print(GPS.latitudeDegrees);
      Serial.print(", Longitude: ");
      Serial.println(GPS.longitudeDegrees);
    }
  }

  // auto polling of radios
  if(poll_time != 0 && millis() - poll_time >= poll_interval && r1_lock != 3 && r2_lock != 3){
    poll_time = r1_time = r2_time = millis();
    //Serial.println("polling radios");
    r1.write(read_freq, sizeof(read_freq));             // TODO: reduce poll interval when timed out, hard to turn on/off radio otherwise
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
    if(disp_mode==0) updateDisplay();
  }else if(r1_req == 0 && r1.available()){              // flush unrequested data
    r1.read();
  }
  if(r1_req > 0 && millis() > r1_time + timeout){       // request timeout
    r1_freq = 1;
    r1_req = 0;
    if(disp_mode==0) updateDisplay();
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
    if(disp_mode==0) updateDisplay();
  }else if(r2_req == 0 && r2.available()){              // flush unrequested
    r2.read();
  }
  if(r2_req > 0 && millis() > r2_time + timeout){       // request timeout
    r2_freq = 1;
    r2_req = 0;
    if(disp_mode==0) updateDisplay();
  }

  // locked vfo with r1 as master
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
      if(disp_mode==0) updateDisplay();
    }
  }else if(r1_lock == 3){ // doppler controlled
    if(poll_time != 0 && millis() - poll_time >= poll_interval){
      poll_time = millis();
      //r1_freq = downlink + sat_speed / 29979 * uplink
      //to_bcd_be(write_freq, r1_freq, 8);
      //r1.write(write_freq, sizeof(write_freq));
      //to_bcd_be(write_freq, r2_freq, 8);
      //r2.write(write_freq, sizeof(write_freq));
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
      if(disp_mode==0) updateDisplay();
    }
  }else if(r2_lock == 3){
    // handled in r1 above
  }else{
    r2_lock = 0;
  }

}

