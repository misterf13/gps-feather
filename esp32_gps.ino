#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Enable or disable ADA_IO
#define ADA_IO false

/************************** Configuration ***********************************/
// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#if ADA_IO
#include "config.h"
// set up the ESP32 feed as 'esp_feed'
AdafruitIO_Feed *esp_feed = io.feed("ESP32_volts");
#endif

#define VBATPIN A13
#define A_RES    12

// Serial1 in v1.0.2 Serial2 in v1.0.1
HardwareSerial GPSSerial(2);

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

uint32_t timer       = millis();
uint32_t timer_delay = 1000;

#if defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#endif

void setup() {
  analogReadResolution(A_RES);
  // wait for hardware serial to appear
  while (!Serial);
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

  Serial.println("Bringing up GPS...");

  GPS.begin(115200);
  // Command to set baud rate to 115200
  //GPS.sendCommand("$PMTK251,115200*1F");
  // Uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // Position fix 5 times a second
   
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  Serial.println("OLED begun");
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
 
  // Clear the buffer.
  display.clearDisplay();
  display.display();
  
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
   
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.display(); // actually display all of the above
  delay(1000);

#if ADA_IO
  // connect to io.adafruit.com
  display.println("Adafruit IO: Enabled");
  display.print("Connecting");
  io.connect();
  
  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    display.print(".");
    display.display();
    delay(200);
  }
  
  // we are connected
  display.println();
  display.println(io.statusText());
  display.display();
  delay(5000);
#endif
  display.setCursor(0,0);
  display.clearDisplay();
  display.display();
}

int display_msg = 0;

void loop() {
#if ADA_IO
  // We need this but it causes delays. Comment out for now.
  //io.run();
#endif
  if(!digitalRead(BUTTON_A)) display_msg = 0;
  if(!digitalRead(BUTTON_B)) display_msg = 1;
  if(!digitalRead(BUTTON_C)) display_msg = 2;
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every second or so, print out the current stats
  if (millis() - timer > timer_delay) {
    timer = millis(); // reset the timer
    if (display_msg == 0) {
      timer_delay = 1000;
      msg_1();
    } else if (display_msg == 1) {
#if ADA_IO
      // We set timer_delay to 2 seconds to avide by adafruit.io rules
      timer_delay = 2000;
#endif
      msg_2();
    } else if (display_msg == 2) {
      timer_delay = 1000;
      msg_3();
    }
    display.display();
    display.clearDisplay();
    display.setCursor(0,0);
  }
}

void msg_1() {
  if (GPS.fix) {
    display.print("Fix: "); display.print((int)GPS.fix);
    display.print(" Quality: "); display.println((int)GPS.fixquality);
    if (GPS.fixquality == 0) {
      display.println("Invalid");
    } else if (GPS.fixquality == 1) {
      display.println("GPS");
    } else if (GPS.fixquality == 2) {
      display.println("DGPS");
    } else {
      display.println("Unknown quality");
    }
    display.print("Satellites: ");
    display.println((int)GPS.satellites);
  } else {
    display.println("No GPS fix.");
    display.println((char)1);
    display.print("Time is good."); display.println((char)2);
#if ADA_IO
    display.print("IP: "); display.println(WiFi.localIP());
#endif
  }
}

void msg_2() {
  display.print("Date: ");
  display.print(GPS.month, DEC); display.print("/");
  display.print(GPS.day, DEC); display.print("/20");
  display.println(GPS.year, DEC);
  display.print("Time: ");
  if (GPS.hour < 10) display.print(0);
  display.print(GPS.hour, DEC); display.print(':');
  if (GPS.minute < 10) display.print(0);
  display.print(GPS.minute, DEC); display.print(':');
  if (GPS.seconds < 10) display.print(0);
  display.print(GPS.seconds, DEC); display.println(" UTC");
  // To convert the ADC integer value to a real voltage you’ll need to divide it by
  // the maximum value of 4096, then double it (note above that Adafruit halves the voltage),
  // then multiply that by the reference voltage of the ESP32 which is 3.3V and then vinally,
  // multiply that again by the ADC Reference Voltage of 1100mV.
  float volt_read = (float)analogRead(VBATPIN) / (1<<A_RES) * 2.0 * 3.3 * 1.1;
  //float volt_read = ((float)analogRead(VBATPIN) * 2.0 * 3.3) / (1<<A_RES);
  // Voltage is off by .20 according to multimeter
  volt_read += 0.20;
#if ADA_IO
  esp_feed->save(volt_read);
#endif
  Serial.print("Voltage: "); Serial.print(volt_read, 2); Serial.println("v");
  display.print("Voltage: "); display.print(volt_read, 2); display.println("v");
  display.print("E[");
  for (float volts = 3.20; volts <= 4.10; volts += 0.10) {
    if (volt_read >= volts) {
      display.print((char)(218));
    } else {
      display.print(" ");
    }
  }
  display.println("]F");
}

void msg_3() {
  display.print("Lat: ");
  display.print(GPS.latitude, 4); display.println(GPS.lat);
  display.print("Lon: ");
  display.print(GPS.longitude, 4); display.println(GPS.lon);
  display.print("Spd: "); display.print((float)GPS.speed * 1.15078); display.println(" mph");
  display.print("Alt: "); display.print(GPS.altitude); display.println(" m");
}
