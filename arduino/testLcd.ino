//Import Relevant library
#include "U8g2lib.h"

//Configure pins for RepRapDiscount Full Graphic Smart Controller
U8GLIB_ST7920_128X64_1X u8g(23, 17, 16);  // SPI Com: SCK = en = 23, MOSI = rw = 17, CS = di = 16

int i = 0;

void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);
  //u8g.drawStr( 16, 32, "Hello World!");
}

void setup(void) {

  //Initialise Baud Rate for Serial Communication
  Serial.begin(250000);
  
  // flip screen, if required
  // u8g.setRot180();
  
  // set SPI backup if required
  //u8g.setHardwareBackup(u8g_backup_avr_spi);

  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity: default - 3
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  
  pinMode(8, OUTPUT);

}

void loop(void) {

   i += 1;
   
//    if (Serial.available()) {
//      int data  = Serial.read();
//      Serial.println(data);
//    }

  // picture loop
  u8g.firstPage();  
  do {
    
    draw();
    char buf[9];
    sprintf (buf, "%d", i);
    u8g.drawStr(18, 18, buf);
  } while( u8g.nextPage() );
  
  // rebuild the picture after some delay
  delay(50);
}