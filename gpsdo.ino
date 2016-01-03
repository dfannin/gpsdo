
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "TinyGPSplus.h"
#include <Time.h>
#include "ClickButton.h"
#include "Statistic.h"


#define SWVER "1.1c"

TinyGPSPlus gps ;
// get the DOP measurements
TinyGPSCustom gpspdop(gps, "GPGSA", 15);
TinyGPSCustom gpshdop(gps, "GPGSA", 16);
TinyGPSCustom gpsvdop(gps, "GPGSA", 17);




uint32_t recvmsgcnt = 0 ;
uint32_t last_recvmsgcnt = 0 ;
uint32_t lastErrorCheck = 0;
bool guardtime = true ;


// get std deviaiton for the control voltage
// and use it to determine whether the oscillator has locked.
// look for std dev has reduced to less than 0.02 volts.
// sample is set to 10 seconds (100 ms * 100 samples)
Statistic pllctl ;
unsigned long pllctl_update = 0 ;
// ms between measurements
const int PLLCTL_INTERVAL = 100;
// number of counts to use ( INTERVAL * COUNT = sample period )
const long PLLCTL_COUNT = 100;
// stdev levelf for PLL control voltage , 5 = 0.0244 volts
const double PLLCTL_STDEV_LEVEL = 5.0;

// set gps fix
#define pinFix 23
// set pll has locked
#define pinLock 25
// show gps fault
#define pinFault 27
// sample output of phase detector (for pll control voltage)
#define pinPllCtl A8
// input for the pin
#define pinSwitch A9


LiquidCrystal_I2C lcd(0x27, 16, 2) ;

ClickButton mbutton(pinSwitch, HIGH);


struct GPSstate {
  unsigned long msgtimer; //
  char msg[21] ; // status message
  char ctime[9] ; // 00:00:00
  char cdate[11] ; // 00-00-0000
  uint32_t satusing ;
  double pdop;
  double hdop;
  double vdop;
  bool fixed ; // is gps fixed or not?
  double lat;
  double lon;
  double altitudem;
  uint32_t last_charsProcessed;
  uint32_t last_failedChecksum;
  uint32_t last_sentencesWithFix;

}  gps1 ;


void gps_init() {
  strcpy(gps1.msg, "");
  strcpy(gps1.ctime, "00:00:00") ;
  strcpy(gps1.cdate, "00-00-0000") ;
  gps1.satusing = 0L ;
  gps1.fixed = false ;
  gps1.msgtimer = 0 ;
  gps1.pdop = 0.0;
  gps1.hdop = 0.0;
  gps1.vdop = 0.0;
  gps1.lat = 0.0;
  gps1.lon = 0.0;
  gps1.altitudem = 0.0;
  gps1.last_charsProcessed = 0;
  gps1.last_failedChecksum = 0;
  gps1.last_sentencesWithFix = 0;
}





void lcd_init() {
  lcd.begin() ;
  lcd.backlight() ;
  lcd.home();
  lcd.print(F("GPSDO "));
  lcd.print(SWVER);
  lcd.setCursor(0, 1);
  lcd.print(F("(C) 2015 KK6DF"));
  delay(2000) ;
  lcd.clear();
  lcd.home();
  lcd.print(F("Initializing"));
  lcd.setCursor(0, 1) ;
  lcd.print(F("GPS......"));
}

int LCDUPDATE = 4000 ;
const int SCREENS = 4 ;
bool lockscreen = false ;
unsigned long lastupdate = 0 ;
int ButtonClicks = 0 ;



void lcd_main(bool forceupdate) {
  unsigned long current;
  static int screennum = -1 ;

  current = millis() ;

  if ( ( ( current - lastupdate ) < LCDUPDATE ) || forceupdate ) {
    return ;
  }

  if ( !lockscreen && ++screennum >= SCREENS ) screennum = 0 ;

  switch (screennum) {
    case 0:
      lcd.clear() ;
      lcd.home() ;
      lcd.print(gps1.cdate);
      lcd.print(" ") ;
      if (gps1.fixed)  {
        lcd.print("*") ;
        digitalWrite(pinFix, HIGH);
      } else {
        digitalWrite(pinFix, LOW);
      }
      lcd.setCursor(0, 1) ;
      lcd.print(gps1.ctime);
      break;
    case 1:
      lcd.clear() ;
      lcd.home() ;
      lcd.print("S");
      lcd.print(gps1.satusing);
      lcd.print("  ");
      lcd.print("D");
      lcd.print(gps1.pdop, 1);
      lcd.print("  ");
      lcd.print("M");
      recvmsgcnt = gps.passedChecksum() - last_recvmsgcnt ;
      last_recvmsgcnt = gps.passedChecksum() ;
      lcd.print(recvmsgcnt);
      lcd.setCursor(0, 1) ;
      lcd.print("U");
      lcd.print(month(now()) - 1);
      lcd.print(":");
      lcd.print(day(now()) - 1);
      lcd.print(":");
      lcd.print(hour(now()));
      lcd.print(":");
      lcd.print(minute(now()));
      lcd.print(":");
      lcd.print(second(now()));
      break;
    case 2:
      lcd.clear() ;
      lcd.home() ;
      lcd.print("Lat:") ;
      lcd.print(gps1.lat, 6);
      lcd.setCursor(0, 1) ;
      lcd.print("Long:");
      lcd.print(gps1.lon, 6);
      break;
    case 3:
      lcd.clear() ;
      lcd.home() ;
      lcd.print("A:");
      lcd.print(gps1.altitudem, 1);
      lcd.setCursor(0, 1) ;
      lcd.print("p");
      lcd.print(pllctl.count()) ;
      double pca = pllctl.average() * (5.0 / 1023.0) ;
      lcd.print(" ");
      lcd.print(pca, 1) ;
      lcd.print(" ");
      double pcs = pllctl.pop_stdev() * (5.0 / 1023.0) ;
      lcd.print(pcs, 3) ;
      break;
  }


  lastupdate = current ;

}

// @@Wb - swtich to zodiac
//
#define LSWZOD 8
byte switchzodiac[LSWZOD] = {
  0x40, 0x40,
  0x57, 0x62,
  0x01, 0x34,
  0x0d, 0x0a
} ;


// switch to NMEA mode
#define LSWNM  18
byte switchnmea[LSWNM] = {
  0xff, 0x81,
  0x33, 0x05,
  0x3, 0x0,
  0x0, 0x7,
  0xcb, 0x71,
  0x0, 0x0,
  0x0, 0x0,
  0x1, 0x0,
  0xff, 0xff
};


//  measurement time mark
//
#define LMTM  18
byte measuretimemark[LMTM] = {
  0xff, 0x81,
  0x4e, 0x4,
  0x3, 0x0,
  0x0, 0x67,
  0xB0, 0x12,
  0x0, 0x0,
  0x1, 0x0,
  0x0, 0x0,
  0xff, 0xff
};

#define LVSAT  18
byte visiblesats[LVSAT] = {
  0xff, 0x81,
  0xeb, 0x3,
  0x3, 0x0,
  0x0, 0x67,
  0x13, 0x13,
  0x0, 0x0,
  0x1, 0x0,
  0x0, 0x0,
  0xff, 0xff
};

#define LUSET 18
byte usersettings[LUSET] = {
  0xff, 0x81,
  0xf4, 0x3,
  0x3, 0x0,
  0x0, 0x67,
  0x0a, 0x13,
  0x0, 0x0,
  0x1, 0x0,
  0x0, 0x0,
  0xff, 0xff
};

#define NSENDZDA 24

char n_sendzda[NSENDZDA + 1] = {
  '$', 'P', 'R', 'W', 'I', 'I', 'L', 'O', 'G', ',',
  'Z', 'D', 'A', ',',
  'A', ',',
  'T', ',',
  '1', '0', ',',
  '0',
  0xd, 0xa, 0x0
};
#define NSENDGSV 20

char n_sendgsv[NSENDGSV + 1] = {
  '$', 'P', 'R', 'W', 'I', 'I', 'L', 'O', 'G', ',',
  'G', 'S', 'V', ',',
  'V', ',',
  ',',
  ',',
  0xd, 0xa, 0x0
};

#define LRSETGPS 26

char resetgps[LRSETGPS + 1] = {
  '$', 'P', 'R', 'W', 'I', 'I', 'N', 'I', 'T', ',',
  'A', ',',
  ',', ',', ',', ',', ',', ',', ',', ',', ',', ',', ',', ',',
  0xd, 0xa, 0x0
};

void sendmsg(byte msg[], int len) {
  for (int i = 0; i < len; i++ ) {
    Serial1.write(msg[i]);
  }
}


bool debug = true ;

void logger(char* tag, char* logmsg) {

  if ( !debug ) return ;
  char outbuf[120] ;
  sprintf(outbuf, "%04d%02d%02d%02d%02d%02d,%s,%s\n",
          year(), month(), day(), hour(), minute(), second(),
          tag, logmsg);
  Serial.print(outbuf);
}



#define SDELAY 1500
void setup_gps() {
  gps_init() ;

  logger("boot", "setting up gps");

  delay(SDELAY + 5000);
  logger("boot", "1 sending switch to zodiac");
  sendmsg(switchzodiac, LSWZOD);

  /*  not valid to use
  delay(SDELAY);
  logger("boot", "2 sending adjust measure time mark");
  sendmsg(measuretimemark, LMTM);

    not valid
  delay(SDELAY);
  logger("boot", "3 sending  adjust visible sats");
  sendmsg(visiblesats, LVSAT);


  don't need these
  delay(SDELAY);
  logger("boot", "5 send date/time zda messages");
  sendmsg((byte *) n_sendzda, NSENDZDA);

  user setting are ok, so don't use
  delay(SDELAY);
  logger("boot", "4 sending  adjust user settings");
  sendmsg(usersettings, LUSET);
  */

  delay(SDELAY);
  logger("boot", "2 sending switch to nmea");
  sendmsg(switchnmea, LSWNM);
  /*
   removing this , it caused the GPS to be reset and lose NMEA messages - bad juju
  delay(SDELAY);
  logger("boot", "3 resetting GPS");
  sendmsg((byte *) resetgps, LRSETGPS);
  */

  delay(SDELAY);
  logger("boot", "3 turn off gpgsv messages");
  sendmsg((byte *) n_sendgsv, NSENDGSV);

}





void setup() {


  delay(500) ;
  Serial.begin(9600);
  Serial1.begin(9600);

  logger("boot", "GPSDO start");
  logger("boot", SWVER);
  logger("boot", "Copyright 2015");
  logger("boot", "by David Fannin KK6DF");
  logger("boot", "initializing");


  pinMode(pinFix, OUTPUT);
  pinMode(pinLock, OUTPUT);
  pinMode(pinFault, OUTPUT);

  mbutton.debounceTime = 50;

  pinMode(pinPllCtl, INPUT);

  pllctl.clear() ;

  digitalWrite(pinFix, HIGH);
  delay(500);
  digitalWrite(pinFix, LOW);
  delay(300);
  digitalWrite(pinLock, HIGH);
  delay(500);
  digitalWrite(pinLock, LOW);
  delay(300);
  digitalWrite(pinFault, HIGH);
  delay(500);
  digitalWrite(pinFault, LOW);
  lcd_init() ;
  delay(3000) ;
  setup_gps() ;
}

char inChar ;

void loop() {

  mbutton.Update();
  if ( Serial1.available() > 0 ) {
    inChar = Serial1.read() ;

    if ( gps.encode(inChar)) {


      if (gps.location.isUpdated()) {
        gps1.lat = gps.location.lat();
        gps1.lon = gps.location.lng();
      }

      if (gps.date.isUpdated()) {
        sprintf(gps1.cdate, "%02d-%02d-%04d", gps.date.month(), gps.date.day(), gps.date.year()) ;
      }

      if (gps.time.isUpdated()) {
        sprintf(gps1.ctime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second()) ;
      }

      if (gps.satellites.isUpdated()) {
        gps1.satusing = gps.satellites.value();
      }
      if (gpshdop.isUpdated()) {
        gps1.hdop = atof(gpshdop.value());
      }

      if (gpspdop.isUpdated()) {
        gps1.pdop = atof(gpspdop.value()) ;
      }

      if (gpsvdop.isUpdated()) {
        gps1.vdop = atof(gpsvdop.value()) ;
      }

      if (gps.altitude.isUpdated()) {
        gps1.altitudem = gps.altitude.meters() ;
      }
    }
  }


  // start check state
  //
  if ( guardtime && ( millis() > 60000 )) {
    guardtime = false ;
    gps1.last_sentencesWithFix = gps.sentencesWithFix();
    gps1.last_charsProcessed = gps.charsProcessed();
    gps1.last_failedChecksum = gps.failedChecksum();
  }

  // only check after guard timing
  if ( ! guardtime  && (millis() - lastErrorCheck) > 2000 ) {

    lastErrorCheck = millis() ;

    // check if has a gps fix and set the led
    if (gps.sentencesWithFix() > gps1.last_sentencesWithFix ) {
      gps1.last_sentencesWithFix = gps.sentencesWithFix() ;
      digitalWrite(pinFix, HIGH);
      if ( gps1.fixed == false ) {
        logger("log", "fix change true");
      }
      gps1.fixed = true;
    } else {
      if (gps1.fixed == true) {
        logger("log", "fix change false");
      }
      gps1.fixed = false;

      digitalWrite(pinFix, LOW);
    }


    // check if a fault exists
    // fault = no chars processed or checksum failed
    // during the previous intervals

    bool faultstate = false ;

    if (gps.charsProcessed() > gps1.last_charsProcessed ) {
      gps1.last_charsProcessed = gps.charsProcessed() ;
    } else {
      faultstate = true;
      gps1.last_charsProcessed = gps.charsProcessed() ;
      logger("log", "fault char proc");
    }

    if (gps.failedChecksum() > gps1.last_failedChecksum ) {
      gps1.last_failedChecksum = gps.failedChecksum() ;

      faultstate = true;
      logger("log", "fault failed checksum");
    }

    if (gps.time.age() > 5000 ) {
      faultstate = true;
      logger("log", "time failed age check");
    }

    if ( faultstate == true ) {
      digitalWrite(pinFault, HIGH);
    } else {
      digitalWrite(pinFault, LOW);
    }

    if ( pllctl.pop_stdev() < PLLCTL_STDEV_LEVEL ) {
      digitalWrite(pinLock, HIGH);
    } else {
      digitalWrite(pinLock, LOW);
    }
  }
  // end check state
  //

  // reading the pll lock pin every PLLCTL_INTERVAL ms, and
  // keep for xx secs (PLLCTL_INTERVAL  ms * PLLCTL_COUNT )
  // display statistics in the LCD
  if ( (millis() - pllctl_update ) >= PLLCTL_INTERVAL ) {
    int pc = analogRead(pinPllCtl) ;
    if ( pllctl.count() > PLLCTL_COUNT ) {
      double pctmp = pllctl.average();
      pllctl.clear() ;
      pllctl.add(pctmp);
    }
    pllctl.add( (double) pc);
    pllctl_update = millis() ;
  }

  if ( mbutton.clicks > 0 ) {
    ButtonClicks = 1;
  }

  if (ButtonClicks == 1 ) {
    lastupdate = 0 ;
    ButtonClicks = 0 ;
    if ( lockscreen ) {
      lockscreen = false ;
      LCDUPDATE = 4000 ;
      Serial.println("lock true to false");

    } else {
      lockscreen = true ;
      LCDUPDATE = 1000 ;
      Serial.println("lock false to true");
    }
  }


  lcd_main(false) ;
}
