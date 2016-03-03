#include <Wire.h>
#include <DS1302.h>
#include <SoftwareSerial.h>
#include <dht.h>

//MODES
#define DEBUG


#ifdef DEBUG 
#define PRINT(MSG) Serial.print(MSG); Serial.print ("\t");
#define PRINT2(MSG1, MSG2) Serial.print(MSG1);  Serial.print (" "); Serial.print(MSG2); Serial.print ("\t");
#define PRINT3(MSG1, MSG2, MSG3) Serial.print(MSG1); Serial.print(MSG2); Serial.print(MSG3); Serial.print ("\t");
#define END_LINE Serial.print("\n");
#else
#define PRINT(MSG)
#define END_LINE
#endif

//PINS
#define ECHO_PIN 2
#define TRIG_PIN 3
#define RELE_PIN 5
#define FORCE_PIN 4
#define DHT11_PIN 6
/*RX to TX, TX to RX*/
#define SS_RX_PIN 10 
#define SS_TX_PIN 11




#define LIGHT_SENSOR_PIN A0
#define CLOCK_RST_PIN A3
#define CLOCK_IO_PIN A4
#define CLOCK_SCLK_PIN A5

#define WAIT_TIME 10
#define RANGE 70
#define LUMINANCE_THRESHOLD 300


unsigned long timeWhenSmbWasHere = 0;
long waiting_time = 0;

boolean forceOn = false;
boolean forceWasOn = false;
boolean powerOn = false;

void turnLight(boolean enable) {
  digitalWrite(RELE_PIN, enable);
  powerOn = enable;
}

void printLED(const int line_num, char const * const msg ) {

}

bool night_mode = true;

DS1302 rtc (CLOCK_RST_PIN, CLOCK_IO_PIN, CLOCK_SCLK_PIN);
bool setup_clock () {
  rtc.writeProtect(false);
  rtc.halt(false);
  return true;
}


DHT sensor = DHT();
bool setup_temperature() {
  sensor.attach(DHT11_PIN);
}


bool readDHT() {
  sensor.update();
  switch (sensor.getLastError()) {
        case DHT_ERROR_OK:
        return true;
  }
}

unsigned long lastTimeCheckMs = 0;

class AutolampTime {
  public:
    uint8_t hr, min;

    bool readTime () {
      return true;
    }
};

//TODO: make it wors like a one time trigger
bool tooLate = false;
void checkIfTooLate() {

  unsigned long currentMs = millis();
  bool skip = ((currentMs - lastTimeCheckMs) < (60*1000));
  if (skip) 
    return;

  lastTimeCheckMs = currentMs;
    
  //turn off the light if the current time within this interval
  const int from [2]  = {0, 00};
  const int to [2]  = {07, 00};

  Time now = rtc.time();
  PRINT3 ("TIME:", now.hr, now.min)
  //fix TBD
  tooLate = (now.hr >= from[0]) && (now.hr <= to [0]);

  //PRINT ("TOO_LATE:")
  //PRINT (tooLate)

  //TODO: idea??? -- tooLate should cancel forceOn effect
  if (tooLate) {
    forceWasOn = false;
    forceOn = false;
    PRINT ("TOO LATE")
  }
}


SoftwareSerial BT(SS_RX_PIN, SS_TX_PIN);
bool setup_BT () {
  BT.begin(9600); 
}

void statusBT (char c) {
  BT.print("command :");
  BT.println(c);  
  BT.print("powerOn :");
  BT.println(powerOn);
  BT.print("forceOn:");
  BT.println(forceOn);
  BT.print("forceOn:");
  BT.println(forceOn);
  BT.print("waiting time:");
  BT.println(waiting_time);
  BT.print("too late:");
  BT.println(tooLate);
}

bool checkBT () {
  if (BT.available()) {
    PRINT ("BT_YES")
    char c = BT.read();
    switch (c) {
      case 't':
        readDHT();
        BT.print(sensor.getTemperatureInt()); BT.print ("\t");
        BT.println(sensor.getHumidityInt());
        break;
      case '0':
        //turn off if there is nobody at the desk
        forceOn = false;
        forceWasOn = false;
        break;
      case '1':
        forceOn = true;
        break;
      case 'l':
        checkIfTooLate();
        
        break;
      case 's':
        //status
        break;
      default:
        BT.println("Command is not found");
    }
    statusBT(c);
  } else {
    PRINT ("BT_NO")
  }
}


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin (115200);
  setup_clock ();
  setup_BT ();
  setup_temperature();
 
  // initialize digital pin 13 as an output.
  pinMode(RELE_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);
  pinMode(FORCE_PIN, OUTPUT);
}



boolean isSmbHere () {
  
  int duration, cm; 
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 58*200);
  //duration = pulseIn(ECHO_PIN, HIGH);

  //duration = pulseIn(ECHO_PIN, HIGH);

  PRINT2("DURATION:", duration)

  //hack to reset ??
  if (duration <= 0 ) {
      delay(200);
      pinMode(ECHO_PIN, OUTPUT);
      digitalWrite(ECHO_PIN, LOW);
      digitalWrite(TRIG_PIN, LOW);
      delay(200);
      pinMode(ECHO_PIN, INPUT);
  }
  
  cm = duration / 58;
  PRINT2 ("RAW CM:", cm)

  if (cm <= 0) {
    //infinite
    cm = 5000;
  }
  
  cm = constrain (cm, 5, 200);
  PRINT2 ("CM:", cm)
  
  boolean smbIsHere = false;
  long timeNow = millis();
  PRINT (timeNow)

  if (cm < 15 && !forceWasOn) {
    forceOn = !forceOn;
    printLED(1, forceOn?"FORCE ON":"FORCE OFF" );
  }
  forceWasOn = (cm < 15);

  waiting_time = (timeNow - timeWhenSmbWasHere)/1000;
  PRINT2 ("WT:", waiting_time)

  if (cm < RANGE) {
    smbIsHere = true;
    timeWhenSmbWasHere = timeNow;
    PRINT("SMB IS HERE")
    printLED(0, "SMB IS HERE");
  } else if (waiting_time < WAIT_TIME) {
    smbIsHere = true;
    PRINT2(WAIT_TIME - waiting_time, "HERE????")
    printLED(0,"HERE?");
  } else {
    PRINT("NOBODY IS HERE") 
    printLED(0, "NOBODY IS HERE");
  }

  
  PRINT (smbIsHere)
  PRINT (timeWhenSmbWasHere)
  PRINT(forceWasOn);
  PRINT(forceOn?"FORCE MODE ON":"FORCE MODE OFF");

  return smbIsHere;
}

void flagForceMode (bool on) {
  digitalWrite (FORCE_PIN, on?HIGH:LOW);
}

// the loop function runs over and over again forever
void loop() {


  int light = analogRead (LIGHT_SENSOR_PIN);

  //assume that the room is always dark if the light is ON
  bool room_is_dark = true;
  //If the light is OFF check luminance
  if (!powerOn) {
    room_is_dark = (light - LUMINANCE_THRESHOLD) < 0;
  }
  PRINT (light)
  PRINT (room_is_dark?"DARK":"LIGHT")

  bool smbIsHere = isSmbHere ();
  if ((smbIsHere && room_is_dark) || forceOn) {
    PRINT ("POWER ON")
    turnLight(true);
  } else {
    PRINT ("POWER OFF")
    turnLight(false);
  }
  flagForceMode (forceOn);

  checkBT();
  //check if too late to turn off force mode
  checkIfTooLate ();

  delay(300);


  END_LINE
}
