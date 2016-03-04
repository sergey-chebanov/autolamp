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
#define SMB_HERE_PIN 13




#define LIGHT_SENSOR_PIN A0
#define CLOCK_RST_PIN A3
#define CLOCK_IO_PIN A4
#define CLOCK_SCLK_PIN A5

#define WAIT_TIME 10
#define RANGE 70
#define LUMINANCE_THRESHOLD 300


unsigned long timeWhenSmbWasHere = 0;
long waiting_time = 0;
long luminance = 0;

boolean forceOn = false;
boolean forceWasOn = false;
boolean powerOn = false;
long forceTurnOff = 0;

void turnLight(boolean enable) {
  digitalWrite(RELE_PIN, enable);
  powerOn = enable;
  if (!enable)
    delay(1000); //too loud turn off can awake the sonar 
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
private:
  unsigned long lastTimeCheckMs;
  
public:
  uint8_t hr, min;

  void update () { 
    Time t = rtc.time();
    hr = t.hr;
    min = t.min;
  }

  bool updateInterval(uint32_t sec) {
    uint32_t currentMs = millis();
    bool skip;
    if (skip = ((currentMs - lastTimeCheckMs) < (sec*1000))) 
      return false; 
    lastTimeCheckMs = currentMs;

    update();
    return true;
  }

};
AutolampTime clocks = AutolampTime();

//TODO: make it wors like a one time trigger
bool tooLate = false;
void checkIfTooLate() {
    
  //turn off the light if the current time within this interval
  const int from [2]  = {0, 00};
  const int to [2]  = {07, 00};

  if (!clocks.updateInterval(30))
    return;
    
  PRINT3 ("TIME:", clocks.hr, clocks.min)
  //fix TBD
  tooLate = (clocks.hr >= from[0]) && (clocks.hr <= to [0]);


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
  BT.print("temperature&humidity:");
  BT.print(sensor.getTemperatureInt()); BT.print ("\t");
  BT.println(sensor.getHumidityInt());
  clocks.update();
  BT.print("time is ");
  BT.print(clocks.hr);BT.print(":");
  BT.println(clocks.min);
  BT.print("luminance: ");
  BT.println(luminance);
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
      case 'f':
        forceTurnOff = (forceTurnOff+1)%2;
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
  pinMode(SMB_HERE_PIN, OUTPUT);
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

  digitalWrite(SMB_HERE_PIN, cm < RANGE);

  
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


  luminance = analogRead (LIGHT_SENSOR_PIN);

  //assume that the room is always dark if the light is ON
  bool room_is_dark = true;
  //If the light is OFF check luminance
  if (!powerOn) {
    room_is_dark = (luminance - LUMINANCE_THRESHOLD) < 0;
  }
  PRINT (luminance)
  PRINT (room_is_dark?"DARK":"LIGHT")


  bool smbIsHere = isSmbHere ();
  if (!forceTurnOff && ((smbIsHere && room_is_dark) || forceOn)) {
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

  delay(500);


  END_LINE
}
