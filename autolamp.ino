#include <NewPing.h>
#include <Wire.h>
#include <DS1302.h>
#include <SoftwareSerial.h>
#include <dht.h>
#include <BMP280.h>
#include <SparkFunHTU21D.h>

#define P0 1013.25

//MODES
#define DEBUG


#ifdef DEBUG
#define PRINT(MSG) Serial.print(MSG); Serial.print ("\t");
#define PRINT2(MSG1, MSG2) Serial.print(MSG1);  Serial.print (" "); Serial.print(MSG2); Serial.print ("\t");
#define PRINT3(MSG1, MSG2, MSG3) Serial.print(MSG1); Serial.print(MSG2); Serial.print(MSG3); Serial.print ("\t");
#define END_LINE Serial.print("\n");
#else
#define PRINT(MSG)
#define PRINT2(MSG1, MSG2)
#define PRINT3(MSG1, MSG2, MSG3)
#define END_LINE
#endif

//PINS
#define ECHO_PIN 2
#define TRIG_PIN 3
#define RELE_PIN 4
//#define DHT11_PIN 6
#define CLOCK_RST_PIN 5
#define CLOCK_IO_PIN 6
#define CLOCK_SCLK_PIN 7

//BLUETOOTH
/*RX to TX, TX to RX*/
#define SS_RX_PIN 8
#define SS_TX_PIN 9
#define SMB_HERE_PIN 12
#define FORCE_PIN 13


#define LIGHT_SENSOR_PIN A3

/*
Board  I2C / TWI pins
Uno, Ethernet A4 (SDA), A5 (SCL)
*/

#define WAIT_TIME 10
#define RANGE 90
#define FORCE_DISTANCE 20
#define LUMINANCE_THRESHOLD 300
#define MAX_DISTANCE 200


unsigned long timeWhenSmbWasHere = 0;
long waiting_time = 0;
long luminance = 0;

boolean forceOn = false;
boolean forceWasOn = false;
boolean powerOn = false;
long forceOff = 0;

void turnLight(boolean enable) {
  digitalWrite(RELE_PIN, enable);
  powerOn = enable;
  if (!enable)
    delay(1000); //too loud turn off can awake the sonar
}

bool night_mode = true;

DS1302 rtc (CLOCK_RST_PIN, CLOCK_IO_PIN, CLOCK_SCLK_PIN);
bool setup_clock () {
  rtc.writeProtect(false);
  rtc.halt(false);
  return true;
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

struct {
  BMP280 sensor;
  double T;
  double P;
  void read () {
    char result = sensor.startMeasurment();
 
    if(result!=0){
      delay(result);
      result = sensor.getTemperatureAndPressure(T,P);   
    }
  }
  bool init () {
    if(!sensor.begin()){
      Serial.println("BMP init failed!");
      return false;
    }
    else Serial.println("BMP init success!");
    sensor.setOversampling(4);
    return true;
  }
} bmp280;

struct {
  HTU21D sensor;
  float T;
  float H;
  void read () {
    H = sensor.readHumidity();
    T = sensor.readTemperature();
  } 
  bool init () {
    sensor.begin();
  }
} htu21d;


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
  BT.print("forceOff:");
  BT.println(forceOff);
  BT.print("waiting time:");
  BT.println(waiting_time);
  BT.print("too late:");
  BT.println(tooLate);
  clocks.update();
  BT.print("time is ");
  BT.print(clocks.hr);BT.print(":");
  BT.println(clocks.min);
  BT.print("luminance: ");
  BT.println(luminance);
  BT.print("T = \t"); BT.print(htu21d.T, 2); BT.print(" C\t\t");
  BT.print("H = \t"); BT.print(htu21d.H, 2); BT.println(" %");
  BT.print("T = \t");BT.print(bmp280.T,2); BT.print(" degC\t");
  BT.print("P = \t");BT.print(bmp280.P,2); BT.print(" mBar\t");
  BT.print("P2 = \t");BT.print(bmp280.P*750.06/1000, 2); BT.println(" mm Hg\t");
}

bool checkBT () {
  PRINT2 ("BT:", BT.available())

  if (BT.available()) {
    char c = BT.read();
    switch (c) {
      case 't':
        bmp280.read();
        htu21d.read();
        break;
      case '0':
        //turn off if there is nobody at the desk
        forceOn = false;
        forceWasOn = false;
        break;
      case 'f':
        forceOff = (forceOff+1)%2;
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
  }
}


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin (115200);
  setup_clock ();
  setup_BT ();

  //BMP280 setup
  bmp280.init();

  //HTU21D
  htu21d.init();
  

  // initialize digital pin 13 as an output.
  pinMode(RELE_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FORCE_PIN, OUTPUT);
  pinMode(SMB_HERE_PIN, OUTPUT);
}



NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

boolean isSmbHere () {


  unsigned long microsec = sonar.ping_median(3);
  unsigned int cm =  sonar.convert_cm (microsec);

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


  //detect force gesture. In force mode light is turned on permanently
  //check the previous
  if (cm < FORCE_DISTANCE && !forceWasOn) {
    forceOn = !forceOn;
  }
  forceWasOn = (cm < FORCE_DISTANCE);

  waiting_time = (timeNow - timeWhenSmbWasHere)/1000;
  PRINT2 ("WT:", waiting_time)

  digitalWrite(SMB_HERE_PIN, cm < RANGE);

  if (cm < RANGE) {
    timeWhenSmbWasHere = timeNow;
  }

  smbIsHere = (cm < RANGE) || (waiting_time < WAIT_TIME);

  PRINT (timeWhenSmbWasHere)
  PRINT2("FORCE MODE:", forceOn?"ON":"OFF");


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
    //room_is_dark = (luminance - LUMINANCE_THRESHOLD) < 0;
    room_is_dark = true;
  }
  PRINT (luminance)
  PRINT (room_is_dark?"DARK":"LIGHT")


  bool smbIsHere = isSmbHere ();

  //forceOff has bigger priority
  //forceOn is the second priority
  //
  turnLight (!forceOff && ((smbIsHere && room_is_dark) || forceOn));
  PRINT2 ("POWER:", powerOn?"ON":"OFF")

  flagForceMode (forceOn);

  checkBT ();
  //check if too late to turn off force mode
  checkIfTooLate ();

  delay(300);


  END_LINE
}

