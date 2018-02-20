/*
 * FTS Design Pte Ltd, 2016
 * https://ftsdesign.biz/
 * 
 * Device firmware for iSeeTheLight
 * https://ftsdesign.biz/iseethelight/index.html
 * For compilation using Arduino IDE 1.6.11 and esp8266 board v. 2.3.0, 
 * module "Generic ESP8266 Module"
 * 
 * Released under Creative Commons Attribution 4.0 International License.
 * http://creativecommons.org/licenses/by/4.0/
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "FS.h"
#include <stdlib.h>

const unsigned int PIN_R = 13;
const unsigned int PIN_G = 12;
const unsigned int PIN_B = 14;

const unsigned int PIN_BROWNOUT = 5; // 4 swapped with 5

struct LightPWM {
  // Values can be from 0 to 1023 (PWMRANGE)
  unsigned short r;
  unsigned short g;
  unsigned short b;
};
// This is the original colour sent by the client
LightPWM originalLightPWM = LightPWM {0, 0, 0};
// This is not resettable and only goes up every time brownout condition is triggered
short powerReduction = 0;

const LightPWM PWM_OFF = LightPWM {0, 0, 0};
// Default PWM freq is 1kHz which is too high and unreliable with LED drivers
const unsigned int PWM_FREQ = 200;

unsigned int seqStep = 0;
const unsigned int MAX_SEQ_LEN = 256;
unsigned int seq_colour[MAX_SEQ_LEN];
unsigned int seq_duration[MAX_SEQ_LEN];
const unsigned int MASK_R = 1;
const unsigned int MASK_G = 2;
const unsigned int MASK_B = 4;
const unsigned int PAUSE = 0;
const unsigned int COLOUR_BLACK = 0;
const unsigned int COLOR_RED = MASK_R;
const unsigned int COLOR_GREEN = MASK_G;
const unsigned int COLOR_BLUE = MASK_B;
const unsigned int COLOR_YELLOW = COLOR_RED | COLOR_GREEN;
const unsigned int COLOR_MAGENTA = COLOR_RED | COLOR_BLUE;
const unsigned int COLOR_CYAN = COLOR_GREEN | COLOR_BLUE;
const unsigned int COLOR_WHITE = COLOR_RED | COLOR_GREEN | COLOR_BLUE;

unsigned long lastTick = 0;
unsigned long lastDuration = 0;
unsigned long lastColour = 0;
// When light was switched off last time
unsigned long lastTimeOffMs = 0;

const unsigned int STATE_OFF = 0;
const unsigned int STATE_ON = 1;
unsigned int state = STATE_OFF;

const short MODE_SEQ = 0;
const short MODE_DIRECT = 1;
short mode = MODE_SEQ;

// Brownout handling logic
unsigned long brownoutActionTakenTimeMs = 0;
// See datasheet for CAX903, signal duration on low voltage is 140..240..460ms
const unsigned long MAX_BROWNOUT_DURATION_MS = 460; 
// This is not resettable, once raised stays raised till power down. This is for UI low battery indication.
unsigned int batteryBecameLow = 0;

/*
 * v7 27-Aug-2016
 * Stopped using built-in LED as apparently it may interfere with serial communication.
 */
const unsigned int FIRMWARE_VERSION = 7;

const char SSID_PREFIX[] = "Light-";
char macAsString[18]; // Complete MAC in its proper form
char ssid[32]; // SSID_PREFIX + last 2 bytes of MAC as hex
/*
 * Chip ID - pure numeric password is easier to type in on mobile. 
 * It must be at least 8 characters long, so padded with leading 0 if needed.
 */
char pass[18]; 
IPAddress apIP(192,168,1,1);

// Ambient light sensor
const unsigned long MIN_AMB_LIGHT_SENSOR_READ_INTERVAL_MS = 10000;
const unsigned int AMB_LIGHT_READINGS_COUNT = 18;
/* 
 * We need this because of low pass filter on sensor. We can read  
 * ambient light sensor only after that many milliseconds since we 
 * told it to switch off.
 */
const unsigned long MIN_OFF_DELAY_MS = 150;
unsigned long lastReadAmbLightMs = 0;
unsigned long ambLightThresholdHigh = 40;
unsigned long ambLightThresholdLow = 20;
unsigned int countAmbLightAboveThreshold = 0;
unsigned int countAmbLightBelowThreshold = 0;
unsigned int sensorReadingMv = 0;
unsigned int ambientLightSensorEnabled = 0;
const unsigned int AMB_LIGHT_NIGHT = 0;
const unsigned int AMB_LIGHT_DAY = 1;
unsigned int sensorReadingState = AMB_LIGHT_NIGHT;

// Timer
unsigned long timerDurationMs;
unsigned long timerStartedMs;

// Persistence
unsigned int persistenceEnabled = 1;
const char SEQ_FILE_NAME[] = "/seq.dat";
const char SETTINGS_FILE_NAME[] = "/settings.dat";

// HTTP request params
ESP8266WebServer server(80);

const char SIGNAL_STRING[] = "Let there be light!";
const char PARAM_SEQUENCE[] = "s";
const char PARAM_STATE[] = "state";
const char PARAM_TIMER[] = "timer";
const char PARAM_AMBIENT[] = "ambient";
const char PARAM_DIRECT[] = "d";

unsigned long now = millis();

const char DEMO_SEQUENCE[] = "030103005203005303005403005503005603005703";

// ------------- remove me ------------
unsigned long lastTimeDebug = 0;
const unsigned long MIN_TIME_DEBUG_INTERVAL_MS = 10 * 60 * 1000; 

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

//  pinMode(16, INPUT);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  allLightOffAnalog();

  // Brownout sensor - normal power is HIGH
  pinMode(PIN_BROWNOUT, INPUT_PULLUP);

  getMacAddress();
  sprintf(pass, "%08u", ESP.getChipId());
  Serial.printf("\nFirmware: %u\nssid: %s\npass: %s\nchipId: %u\nflashChipId: %u\nflashChipSize: %u\n\n", 
    FIRMWARE_VERSION, ssid, pass, ESP.getChipId(), ESP.getFlashChipId(), ESP.getFlashChipSize());

  if (persistenceEnabled) {
    if (!SPIFFS.begin()) {
      persistenceEnabled = 0;
      Serial.printf("ERROR: Failed to open file system\n"); 
    }
  }

  seq_duration[0] = 0;
  setState(STATE_OFF);
  readFromFile();
  // If nothing is saved - show demo
  if (seq_duration[0] == 0) {
    setSequenceFromString(DEMO_SEQUENCE);
    setState(STATE_ON);
  }

  restoreSettings();

  Serial.printf("Setting up WiFi AP %s %s\n", ssid, pass);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, pass);
  Serial.printf("WiFi set up\n");

  server.on("/", handleHttp);
  server.onNotFound(handleHttp);
  server.begin();

  if(digitalRead(PIN_BROWNOUT) == LOW) {
    Serial.printf("LOW BATTERY\n"); 
  }

  Serial.printf("Started: %u\n", millis()); 
}

void setState(unsigned int newState) {
  /*
   * For seq mode we just raise a flag here for later execution in the loop.
   * For direct mode we do analog write directly here on state change, as it 
   * is more expensive otherwise.
   */
  if (newState != state) {
    Serial.printf("State %u => %u\n", state, newState); 
    if(newState == STATE_ON && digitalRead(PIN_BROWNOUT) == LOW) {
      Serial.printf("Can't turn light on when voltage is low\n"); 
    } else {
      state = newState;
    }

    if (newState == STATE_OFF) {
      allLightOffAnalog();
    }
    
    if (mode == MODE_DIRECT && state == STATE_ON) {
      lightPWM(originalLightPWM);
    }
  }
}

void getMacAddress() {
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(macAsString, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sprintf(ssid, "%s%02x%02x", SSID_PREFIX, mac[4], mac[5]);
}

void readFromFile() {
  if (persistenceEnabled && SPIFFS.exists(SEQ_FILE_NAME)) {
    Serial.printf("Opening %s\n", SEQ_FILE_NAME);
    File seqFile = SPIFFS.open(SEQ_FILE_NAME, "r");
    if (seqFile) {
      unsigned int seq_colour_tmp[MAX_SEQ_LEN];
      unsigned int seq_duration_tmp[MAX_SEQ_LEN];
      seq_duration_tmp[0] = 0;
      int c;
      int seqPosition = 0;
      int bytesRead = 0;
      int stepCompleted = 1;
      for (bytesRead = 0; bytesRead < MAX_SEQ_LEN * 3; bytesRead++) {
        c = seqFile.read();
        if (c == EOF)
          break;
        int digit = charToDigit(c);
        stepCompleted = 0;
        switch (bytesRead % 3) {
          case 0:
            // Colour
            seq_colour_tmp[seqPosition] = digit;
            break;
          case 1:
            // First digit of duration
            seq_duration_tmp[seqPosition] = digit * 10;
            break;
          case 2:
            // Second digit of duration
            seq_duration_tmp[seqPosition] += digit;
            seq_duration_tmp[seqPosition] *= 100;
            stepCompleted = 1;
//            Serial.printf("step %u %u\n", seq_colour_tmp[seqPosition], seq_duration_tmp[seqPosition]);
            seqPosition++;
            break;
        }
        if (stepCompleted == 1 && seq_duration_tmp[seqPosition - 1] == 0)
          break;
      }
      seqFile.close();
      Serial.printf("Bytes read: %u\n", bytesRead);
  
      if (seqPosition < MAX_SEQ_LEN - 1)
        seq_duration_tmp[seqPosition] = 0;
      if (seqPosition > 0 && stepCompleted) {
        // Copy to working array
        for (int i = 0; i <= seqPosition; i++) {
          seq_colour[i] = seq_colour_tmp[i];
          seq_duration[i] = seq_duration_tmp[i];
        }
        setState(STATE_ON);
      }
    } else {
      Serial.printf("Cannot read\n");
    }
  }
}

void firstReadAmbientLightSensor() {
  sensorReadingMv = analogRead(A0);
  lastReadAmbLightMs = now;
//  Serial.printf("Ambient light: %u\n", sensorReadingMv);
  if (sensorReadingMv >= ambLightThresholdHigh) {
    sensorReadingState = AMB_LIGHT_DAY;
  }
  if (ambientLightSensorEnabled) {
    if (sensorReadingState == AMB_LIGHT_DAY) {
      setState(STATE_OFF);
    } else {
      setState(STATE_ON);
    }
  }
}

void readAmbientLightSensor() {
  unsigned long now = millis();

  /*
   * When we just start up, we need to set things based on just one sensor reading. 
   * Later, we take into account hysteresis and duration.
   */
  if (lastReadAmbLightMs == 0) {
    firstReadAmbientLightSensor();

  } else if (
    mode == MODE_SEQ // We are in sequence mode...
    && lastColour == COLOUR_BLACK // and the light is NOT currently on...
    && lastReadAmbLightMs + MIN_AMB_LIGHT_SENSOR_READ_INTERVAL_MS < now // and the light was switched off earlier, so that RC filter gets it
    && lastTimeOffMs + MIN_OFF_DELAY_MS < now) // and it has been a while since we measured it last time
    { 
    
    sensorReadingMv = analogRead(A0);
    lastReadAmbLightMs = now;
//    Serial.printf("Ambient light: %u\n", sensorReadingMv);

    // We monitor the sensor always, but use its readings to turn the light on/off only if enabled
    if (state == STATE_ON && sensorReadingMv > ambLightThresholdHigh) {
      countAmbLightAboveThreshold++;
      countAmbLightBelowThreshold = 0;
      if (countAmbLightAboveThreshold > AMB_LIGHT_READINGS_COUNT) {
        // off
        sensorReadingState = AMB_LIGHT_DAY;
        if (ambientLightSensorEnabled) {
          Serial.printf("Light off by amb sensor\n");
          setState(STATE_OFF);
        }
      }
    } else if (state == STATE_OFF && sensorReadingMv < ambLightThresholdLow) {
      countAmbLightBelowThreshold++;
      countAmbLightAboveThreshold = 0;
      if (countAmbLightBelowThreshold > AMB_LIGHT_READINGS_COUNT) {
        // on
        sensorReadingState = AMB_LIGHT_NIGHT;
        if (ambientLightSensorEnabled) {
          Serial.printf("Light on by amb sensor\n");
          setState(STATE_ON);
        }
      }
    }
  }
}

void loop() {
  now = millis();

   if (lastTimeDebug + MIN_TIME_DEBUG_INTERVAL_MS < now) {
    lastTimeDebug = now;
    Serial.printf("ts %u\n", now);
   }

  if (mode == MODE_SEQ) {
    loopSeq();
  } else {
    loopDirect();
  }
}

void loopDirect() {
  checkBrownout();
  server.handleClient();
  checkTimer();
}

void loopSeq() {
  checkBrownout();
  readAmbientLightSensor();
  server.handleClient();
  checkTimer();

  now = millis();
  if (state == STATE_ON) {
    if (lastTick + lastDuration < now) {
      // Last duration expired, next step
      unsigned long duration = seq_duration[seqStep];
      unsigned int colour = seq_colour[seqStep];
      // Zero duration marks end of sequence
      if (duration == 0 || seqStep >= MAX_SEQ_LEN ) {
        seqStep = 0;
        duration = seq_duration[seqStep];
        colour = seq_colour[seqStep];
      }

      lastTick = now;
      lastDuration = duration;
      lastColour = colour;

      // For ambient light sensor we need to know when we turned the light off last time
      if (colour == COLOUR_BLACK) {
        lastTimeOffMs = now;
      }
      
      // Don't even try to turn on LED if already in brownout
      if (lastColour == COLOUR_BLACK || digitalRead(PIN_BROWNOUT) != LOW) {
//        Serial.printf("tick2 %u %u %u %u\n", seqStep, lastColour, duration, now);
        lightPWM(lastColour);
      }
      seqStep++;
    }
    
  } else {
    if (lastColour != COLOUR_BLACK) {
      lastColour = COLOUR_BLACK;
      lastTimeOffMs = now;
    }
    allLightOffAnalog();
  }
}

void allLightOffAnalog() {
  analogWrite(PIN_R, 0);
  analogWrite(PIN_G, 0);
  analogWrite(PIN_B, 0);
}

void checkBrownout() {
  if (digitalRead(PIN_BROWNOUT) == LOW) {
    // Brownout
    if (state == STATE_ON) {
      // We need to wait for MAX_BROWNOUT_DURATION_MS before taking action again 
      // Sensor will not flip back for some hundreds ms after blackout detected
      if (brownoutActionTakenTimeMs + MAX_BROWNOUT_DURATION_MS < now) {
        reducePower();
        if (mode == MODE_SEQ) {
          // Update the light with reduced power
          allLightOffAnalog();
          lightPWM(lastColour);
        } else {
          lightPWM(originalLightPWM);
        }

        brownoutActionTakenTimeMs = now;
        // We do the logging only after we're done with the important things
        Serial.printf("Lowpwr %u %u\n", powerReduction, now);
      }
    }
    batteryBecameLow = 1;
  } else {
    // TODO If we were brownout on power up, and MAX_BROWNOUT_DURATION_MS has passed, try turning on now
  }
}

void reducePower() {
  if (state == STATE_ON) {
    powerReduction++;
    if (powerReduction >= 9) {
      setState(STATE_OFF);
      Serial.printf("Lowpwroff, goodbye %u\n", millis());
      // We don't want anything bad (like failed write to flash) to happen when power is very low already, safer to go to sleep
      ESP.deepSleep(0xffffffff, WAKE_RF_DISABLED);
    }
  }
}

void checkTimer() {
  if (timerStartedMs > 0) {
    if (now > timerStartedMs + timerDurationMs) {
      setState(STATE_OFF);
      timerStartedMs = 0;
      seqStep = 0;
    }
  }
}

// To be used only for "UI" signals
void flash(unsigned int colour, unsigned long ms) {
  digitalWrite(PIN_R, isOn(colour, MASK_R));
  digitalWrite(PIN_G, isOn(colour, MASK_G));
  digitalWrite(PIN_B, isOn(colour, MASK_B));
  delay(ms);
  digitalWrite(PIN_R, 0);
  digitalWrite(PIN_G, 0);
  digitalWrite(PIN_B, 0);
}

void lightPWM(struct LightPWM pwm) {
  if (mode == MODE_DIRECT) {
    lightPWM(pwm.r, pwm.g, pwm.b);
  }
}

void lightPWM(int colour) {
  short r = isOn(colour, MASK_R) ? 0x3ff : 0;
  short g = isOn(colour, MASK_G) ? 0x3ff : 0;
  short b = isOn(colour, MASK_B) ? 0x3ff : 0;
  lightPWM(r, g, b);
}

void lightPWM(short r, short g, short b) {
  if (state == STATE_ON) {
    analogWrite(PIN_R, r >> powerReduction);
    analogWrite(PIN_G, g >> powerReduction);
    analogWrite(PIN_B, b >> powerReduction);
//    Serial.printf("lightPWM [%u %u %u] >> %u\n", r, g, b, powerReduction);
  } else {
    allLightOffAnalog(); // Normally don't need, but for safety
  }
}

unsigned int isOn(unsigned int colour, unsigned int mask) {
  if (colour & mask) {
    return HIGH;
  } else {
    return LOW;
  }
}

void handleHttp() {
  Serial.printf("http\n");
  /*
   * Payload format:
   * s=[cdd]+
   * where c is colour
   * dd is duration in 0.1s (0.1 .. 9.9s)
   */
  unsigned int settingsChanged = 0;
  if (server.method() == HTTP_GET) {
    for ( int i = 0; i < server.args(); i++ ) {
      String currArgName = server.argName(i);
      String currArgValue = server.arg(i);
      int argValueLength = currArgValue.length();
      if (argValueLength > 0) {
        if (currArgName.equals(PARAM_DIRECT)) {
          // d=00ff00 [rrggbb]
          if (mode != MODE_DIRECT) {
            analogWriteFreq(PWM_FREQ);
            mode = MODE_DIRECT;
            seqStep = 0;
          }
          settingsChanged = 1;
          setLightPWM(currArgValue.c_str());
          lightPWM(originalLightPWM);
          // Sending any colour other than black sets state to ON automatically
          if ((originalLightPWM.r >> powerReduction | originalLightPWM.g >> powerReduction | originalLightPWM.b >> powerReduction) > 0) {
            setState(STATE_ON);
          } else {
            setState(STATE_OFF);
          }
          
        } else if (currArgName.equals(PARAM_SEQUENCE)) {
          Serial.printf("params %s %s\n", PARAM_SEQUENCE, currArgValue.c_str());
          if (mode != MODE_SEQ) {
            lightPWM(PWM_OFF);
            mode = MODE_SEQ;
          }
          setSequenceFromString(currArgValue);
          persistWorkingSequence();
          if (state == STATE_ON) {
            settingsChanged = 1;
          }

        } else if (currArgName.equals(PARAM_STATE)) {
          unsigned int newState;
          if (currArgValue.equals("0")) {
            newState = STATE_OFF;
            timerStartedMs = 0; // Kill timer
          } else {
            newState = STATE_ON;
          }
          if (newState != state) {
            setState(newState);
            seqStep = 0;
            // If state goes from OFF to ON and timer was preset - initiate it
            if (newState == STATE_ON && timerDurationMs > 0 && timerStartedMs == 0) {
              timerStartedMs = now;
            }
          }

        } else if (currArgName.equals(PARAM_TIMER)) {
          int minutes = atoi(currArgValue.c_str());
          // Save the duration, but start timer only if the light is on
          unsigned long timerDurationMsNew = minutes * 60 * 1000;
          if (minutes > 0 && state == STATE_ON) {
            timerStartedMs = now;
          } else {
            timerStartedMs = 0;
          }
          if (timerDurationMsNew != timerDurationMs) {
            timerDurationMs = timerDurationMsNew;
            settingsChanged = 1;
          }

        } else if (currArgName.equals(PARAM_AMBIENT)) {
          unsigned int ambientLightSensorEnabledNew = !currArgValue.equals("0");
          if (ambientLightSensorEnabledNew != ambientLightSensorEnabled) {
            ambientLightSensorEnabled = ambientLightSensorEnabledNew;
            settingsChanged = 1;
          }
          
        }
      }
    }
  }

  String page = generatePage();
  server.send(200, "text/plain", page);

  if (settingsChanged == 1) {
    // TODO persist settings
    persistSettings();
  }
}

void persistSettings() {
  if (persistenceEnabled) {
    if (digitalRead(PIN_BROWNOUT) == HIGH) {
      Serial.printf("Writing to %s\n", SETTINGS_FILE_NAME);
      File settingsFile = SPIFFS.open(SETTINGS_FILE_NAME, "w");
      if (settingsFile) {
        writeLong(settingsFile, timerDurationMs);
        writeInt(settingsFile, ambientLightSensorEnabled);
        writeByte(settingsFile, mode);
        writeShort(settingsFile, originalLightPWM.r);
        writeShort(settingsFile, originalLightPWM.g);
        writeShort(settingsFile, originalLightPWM.b);
        settingsFile.close();
      }
    } else {
      Serial.printf("Cannot write in brownout\n");
    }
  }
}

// ----- serialization library -----
int writeLong(File f, long value) {
  const int bytesToWrite = sizeof(value);
  for (int i = 0; i < bytesToWrite; i++) {
    byte b = (byte) 0xff & (value >> (i * 8));
    Serial.printf("%02x ", b);
    f.write(b);
  }
  Serial.printf("\n");
  return bytesToWrite;
}

int writeInt(File f, int value) {
  const int bytesToWrite = sizeof(value);
  for (int i = 0; i < bytesToWrite; i++) {
    byte b = (byte) 0xff & (value >> (i * 8));
    Serial.printf("%02x ", b);
    f.write(b);
  }
  Serial.printf("\n");
  return bytesToWrite;
}

int writeShort(File f, short value) {
  const int bytesToWrite = sizeof(value);
  for (int i = 0; i < bytesToWrite; i++) {
    byte b = (byte) 0xff & (value >> (i * 8));
    Serial.printf("%02x ", b);
    f.write(b);
  }
  Serial.printf("\n");
  return bytesToWrite;
}

int writeByte(File f, byte value) {
  f.write(value);
  Serial.printf("%02x\n", value);
  return 1;
}

long readLong(File f) {
  long value = 0;
  int bytesRead = 0;
  while (f.available() && bytesRead < sizeof(value)) {
    byte b = f.read();
    Serial.printf("%02x ", b);
    value |= (b << bytesRead++ * 8);
  }
  Serial.printf("\n");
  if (bytesRead < sizeof(value)) {
    Serial.printf("ERROR read less bytes than expected: %u < %u\n", bytesRead, sizeof(value));
    return 0;
  }
  return value;
}

int readInt(File f) {
  int value = 0;
  int bytesRead = 0;
  while (f.available() && bytesRead < sizeof(value)) {
    byte b = f.read();
    Serial.printf("%02x ", b);
    value |= (b << bytesRead++ * 8);
  }
  Serial.printf("\n");
  if (bytesRead < sizeof(value)) {
    Serial.printf("ERROR read less bytes than expected: %u < %u\n", bytesRead, sizeof(value));
    return 0;
  }
  return value;
}

short readShort(File f) {
  short value = 0;
  int bytesRead = 0;
  while (f.available() && bytesRead < sizeof(value)) {
    byte b = f.read();
    Serial.printf("%02x ", b);
    value |= (b << bytesRead++ * 8);
  }
  Serial.printf("\n");
  if (bytesRead < sizeof(value)) {
    Serial.printf("ERROR read less bytes than expected: %u < %u\n", bytesRead, sizeof(value));
    return 0;
  }
  return value;
}

byte readByte(File f) {
  if (f.available() == 0) {
    Serial.printf("ERROR read less bytes than expected: %u < %u\n", 0, 1);
    return 0;
  }
  byte b = f.read();
  Serial.printf("%02x\n", b);
  return b;
}
// ----- /serialization library -----

void restoreSettings() {
  if (persistenceEnabled && SPIFFS.exists(SETTINGS_FILE_NAME)) {
    File settingsFile = SPIFFS.open(SETTINGS_FILE_NAME, "r");
    if (settingsFile) {
      Serial.printf("Reading from %s\n", SETTINGS_FILE_NAME);
      timerDurationMs = readLong(settingsFile);
      ambientLightSensorEnabled = readInt(settingsFile);
      mode = readByte(settingsFile);
      originalLightPWM.r = readShort(settingsFile);
      originalLightPWM.g = readShort(settingsFile);
      originalLightPWM.b = readShort(settingsFile);
      settingsFile.close();

      if (timerDurationMs > 0) {
        timerStartedMs = millis();
      }

      if (mode == MODE_DIRECT) {
        lightPWM(originalLightPWM);
      }
    }
  }
}

String generatePage() {
  String s = "# ";
  s += SIGNAL_STRING;
  s += "\n";
  
  s += "name=";
  s += ssid;
  s += "\n";
  
  s += "uptime=";
  s += millis();
  s += "\n";
  
  s += "state=";
  s += state;
  s += "\n";
  
  s += "mode=";
  s += mode;
  s += "\n";

  s += "direct.color=";
  s += originalLightPWM.r;
  s += ",";
  s += originalLightPWM.g;
  s += ",";
  s += originalLightPWM.b;
  s += "\n";
  
  s += "timer.state=";
  s += (timerDurationMs > 0);
  s += "\n";
  
  s += "timer.durationMinutes=";
  s += (timerDurationMs / 60000);
  s += "\n";
  
  unsigned long remainingMinutes = 0;
  if (state == STATE_ON && timerStartedMs > 0) {
    remainingMinutes = (timerStartedMs + timerDurationMs - now) / (60000);
  }
  s += "timer.remainingMinutes=";
  s += remainingMinutes;
  s += "\n";

  s += "firmware=";
  s += FIRMWARE_VERSION;
  s += "\n";

  s += "sequence=";
  s += workingSequenceToString();
  s += "\n";

  s += "amblightsensor.enabled=";
  s += ambientLightSensorEnabled;
  s += "\n";

  s += "amblightsensor.value=";
  s += sensorReadingMv;
  s += "\n";

  s += "amblightsensor.state=";
  s += sensorReadingState;
  s += "\n";

  s += "brownout.flag=";
  s += batteryBecameLow;
  s += "\n";

  s += "power.reduction=";
  s += powerReduction;
  s += "\n";

  return s;
}

void setLightPWM(String s) {
  // String is hex RRGGBB, each byte << 2
  if (s.length() >= 6) {
    int i = strtol(s.c_str(), NULL, 16);
    int r = (i & 0xff0000) >> 14;
    int g = (i & 0x00ff00) >> 6;
    int b = (i & 0x0000ff) << 2;
    Serial.printf("RGB: %u %u %u\n", r, g, b);
    originalLightPWM.r = r;
    originalLightPWM.g = g;
    originalLightPWM.b = b;
  }
}

void setSequenceFromString(String payload) {
  unsigned int seq_colour_tmp[MAX_SEQ_LEN];
  unsigned int seq_duration_tmp[MAX_SEQ_LEN];
  seq_duration_tmp[0] = 0;
  unsigned int tmpSeqStep = 0;
  const unsigned int seqStepLen = 3;
  for (int i = 0; i < payload.length(); i += seqStepLen) {
    int colour = charToDigit(payload[i]);
    // Duration is two symbols dd, in 0.1 s
    int digitSeconds = charToDigit(payload[i + 1]);
    int digit110Seconds = charToDigit(payload[i + 2]);
    int durationMs = (digitSeconds * 1000) + (digit110Seconds * 100);
    seq_colour_tmp[tmpSeqStep] = colour;
    seq_duration_tmp[tmpSeqStep] = durationMs;
    tmpSeqStep++;
    if (tmpSeqStep >= MAX_SEQ_LEN)
      break;
  }
  Serial.printf("Sequence set, steps: %u\n", tmpSeqStep);
  if (tmpSeqStep < MAX_SEQ_LEN - 1)
    seq_duration_tmp[tmpSeqStep++] = 0;
  // seq_duration_tmp has 0 at the end

  if (seq_duration_tmp[0] > 0) {
    // Copy to working array
    for (int i = 0; i < tmpSeqStep; i++) {
      seq_colour[i] = seq_colour_tmp[i];
      seq_duration[i] = seq_duration_tmp[i];
    }
  }

  // Restart the sequence from 0
  seqStep = 0;
  lastTick = 0;
  lastDuration = 0;
}

void persistWorkingSequence() {
  if (persistenceEnabled) {
    if (seq_duration[0] > 0) {
      if (digitalRead(PIN_BROWNOUT) == HIGH) {
        File seqFile = SPIFFS.open(SEQ_FILE_NAME, "w");
        if (seqFile) {
          Serial.printf("Writing file %s\n", SEQ_FILE_NAME);
          String seq = workingSequenceToString();
          Serial.printf("Writing %s\n", seq.c_str());
          seqFile.print(seq);
          seqFile.close();
        } else {
          // Cannot write
          Serial.printf("Cannot write file %s\n", SEQ_FILE_NAME);
          flash(COLOR_RED, 1000);
        }
      } else {
        Serial.printf("Cannot persist in brownout\n");
      }
    }
  }
}

String workingSequenceToString() {
  String s = "";
  int bytesWritten = 0;
  for (int i = 0; i < MAX_SEQ_LEN; i++) {
    if (seq_duration[i] == 0)
      break;
    int durationTenthSecond = seq_duration[i] / 100;
    int iColor = seq_colour[i];
    int iDuration1 = durationTenthSecond % 10;
    int iDuration10 = (durationTenthSecond - iDuration1) / 10;
    s += digitToChar(iColor);
    s +=  digitToChar(iDuration10);
    s +=  digitToChar(iDuration1);
  }
  return s;
}

String seqToString() {
  String s = "";
  for (int i = 0; i < MAX_SEQ_LEN; i++) {
    if (seq_duration[i] == 0)
      break;
    s += seq_colour[i];
    if (seq_duration[i] < 10)
      s += "0";
    s += seq_duration[i];
  }
  return s;
}

char digitToChar(int i) {
  return i + '0';
}

int charToDigit(char c) {
  return c - '0'; // This is a naughty C style hack
}

