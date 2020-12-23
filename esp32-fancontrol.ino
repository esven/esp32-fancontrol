#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

typedef uint32_t PortType; // Formerly 'RwReg' but interfered w/CMCIS header

const char* ssid = "YOUR_SSID";
const char* password =  "YOUR_PWD";

// the number of the LED pin
const int ledPin = 32;  // 16 corresponds to GPIO16
const int reedPin = 33;

// setting PWM properties
const int freq = 600;
const int ledChannel = 0;
const int resolution = 10;
int dutyCycle = 0;
byte pwmInterval = 25;
byte fanState = 0;
byte change_available = 0;

unsigned long curRPM;
unsigned long previousRPMMillis;
unsigned long previousMillis;
volatile unsigned long pulses = 0;
unsigned long lastPulses = 0;
float lastElapsedSec = 0.0;
unsigned long lastRPMmillis = 0;
unsigned long interval = 1000;
volatile byte pinstate = 0;

PortType pinmask;
volatile PortType *outsetreg; ///< RGB PORT bit set register
volatile PortType *outclrreg; ///< RGB PORT bit clear register
volatile PortType *inreg;
PortType inpinmask;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

AsyncWebServer server(80);
const char* PARAM_MESSAGE = "message";
const String PAGE_HEADER = F("<!DOCTYPE html>\r\n<html>\r\n" \
    "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\r\n" \
    "<link rel=\"icon\" href=\"data:,\">\r\n" \
    "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\r\n" \
    ".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;\r\n" \
    "text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}\r\n" \
    ".button2 {background-color: #555555;}</style></head>\r\n" \
    "<body><h1>ESP32 Web Server</h1>\r\n" \
    "<p>Fan - RPM 0</p>\r\n");
const String PAGE_FOOTER = F("<p><a href=\"/fan/up\"><button class=\"button\">UP</button></a></p>\r\n" \
    "<p><a href=\"/fan/down\"><button class=\"button button2\">DOWN</button></a></p>\r\n" \
    "</body></html>\r\n");

void IRAM_ATTR countPulse() {
  // just count each pulse (only if hall is low
  //if (!(*inreg & inpinmask)) {
    portENTER_CRITICAL_ISR(&mux);
    if (pinstate == 0) {
      *outsetreg = pinmask;
      pinstate = 1;
    } else {
      *outclrreg = pinmask;
      pinstate = 0;
    }
      pulses++;
      portEXIT_CRITICAL_ISR(&mux);
 // }
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setup(){

  pinMode(reedPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(reedPin), countPulse, FALLING); 
  pinMode(27,OUTPUT);
  digitalWrite(27, LOW);
  outsetreg = &GPIO.out_w1ts;
  outclrreg = &GPIO.out_w1tc;
  if (reedPin < 32) {
    inreg = &GPIO.in;
  } else {
    inreg = &GPIO.in1.val;
  }
  
  pinmask = digitalPinToBitMask(27);
  inpinmask = digitalPinToBitMask(reedPin);
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", buildPage());
  });

  server.on("/fan/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    fanState = 1;
    if (dutyCycle < 100) {
      dutyCycle = 100;
    }
    change_available = 1;
    request->send(200, "text/html", buildPage());
  });

  server.on("/fan/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    fanState = 0;
    change_available = 1;
    request->send(200, "text/html", buildPage());
  });

  server.on("/fan/down", HTTP_GET, [](AsyncWebServerRequest *request) {
     dutyCycle = dutyCycle - pwmInterval;
     if (dutyCycle < 0) {
      dutyCycle = 0;
     }
    change_available = 1;
    request->send(200, "text/html", buildPage());
  });

  server.on("/fan/up", HTTP_GET, [](AsyncWebServerRequest *request) {
     dutyCycle = dutyCycle + pwmInterval;
     if (dutyCycle > 1023) {
      dutyCycle = 1023;
     }
    change_available = 1;
    request->send(200, "text/html", buildPage());
  });

  server.on("/fan/1", HTTP_GET, [](AsyncWebServerRequest *request) {
     dutyCycle = 1;
    change_available = 1;
    request->send(200, "text/html", buildPage());
  });

  server.on("/fan/255", HTTP_GET, [](AsyncWebServerRequest *request) {
     dutyCycle = 1023;
    change_available = 1;
    request->send(200, "text/html", buildPage());
  });

  server.onNotFound(notFound);

  server.begin();
}

String buildPage() {
  String responsePage = "<!DOCTYPE html>\r\n<html>\r\n";
  responsePage.concat("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\r\n");
  responsePage.concat("<meta http-equiv=\"refresh\" content=\"3; URL=http://");
  responsePage.concat(WiFi.localIP().toString());
  responsePage.concat("/\">\r\n");
  responsePage.concat("<link rel=\"icon\" href=\"data:,\">\r\n");
  // CSS to style the on/off buttons 
  // Feel free to change the background-color and font-size attributes to fit your preferences
  responsePage.concat("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\r\n");
  responsePage.concat(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;\r\n");
  responsePage.concat("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}\r\n");
  responsePage.concat(".button2 {background-color: #555555;}</style></head>\r\n");
  
  // Web Page Heading
  responsePage.concat("<body><h1>ESP32 Web Server</h1>\r\n");
  
  // Display current state, and ON/OFF buttons for GPIO 26  
  responsePage.concat("<p>Fan - RPM " + String(curRPM) + "</p>\r\n");
  responsePage.concat("<p>Fan - Pulses " + String(lastPulses) + "</p>\r\n");
  responsePage.concat("<p>Fan - ElapsedSec " + String(lastElapsedSec) + "</p>\r\n");
  // If the output26State is off, it displays the ON button       
  if (fanState==0) {
    responsePage.concat("<p><a href=\"/fan/on\"><button class=\"button\">ON</button></a></p>\r\n");
  } else {
    responsePage.concat("<p><a href=\"/fan/off\"><button class=\"button button2\">OFF</button></a></p>\r\n");
  }
  responsePage.concat("<p>Fan - dutyCycle " + String(dutyCycle) + "</p>\r\n");
  responsePage.concat("<p><a href=\"/fan/up\"><button class=\"button\">UP</button></a><a href=\"/fan/255\"><button class=\"button\">255</button></a></p>\r\n");
  responsePage.concat("<p><a href=\"/fan/down\"><button class=\"button button2\">DOWN</button></a><a href=\"/fan/1\"><button class=\"button button2\">1</button></a></p>\r\n");
  responsePage.concat("</body></html>\r\n");
  return responsePage;
}

unsigned long calculateRPM(unsigned long nowMillis) {
  unsigned long RPM;
  noInterrupts();
  portENTER_CRITICAL(&mux);
  float elapsedSec = ((float)(nowMillis - lastRPMmillis))/1000.0;
  lastElapsedSec = elapsedSec;
  unsigned long revolutions = pulses/2;
  float revPerSec = (float)revolutions / elapsedSec;
  RPM = revPerSec * (float)60.0;
  lastRPMmillis = millis();
  lastPulses = pulses;
  pulses=0;
  portEXIT_CRITICAL(&mux);
  interrupts();
  return RPM;
}
 
void loop(){
  unsigned long nowMillis = millis();
  if (nowMillis - previousMillis > interval) {
    curRPM = calculateRPM(nowMillis);
    previousMillis = nowMillis;
  }
  if (change_available != 0) {
    if (fanState == 0) {
      // Switch Fan off
      ledcWrite(ledChannel, 0);
    } else {
      // changing the FAN RPM with PWM
      ledcWrite(ledChannel, dutyCycle);
    }
    change_available = 0;
  }
}
