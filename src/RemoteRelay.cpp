#define DEBUG true

#include <secrets.h>
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_Fingerprint.h>

#define RELAY_LOCK_PIN 13
#define RELAY_LIGHTS_PIN 12
#define RELAY_LOCK_OPEN_INTERVAL 30*1000 // 30 seconds
#define LIGHTS_INTERVAL 3*60*1000 // 3 minutes
#define FINGERPRINT_RX 2
#define FINGERPRINT_TX 3

char ssid[] = SECRET_SSID;           // your network SSID (name)
char pass[] = SECRET_PASS;           // your network password
int status = WL_IDLE_STATUS;         // the WiFi connection status
WiFiServer server(80);               // create a server at port 80
bool relayLockOn = false;            // flag to track lock relay state
bool relayLightsOn = false;          // flag to track lights relay state
bool registeringFingerprint = false; // flag to track fingerprint registration state
unsigned long lightsPreviousMillis = 0;
unsigned long lockPreviousMillis = 0;

void printWifiStatus();
void processClientRequest(WiFiClient client);
void handleFingerprintEnroll();
void handleFingerprintDetection();
bool checkForSubstring(String request, String subStr);
void startFingerprintRegistration();

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial1);

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  if (DEBUG) Serial.println("Serial communication initialized.");

  finger.begin(57600);
  delay(5);

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    if (DEBUG) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
    }
    status = WiFi.begin(ssid, pass);
    delay(5000);  // wait 5 seconds for connection
  }

  // you're connected now, so print out the status
  if (DEBUG) {
    Serial.println("Connected to WiFi");
    printWifiStatus();
    Serial.println("Web server started");
  }

  // start the web server
  server.begin();

  // set pin 13 and 12 as output for relay control
  pinMode(RELAY_LOCK_PIN, OUTPUT);
  digitalWrite(RELAY_LOCK_PIN, LOW);  // ensure lock relay is initially off
  pinMode(RELAY_LIGHTS_PIN, OUTPUT);
  digitalWrite(RELAY_LIGHTS_PIN, LOW);  // ensure lights relay is initially off
}

void loop() {
  // listen for incoming clients
  WiFiClient client = server.available();
  
  if (client) {
    if (DEBUG) Serial.println("New client");
    // process client request
    processClientRequest(client);
  }

  // check if the server has been requested to turn the lock relay off
  if (relayLockOn && millis() - lockPreviousMillis >= RELAY_LOCK_OPEN_INTERVAL) {
    if (DEBUG) Serial.println("Turning lock relay OFF");
    digitalWrite(RELAY_LOCK_PIN, LOW);  // turn the lock relay off
    relayLockOn = false;  // update lock relay state
  }

  // check if the server has been requested to turn the lights relay off
  if (relayLightsOn && millis() - lightsPreviousMillis >= LIGHTS_INTERVAL) {
    if (DEBUG) Serial.println("Turning lights relay OFF");
    digitalWrite(RELAY_LIGHTS_PIN, LOW);  // turn the lights relay off
    relayLightsOn = false;  // update lights relay state
  }

  // handle fingerprint registration
  if (registeringFingerprint) {
    handleFingerprintEnroll();
  } else {
    // check for fingerprint detection only if not registering
    handleFingerprintDetection();
  }
}

void handleOn() {
  if (!relayLockOn) {
    if (DEBUG) Serial.println("Turning lock relay ON");
    digitalWrite(RELAY_LOCK_PIN, HIGH);  // turn the lock relay on
    relayLockOn = true;  // update lock relay state
    lockPreviousMillis = millis();  // reset lock timer
  }

  if (!relayLightsOn) {
    if (DEBUG) Serial.println("Turning lights relay ON");
    digitalWrite(RELAY_LIGHTS_PIN, HIGH);  // turn the lights relay on
    relayLightsOn = true;  // update lights relay state
    lightsPreviousMillis = millis();  // reset lights timer
  }
}

void handleOff() {
  if (DEBUG) Serial.println("Turning lock relay OFF");
  digitalWrite(RELAY_LOCK_PIN, LOW);  // turn the lock relay off
  relayLockOn = false;  // update lock relay state

  if (DEBUG) Serial.println("Turning lights relay OFF");
  digitalWrite(RELAY_LIGHTS_PIN, LOW);  // turn the lights relay off
  relayLightsOn = false;  // update lights relay state
}

void processClientRequest(WiFiClient client) {
  // an http request ends with a blank line
  boolean currentLineIsBlank = true;
  String request = "";
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      if (DEBUG) Serial.write(c);
      request += c;
      if (c == '\n' && currentLineIsBlank) {
        // send a standard http response header
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();
        // output the HTML content
        client.println("<html><head><title>Relay Control</title></head><body>");
        client.println("<h1>Relay Control</h1>");
        client.println("<p>Click <a href=\"/on\">here</a> to turn the relays on</p>");
        client.println("<p>Click <a href=\"/off\">here</a> to turn the relays off</p>");
        client.println("<p>Click <a href=\"/register\">here</a> to register a fingerprint</p>");
        client.println("</body></html>");
        break;
      }
      if (c == '\n') {
        // you're starting a new line
        currentLineIsBlank = true;
      } 
      else if (c != '\r') {
        // you've gotten a character on the current line
        currentLineIsBlank = false;
      }
    }
  }
  // close the connection
  client.stop();
  if (DEBUG) Serial.println("Client disconnected");

  // handle client request
  if (checkForSubstring(request, "/on")) {
    handleOn();
  }
  if (checkForSubstring(request, "/off")) {
    handleOff();
  }
  if (checkForSubstring(request, "/register")) {
    startFingerprintRegistration();
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

bool checkForSubstring(String request, String subStr) {
  if (DEBUG) {
    Serial.println("---- Start check ");
    Serial.println(request);
    Serial.println("---- end check");
  }
  return request.indexOf(subStr) != -1;
}

void startFingerprintRegistration() {
  if (DEBUG) Serial.println("Starting fingerprint registration...");
  registeringFingerprint = true;
}

void handleFingerprintEnroll() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) {
    Serial.println("No finger detected.");
    return;
  }

  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) {
    Serial.println("Failed to convert image to template.");
    return;
  }

  p = finger.createModel();
  if (p != FINGERPRINT_OK) {
    Serial.println("Failed to create fingerprint model.");
    return;
  }

  p = finger.storeModel(1);
  if (p != FINGERPRINT_OK) {
    Serial.println("Failed to store fingerprint template.");
    return;
  }

  Serial.println("Fingerprint enrolled successfully!");
}

void handleFingerprintDetection() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return;

  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    if (DEBUG) Serial.println("Fingerprint detected.");
    handleOn(); // Turn relays on after fingerprint detection
  }
}
