#include <PixhawkArduinoMAVLink.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <HardwareSerial.h>

HardwareSerial hc12Serial(0);  // Create a new HardwareSerial class for HC12 modul.

////////////////////////////////////////////

// SH1106 OLED Display
#define I2C_ADDRESS 0x3C  // Initialize with the I2C address of your OLED display

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin for OLED (not used in this code)
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

////////////////////////////////////////////

// Variables to store MAVLink data
float voltage = 0.0;       // Main battery voltage
float current = 0.0;       // Current
float heading = 0.0;       // Quad Heading
uint8_t mode = 0;          // Flight Mode
bool armed = false;        // Flag to indicate Arm status
bool rcConnected = false;  // Flag to indicate RC controller connection status

unsigned long lastUpdateTime = 0;          // Last time data was updated
const unsigned long updateInterval = 500;  // Interval between data updates (in milliseconds)

#define MAX_STATUS_TEXT_LEN 50
char statusMessage[MAX_STATUS_TEXT_LEN];

////////////////////////////////////////////

// Display ArduCopter Status Messages
void displayStatusMessage() {
  display.setCursor(0, 50);
  display.print("Status: ");
  display.println(statusMessage);
}

//////////////////////

//Active connection to Pixhawk
const unsigned long BLINK_INTERVAL = 500;  // Blink interval in milliseconds
unsigned long lastBlinkTime = 0;           // Last time the symbol blinked
bool blinkState = false;                   // Flag to indicate blinkState status

bool isConnectionActive = false;  // Flag to track connection status

//Check if there is an active connection to Pixhawk
void checkConnectionStatus() {
  static unsigned long lastDataTime = 0;
  const unsigned long connectionTimeout = 3000;  // Adjust the timeout value as needed

  // Check if data has been received within the timeout period
  if (hc12Serial.available() > 0) {
    lastDataTime = millis();    // Reset the timeout
    isConnectionActive = true;  // Connection is active
  }

  // Check if the connection timeout has elapsed
  if (millis() - lastDataTime > connectionTimeout) {
    isConnectionActive = false;  // Connection is inactive
  }
}

///////////////////////////////////////////////

void displayInitializingDots(byte x, byte y) {
  static unsigned long previousMillis = 0;
  const unsigned long interval = 400;  // Interval between dot changes in milliseconds

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    static byte numDots = 0;
    const byte maxDots = 4;  // Maximum number of dots

    display.setCursor(x, y);  // Set the cursor position based on the parameters
    for (byte i = 0; i < maxDots; i++) {
      if (i < numDots) {
        display.print(".");
      } else {
        display.print(" ");
      }
    }

    numDots = (numDots + 1) % (maxDots + 1);
    display.display();  // Update the display
  }
}

///////////////////////////////////////////////

float voltageThreshold = 7.2;  // Define the voltage threshold (Battery Warning Voltage)
const unsigned long flashInterval = 100;
unsigned long lastFlashTime = 0;  // Last time the symbol blinked
bool showWarning = false;         // Flag to indicate Bat Warning status

///////////////////////////////////////////////

void setup() {
  Serial.begin(57600);                          // Initialize the USB serial communication
  hc12Serial.begin(57600, SERIAL_8N1, -1, -1);  // Initialize the telemetry module serial communication
  delay(100);

  display.begin(I2C_ADDRESS, true);  // Initialize the display (adjust the address if necessary)
  display.clearDisplay();
  display.display();
  delay(100);

  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  display.println("Initializing");

  unsigned long startTime1 = millis();

  while (millis() - startTime1 < 3000) {  // Run for 3 seconds or adjust the duration as needed
    displayInitializingDots(75, 0);       // Adjust the cursor position as needed
  }

  delay(100);

  // Wait until HC12 is available
  while (!hc12Serial) {
    // Do nothing, keep waiting for HC12
  }

  display.setCursor(0, 10);
  display.println("HC12 available");
  display.display();
  delay(500);

  // Wait until data is incoming from HC12
  while (hc12Serial.available() <= 0) {
    // Do nothing, keep waiting for incoming data
    display.setCursor(0, 20);
    display.print("Powerup Pixhawk");
    display.display();
  }
  delay(500);

  display.setCursor(0, 20);
  display.setTextColor(0xFFFF, 0);
  display.print("               ");

  display.display();
  delay(100);

  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 20);
  display.print("Recv Heartbeat");

  unsigned long startTime2 = millis();

  while (millis() - startTime2 < 3000) {  // Run for 3 seconds or adjust the duration as needed
    displayInitializingDots(90, 20);
  }

  display.display();
  delay(1000);

  requestSysStatus();  // Request MAVLink SYS_STATUS message
  delay(1000);         // Wait for 1 second
}


void loop() {

  // Check for serial input from USB port
  while (Serial.available()) {
    hc12Serial.write(Serial.read());  // Forward the data received from USB to the Pixhawk
  }

  while (hc12Serial.available()) {
    uint8_t c = hc12Serial.read();

    Serial.write(c);  // Forward the data received from the Pixhawk to USB and process the MAVLink data

    // Variables for MAVLink Msg
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      processMavlinkData(msg);
    }
    blinkState = !blinkState;
  }

  checkConnectionStatus();  // Check the connection status to Pixhawk

  unsigned long currentMillis = millis();

  if (isConnectionActive) {                                  // Check if it if there is an active link to the Pixhawk
    if (currentMillis - lastUpdateTime >= updateInterval) {  // Check if it's time to update the data on OLED

      // Display the data on the OLED display
      display.clearDisplay();
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 0);
      display.print("Voltage: ");
      display.print(voltage);
      display.setCursor(0, 10);
      display.print("Current: ");
      display.println(current);
      display.setCursor(0, 20);
      display.print("Heading: ");
      display.println(heading);
      displayFlightMode();  // Display flight mode
      display.setCursor(0, 40);
      display.print("Status: ");
      display.println(armed ? "ARMED" : "DISARMED");  // Display arm status

      //displayStatusMessage();                         // Display status message
      //display.setCursor(0, 50);
      //display.print("RC: ");
      //display.println(rcConnected ? "Connected" : "Disconnected");  // Display armes status

      displayRCLink();  //Display if RC controller is connected to PX or not


      if (voltage < voltageThreshold) {  // Check if voltage is below the threshold and display warning

        if ((currentMillis - lastFlashTime) >= flashInterval) {
          showWarning = !showWarning;     // Toggle the flag
          lastFlashTime = currentMillis;  // Update the last flash time
        }

        if (showWarning) {
          display.setCursor(85, 0);
          display.println("BAT LOW");
        } else {
          display.setCursor(85, 0);
          display.setTextColor(0xFFFF, 0);
          display.println("       ");
        }
      }
      display.display();
      lastUpdateTime = currentMillis;  // Update the last update time
      
    }

    // Draw indicator if data is being received
    if (blinkState) {
      
      display.fillRect(117, 25, 8, 8, SH110X_WHITE);
      display.display();
    }
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextColor(SH110X_WHITE);
    display.println("No Link to PX");
    display.println("Wait a few Seconds");
    display.display();
  }
}
