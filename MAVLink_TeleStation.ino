#include <PixhawkArduinoMAVLink.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

HardwareSerial hc12Serial(0);  // Create a new HardwareSerial class.

////////////////////////////////////////////

// SH1106 OLED Display
#define I2C_ADDRESS 0x3C  // Initialize with the I2C address of your OLED display

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin for OLED (not used in this code)
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

////////////////////////////////////////////

// Variables to store MAVLink data
float voltage = 0.0;
float current = 0.0;
float heading = 0.0;
uint8_t mode = 0;
bool armed = false;
bool rcConnected = false;  // Flag to indicate RC controller connection status

unsigned long lastUpdateTime = 0;           // Last time data was updated
const unsigned long updateInterval = 1000;  // Interval between data updates (in milliseconds)

#define MAX_STATUS_TEXT_LEN 50
char statusMessage[MAX_STATUS_TEXT_LEN];

////////////////////////////////////////////

//Process MAVLink Messages
void processMavlinkData(const mavlink_message_t& msg) {
  switch (msg.msgid) {

    case MAVLINK_MSG_ID_SYS_STATUS:
      {
        mavlink_sys_status_t sysStatus;
        mavlink_msg_sys_status_decode(&msg, &sysStatus);

        // Update voltage and current
        voltage = sysStatus.voltage_battery / 1000.0;  // Assuming voltage_battery is in millivolts
        current = sysStatus.current_battery / 100.0;   // Assuming current_battery is in centiamperes
        // Check if RC receiver is connected
        rcConnected = (sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_RC_RECEIVER);

        break;
      }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      {
        mavlink_global_position_int_t globalPosition;
        mavlink_msg_global_position_int_decode(&msg, &globalPosition);

        // Update heading (already in degrees)
        heading = globalPosition.hdg / 100.0;  // Assuming hdg is in centidegrees

        break;
      }

    case MAVLINK_MSG_ID_HEARTBEAT:
      {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(&msg, &heartbeat);

        // Update flight mode
        mode = heartbeat.custom_mode;

        // Check if armed
        armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);

        break;
      }

    case MAVLINK_MSG_ID_STATUSTEXT:
      {
        mavlink_statustext_t statusText;
        mavlink_msg_statustext_decode(&msg, &statusText);

        // Check if the severity is critical
        if (statusText.severity == MAV_SEVERITY_WARNING) {
          // Copy the status message to the global variable
          strncpy(statusMessage, statusText.text, MAX_STATUS_TEXT_LEN - 1);
          statusMessage[MAX_STATUS_TEXT_LEN - 1] = '\0';  // Ensure null-termination
        }
        break;
      }

      // Add cases for other MAVLink messages you want to process and display
  }
}

////////////////////////////////////////////

//Request MAVLink Data from Pixhawk
void requestSysStatus() {       //Request Data from Pixhawk
  uint8_t systemId = 255;       // ID of the computer sending the command
  uint8_t componentId = 2;      // Component ID
  uint8_t targetSystem = 1;     // ID of the Pixhawk
  uint8_t targetComponent = 0;  // Target component (0 = all)
  uint8_t reqStreamId = MAV_DATA_STREAM_ALL;
  uint16_t reqMessageRate = 0x01;  // Number of times per second to request the data in hex
  uint8_t startStop = 1;           // 1 = start, 0 = stop

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(systemId, componentId, &msg, targetSystem, targetComponent, reqStreamId, reqMessageRate, startStop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message

  hc12Serial.write(buf, len);  // Write data to the HC-12 module
}

////////////////////////////////////////////

// Display ArduCopter Flight Modes
void displayFlightMode() {
  const char* flightModeString;

  switch (mode) {
    case 0:
      flightModeString = "Stabilize";
      break;
    case 2:
      flightModeString = "Altitude Hold";
      break;
    case 16:
      flightModeString = "Position Hold";
      break;
    case 7:
      flightModeString = "Follow";
      break;
    case 8:
      flightModeString = "Guided";
      break;
    case 10:
      flightModeString = "Auto";
      break;
    case 11:
      flightModeString = "Land";
      break;
  }

  display.setCursor(0, 30);
  display.print("Mode: ");
  display.println(flightModeString);
}

////////////////////////////////////////////

// Display ArduCopter Status Messages
void displayStatusMessage() {
  display.setCursor(0, 50);
  display.print("Status: ");
  display.println(statusMessage);
}

//////////////////////

//Incoming data indicator
const unsigned long BLINK_INTERVAL = 500;  // Blink interval in milliseconds
unsigned long lastBlinkTime = 0;           // Last time the symbol blinked
bool blinkState = false;

///////////////////



///////////////////////////////////////////////
// Bitmap for "RC"
const unsigned char rcBitmap[] PROGMEM = {
 0x00, 0x00, 0xf1, 0x80, 0x8a, 0x40, 0x8a, 0x00, 0x8a, 0x00, 0xf2, 0x00, 0x92, 0x00, 0x8a, 0x40, 
	0x89, 0x80, 0x00, 0x00
};


// Bitmap for "RC crossed out"
const unsigned char rcCrossedOutBitmap[] PROGMEM = {
  0x00, 0x00, 0x78, 0xc0, 0x45, 0x20, 0x45, 0x00, 0x45, 0x00, 0xff, 0xf0, 0x49, 0x00, 0x45, 0x20, 
	0x44, 0xc0, 0x00, 0x00
};

///////////////////////////////////////////////



void setup() {
  Serial.begin(57600);                          // Initialize the USB serial communication
  hc12Serial.begin(57600, SERIAL_8N1, -1, -1);  // Initialize the telemetry module serial communication
  delay(500);

  display.begin(I2C_ADDRESS, true);  // Initialize the display (adjust the address if necessary)
  display.clearDisplay();
  display.display();
  delay(100);

  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();
  delay(500);

  // Wait until HC12 is available
  while (!hc12Serial) {
    // Do nothing, keep waiting for HC12
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("HC12 available");
  display.display();

  // Wait until data is incoming from HC12
  while (hc12Serial.available() <= 0) {
    // Do nothing, keep waiting for incoming data
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.println("Powerup Pixhawk");
    display.display();
  }

  display.clearDisplay();
  display.println("Data incoming");
  display.display();

  // Request MAVLink SYS_STATUS message
  requestSysStatus();
  delay(1000);  // Wait for 1 second
  requestSysStatus();
  delay(500);

  // Reinitialize HC12 module or HC12 serial port here
  hc12Serial.end();
  hc12Serial.begin(57600, SERIAL_8N1, -1, -1);
  // Wait until the hc12Serial is available and data is incoming
  while (!hc12Serial || !hc12Serial.available()) {
    delay(100);
  }

  display.display();
}

void loop() {

  // Check for serial input from USB port
  while (Serial.available()) {
    uint8_t c = Serial.read();
    hc12Serial.write(c);  // Forward the data received from USB to the Pixhawk
  }


  while (hc12Serial.available()) {
    uint8_t c = hc12Serial.read();
    // Forward the data received from the Pixhawk to USB and process the MAVLink data
    Serial.write(c);

    blinkState = !blinkState;

    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      processMavlinkData(msg);
    }
  }

  unsigned long currentMillis = millis();

  // Check if it's time to update the data
  if (currentMillis - lastUpdateTime >= updateInterval) {

    // Display the data on the OLED display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Voltage: ");
    display.println(voltage);
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
    display.setCursor(0, 50);
    display.print("RC: ");
    display.println(rcConnected ? "Connected" : "Disconnected");  // Display armes status
    if (rcConnected) {
      display.drawBitmap(116, 12, rcBitmap, 10, 10, SH110X_WHITE);
    } else {
      display.drawBitmap(115, 12, rcCrossedOutBitmap, 12, 10, SH110X_WHITE);
    }
	// Update the last update time
    lastUpdateTime = currentMillis;
  }

  // Draw the blinking symbol
  if (blinkState) {
    display.fillRect(120, 0, 8, 8, SH110X_WHITE);
  }

  display.display();
}
