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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////


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