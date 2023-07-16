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