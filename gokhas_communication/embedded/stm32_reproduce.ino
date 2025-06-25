// STM32 Communication Reproduction with Separate Packet Structures
// Structure 1: ControlMessage (2 bytes) - comStatus, calibStatus  
// Structure 2: JointMessage (6 bytes) - control_bits, j1p, j1s, j2p, j2s, ap

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Packet size definitions
  const byte CONTROL_PACKET_SIZE = 2;  // Structure 1
  const byte JOINT_PACKET_SIZE = 6;    // Structure 2
  
  static byte buffer[8]; // Use sufficient buffer size
  static byte index = 0;
  static unsigned long last_byte_time = 0;
  
  // Check for incoming data
  if (Serial.available() > 0) {
    // Read byte and store timestamp
    buffer[index] = Serial.read();
    last_byte_time = millis();
    index++;
    
    // Check if we have enough data for either packet type
    if (index >= CONTROL_PACKET_SIZE) {
      // Wait a bit to see if more data arrives
      delay(50);
      
      if (index == CONTROL_PACKET_SIZE && Serial.available() == 0) {
        // Structure 1 (ControlMessage) - 2 bytes
        byte comStatus = buffer[0];
        byte calibStatus = buffer[1];
        
        // Send back the same Structure 1
        Serial.write(buffer, CONTROL_PACKET_SIZE);
        
        // If calibStatus is 1, sleep for 2 seconds
        if (calibStatus == 1) {
          delay(2000);
        }
        
        // Reset buffer
        index = 0;
        
      } else if (index >= JOINT_PACKET_SIZE) {
        // Structure 2 (JointMessage) - 6 bytes
        byte control_bits = buffer[0];
        byte j1p = buffer[1];
        byte j1s = buffer[2];
        byte j2p = buffer[3];
        byte j2s = buffer[4];
        byte ap = buffer[5];
        
        // Send back the same Structure 2
        Serial.write(buffer, JOINT_PACKET_SIZE);
        
        // Reset buffer
        index = 0;
      }
    }
  }
  
  // Reset buffer if no data received for too long (timeout)
  if (index > 0 && (millis() - last_byte_time) > 100) {
    index = 0;
  }
}
