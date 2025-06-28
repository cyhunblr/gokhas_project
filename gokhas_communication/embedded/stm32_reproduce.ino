// STM32 Communication Reproduction with Modified Calibration Timing
// Structure 1: ControlMessage (2 bytes) - comStatus, calibStatus  
// Structure 2: JointMessage (6 bytes) - control_bits, j1p, j1s, j2p, j2s, ap

// Basic definitions for STM32 compatibility
typedef unsigned char byte;
typedef unsigned long uint32_t;

// Mock Arduino functions for compilation
unsigned long millis() { 
    // In real STM32 implementation, this should return system tick count
    static unsigned long counter = 0;
    return counter += 10; // Mock increment
}

void delayMicroseconds(int us) {
    // Mock delay function
    volatile int i;
    for (i = 0; i < us * 10; i++);
}

void delay(int ms) {
    // Mock delay function
    delayMicroseconds(ms * 1000);
}

// Mock Serial class
class MockSerial {
public:
    void begin(int baud) { /* Mock initialization */ }
    int available() { return 0; /* Mock - no data available */ }
    byte read() { return 0; /* Mock read */ }
    void write(byte* data, int len) { /* Mock write */ }
    void write(byte data) { /* Mock write single byte */ }
};

MockSerial Serial;

// Motor calibration status check function
// In real implementation, this should communicate with motor controller
// Global variables needed for calibration timing
static unsigned long calibration_start_time = 0;

bool check_motor_calibration_status() {
  unsigned long elapsed = millis() - calibration_start_time;
  
  // Fixed 2-second calibration delay as requested
  // Interface remains dynamic to handle variable times in future
  return (elapsed >= 2000);
}

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
  static unsigned long last_response_time = 0;
  static bool calibration_in_progress = false;
  static unsigned long calibration_start_time = 0;
  static bool calibration_completed_sent = false;
  
  // Handle ongoing calibration process - Send completion when motor actually finishes
  if (calibration_in_progress && !calibration_completed_sent) {
    // Check if motor calibration is actually complete
    // This should be replaced with real motor controller communication
    bool motor_calibration_complete = check_motor_calibration_status();
    
    if (motor_calibration_complete) {
      // Send completion when motor actually finishes (calibStatus = 0)
      byte completion_response[2];
      completion_response[0] = 1;  // comStatus = 1 (active)
      completion_response[1] = 0;  // calibStatus = 0 (COMPLETION)
      Serial.write(completion_response, CONTROL_PACKET_SIZE);
      
      calibration_completed_sent = true;
      calibration_in_progress = false;
      last_response_time = millis();
    }
  }
  
  // Check for incoming data
  if (Serial.available() > 0) {
    // Read all available bytes quickly
    while (Serial.available() > 0 && index < 8) {
      buffer[index] = Serial.read();
      last_byte_time = millis();
      index++;
      
      // Small delay to allow more bytes to arrive
      delayMicroseconds(500);
    }
    
    // Process complete packets
    if (index == CONTROL_PACKET_SIZE) {
      // Check if this might be start of Structure 2 by waiting briefly
      delay(20);
      
      if (Serial.available() == 0) {
        // Structure 1 (ControlMessage) - 2 bytes
        byte comStatus = buffer[0];
        byte calibStatus = buffer[1];
        
        // Prevent too frequent responses (minimum 50ms between responses)
        if (millis() - last_response_time >= 50) {
          if (calibStatus == 1 && !calibration_in_progress) {
            // Start calibration process - Send immediate ACK first
            Serial.write(buffer, CONTROL_PACKET_SIZE);  // Echo back calibStatus=1 as ACK
            last_response_time = millis();
            
            // Then start calibration process
            calibration_in_progress = true;
            calibration_start_time = millis();
            calibration_completed_sent = false;
            
          } else if (calibStatus == 0 && !calibration_in_progress) {
            // Normal non-calibration command - echo back
            Serial.write(buffer, CONTROL_PACKET_SIZE);
            last_response_time = millis();
          }
          // IGNORE all other commands during calibration (prevents duplicate ACKs)
        }
        
        // Reset buffer
        index = 0;
      }
    } else if (index >= JOINT_PACKET_SIZE) {
      // Structure 2 (JointMessage) - 6 bytes
      byte control_bits = buffer[0];
      byte j1p = buffer[1];
      byte j1s = buffer[2];
      byte j2p = buffer[3];
      byte j2s = buffer[4];
      byte ap = buffer[5];
      
      // Prevent too frequent responses (minimum 50ms between responses)
      if (millis() - last_response_time >= 50) {
        // Send back the same Structure 2
        Serial.write(buffer, JOINT_PACKET_SIZE);
        last_response_time = millis();
      }
      
      // Reset buffer
      index = 0;
    }
  }
  
  // Reset buffer if no data received for too long (timeout)
  if (index > 0 && (millis() - last_byte_time) > 200) {
    index = 0;
  }
}
