// Example Arduino sketch for ESP32 to receive UART data from the FPGA
// and print the received value in decimal format.

#define RX_PIN 16
#define TX_PIN 17

void setup() {
  // Initialize the Serial monitor (USB) for debugging at 115200 baud.
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready.
  }
  Serial.println("ESP32 UART Receiver Started");

  // Initialize Serial1 at 9600 baud.
  // We're only using RX here since the FPGA only sends data.
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  // Check if data is available on Serial1.
  while (Serial1.available() > 0) {
    // Read a byte from Serial1.
    uint8_t byteReceived = Serial1.read();
    // Print the received byte in decimal format.
    Serial.print("Received: ");
    Serial.println(byteReceived, DEC);  // Print as decimal.
  }
  
  // Optional: add a short delay to avoid flooding the serial monitor.
  delay(10);
}
