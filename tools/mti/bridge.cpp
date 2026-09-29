#include <Arduino.h>

// Maintenance-only firmware. No VCU, CAN, motors or boot text.
// ESP32 RX19 <- RS232 converter TTL output; TX21 -> converter TTL input.
static void forward(HardwareSerial &from, HardwareSerial &to) {
    uint8_t buffer[128];
    const int available = from.available();
    if (available <= 0) return;
    const size_t count = min(static_cast<size_t>(available), sizeof(buffer));
    const size_t received = from.readBytes(buffer, count);
    if (received) to.write(buffer, received);
}

void setup() {
    Serial.setRxBufferSize(2048);
    Serial2.setRxBufferSize(2048);
    Serial.begin(115200, SERIAL_8N1);
    Serial2.begin(115200, SERIAL_8N1, 19, 21);
}

void loop() {
    forward(Serial, Serial2);
    forward(Serial2, Serial);
    delay(0);
}
