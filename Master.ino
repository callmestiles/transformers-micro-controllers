#include <esp_now.h>
#include <WiFi.h>

// IMPORTANT: Replace this with the MAC address of your RECEIVER Deneyap Kart.
// You can find the MAC address by uploading a simple sketch to the receiver
// that prints WiFi.macAddress() to the Serial Monitor.
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// This struct MUST EXACTLY match the one in your QT application's ControlData.h
// The #pragma pack directives are crucial to ensure the memory layout is identical
// and no padding bytes are added by the compiler.
#pragma pack(push, 1)
struct ControlData {
    // Car Control
    int16_t left_motor_speed;
    int16_t right_motor_speed;

    // Arm Control
    float arm_base_angle;
    float arm_shoulder_angle;
    float arm_elbow_angle;

    // Gripper and Dumper
    uint8_t gripper_state;
    uint8_t dumper_state;

    // Flags
    uint8_t reset_arm_flag;
    uint8_t auto_release_flag;
};
#pragma pack(pop)

// Create a variable to hold the incoming data from the Raspberry Pi
ControlData incomingData;

// A peer info structure to hold the receiver's information.
esp_now_peer_info_t peerInfo;

// Callback function that is executed when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // This is useful for debugging. You can check the Serial Monitor
  // to see if packets are being sent successfully.
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the callback function to get send status
  esp_now_register_send_cb(OnDataSent);

  // Register the receiver peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW Sender Initialized. Waiting for data from Raspberry Pi...");
}

void loop() {
  // Check if the number of bytes available in the serial buffer is
  // enough to form a complete ControlData struct.
  if (Serial.available() >= sizeof(incomingData)) {
    // Read the raw bytes from the serial port directly into our struct variable.
    // This is efficient and avoids any parsing.
    Serial.readBytes((char*)&incomingData, sizeof(incomingData));

    // Send the received data packet via ESP-NOW to the other Deneyap Kart.
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &incomingData, sizeof(incomingData));

    if (result == ESP_OK) {
      // You can uncomment this for verbose debugging, but it might add slight delay.
      // Serial.println("Sent data packet via ESP-NOW");
    } else {
      Serial.println("Error sending the data");
    }
  }
}
