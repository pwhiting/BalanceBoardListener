#include <Arduino.h>
#include <Wiimote.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(2); // Use UART2

// UART Configuration for ESP32 (connects to ESP32-C3 D3/D6)
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD   9600

//#define BALANCE_BOARD_REPORT_MODE  0x34
//#define DATA_HEADER_INDEX          1
#define BUTTON_BYTE_INDEX          3
#define EXT_DATA_INDEX             4
#define UART_SEND_INTERVAL_MS      500

Wiimote wiimote;
void wiimote_callback(wiimote_event_type_t event_type, uint16_t handle, uint8_t *data, size_t len);

uint8_t static_mac[6];

void generate_unique_mac() {
  // Get the ESP32's unique chip ID
  uint64_t chipid = ESP.getEfuseMac();
  
  // Create a unique MAC address based on chip ID
  // Using a custom OUI (first 3 bytes) - make sure it's locally administered
  static_mac[0] = 0x02;  // Locally administered bit set (bit 1 of first byte)
  static_mac[1] = 0x00;
  static_mac[2] = 0x00;
  
  // Use parts of the chip ID for the last 3 bytes
  static_mac[3] = (uint8_t)(chipid >> 16);
  static_mac[4] = (uint8_t)(chipid >> 8);
  static_mac[5] = (uint8_t)(chipid);
  
  Serial.print("Generated unique MAC address: ");
  for (int i = 0; i < 6; i++) {
    if (static_mac[i] < 16) Serial.print("0");
    Serial.print(static_mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}




void setup() {
  Serial.begin(115200);
  Serial.println("Wiimote to UART Bridge Starting...");

  
  generate_unique_mac(); 
  // Set the new MAC address
  esp_base_mac_addr_set(static_mac);


//  esp_bt_controller_disable();
//  esp_bt_controller_deinit();
  delay(1000);

  MySerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("Wiimote to UART Bridge Started...");
  
  // let everything settle
  delay(4000);

  wiimote.init(wiimote_callback);
  Serial.println("Wiimote initialized. Waiting for connections...");
}

void loop() {
  wiimote.handle();
}

void handle_balance_data(uint8_t *data, size_t len, unsigned long &last_uart_send) {
  if (len < EXT_DATA_INDEX + 1) {
    Serial.println("Data too short");
    return; 
  }

  float weight[4];
  wiimote.get_balance_weight(data, weight);
  unsigned long current_time = millis();

  if (current_time - last_uart_send >= UART_SEND_INTERVAL_MS) {
    char msg[64];
    snprintf(msg, sizeof(msg), "%.2f %.2f %.2f %.2f\n",
             weight[BALANCE_POSITION_TOP_RIGHT],
             weight[BALANCE_POSITION_BOTTOM_RIGHT],
             weight[BALANCE_POSITION_TOP_LEFT],
             weight[BALANCE_POSITION_BOTTOM_LEFT]);
    MySerial.print(msg);
    Serial.printf("sent: %s", msg);
    last_uart_send = current_time;
  }

  bool wiimote_button_A = (data[BUTTON_BYTE_INDEX] & 0x08) != 0;
  if (wiimote_button_A) {
    Serial.println("Balance Board A button pressed: restarting...");
    ESP.restart();
  }
}

void wiimote_callback(wiimote_event_type_t event_type, uint16_t handle, uint8_t *data, size_t len) {
  static int connection_count = 0;
  static unsigned long last_uart_send = 0;

  switch (event_type) {
case WIIMOTE_EVENT_CONNECT:
  Serial.println("event_type=WIIMOTE_EVENT_CONNECT");
  
  // Extended initialization sequence
  delay(500);  // Let connection settle
  
  // Cycle through LEDs to ensure communication is working
  for (int i = 1; i <= 4; i++) {
    wiimote.set_led(handle, 1 << (i-1));
    delay(200);
  }
  
  // Brief rumble to "wake up" the device
  wiimote.set_rumble(handle, true);
  delay(100);
  wiimote.set_rumble(handle, false);
  delay(200);
  
  // Set final LED state
  wiimote.set_led(handle, 1 << connection_count);
  
  connection_count++;
  break;

//    case WIIMOTE_EVENT_CONNECT:
//      Serial.println("event_type=WIIMOTE_EVENT_CONNECT");
 //     wiimote.set_led(handle, 1 << connection_count);
//      connection_count++;
        // Force LED change to trigger communication that might help initialization
//      delay(100);
//      wiimote.set_led(handle, 0);
//      delay(100);  
//      wiimote.set_led(handle, 1 << connection_count);
  
//      break;

    case WIIMOTE_EVENT_DISCONNECT:
      Serial.println("event_type=WIIMOTE_EVENT_DISCONNECT");
      connection_count--;
      wiimote.scan(true);
      break;

    case WIIMOTE_EVENT_INITIALIZE:
      Serial.println("event_type=WIIMOTE_EVENT_INITIALIZE");
      wiimote.scan(true);
      break;

    case WIIMOTE_EVENT_SCAN_START:
      Serial.println("event_type=WIIMOTE_EVENT_SCAN_START");
      break;

    case WIIMOTE_EVENT_SCAN_STOP:
      Serial.println("event_type=WIIMOTE_EVENT_SCAN_STOP");
      if (connection_count == 0) {
        wiimote.scan(true);
      }
      break;

    case WIIMOTE_EVENT_DATA:
      handle_balance_data(data, len, last_uart_send);
      break;

    default:
      Serial.printf("event_type=%d\n", event_type);
      break;
  }
}

