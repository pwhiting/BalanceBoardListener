#include <Arduino.h>
#include <Wiimote.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(2); // Use UART2
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_BAUD   9600
#define BUTTON_BYTE_INDEX          3
#define EXT_DATA_INDEX             4
#define UART_SEND_INTERVAL_MS      100
#define ACTIVITY_THRESHOLD         15.0
#define TIMEOUT_MINUTES            10
#define TIMEOUT_MS                 (TIMEOUT_MINUTES * 60 * 1000UL)

Wiimote wiimote;
unsigned long last_activity_time = 0;

void wiimote_callback(wiimote_event_type_t event_type, uint16_t handle, uint8_t *data, size_t len);

void setup() {
  Serial.begin(115200);
  Serial.println("Wiimote to UART Bridge Starting...");
  
  MySerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("Wiimote to UART Bridge Started...");
  
  // let everything settle
  delay(4000);
  wiimote.init(wiimote_callback);
  Serial.println("Wiimote initialized. Waiting for connections...");
  
  // Initialize activity timer
  last_activity_time = millis();
}

void loop() {
  wiimote.handle();
  
  // Check for timeout (10 minutes of inactivity)
  unsigned long current_time = millis();
  if (current_time - last_activity_time >= TIMEOUT_MS) {
    Serial.println("10 minutes of inactivity detected. Restarting...");
    ESP.restart();
  }
}

void handle_balance_data(uint8_t *data, size_t len, unsigned long &last_uart_send) {
  if (len < EXT_DATA_INDEX + 1) {
    Serial.println("Data too short");
    return; 
  }
  
  float weight[4];
  wiimote.get_balance_weight(data, weight);
  
  // Check for activity (any weight value over threshold)
  bool activity_detected = false;
  if (weight[BALANCE_POSITION_TOP_RIGHT] > ACTIVITY_THRESHOLD ||
      weight[BALANCE_POSITION_BOTTOM_RIGHT] > ACTIVITY_THRESHOLD ||
      weight[BALANCE_POSITION_TOP_LEFT] > ACTIVITY_THRESHOLD ||
      weight[BALANCE_POSITION_BOTTOM_LEFT] > ACTIVITY_THRESHOLD) {
    activity_detected = true;
    last_activity_time = millis(); // Reset the timeout timer
  }
  
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
      wiimote.set_led(handle, 1 << connection_count);
      connection_count++;
      // Reset activity timer on new connection
      last_activity_time = millis();
      break;
      
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