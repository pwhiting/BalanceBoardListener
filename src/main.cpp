#include <Arduino.h>
#include <Wiimote.h>
#include <HardwareSerial.h>
HardwareSerial MySerial(2); // Use UART2

// UART Configuration for ESP32 (connects to ESP32-C3 D3/D6)
#define UART_TX_PIN 17  // GPIO17 for TX (connects to ESP32-C3 D3/GPIO3 RX)
#define UART_RX_PIN 16  // GPIO16 for RX (connects to ESP32-C3 D6/GPIO6 TX)
#define UART_BAUD 9600

Wiimote wiimote;
void wiimote_callback(wiimote_event_type_t event_type, uint16_t handle, uint8_t *data, size_t len);

void setup() {
  // Initialize debug serial (USB)
  Serial.begin(115200);
  Serial.println("Wiimote to UART Bridge Starting...");
  // Initialize UART for sending data (using Serial2 on ESP32)
  MySerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("Wiimote to UART Bridge Started...");
  
  // Initialize Wiimote
  wiimote.init(wiimote_callback);
  Serial.println("Wiimote initialized. Waiting for connections...");
}

void loop() {
  wiimote.handle();
}

void wiimote_callback(wiimote_event_type_t event_type, uint16_t handle, uint8_t *data, size_t len) {
  static int connection_count = 0;
  static unsigned long last_uart_send = 0;
  
  // Debug output to USB serial
  if(event_type == WIIMOTE_EVENT_DATA){
    if(data[1]==0x34){
      uint8_t* ext = data+4;
      float weight[4];
      wiimote.get_balance_weight(data, weight);
      
      // Debug output to USB serial
      if(0) Serial.printf(" ... Wii Balance Board: TopRight=%.2f BottomRight=%.2f TopLeft=%.2f BottomLeft=%.2f\n",
        weight[BALANCE_POSITION_TOP_RIGHT],
        weight[BALANCE_POSITION_BOTTOM_RIGHT],
        weight[BALANCE_POSITION_TOP_LEFT],
        weight[BALANCE_POSITION_BOTTOM_LEFT]
      );
      unsigned long current_time = millis();
      if (current_time - last_uart_send >= 500) {
        MySerial.printf("%.2f %.2f %.2f %.2f\n", 
           weight[BALANCE_POSITION_TOP_RIGHT],
           weight[BALANCE_POSITION_BOTTOM_RIGHT],
           weight[BALANCE_POSITION_TOP_LEFT],
           weight[BALANCE_POSITION_BOTTOM_LEFT]);
        Serial.printf("sent: %.2f %.2f %.2f %.2f\n", 
           weight[BALANCE_POSITION_TOP_RIGHT],
           weight[BALANCE_POSITION_BOTTOM_RIGHT],
           weight[BALANCE_POSITION_TOP_LEFT],
           weight[BALANCE_POSITION_BOTTOM_LEFT]);
        last_uart_send = current_time;
      }
    }
    
    // Handle Wiimote button presses
    bool wiimote_button_down  = (data[2] & 0x01) != 0;
    bool wiimote_button_up    = (data[2] & 0x02) != 0;
    bool wiimote_button_right = (data[2] & 0x04) != 0;
    bool wiimote_button_left  = (data[2] & 0x08) != 0;
    bool wiimote_button_plus  = (data[2] & 0x10) != 0;
    bool wiimote_button_2     = (data[3] & 0x01) != 0;
    bool wiimote_button_1     = (data[3] & 0x02) != 0;
    bool wiimote_button_B     = (data[3] & 0x04) != 0;
    bool wiimote_button_A     = (data[3] & 0x08) != 0;
    bool wiimote_button_minus = (data[3] & 0x10) != 0;
    bool wiimote_button_home  = (data[3] & 0x80) != 0;

    if (wiimote_button_A) {
      Serial.println("Balance Board A button pressed:");
      ESP.restart();
    }
  }
  else if(event_type == WIIMOTE_EVENT_INITIALIZE){
    Serial.printf("  event_type=WIIMOTE_EVENT_INITIALIZE\n");
    wiimote.scan(true);
  }
  else if(event_type == WIIMOTE_EVENT_SCAN_START){
    Serial.printf("  event_type=WIIMOTE_EVENT_SCAN_START\n");
  }
  else if(event_type == WIIMOTE_EVENT_SCAN_STOP){
    Serial.printf("  event_type=WIIMOTE_EVENT_SCAN_STOP\n");
    if(connection_count==0){
      wiimote.scan(true);
    }
  }
  else if(event_type == WIIMOTE_EVENT_CONNECT){
    Serial.printf("  event_type=WIIMOTE_EVENT_CONNECT\n");
    wiimote.set_led(handle, 1<<connection_count);
    connection_count++;
  }
  else if(event_type == WIIMOTE_EVENT_DISCONNECT){
    Serial.printf("  event_type=WIIMOTE_EVENT_DISCONNECT\n");
    connection_count--;
    wiimote.scan(true);
  }
  else{
    Serial.printf("  event_type=%d\n", event_type);
  }
  delay(5);
}
