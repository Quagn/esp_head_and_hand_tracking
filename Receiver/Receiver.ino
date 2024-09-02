#include <ESP8266WiFi.h>
#include <espnow.h>

#define CHANNEL_COUNT 3               // 3 channel pitch, roll, yaw
#define PPM_PIN 2                     // PPM pin out

typedef struct struct_message {
  int8_t Pitch;
  int8_t Roll;
  int8_t Yaw;
} struct_message;
struct_message receivedData;

unsigned long lastPPMUpdate = 0;
uint16_t ppmChannels[CHANNEL_COUNT] = {1500, 1500, 1500};   // Initial PPM pulse widths (neutral)

volatile bool ppmHigh = false;
volatile int currentChannel = 0;

// Callback when data is received
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  memcpy(&receivedData, data, sizeof(receivedData));

  uint16_t pitchPWM = map(receivedData.Pitch, -30 , 30 , 1000, 2000);
  uint16_t rollPWM  = map(receivedData.Roll , -30 , 30 , 1000, 2000);
  uint16_t yawPWM   = map(receivedData.Yaw  , -180, 180, 1000, 2000);

  ppmChannels[0] = constrain(pitchPWM, 1000, 2000);
  ppmChannels[1] = constrain(rollPWM , 1000, 2000);
  ppmChannels[2] = constrain(yawPWM  , 1000, 2000);
}

// Timer interrupt handler
void IRAM_ATTR ppmTimerISR() {
  if (ppmHigh) {
    GPOC = (1 << PPM_PIN);        // PPM LOW
    ppmHigh = false;
  } else {
    if (currentChannel < CHANNEL_COUNT) {
      GPOS = (1 << PPM_PIN);      // PPM HIGH
      ppmHigh = true;
      timer1_write(500 + ppmChannels[currentChannel]); // 500 us is PPM sync pulse, add channel value
      currentChannel++;
    } else {
      GPOC = (1 << PPM_PIN);      // PPM LOW
      ppmHigh = false;
      currentChannel = 0;
      timer1_write(22000); // Gap before starting the next frame (20 ms frame duration - total channels duration)
    }
  }
}

void setup() {
  //Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(PPM_PIN, OUTPUT);                     // PPM Pin GPIO2

  cli(); // Disable interrupts

  // Configure timer1 for PPM signal generation
  timer1_isr_init();
  timer1_attachInterrupt(ppmTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(22000); // Gap before starting the first frame

  sei(); // Enable interrupts
}

void loop() {
  // Your other non-blocking tasks can go here
}
