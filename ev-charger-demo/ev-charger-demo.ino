#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <queue.h>

// Pins
#define STATUS_LED_PIN 13
#define CHARGER_CAR_DETECT_PIN 10
#define BOLLARD_LED_PIN 6
#define CHARGING_LED PIN 7

// FreeRTOS Params and Configuration
#define STATE_QUEUE_SIZE 5
#define STATE_QUEUE_RX_WAIT pdMS_TO_TICKS(0)
#define AUTO_RELOAD_TIMER_PERIOD_STATUS pdMS_TO_TICKS(200)
#define CHARGER_TASK_DELAY pdMS_TO_TICKS(50)

// Neo Pixel Params
#define BOLLARD_LED_PIXELS 7
#define CHARGING_LED_PIXELS 8
#define PIXEL_FULL_WHITE 0xFF000000
#define PIXEL_FULL_RED 0x00FF0000
#define PIXEL_FULL_GREEN 0x0000FF00
#define PIXEL_FULL_BLUE 0x000000FF

// Charging Stuff
#define MAX_CHARGING 400

// Charge State
enum class eChargerState {
  Null,
  VehicleVacant,
  VehicleDetected,
  ChargeinProgress,
  ChargeComplete
};

// FreeRTOS Handles
static TimerHandle_t xStatusTimer;
static TaskHandle_t xChargerTask;
static QueueHandle_t xStateQueue;

// NeoPixel Handles
Adafruit_NeoPixel xBollardLED, xChargingLED;

// Function Prototypes
void prvStatusCallback(TimerHandle_t xTimer);
void prvCarDetectCallback(TimerHandle_t xTimer);
void prvChargingTask(void* pvParameters);
void prvUpdatePixel(uint16_t  pixelNum, Adafruit_NeoPixel& pixelHandle, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

void setup() {
  // Set pin mode
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(CHARGER_CAR_DETECT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CHARGER_CAR_DETECT_PIN), prvCarDetectISR, CHANGE);
  // Set the NeoPixles
  xBollardLED = Adafruit_NeoPixel(BOLLARD_LED_PIXELS, BOLLARD_LED_PIN, NEO_GRB + NEO_KHZ800);
  xChargingLED = Adafruit_NeoPixel(CHARGING_LED_PIXELS, CHARGING_LED_PIXELS, NEO_GRB + NEO_KHZ800);
  // Setup the queue
  xStateQueue = xQueueCreate(STATE_QUEUE_SIZE, sizeof(eChargerState));
  // Status Timer
  xStatusTimer = xTimerCreate(
                   "Status",
                   AUTO_RELOAD_TIMER_PERIOD_STATUS,
                   pdTRUE,
                   0,
                   prvStatusCallback);
  // Charging Task
  xChargerTask = xTaskCreate(
                   prvChargingTask,
                   "Charging",
                   128,
                   NULL,
                   1,
                   NULL);
  // Start scheduler
  // I should probably check if populated/started correctly
  xTimerStart(xStatusTimer, 0);
  vTaskStartScheduler();
  // Start Pixels
  xBollardLED.begin();
  xChargingLED.begin();
}

void loop() {
  // NA
}

// Provides a blink status at the board led
static void prvStatusCallback(TimerHandle_t xTimer) {
  (void) xTimer;
  // Toggle board LED
  digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
}

// Detects if a car is detected
static void prvCarDetectISR() {
  BaseType_t xHigherPriorityTaskWoken;
  eChargerState transition;
  // Check if high or low
  digitalRead(STATUS_LED_PIN) ? transition = eChargerState::VehicleDetected : eChargerState::VehicleVacant;
  xQueueSendFromISR(xStateQueue, &transition, xHigherPriorityTaskWoken);
}

// Manages the charging task
static void prvChargingTask(void* pvParameters) {
  (void) pvParameters;
  // Variables
  uint16_t count = 0;
  eChargerState state = eChargerState::Null;
  // Clear Pixels
  xBollardLED.clear();
  xChargingLED.clear();
  // Set Bollard White
  xBollardLED.fill(PIXEL_FULL_WHITE, 0, 0);
  xBollardLED.show();
  // Run
  for (;;) {
    // Check state queue
    eChargerState transition;
    if (xQueueReceive(xStateQueue, &transition, STATE_QUEUE_RX_WAIT) == pdPASS) {
      // We got new transition, do something...
      state = transition;
    }
    // Execute based on state
    switch (state) {
      case eChargerState::VehicleVacant:
        // Clear and wait for a vehicle
        xChargingLED.clear();
        break;
      case eChargerState::VehicleDetected:
        // Clear and transistio to in progress
        xChargingLED.clear();
        count = 0;
        state = eChargerState::ChargeinProgress;
        break;
      case eChargerState::ChargeinProgress:
        // Increment and update
        prvUpdatePixel(++count / 100, xChargingLED, 0, 255, 0, 0);
        // Check if at full charge
        if (count >= 400) {
          // Complete
          state = eChargerState::ChargeComplete;
        }
        break;
      case eChargerState::ChargeComplete:
        // Set all green
        xBollardLED.fill(PIXEL_FULL_GREEN, 0, 0);
        xBollardLED.show();
        break;
      default:
        // TODO
        break;
    }
    // Delay
    vTaskDelay(CHARGER_TASK_DELAY);
  }
}

static void prvUpdatePixel(uint16_t  pixelNum, Adafruit_NeoPixel& pixelHandle, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  for (int i = 0; i < pixelNum; i++) {
    pixelHandle.setPixelColor(i, pixelHandle.Color(r, g, b, w));
    pixelHandle.show();
  }
}
