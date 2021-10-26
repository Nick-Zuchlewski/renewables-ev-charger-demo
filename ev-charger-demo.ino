#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <queue.h>

// Pin Defines
#define STATUS_LED 13
#define CHARGER_CAR_DETECT 10

// FreeRTOS Params and Configuration
#define STATE_QUEUE_SIZE 5
#define STATE_QUEUE_RX_WAIT pdMS_TO_TICKS(0)
#define AUTO_RELOAD_TIMER_PERIOD_STATUS pdMS_TO_TICKS(200)

// Charge State
enum class eChargerState {
  Null,
  VehicleVacant,
  VehicleDetected,
  ChargeinProgress,
  ChargeComplete
};

// Handles
static TimerHandle_t xStatusTimer;
static TaskHandle_t xChargerTask;
static QueueHandle_t xStateQueue;

// Timer Callbacks/Tasks Prototypes
void prvStatusCallback(TimerHandle_t xTimer);
void prvCarDetectCallback(TimerHandle_t xTimer);
void prvChargingTask(void* pvParameters);

void setup() {
  // Set pin mode
  pinMode(STATUS_LED, OUTPUT);
  pinMode(CHARGER_CAR_DETECT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CHARGER_CAR_DETECT), prvCarDetectISR, CHANGE);
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
}

void loop() {
  // NA
}

// Provides a blink status at the board led
static void prvStatusCallback(TimerHandle_t xTimer) {
  (void) xTimer;
  // Toggle board LED
  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

// Detects if a car is detected
static void prvCarDetectISR() {
  BaseType_t xHigherPriorityTaskWoken;
  eChargerState transition;
  // Check if high or low
  digitalRead(STATUS_LED) ? transition = eChargerState::VehicleDetected : eChargerState::VehicleVacant;
  xQueueSendFromISR(xStateQueue, &transition, xHigherPriorityTaskWoken);
}

// Manages the charging task
static void prvChargingTask(void* pvParameters) {
  (void) pvParameters;
  eChargerState state = eChargerState::Null;
  // Run
  for (;;) {
    // Check state queue
    eChargerState transition;
    if (xQueueReceive(xStateQueue, &transition, STATE_QUEUE_RX_WAIT) == pdPASS)
    {
      // We got new transition, do something...
      // TODO
    }
    // Execute based on state
    switch (state) {
      case eChargerState::VehicleVacant:
        // TODO
        break;
      case eChargerState::VehicleDetected:
        // TODO
        break;
      case eChargerState::ChargeinProgress:
        // TODO
        break;
      case eChargerState::ChargeComplete:
        // TODO
        break;
      default:
        // TODO
        break;
    }
  }
}
