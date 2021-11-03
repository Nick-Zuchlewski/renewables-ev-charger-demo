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
#define CHARGER_CAR_DETECT_PIN 2
#define BOLLARD_LED_PIN 7
#define CHARGING_LED_PIN 6

// FreeRTOS Params and Configuration
#define STATE_QUEUE_SIZE 5
#define STATE_QUEUE_RX_WAIT pdMS_TO_TICKS(0)
#define AUTO_RELOAD_TIMER_PERIOD_STATUS pdMS_TO_TICKS(200)
#define CHARGER_TASK_DELAY pdMS_TO_TICKS(50)

// Neo Pixel Params
#define BOLLARD_LED_PIXELS 7
#define CHARGING_LED_PIXELS 8
#define PIXEL_FULL_OFF 0x00000000
#define PIXEL_FULL_WHITE 0xFF000000
#define PIXEL_FULL_RED 0x00FF0000
#define PIXEL_FULL_GREEN 0x0000FF00
#define PIXEL_FULL_BLUE 0x000000FF

// Charging Stuff
#define MAX_CHARGING 80 // The total "wakes" of inprogress for 50ms over 4 seconds
#define CHARGE_UPDATE_INTVERVAL (MAX_CHARGING / CHARGING_LED_PIXELS) // When to update a pixel
#define PRE_CHARGE_UPDATE_INTVERVAL 5 // How often to flag in pre-charge (Car detected)
#define PRE_CHARGE_FULL_TIME ((PRE_CHARGE_UPDATE_INTVERVAL * 2) * 5) // The number of times it should flash in pre-charge

// Charge State
enum class eChargerState : int {
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
// NOTE: For some reason I can init (configure) the instance in setup or they clone each other...
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel xBollardLED(BOLLARD_LED_PIXELS, BOLLARD_LED_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel xChargingLED(CHARGING_LED_PIXELS, CHARGING_LED_PIN, NEO_GRBW + NEO_KHZ800);

// Function Prototypes
void prvStatusCallback(TimerHandle_t xTimer);
void prvCarDetectCallback(TimerHandle_t xTimer);
void prvChargingTask(void* pvParameters);
void prvUpdatePixel(uint16_t  pixelNum, Adafruit_NeoPixel& pixelHandle, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  //  Serial.begin(9600);
  // END of Trinket-specific code.

  // Set pin mode
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(CHARGER_CAR_DETECT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CHARGER_CAR_DETECT_PIN), prvCarDetectISR, CHANGE);

  // Start Pixels
  xBollardLED.begin();
  xChargingLED.begin();

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
  digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
}


// Detects if a car is detected
static void prvCarDetectISR() {
  BaseType_t xHigherPriorityTaskWoken;
  eChargerState transition;
  // Check if high or low and enqueue it
  digitalRead(CHARGER_CAR_DETECT_PIN) ? transition = eChargerState::VehicleVacant : transition = eChargerState::VehicleDetected;
  xQueueSendFromISR(xStateQueue, &transition, xHigherPriorityTaskWoken);
}

// Manages the charging task
static void prvChargingTask(void* pvParameters) {
  (void) pvParameters;

  // Variables
  uint16_t count = 0;
  uint16_t pixelNum = 0;
  eChargerState state = eChargerState::VehicleVacant;
  bool stateChange = true;

  // Clear LEDs
  xBollardLED.fill(PIXEL_FULL_WHITE, 0, 0);
  xChargingLED.clear();
  xBollardLED.show();
  xChargingLED.show();

  // Run
  for (;;) {
    // Check state queue
    eChargerState transition;
    if (xQueueReceive(xStateQueue, &transition, STATE_QUEUE_RX_WAIT) == pdPASS) {
      // We got new transition, do something...
      state = transition;
      bool stateChange = true;
    }

    // Execute based on state
    switch (state) {
      case eChargerState::VehicleVacant:
        // Clear and wait for a vehicle
        pixelNum = 0;
        count = 0;
        xChargingLED.fill(0x3F000000, 0, 0);
        break;
      case eChargerState::VehicleDetected:
        // Flash blue a few times
        if ((++count / PRE_CHARGE_UPDATE_INTVERVAL) % 2 == 0) {
          xChargingLED.fill(PIXEL_FULL_BLUE, 0, 0);
        } else {
          xChargingLED.clear();
        }
        // Check if ready to transition
        if (count == PRE_CHARGE_FULL_TIME) {
          // Clear and transistion to in progress
          pixelNum = 0;
          count = 0;
          xChargingLED.clear();
          state = eChargerState::ChargeinProgress;
        }
        break;
      case eChargerState::ChargeinProgress:
        // Increment and update
        prvUpdatePixel(++count / CHARGE_UPDATE_INTVERVAL, xChargingLED, 0, 255, 0, 0);
        // Check if at full charge
        if (count >= MAX_CHARGING) {
          // Complete
          state = eChargerState::ChargeComplete;
          // xChargingLED.clear();
        }
        break;
      case eChargerState::ChargeComplete:
        // Set all green
        xChargingLED.fill(PIXEL_FULL_GREEN, 0, 0);
        break;
      default:
        // Should never reach
        xChargingLED.fill(PIXEL_FULL_RED, 0, 0);
        // xChargingLED.getPixelColor(0) > 0 ? xChargingLED.fill(0, 0, 0) : xChargingLED.fill(PIXEL_FULL_RED, 0, 0);
        break;
    }

    // Show
    xBollardLED.show();
    xChargingLED.show();

    // Delay
    vTaskDelay(CHARGER_TASK_DELAY);
  }
}

static void prvUpdatePixel(uint16_t  pixelNum, Adafruit_NeoPixel & pixelHandle, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  for (int i = 0; i < pixelNum; i++) {
    pixelHandle.setPixelColor(i, pixelHandle.Color(r, g, b, w));
    pixelHandle.show();
  }
}
