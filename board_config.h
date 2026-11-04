#pragma once

// Board-level hardware configuration.
// Selected automatically for ESP32, ESP32-C3, and ESP32-S3.

#ifdef CONFIG_IDF_TARGET_ESP32C3
// ---------------------- ESP32-C3 -------------------------

// ---- Motor pins (order: RL, RR, FR, FL, matching motors.ino) ----
// GPIO0/1/3/10 have no internal pull-up and are not strapping pins (C3 strapping pins are GPIO2/8/9 only).
#define BOARD_MOTOR_PINS   {3, 10, 0, 1}    // RL=GPIO3, RR=GPIO10, FR=GPIO0, FL=GPIO1

// ---- Battery ADC ----
// GPIO2 is a strapping pin (weak internal pull-up). In ADC mode that pull-up does not affect the reading.
// Strapping is sampled at boot; by the time analogRead() runs the pin is a normal input.
#define BOARD_VBAT_ADC_PIN 2               // ADC1_CH2 (GPIO2)

// ---- RC UART (CRSF/ELRS by default; SBUS via RC_PROTOCOL=0) ----
// GPIO9 is the CRSF RX pin. Standard logic levels, no inverter.
#define BOARD_RC_SERIAL    Serial1
#define BOARD_RC_RX_PIN    9
#define BOARD_RC_TX_PIN    -1          // telemetry TX reserved, -1 = unused
#define BOARD_RC_PROTOCOL  1           // 0=SBUS, 1=CRSF (ELRS)
#define BOARD_RC_BAUD      420000      // CRSF default 420000; SBUS is 100000

// ---- IMU SPI (Arduino pins_arduino.h defaults) ----
// Matches the Arduino ESP32-C3 SPI defaults, so SPI.begin() can be called with no arguments.
#define BOARD_SPI_SCK      4               // SCK  = GPIO4
#define BOARD_SPI_MISO     5               // MISO = GPIO5
#define BOARD_SPI_MOSI     6               // MOSI = GPIO6
#define BOARD_SPI_CS       7               // SS   = GPIO7

// ---- I2C (ESP32-C3 default pins) ----
#define BOARD_I2C_SDA          (-1)        // GPIO8 (shared with the LED; do not enable both on an expansion board)
#define BOARD_I2C_SCL          (-1)        // GPIO9

// ---- LED: GPIO8, blue status LED, active low ----
#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      8
#define BOARD_LED_INVERTED 1               // lit when the pin is low

// ---- Performance (C3 is kept light) ----
#define BOARD_VBAT_ADC_SAMPLES       8    // ADC samples. ESP32/S3 default is 16
#define BOARD_LOG_DURATION           4    // 4 s log buffer, saves about 22 KB RAM. ESP32/S3 default is 8 s
#define BOARD_CONSOLE_LINES          20   // console lines, saves about 7 KB RAM. ESP32/S3 default is 50
#define BOARD_CONSOLE_LINE_LEN       160  // characters per line. ESP32/S3 default is 240
#define BOARD_MAVLINK_TELEM_FAST_HZ  5    // slower fast telemetry to leave CPU for the flight loop. ESP32/S3 default is 10 Hz
#define BOARD_WIFI_ENABLED           0    // C3 is single-core: Wi-Fi shares the CPU with the flight loop, so it is off
#define BOARD_WEB_RC_ENABLED         0    // web remote needs Wi-Fi, so it is off with BOARD_WIFI_ENABLED

// ---- Expansion-board sensors ----
#undef  EXPANSION_BOARD_ENABLED
#define EXPANSION_BOARD_ENABLED      0    // pin conflict on C3, expansion board forced off
#define BOARD_VL53_XSHUT_PIN   (-1)        // VL53L1X XSHUT, tentatively GPIO10
#define BOARD_PMW_CS_PIN       (-1)        // PMW3901 CS, unassigned on C3, -1 disables it

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
// ---------------------- ESP32-S3 -------------------------

// ---- Motor pins: GPIO4-7, away from the FSPI pins (GPIO10-13), no strapping constraint ----
#define BOARD_MOTOR_PINS   {4, 5, 6, 7}    // RL=GPIO4, RR=GPIO5, FR=GPIO6, FL=GPIO7

// ---- Battery ADC: GPIO1 (ADC1_CH0; on S3, ADC1 is GPIO1-10) ----
#define BOARD_VBAT_ADC_PIN 1

// ---- RC UART (CRSF/ELRS by default): GPIO8, clear of FSPI and UART0 ----
#define BOARD_RC_SERIAL    Serial2
#define BOARD_RC_RX_PIN    8
#define BOARD_RC_TX_PIN    -1
#define BOARD_RC_PROTOCOL  1
#define BOARD_RC_BAUD      420000

// ---- IMU SPI: ESP32-S3 FSPI defaults ----
// SCK=12, MISO=13, MOSI=11, CS=10 (default SS)
#define BOARD_SPI_SCK      12
#define BOARD_SPI_MISO     13
#define BOARD_SPI_MOSI     11
#define BOARD_SPI_CS       10

// ---- I2C (ESP32-S3 default pins) ----
#define BOARD_I2C_SDA          35          // GPIO35
#define BOARD_I2C_SCL          36          // GPIO36

// ---- LED: GPIO2 on the current PCB, same drive as the classic ESP32 ----
#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      2
#define BOARD_LED_INVERTED 0

// ---- Performance (S3, standard) ----
#define BOARD_VBAT_ADC_SAMPLES       16 // ADC samples
#define BOARD_LOG_DURATION           8  // log buffer, seconds
#define BOARD_CONSOLE_LINES          50  // console lines
#define BOARD_CONSOLE_LINE_LEN       240  // characters per line
#define BOARD_MAVLINK_TELEM_FAST_HZ  10  // fast MAVLink telemetry rate
#define BOARD_WIFI_ENABLED           1
#define BOARD_WEB_RC_ENABLED         1

// ---- Expansion-board sensors ----
#define EXPANSION_BOARD_ENABLED      1 // 1 = probe BMP388/VL53L1X/QMC5883L/PMW3901; 0 = skip, capability flags stay false, basic flight is unchanged
#define BOARD_VL53_XSHUT_PIN   37      // VL53L1X XSHUT, tentatively GPIO37
#define BOARD_PMW_CS_PIN       38      // PMW3901 CS, tentatively GPIO38

#else  // ---- ESP32 default ----
// ---------------------- ESP32 (default) -------------------------

#define BOARD_MOTOR_PINS   {12, 13, 15, 14}  // motors RL=GPIO12, RR=GPIO13, FR=GPIO15, FL=GPIO14

#define BOARD_VBAT_ADC_PIN 36              // battery ADC (GPIO36, input only)

#define BOARD_RC_SERIAL    Serial2         // RC UART (CRSF/ELRS by default)
#define BOARD_RC_RX_PIN    4               // RC RX (GPIO4)
#define BOARD_RC_TX_PIN    -1              // telemetry TX reserved, -1 = unused
#define BOARD_RC_PROTOCOL  1               // 0=SBUS, 1=CRSF (ELRS)
#define BOARD_RC_BAUD      420000          // CRSF default 420000; SBUS is 100000

#define BOARD_SPI_SCK      18              // SPI clock (GPIO18)
#define BOARD_SPI_MISO     19              // SPI MISO (GPIO19)
#define BOARD_SPI_MOSI     23              // SPI MOSI (GPIO23)
#define BOARD_SPI_CS       5               // SPI chip select (GPIO5)

#define BOARD_I2C_SDA      21              // GPIO21 (ESP32 default SDA)
#define BOARD_I2C_SCL      22              // GPIO22 (ESP32 default SCL)

#define BOARD_LED_ENABLED  1               // onboard LED
#define BOARD_LED_PIN      2               // LED pin (GPIO2)
#define BOARD_LED_INVERTED 0               // lit when the pin is high

// ---- Performance (ESP32, standard) ----
#define BOARD_VBAT_ADC_SAMPLES       16 // ADC samples
#define BOARD_LOG_DURATION           8  // log buffer, seconds
#define BOARD_CONSOLE_LINES          50  // console lines
#define BOARD_CONSOLE_LINE_LEN       240  // characters per line
#define BOARD_MAVLINK_TELEM_FAST_HZ  10  // fast MAVLink telemetry rate
#define BOARD_WIFI_ENABLED           1  // Wi-Fi switch
#define BOARD_WEB_RC_ENABLED         1  // web remote switch

// ---- Expansion-board sensors ----
#define EXPANSION_BOARD_ENABLED      1 // 1 = probe BMP388/VL53L1X/QMC5883L/PMW3901; 0 = skip, capability flags stay false, basic flight is unchanged
#define BOARD_VL53_XSHUT_PIN   32          // VL53L1X XSHUT (GPIO32)
#define BOARD_PMW_CS_PIN       33          // PMW3901 SPI CS (GPIO33)

#endif

// ---- Fallbacks (unknown chip: features on) ----

// ---- Expansion board ----
//   1 = probe BMP388 / VL53L1X / QMC5883L / PMW3901
//   0 = skip probing; capability flags stay false; basic flight is unchanged
#if !defined(EXPANSION_BOARD_ENABLED)
#define EXPANSION_BOARD_ENABLED 1  // fallback: unknown chip, probing enabled
#endif

// ---- Wi-Fi / web RC fallback (unknown chip: both on) ----
#if !defined(BOARD_WIFI_ENABLED)
#define BOARD_WIFI_ENABLED   1
#endif
#if !defined(BOARD_WEB_RC_ENABLED)
#define BOARD_WEB_RC_ENABLED 1
#endif
