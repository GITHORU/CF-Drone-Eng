# ESP32 drone flight-controller firmware

# Hardware

## Supported boards
ESP32 and ESP32-S3.
IMU: MPU6500 / MPU9250.

## Official hardware
Designed for the Cherglow E1 drone series. OSHWHub project links:
E1_MINI https://oshwhub.com/songge8/project_qqqyfdkm
E1_Nano https://oshwhub.com/songge8/project_fnvbukbz

# Changelog

## 20260521
* Attitude control tuned

## 20260520
* ESP32-C3 support
* PWM frequency tuned to reduce MOSFET stress

## 20260515
* Battery voltage measurement accuracy improved
* Low-battery thresholds adjusted

## 20260430
* Low-battery behavior: block arming, auto-disarm, auto-land
* Throttle mapping limits MOT_THR_MIN and MOT_THR_MAX
* Web remote UI updated

## 20260428
* ESP32-C3 and ESP32-S3 board profiles

## 20260417
* Debug log no longer truncated; command help cleaned up

## 20260416
* RC debug logs printed less often
* Server-side link-quality monitor removed; the web UI already reports it

## 20260415
* Status LED: slow blink while armed, fast blink on low battery, link loss, and similar warnings
* Motor pins configurable from the debug console
* Wi-Fi SSID and password configurable from the debug console
* Network latency calculation fixed
* Web remote send rate raised to 20 Hz; do not go higher if you want to keep loop time
* Other routine updates

## 20260328
* Web remote transport reworked
* Unused web-remote server code removed

## 20260327
* Web remote heavily reworked
* Web remote debug console
* Inverted-vehicle cutoff: a long inversion is treated as a fault and motors stop
