#pragma once

#include <Arduino.h>

// LoRa RFM95
#define LORA_FREQUENCY_HZ   915E6
#define LORA_CS_PIN         10
#define LORA_RST_PIN        9
#define LORA_DIO0_PIN       2
#define LORA_SPI_FREQ_HZ    8000000

// GPS GT-U7
#define GPS_SERIAL          Serial1
#define GPS_BAUD            9600

// Battery monitor
#define VBAT_PIN            A0
#define VBAT_R1_OHMS        330000.0f
#define VBAT_R2_OHMS        100000.0f
#define ADC_REF_V           3.3f
#define ADC_MAX_COUNTS      4095.0f
#define BATT_WARN_V         6.8f

// Sensor settings
#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define MS5607_ADDR_0       0x76
#define MS5607_ADDR_1       0x77

// Timing
#define BARO_UPDATE_MS      50
#define FLIGHT_TX_MS        200
#define NAV_TX_MS           1000
#define STATUS_TX_MS        2000
#define LOG_UPDATE_MS       200
#define STATUS_PRINT_MS     1000

// Packet types
#define PKT_TYPE_FLIGHT_V7  0x01
#define PKT_TYPE_NAV_V7     0x02
#define PKT_TYPE_STATUS_V8  0x03

// Health bits for status packet
#define HEALTH_BARO_OK      (1u << 0)
#define HEALTH_IMU_OK       (1u << 1)
#define HEALTH_GPS_OK       (1u << 2)
#define HEALTH_SD_OK        (1u << 3)
#define HEALTH_NAND_OK      (1u << 4)
#define HEALTH_LOG_OK       (1u << 5)
#define HEALTH_BATT_OK      (1u << 6)
