// config.h - Configuration file with sensitive data
// DO NOT COMMIT TO GIT - This file is excluded via .gitignore

#ifndef CONFIG_H
#define CONFIG_H

// Your phone number for SMS alerts
#define ALERT_PHONE "+61408436241"

// SMS Service Centre number for Aldi Mobile
#define SMSC_NUMBER "+61418706275"

// Alert thresholds
#define PUMP_THRESHOLD_VALUE 5
#define LOW_VOLTAGE_VALUE 11.5
#define HIGH_TEMP_VALUE 45.0
#define HIGH_HUMIDITY_VALUE 85.0

#endif