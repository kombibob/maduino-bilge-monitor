// config_example.h
// Copy this file to config.h and fill in your details.
// config.h is excluded from Git — never commit your real phone numbers.

#ifndef CONFIG_H
#define CONFIG_H

// Phone number to receive SMS alerts. This is also the only number the device
// accepts SMS commands from; commands from any other number are ignored.
#define ALERT_PHONE "+61400000000"

// SMS Service Centre number for your carrier
// Aldi Mobile (Telstra network): +61418706275
// Optus: +61418706275
// Vodafone: +61415012008
#define SMSC_NUMBER "+61418706275"

#endif