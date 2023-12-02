#include <Arduino.h>

/**
 * @brief LEDs for the tachometer, self explanitory
 * 
 * G = Green
 * O = Orange
 * R = Red
 * W = White
 */
#define LED_G 34
#define LED_O 35
#define LED_R 36
#define LED_W 37

/**
 * @brief Subbing in the HIGH and LOW for on and off because the lights need to be grounded
 * (driven low) to turn on
 * 
 */
#define ON LOW
#define OFF HIGH

/**
 * @brief Below are the best calculated RPM values per gear to shift at as calculated by Engines Cptn
 * Ayson Mar (2022-23)
 * 
 * Slide 21 in this slideshow: https://livecsupomona-my.sharepoint.com/:p:/g/personal/mpsevilla_cpp_edu/EcM-7jeAgKxNnuO7VG1wmRkBhsLgbpXqv8pT9bATYowiqg?e=LsJPPc
 * 
 */
unsigned int gears[6] = {7000, 13600, 13000, 12500, 12200, 12200};

void flashyOnSequence();
void checkRPM();