/*
* Example: Custom SPI Pins
* Shows how to re-map the SPI bus to any pins you want.
*/
#include <MCP3911.h>
#include <SPI.h>

// PIN CONFIGURATION (Edit these so they match your board)
// Example for Seeed XIAO ESP32S3
#define MY_SCK   D8
#define MY_MISO  D9
#define MY_MOSI  D10
#define MY_CS    D1

MCP3911 adc;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Configure the SPI bus explicitly BEFORE passing it to the driver
    // syntax: SPI.begin(SCK, MISO, MOSI, CS);
    SPI.begin(MY_SCK, MY_MISO, MY_MOSI, MY_CS);

    // Pass the configured 'SPI' object to the driver
    // The driver will now use *your* pins.
    adc.begin(MY_CS, SPI);

    // Simple Config
    ConfigRegister confReg;
    confReg.rawValue = 0;
    confReg.map.osr = OSR_256;
    confReg.map.vrefext = VREF_INTERNAL;
    confReg.map.clkext = CLK_INTERNAL_CRYSTAL;
    adc.setConfig(confReg);
    
    Serial.println("MCP3911 Started on CUSTOM SPI Pins.");
}

void loop() {
    long val1 = adc.readChannel(CH1_REG);
    Serial.print("CH1: "); Serial.println(val1);
    delay(100);
}