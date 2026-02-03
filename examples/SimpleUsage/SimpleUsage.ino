/*
* Example: Basic Usage
* Uses the default SPI pins for your board
*/
#include <MCP3911.h>
#include <SPI.h>

#define CS_PIN 10 

MCP3911 adc;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize with just the CS pin.
    // The library automatically picks the default 'SPI' bus.
    adc.begin(CS_PIN);

    // Simple Config
    GainRegister gainReg;
    gainReg.map.ch0_gain = GAIN_1;
    gainReg.map.ch1_gain = GAIN_1;
    gainReg.map.boost    = BOOST_X1;
    adc.setGain(gainReg);
    
    Serial.println("MCP3911 Started on Default SPI Pins.");
}

void loop() {
    long val0 = adc.readChannel(CH0_REG);
    Serial.print("CH0: "); Serial.println(val0);
    delay(100);
}