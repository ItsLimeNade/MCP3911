#include "MCP3911.h"
#include "Arduino.h"
#include "SPI.h"

MCP3911::MCP3911() {}

// MCP3911 supports up to 20MHz SPI
// Source: Page ONE of the Datasheet!! :D
SPISettings mcpSettings(4000000, MSBFIRST, SPI_MODE0);

//=======================
//|| Builder Functions ||
//=======================
void MCP3911::begin(int _cs_pin, SPIClass &bus) {
    cs_pin = _cs_pin;
    spi = &bus;
    
    // Setup SPI & CS Pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    
    spi->begin();
}

//======================
//|| Config Functions ||
//======================
void MCP3911::setGain(GainRegister settings) {
    writeRegister(GAIN_REG, settings.rawByte);
}

void MCP3911::setStatusCom(StatuscomRegister settings) {
    writeRegister(STATUSCOM_REG, settings.rawValue);
}

void MCP3911::setConfig(ConfigRegister settings) {
    writeRegister(CONFIG_REG, settings.rawValue);
}

void MCP3911::setPhase(PhaseRegister settings) {
    writeRegister(PHASE_REG, settings.rawValue);
}

void MCP3911::setMod(ModRegister settings) {
    writeRegister(MOD_REG, settings.rawByte);
}

//======================
//|| Helper Functions ||
//======================
uint8_t MCP3911::readRegister8(uint8_t address) {
    spi->beginTransaction(mcpSettings);
    digitalWrite(cs_pin, LOW);

    spi->transfer((address << 1) | 1); 

    // We send 0x00 (Dummy Byte) just to push the clock line.
    // The chip sends the data back at the same time.
    uint8_t result = spi->transfer(0x00);

    digitalWrite(cs_pin, HIGH);
    spi->endTransaction();
    return result;
}

uint16_t MCP3911::readRegister16(uint8_t address) {
    spi->beginTransaction(mcpSettings);
    digitalWrite(cs_pin, LOW);

    spi->transfer((address << 1) | 1);
    uint16_t result = spi->transfer16(0x0000);

    digitalWrite(cs_pin, HIGH);
    spi->endTransaction();
    return result;
}


long MCP3911::readChannel(uint8_t channelAddress) {
    spi->beginTransaction(mcpSettings);
    digitalWrite(cs_pin, LOW);

    spi->transfer((channelAddress << 1) | 1);

    uint8_t upper  = spi->transfer(0x00);
    uint8_t middle = spi->transfer(0x00);
    uint8_t lower  = spi->transfer(0x00);

    digitalWrite(cs_pin, HIGH);
    spi->endTransaction();

    long combinedValue = ((long)upper << 16) | ((long)middle << 8) | (long)lower;

    if (upper & 0x80) { 
        combinedValue |= 0xFF000000;
    }

    return combinedValue;
}

void MCP3911::writeRegister(uint8_t address, uint8_t data8) {
    spi->beginTransaction(mcpSettings);
    digitalWrite(cs_pin, LOW);
    
    spi->transfer((address << 1) | 0); 
    
    spi->transfer(data8);
    
    digitalWrite(cs_pin, HIGH);
    spi->endTransaction();
}

void MCP3911::writeRegister(uint8_t address, uint16_t data16) {
    spi->beginTransaction(mcpSettings);
    digitalWrite(cs_pin, LOW);
    
    spi->transfer((address << 1) | 0); // Write mode cuz of the bitwise or 0
    
    spi->transfer16(data16);
    
    digitalWrite(cs_pin, HIGH);
    spi->endTransaction();
}
