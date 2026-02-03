// The ifndef is called a "Include Guard", it acts like a security gate
// to avoid the library get included more than once.
// For example if an other library uses this package, and the user imports
// it in their Arduino sketch too, it will get imported only once to
// avoid any redefinition errors.
#ifndef MCP3911_h
#define MCP3911_h

#include "Arduino.h"
#include "SPI.h"

// Register addresses, you can find a list of them with their descriptions
// in the MCP3911 datasheet at "TABLE 7-1: REGISTER MAP"
// -----------------------------------------------------------------------
#define CH0_REG           0x00    // 24-bits, read-only
#define CH1_REG           0x03    // 24-bits, read-only
#define MOD_REG           0x06    // 8-bits , read-write
#define PHASE_REG         0x07    // 16-bits, read-write
#define GAIN_REG          0x09    // 8-bits , read-write
#define STATUSCOM_REG     0x0A    // 16-bits, read-write
#define CONFIG_REG        0x0C    // 16-bits, read-write
#define OFFCAL_CH0_REG    0x0E    // 24-bits, read-write
#define GAINCAL_CH0_REG   0x11    // 24-bits, read-write
#define OFFCAL_CH1_REG    0x14    // 24-bits, read-write
#define GAINCAL_CH1_REG   0x17    // 24-bits, read-write
#define VREFCAL           0x1A    // 8-bits , read-write

// ---- Defining for the gain and boost register ----
// Most infos can be found at the section "7.4 GAIN – Gain and Boost Configuration Register" of the datasheet
// name: GAIN | bits: 8 | adress: 0x09 | cof: R/W

// Writing enums for a safe usage, forcing the user to input enums rather than needing to use bits 
// (that can cause errors if not handeled properly)

typedef enum {
    GAIN_32 = 0b101,
    GAIN_16 = 0b100,
    GAIN_8  = 0b011,
    GAIN_4  = 0b010,
    GAIN_2  = 0b001,
    GAIN_1  = 0b000, // Default
} Gain;

typedef enum {
    BOOST_X2   = 0b11,
    BOOST_X1   = 0b10, // Default
    BOOST_X066 = 0b01,
    BOOST_X_05 = 0b00,
} Boost;

// The LSB has to be put first and the MSB at the end.
typedef struct __attribute__ ((__packed__)) {
    Gain    ch0_gain: 3; // Bits 0->2
    Gain    ch1_gain: 3; // Bits 3->5
    Boost      boost: 2; // Bits 6->7
} GainRegisterMap;

// Union allows for a datatype to be seen in 2 different ways.
// GainRegister.map will give us direct control over each bits
// for example GainRegister.map.cho0_gain
// On the other hand rawByte allows us to get the raw value of the
// bitstream that we can directly feed into the SPI library!
typedef union {
    GainRegisterMap map;  
    uint8_t rawByte;
} GainRegister;

// ---- Defining for the status and communication register ----
// Most infos can be found at the section "7.5 STATUSCOM Register – Status and Communication Register" of the datasheet
// name: STATUSCOM | bits: 16 | adress: 0x0A | cof: R/W

typedef enum {
    CAL_DISABLED = 0,
    CAL_ENABLED  = 1
} CalibrationMode;

// MSB represents CH1 and LSB represents CH0
//! DATASHEET HAS ERROR, MIGHT NOT WORK, NEED TO TEST WHEN
//! I GET THE CHIP!
typedef enum {
    WIDTH_24_24 = 0b11, // Both 24-bit (Default)
    WIDTH_24_16 = 0b10, // CH1 24-bit, CH0 16-bit
    WIDTH_16_24 = 0b01, // CH1 16-bit, CH0 24-bit
    WIDTH_16_16 = 0b00, // Both 16-bit

} WidthConf;

typedef enum {
    LOOP    = 0b1, // Address counter loops on entire register map (default)
    NO_LOOP = 0b0
} WriteConf;

typedef enum {
    FULL_INCREMENT  = 0b11,
    TYPE_INCREMENT  = 0b10, // Default
    GROUP_INCREMENT = 0b01,
    NO_INCREMENT    = 0b00,
} ReadConf;

// MSB represents CH1 and LSB represents CH0
// Y represents "YES", meaning data is ready
// N represents "NO", meaning data is not ready
typedef enum {
    READY_N_N = 0b11, // Both not ready (Default)
    READY_N_Y = 0b10, // CH1 Not ready, CH0 Ready
    READY_Y_N = 0b01, // CH1 Ready, CH0 Not Ready
    READY_Y_Y = 0b00, // CH1 Ready, CH0 Ready
} DRStatus;

// MSB represents CH1 and LSB represents CH0                          __
// Y represents "YES", meaning Data Ready pulse will be output on the DR pin
//                                                                       __
// N represents "NO", meaning Data Ready pulse will not be output on the DR pin
typedef enum {
    DR_BOTH_PULSE = 0b11, // Both Channels output pulses on DR pin
    DR_CH1_ONLY   = 0b10, // Only CH1 outputs pulses
    DR_CH0_ONLY   = 0b01, // Only CH0 outputs pulses
    DR_LAGGING    = 0b00  // (Default) Output pulse from whichever ADC finishes last
} DrModeConf;

// Mode to decice what's the logic level of the DR pin when
// data is NOT ready.
typedef enum {
    DR_IDLE_HIGH_Z = 1, // Pin floats (High Impedance) when not ready
    DR_IDLE_HIGH   = 0  // Pin drives Logic High when not ready
} DrHizConf;

// Controls whether the raw ADC bitstream is output on the MDAT pins.
// Useful for debugging or custom filtering.
typedef enum {
    MOD_OUT_OFF_OFF = 0b00, // Both Disabled (Default)
    MOD_OUT_OFF_ON  = 0b01, // CH1 Off, CH0 On
    MOD_OUT_ON_OFF  = 0b10, // CH1 On, CH0 Off
    MOD_OUT_ON_ON   = 0b11  // Both Enabled
} ModOutConf;

typedef struct __attribute__ ((__packed__)) {
    // --- Lower Byte ---
    uint8_t          unused_0: 1; // Bit 0
    CalibrationMode  gain_cal: 1; // Bit 1
    CalibrationMode   off_cal: 1; // Bit 2
    WidthConf           width: 2; // Bits 3-4
    WriteConf           write: 1; // Bit 5
    ReadConf             read: 2; // Bits 6-7

    // --- Upper Byte ---
    DRStatus       data_ready: 2; // Bits 8-9 
    DrModeConf        dr_mode: 2; // Bits 10-11
    DrHizConf          dr_hiz: 1; // Bit 12
    uint8_t         unused_13: 1; // Bit 13
    ModOutConf        mod_out: 2; // Bits 14-15
} StatuscomRegisterMap;

typedef union {
    StatuscomRegisterMap map;  
    uint16_t rawValue;
} StatuscomRegister;

// ---- Defining for the configuration register ----
// Most infos can be found at the section "7.6 CONFIG Register – Configuration Register" of the datasheet
// name: CONFIG | bits: 16 | adress: 0x0C | cof: R/W

// Divides the input clock (MCLK) to create the internal AMCLK
// USE CASE: Generally left at PRE_1 (Default). You only increase this if your
// input clock is extremely fast (>4MHz) and you need to slow the chip down
typedef enum {
    PRE_1 = 0b00, // AMCLK = MCLK (Default)
    PRE_2 = 0b01, // AMCLK = MCLK / 2
    PRE_4 = 0b10, // AMCLK = MCLK / 4
    PRE_8 = 0b11,  // AMCLK = MCLK / 8
} PrescalerConf;

// Determines how many internal samples are averaged to create one output result
// - LOW OSR  => Fast Speed, Lower Resolution  (More Noise)
// - HIGH OSR => Slow Speed, Higher Resolution (Less Noise)
typedef enum {
    OSR_32   = 0b000, 
    OSR_64   = 0b001,
    OSR_128  = 0b010,
    OSR_256  = 0b011, // Default
    OSR_512  = 0b100,
    OSR_1024 = 0b101,
    OSR_2048 = 0b110,
    OSR_4096 = 0b111,
} OsrConf;

// Adds random noise to the signal to break up "idle tones"
// https://en.wikipedia.org/wiki/Dither
typedef enum {
    DITHER_OFF = 0b00,
    DITHER_MIN = 0b01,
    DITHER_MED = 0b10,
    DITHER_MAX = 0b11, // Recommended for DC measurements apparently (Default)
} DitherConf;

typedef enum {
    AZ_FREQ_LOW  = 0, // Default
    AZ_FREQ_HIGH = 1,
} AutoZeroConf;

// Allows you to "Soft Reset" the ADCs without cutting power
// These bits self-clear (they go back to 0 after the reset happens)
typedef enum {
    RESET_NONE = 0b00, // Default
    RESET_BOTH = 0b11,
    RESET_CH1  = 0b10, 
    RESET_CH0  = 0b01,
} ResetConf;

// Used to shutdown a channel to save some power
typedef enum {
    SHUTDOWN_NONE = 0b00, // Both Channels Active (Default)
    SHUTDOWN_CH0  = 0b01, // Channel 0 is OFF
    SHUTDOWN_CH1  = 0b10, // Channel 1 is OFF
    SHUTDOWN_BOTH = 0b11  // Both Channels OFF
} ShutdownConf;

// Decides where the chip gets its voltage reference
typedef enum {
    VREF_INTERNAL = 0, // Internal reference (Default)
    VREF_EXTERNAL = 1  // External reference on Vref+ / Vref- pins
} VrefConf;

// Decides if the chip runs on its own crystal or an Arduino signal
typedef enum {
    CLK_INTERNAL_CRYSTAL = 0, // Crystal Oscillator (Default)
    CLK_EXTERNAL_SQUARE  = 1  // External Clock Signal on OSC1 and OSC2 pins
} ClockConf;

typedef struct __attribute__ ((__packed__)) {
    // ---- LOWER BYTE ----
    uint8_t      unused_0: 1; // Bit 0 (Unused)
    ClockConf      clkext: 1; // Bit 1
    VrefConf      vrefext: 1; // Bit 2
    uint8_t      unused_3: 1; // Bit 3 (Unused)
    ShutdownConf shutdown: 2; // Bits 4-5
    ResetConf       reset: 2; // Bits 6-7

    // ---- UPPER BYTE ----
    AutoZeroConf  az_freq: 1; // Bit 8
    DitherConf     dither: 2; // Bits 9-10
    OsrConf           osr: 3; // Bits 11-13
    PrescalerConf     pre: 2; // Bits 14-15
} ConfigRegisterMap;

typedef union {
    ConfigRegisterMap map;
    uint16_t rawValue;
} ConfigRegister;

// ---- Defining for the Modulators Output Register register ----
// Most infos can be found at the section "7.2 MOD Register – Modulators Output Register" of the datasheet
// name: MOD | bits: 8 | adress: 0x06 | cof: R/W

// Niche stuff I will not cover in this driver.

typedef struct __attribute__ ((__packed__)) {
    uint8_t comp_ch0 : 4; // Comparator setting for CH0 (Default 0011)
    uint8_t comp_ch1 : 4; // Comparator setting for CH1 (Default 0011)
} ModRegisterMap;

typedef union {
    ModRegisterMap map;
    uint8_t rawByte;
} ModRegister;

// ---- Defining for the configuration register ----
// Most infos can be found at the section "7.3 PHASE Register – Phase Configuration Register" of the datasheet
// name: PHASE | bits: 16 | adress: 0x07 | cof: R/W

typedef struct __attribute__ ((__packed__)) {
    // LOWER BYTE
    uint16_t phase_value_low : 8; // Lower 8 bits of the delay count

    // UPPER BYTE (SPI is an 8-bit protocol, we need to split this 12-bit int)
    uint16_t phase_value_high: 4; // Upper 4 bits
    uint16_t unused_12       : 4; // Top 4 bits are unused
} PhaseRegisterMap;

typedef union {
    PhaseRegisterMap map;
    uint16_t rawValue;
} PhaseRegister;

// -------------------------------------------
// Defining the main class here:
class MCP3911 {
    public:
        // Constructor & Setup
        MCP3911(); 
        // If external Crystal
        void begin(int _cs_pin, SPIClass &bus = SPI);

        // High-Level configuration functions
        void setGain(GainRegister settings);
        void setStatusCom(StatuscomRegister settings);
        void setConfig(ConfigRegister settings);
        void setPhase(PhaseRegister settings);
        void setMod(ModRegister settings);

        // Reads a 24-bit channel and returns it as a signed long integer
        long readChannel(uint8_t channelAddress);
        
        // Low-Level Helpers
        void writeRegister(uint8_t address, uint16_t data16); // For 16-bit regs
        void writeRegister(uint8_t address, uint8_t data8);   // For 8-bit regs
        uint16_t readRegister16(uint8_t address);             // Read 16-bit reg
        uint8_t readRegister8(uint8_t address);               // Read 8-bit reg
    private:
        int cs_pin;
        SPIClass *spi;
};

#endif