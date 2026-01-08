// =============================================================================
// NoNag v2 - EGS/NAG52 Emulator for Mercedes W211 Manual Swap
// https://github.com/gareth10203/NoNag.git
// =============================================================================

#include <SPI.h>
#include <mcp_canbus.h>
#include <Arduino.h>


#define SPI_CS_PIN  17  // CS Pin for CANBed board
#define REVERSE_SWITCH_PIN 8 // GPIO pin for reverse switch
#define LED_PIN 13 // Onboard LED

MCP_CAN CAN(SPI_CS_PIN);  // Create MCP_CAN instance

// Function prototypes
void sendGear(unsigned char gear, const char* gearName);
void sendEGS(unsigned char gear);
void handleHandbrake();
void updateLED(unsigned char gear);

// CAN message intervals
unsigned long lastGearTime = 0;
unsigned long lastEGSTime = 0;
unsigned long lastHandbrakeCheckTime = 0;
const unsigned long interval10ms = 10;
const unsigned long interval20ms = 20;

// Handbrake state
bool handbrakeOn = true;
bool lastHandbrakeState = true;

// Gear tracking
unsigned char lastGearSent = 0x00;

// EGS message counters
static unsigned char cvnCounter = 0;      // Error/CVN counter (5-bit, 0-31)
static bool timeToToggle = false;         // Toggle control (40ms period)
static unsigned int frameCounter = 0;     // Frame counter for CALID sequence

// CALID data from real NAG52 capture (16 bytes, repeating)
static const unsigned char CALID_DATA[16] = {
    0x30, 0x33, 0x34, 0x35, 0x34, 0x35, 0x34, 0x33,
    0x33, 0x32, 0x30, 0x32, 0x30, 0x30, 0x30, 0x32
};

// LED pulsing variables
unsigned long lastLEDToggleTime = 0;
const unsigned long pulseInterval = 500;
bool ledState = LOW;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("NoNag v2 - EGS Emulator for W211 Manual Swap");
    Serial.println("=============================================");

    pinMode(REVERSE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    // Initialize CAN at 500kbps
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");

    // Configure CAN filters to only receive 0x240 (handbrake status)
    CAN.init_Mask(0, 0, 0x7FF);
    CAN.init_Mask(1, 0, 0x7FF);
    CAN.init_Filt(0, 0, 0x240);
}

void loop() {
    unsigned long currentTime = millis();

    // Check handbrake status every 20ms
    if (currentTime - lastHandbrakeCheckTime >= interval20ms) {
        handleHandbrake();
        lastHandbrakeCheckTime = currentTime;
    }

    // Gear logic: Reverse has priority, then handbrake determines Drive/Park
    unsigned char currentGear;
    if (digitalRead(REVERSE_SWITCH_PIN) == LOW) {
        currentGear = 0x07; // Reverse
        sendGear(currentGear, "Reverse");
    } else if (!handbrakeOn) {
        currentGear = 0x09; // Drive
        sendGear(currentGear, "Drive");
    } else {
        currentGear = 0x08; // Park
        sendGear(currentGear, "Park");
    }

    // Send EGS messages at 20ms interval
    sendEGS(currentGear);

    // Update LED based on current gear
    updateLED(currentGear);
}

// Send 0x230 - Shifter position (every 10ms)
void sendGear(unsigned char gear, const char* gearName) {
    unsigned long currentTime = millis();

    if (currentTime - lastGearTime >= interval10ms) {
        unsigned char data[1] = {gear};
        CAN.sendMsgBuf(0x230, 0, 1, data);

        if (lastGearSent != gear) {
            Serial.print("Sent: ");
            Serial.println(gearName);
            lastGearSent = gear;
        }

        lastGearTime = currentTime;
    }
}

// Send EGS messages: 0x218, 0x338, 0x418 (every 20ms)
void sendEGS(unsigned char gear) {
    unsigned long currentTime = millis();

    if (currentTime - lastEGSTime >= interval20ms) {
        static bool toggle = false;
        
        // =================================================================
        // 0x338 - Transmission speeds
        // Bytes 0-1: Output shaft speed (0xFFFF = not available)
        // Byte 2: Status flags including toggle bit
        // Bytes 6-7: Turbine speed (0x0000 = not available)
        // =================================================================
        unsigned char data338[8] = {0xFF, 0xFF, 0x1F, 0xFF, 0x00, 0xFF, 0x00, 0x00};
        CAN.sendMsgBuf(0x338, 0, 8, data338);

        // =================================================================
        // 0x218 - Main EGS status (critical for preventing torque limiting)
        // =================================================================
        unsigned char data218[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        
        // Byte 0: Toggle bit at MSB (alternates 0x00/0x80 every 40ms)
        data218[0] = toggle ? 0x80 : 0x00;
        
        // Byte 1: Engine torque request (0 = no request)
        data218[1] = 0x00;
        
        // Byte 2: Gear codes (GIC/GZC - actual/target gear)
        // Byte 4: Status flags (GET_OK, ALF, KS)
        // Byte 5: Parity (matches toggle)
        unsigned char gicGzc;
        unsigned char statusByte4;
        unsigned char statusByte5;
        switch (gear) {
            case 0x07: // Reverse
                gicGzc = 0xBB;       // Reverse gear code
                statusByte4 = 0x03;  // Gearbox OK + Ready to move
                statusByte5 = toggle ? 0x80 : 0x00;
                break;
            case 0x08: // Park
                gicGzc = 0xDD;       // Park gear code
                statusByte4 = 0x23;  // Gearbox OK + Starter enabled
                statusByte5 = toggle ? 0x80 : 0x00;
                break;
            case 0x09: // Drive
            default:
                gicGzc = 0x11;       // 1st gear code
                statusByte4 = 0x03;  // Gearbox OK + Ready to move
                statusByte5 = toggle ? 0x80 : 0x00;
                break;
        }
        data218[2] = gicGzc;
        
        // Byte 3: Converter/shift status flags
        data218[3] = 0x48;
        
        // Byte 4: Critical status - GET_OK must be set, GS_NOTL must be clear
        data218[4] = statusByte4;
        
        // Byte 5: Parity bit (follows toggle)
        data218[5] = statusByte5;
        
        // Bytes 6 & 7: CALID/CVN sequence from real NAG52 capture
        // Pattern repeats every 20 frames:
        //   Frames 0-3:   Byte6=0x00, Byte7=0x30-0x33 (CALID header)
        //   Frames 4-19:  Byte6=CALID_DATA, Byte7=0x20-0x2F (CALID data)
        unsigned int seqPos = frameCounter % 20;
        if (seqPos < 4) {
            // CALID header phase
            data218[6] = 0x00;
            data218[7] = 0x30 + seqPos;
        } else {
            // CALID data phase
            data218[6] = CALID_DATA[seqPos - 4];
            data218[7] = 0x20 + (seqPos - 4);
        }
        frameCounter++;
        
        CAN.sendMsgBuf(0x218, 0, 8, data218);

        // =================================================================
        // 0x418 - Gear display and wheel torque
        // =================================================================
        unsigned char data418[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        
        // Byte 0: Display character (P/R/D)
        switch (gear) {
            case 0x07: data418[0] = 0x52; break;  // 'R'
            case 0x08: data418[0] = 0x50; break;  // 'P'
            case 0x09: data418[0] = 0x44; break;  // 'D'
            default:   data418[0] = 0x44; break;
        }
        
        // Byte 1: Driving program ('S' = Sport)
        data418[1] = 0x53;
        
        // Byte 2: Oil temperature (0x00 = cold/startup)
        data418[2] = 0x00;
        
        // Byte 3: Transmission type flags
        data418[3] = 0x04;
        
        // Byte 4: Gear codes (same format as 0x218)
        switch (gear) {
            case 0x07: data418[4] = 0xBB; break;  // Reverse
            case 0x08: data418[4] = 0xDD; break;  // Park
            case 0x09: data418[4] = 0x11; break;  // 1st gear
            default:   data418[4] = 0x11; break;
        }
        
        // Byte 5: Loss torque
        switch (gear) {
            case 0x07: data418[5] = 0x00; break;
            case 0x08: data418[5] = 0x00; break;
            case 0x09: data418[5] = 0x14; break;
            default:   data418[5] = 0x00; break;
        }
        
        // Byte 6: Toggle + wheel torque factor
        unsigned char byte6Base;
        switch (gear) {
            case 0x07: byte6Base = 0x08; break;  // Reverse
            case 0x08: byte6Base = 0x00; break;  // Park
            case 0x09: byte6Base = 0x61; break;  // Drive
            default:   byte6Base = 0x00; break;
        }
        if (gear == 0x09) {
            data418[6] = toggle ? (byte6Base | 0x40) : byte6Base;
        } else {
            data418[6] = toggle ? (byte6Base | 0xC0) : byte6Base;
        }
        
        // Byte 7: Wheel torque factor upper bits
        switch (gear) {
            case 0x07: data418[7] = 0x00; break;
            case 0x08: data418[7] = 0x00; break;
            case 0x09: data418[7] = 0xB1; break;
            default:   data418[7] = 0x00; break;
        }
        
        CAN.sendMsgBuf(0x418, 0, 8, data418);

        // Toggle flips every 40ms (every other 20ms frame)
        if (timeToToggle) {
            toggle = !toggle;
        }
        timeToToggle = !timeToToggle;
        
        // Increment CVN counter (5-bit, wraps at 32)
        cvnCounter = (cvnCounter + 1) & 0x1F;
        
        lastEGSTime = currentTime;
    }
}

// Read handbrake status from CAN 0x240
void handleHandbrake() {
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        CAN.readMsgBuf(&len, buf);
        unsigned long canId = CAN.getCanId();

        if (canId == 0x240 && len == 8) {
            // Byte 4, Bit 4 = handbrake status
            bool currentHandbrakeState = (buf[4] & 0x10);

            if (currentHandbrakeState != lastHandbrakeState) {
                if (currentHandbrakeState) {
                    Serial.println("Handbrake is ON");
                } else {
                    Serial.println("Handbrake is OFF");
                }
                lastHandbrakeState = currentHandbrakeState;
            }

            handbrakeOn = currentHandbrakeState;
        }
    }
}

// LED indicator: Solid = Park, Off = Drive, Pulsing = Reverse
void updateLED(unsigned char gear) {
    unsigned long currentTime = millis();

    if (gear == 0x08) {
        digitalWrite(LED_PIN, HIGH);
    } else if (gear == 0x09) {
        digitalWrite(LED_PIN, LOW);
    } else if (gear == 0x07) {
        if (currentTime - lastLEDToggleTime >= pulseInterval) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
            lastLEDToggleTime = currentTime;
        }
    }
}
