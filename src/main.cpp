// =============================================================================
// NoNag - EGS/NAG52 Emulator for Mercedes W211 Manual Swap
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
bool lastHandbrakeState = true; // To track state changes

// Gear tracking
unsigned char lastGearSent = 0x00; // Initialize to an invalid gear state

// EGS message counters
static unsigned char cvnCounter = 0;      // FEHLER/CVN counter (5-bit, 0-31)
static bool timeToToggle = false;         // Toggle control (40ms period)

// LED pulsing variables
unsigned long lastLEDToggleTime = 0;
const unsigned long pulseInterval = 500; // LED pulse interval in milliseconds
bool ledState = LOW;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("NoNag - EGS Emulator for W211 Manual Swap");
    Serial.println("=========================================");

    pinMode(REVERSE_SWITCH_PIN, INPUT_PULLUP); // Configure reverse switch as input with pull-up resistor
    pinMode(LED_PIN, OUTPUT); // Configure LED pin as output

    // Initialize CAN at 500kbps
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");

    // -----------------------------------------------------
    // 1) Configure masks (we use 0x7FF so that we care about all bits)
    // -----------------------------------------------------
    CAN.init_Mask(0, 0, 0x7FF);
    CAN.init_Mask(1, 0, 0x7FF);

    // -----------------------------------------------------
    // 2) Configure filters to accept only required ID's 0x240
    // -----------------------------------------------------
    CAN.init_Filt(0, 0, 0x240);
    //CAN.init_Filt(1, 0, 0x240);
    //CAN.init_Filt(2, 0, 0x240);
    }

void loop() {

    unsigned long currentTime = millis();

    // Check handbrake status every 20ms
    if (currentTime - lastHandbrakeCheckTime >= interval20ms) {
        handleHandbrake();
        lastHandbrakeCheckTime = currentTime; // Update the last time handbrake was checked
    }

    // Gear logic: Reverse has priority, then handbrake state determines Drive/Park
    unsigned char currentGear;
    if (digitalRead(REVERSE_SWITCH_PIN) == LOW) { // Reverse switch is pressed (highest priority)
        currentGear = 0x07; // Reverse
        sendGear(currentGear, "Reverse");
    } else if (!handbrakeOn) { // Handbrake is off
        currentGear = 0x09; // Drive
        sendGear(currentGear, "Drive");
    } else { // Handbrake is on
        currentGear = 0x08; // Park
        sendGear(currentGear, "Park");
    }

    // Send EGS messages at 20ms interval
    sendEGS(currentGear);

    // Update the LED behavior based on the current gear
    updateLED(currentGear);
}

void sendGear(unsigned char gear, const char* gearName) {
    unsigned long currentTime = millis();

	// Send the CAN message every 10ms (Match the ECU's expected rate)
    if (currentTime - lastGearTime >= interval10ms) {
        unsigned char data[1] = {gear};
        CAN.sendMsgBuf(0x230, 0, 1, data);

        // Print to Serial only if the gear has changed
        if (lastGearSent != gear) {
            Serial.print("Sent: ");
            Serial.println(gearName);
            lastGearSent = gear;
        }

        lastGearTime = currentTime; // Update the last time a message was sent
    }
}

// -----------------------------------------------------------------------------
// 0x218 (GS_218h) - Transmission status, torque requests
// 0x338 (GS_338h) - Shaft speeds (output & turbine)
// 0x418 (GS_418h) - Gear display, driving program, wheel torque factor
// -----------------------------------------------------------------------------

// Helper: Calculate even parity for a byte range
unsigned char calcEvenParity(unsigned char* data, int startBit, int numBits) {
    int ones = 0;
    for (int i = 0; i < numBits; i++) {
        int byteIdx = (startBit + i) / 8;
        int bitIdx = (startBit + i) % 8;
        if (data[byteIdx] & (1 << (7 - bitIdx))) ones++;
    }
    return (ones % 2) ? 1 : 0; // Return 1 if odd count (to make even parity)
}

void sendEGS(unsigned char gear) {
    unsigned long currentTime = millis();

    if (currentTime - lastEGSTime >= interval20ms) {
        static bool toggle = false;
        
        // =================================================================
        // 0x338 (GS_338h) - Transmission speeds
        // Bit 0-15:  NAB - Output shaft speed (0xFFFF = not available)
        // Bit 48-63: NTURBINE - Turbine speed (0xFFFF = not available)
        // =================================================================
        unsigned char data338[8] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};
        CAN.sendMsgBuf(0x338, 0, 8, data338);

        // =================================================================
        // 0x218 (GS_218h) - Main EGS status message
        // This is critical for preventing torque limiting!
        // =================================================================
        unsigned char data218[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        
        // Byte 0: Torque request control
        // Bit 0: MTGL_EGS - Toggle bit (40ms toggle, not 20ms!)
        // Bit 1: MMIN_EGS - Torque request min (0 = not requesting)
        // Bit 2: MMAX_EGS - Torque request max (0 = not requesting)
        // Bit 3-7: M_EGS MSB - Requested engine torque
        data218[0] = toggle ? 0x01 : 0x00;  // Only toggle bit in LSB position
        
        // Byte 1: M_EGS LSB - Requested engine torque (0 = no intervention)
        data218[1] = 0x00;
        
        // Byte 2: Gear information
        // Bit 16-19: GZC - Target gear 
        // Bit 20-23: GIC - Actual gear
        unsigned char gicGzc;
        unsigned char statusByte4;
        switch (gear) {
            case 0x07: // Reverse
                gicGzc = 0xBB;      // R=11 for both GZC and GIC
                statusByte4 = 0x53; // GET_OK=1, ALF=0, GS_NOTL=0, GSP_OK=1
                break;
            case 0x08: // Park
                gicGzc = 0xDD;      // P=13 for both GZC and GIC
                statusByte4 = 0x77; // GET_OK=1, ALF=1, GS_NOTL=0, GSP_OK=1
                break;
            case 0x09: // Drive
            default:
                gicGzc = 0x55;      // D5=5 for both (or use 0xFF)
                statusByte4 = 0x53; // GET_OK=1, ALF=0, GS_NOTL=0, GSP_OK=1
                break;
        }
        data218[2] = gicGzc;
        
        // Byte 3: Status flags
        // Bit 24: TORQUE_REQ_EN - Torque request enable (0)
        // Bit 25: SE - Schaltungseinleitung/shift initiation (0)
        // Bit 26: KICKDOWN (0)
        // Bit 27-29: Reserved
        // Bit 30: HSM - Hand switching mode (0)
        // Bit 31: SCHALT - Shifting (0)
        data218[3] = 0x48;  // From real EGS capture
        
        // Byte 4: Critical status flags
        // Bit 32: GET_OK - Gearbox OK (1) - CRITICAL!
        // Bit 33: KS - Ball start / garage shift (0)
        // Bit 34: ALF - Start enable (1 for Park, 0 for Drive/Reverse)
        // Bit 35: GS_NOTL - Emergency mode (0) - must be 0!
        // Bit 36: UEHITZ_GET - Overtemperature (0)
        // Bit 37: KD - Kickdown (0)
        // Bit 38: FPC_AAD LSB (0)
        // Bit 39: GSP_OK - Gearbox profile OK (1) - CRITICAL!
        data218[4] = statusByte4;
        
        // Byte 5: More status flags
        // Bit 40: FPC_AAD MSB (0)
        // Bit 41: DYN0_AMR_EGS (0)
        // Bit 42: DYN1_EGS (0)
        // Bit 43: MPAR_EGS - Torque parity (straight parity of torque request)
        // Bit 44-45: Reserved
        // Bit 46: MOT_NAUS_CNF - Confirm bit (0)
        // Bit 47: MOT_NAUS - Emergency shutdown (0) - must be 0!
        // Parity calculation: XOR of all bits in torque request (bytes 0-1)
        // Since we only have toggle bit set, parity = toggle
        data218[5] = toggle ? 0x08 : 0x00;  // MPAR_EGS at bit 3 of byte 5
        
        // Byte 6: MKRIECH - Creep torque
        // For EGS/manual, set to 0xFF (not applicable)
        data218[6] = 0xFF;
        
        // Byte 7: Error status and CVN counter (CRITICAL)
        // Bit 56-57: FEHLPRF_ST - Error check status
        //   0 = WAIT (not finished)
        //   1 = OK (finished, no errors) 
        //   2 = ERROR (error detected)
        //   3 = UNKNOWN
        // Bit 58: CALID_CVN_AKT - CVN transmission active (0)
        // Bit 59-63: FEHLER - Error/CVN counter (5-bit, increments each frame)
        // 
        // Value = (FEHLER << 3) | (CALID_CVN_AKT << 2) | FEHLPRF_ST
        // With FEHLPRF_ST=1 (OK), CALID_CVN_AKT=0, FEHLER=counter
        data218[7] = (cvnCounter << 3) | 0x01;  // 0x01 = FEHLPRF_ST OK
        
        CAN.sendMsgBuf(0x218, 0, 8, data218);

        // =================================================================
        // 0x418 (GS_418h) - Gear display and wheel torque
        // Structure from real EGS capture (DO NOT CHANGE byte order!)
        // =================================================================
        unsigned char data418[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        
        // Byte 0: FSC - Driving position display character
        switch (gear) {
            case 0x07: data418[0] = 0x52; break;  // 'R'
            case 0x08: data418[0] = 0x50; break;  // 'P'
            case 0x09: data418[0] = 0x44; break;  // 'D'
            default:   data418[0] = 0x44; break;
        }
        
        // Byte 1: FPC - Driving program character ('S' = Standard)
        data418[1] = 0x53;
        
        // Byte 2: T_GET - Transmission oil temp (0xFF = not available)
        data418[2] = 0xFF;
        
        // Byte 3: Flags
        data418[3] = 0x00;
        
        // Byte 4: GIC/GZC - Actual/target gear (same as 0x218 byte 2)
        switch (gear) {
            case 0x07: data418[4] = 0xBB; break;  // Reverse
            case 0x08: data418[4] = 0xDD; break;  // Park
            case 0x09: data418[4] = 0xFF; break;  // Drive
            default:   data418[4] = 0xFF; break;
        }
        
        // Byte 5: M_VERL - Loss torque (0xFF = not applicable)
        data418[5] = 0xFF;
        
        // Byte 6: Toggle and wheel torque factor (from real EGS capture)
        // Pattern: base value when toggle=1, base+0x40 when toggle=0
        unsigned char byte6Base;
        switch (gear) {
            case 0x07: byte6Base = 0x4F; break;  // Reverse
            case 0x08: byte6Base = 0x47; break;  // Park
            case 0x09: byte6Base = 0x67; break;  // Drive
            default:   byte6Base = 0x67; break;
        }
        data418[6] = toggle ? byte6Base : (byte6Base | 0x40);
        
        // Byte 7: Always 0xFF
        data418[7] = 0xFF;
        
        CAN.sendMsgBuf(0x418, 0, 8, data418);

        // Update toggle for next cycle (40ms period, flip every other 20ms frame)
        if (timeToToggle) {
            toggle = !toggle;
        }
        timeToToggle = !timeToToggle;
        
        // Increment CVN counter (5-bit, wraps at 32)
        cvnCounter = (cvnCounter + 1) & 0x1F;
        
        lastEGSTime = currentTime;
    }
}

void handleHandbrake() {
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive()) { // Check for incoming CAN messages
        CAN.readMsgBuf(&len, buf); // Read the data
        unsigned long canId = CAN.getCanId();

        // Because of our filter, we *should* only ever see 0x240 here:
        if (canId == 0x240 && len == 8) {
            // Handbrake detection: Byte 4, Bit 4 (mask 0x10)
            // Note: DBC labels this as KL_31B (wiper), but real capture shows it's handbrake
            // Bit 3 (0x08) is actually wiper outside park position
            // Using bit mask so wipers don't interfere with handbrake detection
            bool currentHandbrakeState = (buf[4] & 0x10); // Bit 4 = handbrake ON

            if (currentHandbrakeState != lastHandbrakeState) { // Only print if state changes
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

void updateLED(unsigned char gear) {
    unsigned long currentTime = millis();

    if (gear == 0x08) { // Park
        digitalWrite(LED_PIN, HIGH); // Turn LED on
    } else if (gear == 0x09) { // Drive
        digitalWrite(LED_PIN, LOW); // Turn LED off
    } else if (gear == 0x07) { // Reverse
        // Pulse the LED (normal speed)
        if (currentTime - lastLEDToggleTime >= pulseInterval) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
            lastLEDToggleTime = currentTime;
        }
    }
}

