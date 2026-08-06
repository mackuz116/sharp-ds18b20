/*
 =====================================================================================
           STEROWNIK WZMACNIACZA A-136 (RAD FAN + TRAFO PROTECT + SELECTOR + IR)
 =====================================================================================
 Hardware: Arduino Pro Mini (5V / 16MHz) + Moduł TM1638 (HW-154) + Odbiornik IR
 Zasilanie: Małe trafko Standby (Arduino działa non-stop)
 -------------------------------------------------------------------------------------
 PEŁNA MAPA POŁĄCZEŃ PINÓW (HARDWARE PINOUT):
 -------------------------------------------------------------------------------------
 Pin Arduino   | Funkcja / Moduł                 | Uwagi / Opis
 -------------+---------------------------------+-------------------------------------
 D0 (RX)      | Serial Monitor (Programowanie)  | Narzędzie diagnostyczne (9600 baud)
 D1 (TX)      | Serial Monitor (Programowanie)  | Narzędzie diagnostyczne (9600 baud)
 D2           | Wentylator 1 - TACHO (RPM)      | Przerwanie sprzętowe INT0
 D3           | WOLNY                           | 
 D4           | Selektor Przekaźników - CH1     | Wyjście cyfrowe Break-Before-Make
 D5           | Odbiornik IR (DATA / OUT)       | Odbiornik IR (np. VS1838B)
 D6           | Moduł HW-154 - DIO              | Linia danych TM1638
 D7           | Selektor Przekaźników - CH2     | Wyjście cyfrowe Break-Before-Make
 D8           | Czujniki Temp. DS18B20 (1-Wire) | Radiator (Czujnik 1) + Trafo (Czujnik 2)
 D9           | Wentylator 1 - Sterowanie PWM   | Wyjście PWM (Timer 1)
 D10          | PRZEKAŹNIK ZASILANIA GŁÓWNEGO   | High = ON (Wzmacniacz włączony)
 D11          | Moduł HW-154 - STB (Strobe)     | Linia wyboru układu TM1638
 D12          | Moduł HW-154 - CLK (Clock)      | Linia zegarowa TM1638
 D13          | Selektor Przekaźników - CH8     | Wyjście cyfrowe GPIO
 A0           | Selektor Przekaźników - CH3     | Wyjście cyfrowe GPIO
 A1           | Selektor Przekaźników - CH4     | Wyjście cyfrowe GPIO
 A2           | Selektor Przekaźników - CH5     | Wyjście cyfrowe GPIO
 A3           | Sygnalizacja Alarmu - Wentylator| Wyjście sygnału awarii wentylatora
 A4           | Selektor Przekaźników - CH6     | Wyjście cyfrowe GPIO
 A5           | Selektor Przekaźników - CH7     | Wyjście cyfrowe GPIO
 A6 / A7      | WOLNE                           | Dostępne tylko jako wejścia ADC
 -------------------------------------------------------------------------------------
 AUTOMATYCZNY SKRYPT GIT (Uruchamiany w terminalu Linuxa w folderze projektu):
 -------------------------------------------------------------------------------------
 inotifywait -m -e close_write,modify,moved_to . | while read -r path action file; do
     if [[ "$file" == *.ino ]]; then
         git add .
         if git commit -m "Auto-save: $file $(date '+%Y-%m-%d %H:%M:%S')"; then
             git push
             echo "✔ [GIT] Zapisano i wysłano (push) dla: $file"
         fi
     fi
 done
 =====================================================================================
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <TM1638plus.h>
#include <IRremote.hpp>

// ===================================================================================
//             ZWERYFIKOWANE KODY TWOJEGO PILOTA IR (PROTOKÓŁ NEC)
// ===================================================================================
const uint32_t IR_CODE_POWER = 0x45; // Przycisk Power na pilocie

const uint32_t IR_CODE_CH1   = 0x0C; // Przycisk 1 (CD)
const uint32_t IR_CODE_CH2   = 0x18; // Przycisk 2 (DAC)
const uint32_t IR_CODE_CH3   = 0x5E; // Przycisk 3 (TUBE)
const uint32_t IR_CODE_CH4   = 0x08; // Przycisk 4 (AUX1)
const uint32_t IR_CODE_CH5   = 0x1C; // Przycisk 5 (AUX2)
const uint32_t IR_CODE_CH6   = 0x5A; // Przycisk 6 (TAPE)
const uint32_t IR_CODE_CH7   = 0x42; // Przycisk 7 (PHON)
const uint32_t IR_CODE_CH8   = 0x52; // Przycisk 8 (TUNR)

const uint32_t IR_CODE_NEXT  = 0x15; // Przycisk Ch+ / Strzałka w prawo
const uint32_t IR_CODE_PREV  = 0x07; // Przycisk Ch- / Strzałka w lewo
// ===================================================================================

// Nazwy wejść wyświetlane na panelu (max 4 znaki)
const char* channelNames[8] = {
    "CD  ",   // CH 1
    "DAC ",   // CH 2
    "TUBE",   // CH 3
    "AUX1",   // CH 4
    "AUX2",   // CH 5
    "TAPE",   // CH 6
    "PHON",   // CH 7
    "TUNR"    // CH 8
};

const int EEPROM_ADDR_CHANNEL = 0;

// --- MODUŁ HW-154 ---
#define STROBE_TM 11
#define CLOCK_TM  12
#define DIO_TM    6   

TM1638plus tm(STROBE_TM, CLOCK_TM, DIO_TM, false);

// --- ODBIORNIK IR ---
#define IR_RECEIVE_PIN 5

// --- STEROWANIE ZASILANIEM GŁÓWNYM ---
#define POWER_RELAY_PIN 10 // Pin sterujący przekaźnikiem zasilania 230V
bool powerState = false;   // false = Standby (OFF), true = Praca (ON)
bool waitButtonsRelease = false;

// --- CZUJNIKI TEMPERATURY (DS18B20) ---
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Adresy czujników na magistrali 1-Wire
DeviceAddress radSensor   = { 0x28, 0xFF, 0x20, 0x64, 0x68, 0x14, 0x02, 0xC5 }; // Czujnik 1: Radiator
DeviceAddress trafoSensor = { 0x28, 0x3A, 0xE0, 0xAC, 0x08, 0x00, 0x00, 0x78 }; // Czujnik 2: Trafo

// Nastawy temperaturowe dla radiatora (Wentylator)
const float RAD_TEMP_START = 40.0; 
const float RAD_TEMP_MAX   = 70.0; 
const int PWM_MIN          = 45; 

// Nastawa krytyczna dla Trafa (Wyłączenie wzmacniacza)
const float TRAFO_MAX_TEMP = 85.0; 

const int FAN_PWM_PIN   = 9;  
const int FAN_TACHO_PIN = 2;   
const int FAN_ALARM_PIN = A3;

volatile int tachoCount = 0;
unsigned long lastTempCheck = 0;
const unsigned long tempInterval = 1000;

bool fanAlarmState = false;
bool trafoOverheatState = false;

void countTacho() { tachoCount++; }

// --- SELEKTOR WEJŚĆ ---
const int selectorPins[8] = {4, 7, A0, A1, A2, A4, A5, 13}; // CH1..CH8
int currentChannel = 0;
float currentRadTemp = 0.0;
float currentTrafoTemp = 0.0;
bool showTrafoTempOnDisplay = false;

void updateDisplay() {
    if (!powerState) return; 

    char displayBuffer[9];

    if (fanAlarmState) {
        snprintf(displayBuffer, sizeof(displayBuffer), "%-4sERR FAN", channelNames[currentChannel]);
    } else if (trafoOverheatState) {
        snprintf(displayBuffer, sizeof(displayBuffer), "HOT TRF!");
    } else {
        // Wyświetlanie na przemian temperatury Radiatora ("r") i Trafa ("t")
        if (showTrafoTempOnDisplay) {
            int displayTemp = (currentTrafoTemp > 0 && currentTrafoTemp < 125) ? (int)currentTrafoTemp : 0;
            snprintf(displayBuffer, sizeof(displayBuffer), "%-4st%2dC", channelNames[currentChannel], displayTemp);
        } else {
            int displayTemp = (currentRadTemp > 0 && currentRadTemp < 125) ? (int)currentRadTemp : 0;
            snprintf(displayBuffer, sizeof(displayBuffer), "%-4sr%2dC", channelNames[currentChannel], displayTemp);
        }
    }

    tm.displayText(displayBuffer);
}

void selectChannel(int channel) {
    if (!powerState) return; 

    if (channel < 0) channel = 7;
    if (channel > 7) channel = 0;

    currentChannel = channel;

    // 1. Break-Before-Make
    for (int i = 0; i < 8; i++) {
        digitalWrite(selectorPins[i], LOW);
    }
    delay(2); 

    // 2. Załączenie wybranego przekaźnika
    digitalWrite(selectorPins[currentChannel], HIGH);

    // 3. Zapis w EEPROM
    EEPROM.update(EEPROM_ADDR_CHANNEL, currentChannel);

    // 4. Wskaźniki LED na panelu
    for (int i = 0; i < 8; i++) {
        tm.setLED(i, (i == currentChannel) ? 1 : 0);
    }

    updateDisplay();

    Serial.print(">>> AKTYWNE WEJŚCIE: CH ");
    Serial.print(currentChannel + 1);
    Serial.print(" [");
    Serial.print(channelNames[currentChannel]);
    Serial.println("]");
}

void setPower(bool turnOn) {
    powerState = turnOn;

    if (powerState) {
        // --- WŁĄCZANIE WZMACNIACZA (POWER ON) ---
        digitalWrite(POWER_RELAY_PIN, HIGH); 
        
        tm.displayBegin();
        tm.reset();
        tm.brightness(3);

        tm.displayText("A-136 OK");
        for (int i = 0; i < 8; i++) {
            tm.setLED(i, 1);
            delay(50);
            tm.setLED(i, 0);
        }
        delay(200);

        byte savedChannel = EEPROM.read(EEPROM_ADDR_CHANNEL);
        if (savedChannel > 7) savedChannel = 0;
        selectChannel(savedChannel);

        Serial.println(">>> WZMACNIACZ WŁĄCZONY [POWER ON]");
    } else {
        // --- WYŁĄCZANIE WZMACNIACZA (STANDBY / OFF) ---
        for (int i = 0; i < 8; i++) {
            digitalWrite(selectorPins[i], LOW);
        }

        digitalWrite(POWER_RELAY_PIN, LOW);

        analogWrite(FAN_PWM_PIN, 0);
        digitalWrite(FAN_ALARM_PIN, LOW);
        fanAlarmState = false;

        tm.reset();
        waitButtonsRelease = true;

        Serial.println(">>> WZMACNIACZ WYŁĄCZONY [STANDBY]");
    }
}

// --- OBSŁUGA PILOTA IR ---
void handleIR() {
    if (IrReceiver.decode()) {
        if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
            uint32_t irCommand = IrReceiver.decodedIRData.command;

            if (irCommand != 0) {
                Serial.print("[IR DETECT] Komenda HEX: 0x");
                Serial.print(irCommand, HEX);

                if (irCommand == IR_CODE_POWER) {
                    Serial.println(" -> Akcja: [POWER TOGGLE]");
                    setPower(!powerState);
                } 
                else if (powerState) {
                    if (irCommand == IR_CODE_CH1)      selectChannel(0);
                    else if (irCommand == IR_CODE_CH2) selectChannel(1);
                    else if (irCommand == IR_CODE_CH3) selectChannel(2);
                    else if (irCommand == IR_CODE_CH4) selectChannel(3);
                    else if (irCommand == IR_CODE_CH5) selectChannel(4);
                    else if (irCommand == IR_CODE_CH6) selectChannel(5);
                    else if (irCommand == IR_CODE_CH7) selectChannel(6);
                    else if (irCommand == IR_CODE_CH8) selectChannel(7);
                    else if (irCommand == IR_CODE_NEXT) selectChannel(currentChannel + 1);
                    else if (irCommand == IR_CODE_PREV) selectChannel(currentChannel - 1);
                }
            }
        }
        IrReceiver.resume();
    }
}

void setup() {
    Serial.begin(9600);
    sensors.begin();
    sensors.setWaitForConversion(false);

    pinMode(POWER_RELAY_PIN, OUTPUT);
    digitalWrite(POWER_RELAY_PIN, LOW);

    tm.displayBegin();
    tm.reset();

    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

    pinMode(FAN_PWM_PIN, OUTPUT);
    pinMode(FAN_ALARM_PIN, OUTPUT);
    pinMode(FAN_TACHO_PIN, INPUT_PULLUP);
    digitalWrite(FAN_ALARM_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(FAN_TACHO_PIN), countTacho, FALLING);

    for (int i = 0; i < 8; i++) {
        pinMode(selectorPins[i], OUTPUT);
        digitalWrite(selectorPins[i], LOW);
    }

    setPower(false);

    sensors.requestTemperatures();
    Serial.println("Sterownik A-136 gotowy (Tryb Standby z ochroną Trafa).");
    Serial.println("-------------------------------------------------------");
}

void loop() {
    // 1. OBSŁUGA PILOTA IR
    handleIR();

    // 2. PRZYCISKI PANELU HW-154
    uint8_t buttons = tm.readButtons();
    static unsigned long s1PressStartTime = 0;
    static bool s1LongPressTriggered = false;

    if (buttons == 0) {
        waitButtonsRelease = false;

        if (s1PressStartTime != 0 && !s1LongPressTriggered && powerState) {
            selectChannel(0);
        }

        s1PressStartTime = 0;
        s1LongPressTriggered = false;
    }

    if (!powerState) {
        if (buttons != 0 && !waitButtonsRelease) {
            setPower(true);
            waitButtonsRelease = true;
        }
    } else {
        if (buttons & 0x01) { 
            if (s1PressStartTime == 0) {
                s1PressStartTime = millis();
            } else if (!s1LongPressTriggered && (millis() - s1PressStartTime >= 1200)) {
                s1LongPressTriggered = true;
                setPower(false);
            }
        } 
        else if (buttons != 0) { 
            int pressedBit = -1;
            for (int i = 1; i < 8; i++) {
                if (buttons & (1 << i)) {
                    pressedBit = i;
                    break;
                }
            }

            if (pressedBit != -1 && pressedBit != currentChannel) {
                selectChannel(pressedBit);
                delay(200); 
            }
        }
    }

    // 3. OBSŁUGA KONTROLI TEMPERATURY I CHŁODZENIA (Tylko w trybie ON)
    if (powerState) {
        unsigned long currentMillis = millis();

        if (currentMillis - lastTempCheck >= tempInterval) {
            lastTempCheck = currentMillis;

            float tRad   = sensors.getTempC(radSensor);
            float tTrafo = sensors.getTempC(trafoSensor);
            sensors.requestTemperatures();

            currentRadTemp   = tRad;
            currentTrafoTemp = tTrafo;

            // --- ZABEZPIECZENIE TRAFA ---
            if (tTrafo >= TRAFO_MAX_TEMP && tTrafo < 125.0) {
                Serial.print("[ALARM KRYTYCZNY] Przegrzanie Trafa: ");
                Serial.print(tTrafo);
                Serial.println("°C! Awaryjne wyłączenie zasilania.");

                trafoOverheatState = true;
                setPower(false); // Natychmiastowe odcięcie zasilania 230V!
                return;
            } else {
                trafoOverheatState = false;
            }

            // --- STEROWANIE WENTYLATOREM RADIATORA ---
            int pwmValue = 0;

            noInterrupts();
            int rpm = (tachoCount / 2) * 60;
            tachoCount = 0;
            interrupts();

            if (tRad < RAD_TEMP_START) {
                pwmValue = 0;
            } else {
                float constrainedTemp = constrain(tRad, RAD_TEMP_START, RAD_TEMP_MAX);
                pwmValue = map(constrainedTemp, RAD_TEMP_START, RAD_TEMP_MAX, PWM_MIN, 255);
            }
            analogWrite(FAN_PWM_PIN, pwmValue);

            // Weryfikacja uszkodzenia wentylatora
            if (pwmValue > 0 && rpm == 0) {
                digitalWrite(FAN_ALARM_PIN, HIGH);
                fanAlarmState = true;
            } else {
                digitalWrite(FAN_ALARM_PIN, LOW);
                fanAlarmState = false;
            }

            // Raport diagnostyczny w Serial Monitorze
            Serial.print("[STAT] CH:"); Serial.print(currentChannel + 1);
            Serial.print(" ("); Serial.print(channelNames[currentChannel]); Serial.print(") | ");
            Serial.print("RAD: "); Serial.print(tRad, 1); Serial.print("°C (PWM:");
            Serial.print(map(pwmValue, 0, 255, 0, 100)); Serial.print("%, RPM:");
            Serial.print(rpm); Serial.print(") | ");
            Serial.print("TRAFO: "); Serial.print(tTrafo, 1); Serial.println("°C");

            // Przełączanie wskaźnika temp na wyświetlaczu (Radiator / Trafo)
            showTrafoTempOnDisplay = !showTrafoTempOnDisplay;
            updateDisplay();
        }
    }
}
