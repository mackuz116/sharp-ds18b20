/*
 =====================================================================================
           STEROWNIK WZMACNIACZA A-136 (CHŁODZENIE + SELEKTOR 8 CH + HW-154)
 =====================================================================================
 Hardware: Arduino Pro Mini (5V / 16MHz) + Moduł HW-154 (TM1638)
 =====================================================================================
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <TM1638plus.h>

// Nazwy wejść wyświetlane na panelu (max 4 znaki, np. "CD  ", "DAC ", "TUBE")
const char* channelNames[8] = {
    "CD  ",   // CH 1
    "DAC ",   // CH 2
    "TUBE",   // CH 3 (przedwzmacniacz lampowy)
    "AUX1",   // CH 4
    "AUX2",   // CH 5
    "TAPE",   // CH 6
    "PHON",   // CH 7
    "TUNR"    // CH 8
};
/ss

const int EEPROM_ADDR_CHANNEL = 0;

// --- MODUŁ HW-154 (NOWA ROZPISKA) ---
#define STROBE_TM 11
#define CLOCK_TM  12
#define DIO_TM    6   // Pin 6 (zamiast Pinu 13 z diodą L)

TM1638plus tm(STROBE_TM, CLOCK_TM, DIO_TM, false);

// --- CZUJNIKI TEMPERATURY I CHŁODZENIE ---
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress fan1Sensor = { 0x28, 0xFF, 0x20, 0x64, 0x68, 0x14, 0x02, 0xC5 }; 
DeviceAddress fan2Sensor = { 0x28, 0x3A, 0xE0, 0xAC, 0x08, 0x00, 0x00, 0x78 }; 

const float TEMP_START = 40.0; 
const float TEMP_MAX   = 70.0; 
const int PWM_MIN      = 45; 

const int pwmPins[2]   = {9, 10};  
const int tachoPins[2] = {2, 3};   
const int alarmPins[2] = {5, 13}; // Pin 13 jako fizyczny wyjście alarmowe LED

bool fanEnabled[2]   = {true, true};
const int fanType[2] = {1, 1};      

volatile int tachoCount[2] = {0, 0};
unsigned long lastTempCheck = 0;
const unsigned long tempInterval = 1000;

// Flagi alarmowe dla wentylatorów
bool fanAlarmState[2] = {false, false};

void countTacho0() { tachoCount[0]++; }
void countTacho1() { tachoCount[1]++; }

// --- SELEKTOR WEJŚĆ ---
const int selectorPins[8] = {4, 7, A0, A1, A2, A3, A4, A5};
int currentChannel = 0;
float currentTempToDisplay = 0.0;

void updateDisplay() {
    char displayBuffer[9];

    // Priorytet: Wyświetlanie błędu awarii wentylatora na LCD
    if (fanAlarmState[0] && fanAlarmState[1]) {
        snprintf(displayBuffer, sizeof(displayBuffer), "%-4sERR ALL", channelNames[currentChannel]);
    } else if (fanAlarmState[0]) {
        snprintf(displayBuffer, sizeof(displayBuffer), "%-4sERR F1", channelNames[currentChannel]);
    } else if (fanAlarmState[1]) {
        snprintf(displayBuffer, sizeof(displayBuffer), "%-4sERR F2", channelNames[currentChannel]);
    } else {
        // Normalny tryb wyświetlania (Nazwa + Temp)
        int displayTemp = (currentTempToDisplay > 0 && currentTempToDisplay < 125) ? (int)currentTempToDisplay : 0;
        snprintf(displayBuffer, sizeof(displayBuffer), "%-4s%2d C", channelNames[currentChannel], displayTemp);
    }

    tm.displayText(displayBuffer);
}

void selectChannel(int channel) {
    if (channel < 0) channel = 7;
    if (channel > 7) channel = 0;

    currentChannel = channel;

    // 1. Break-Before-Make
    for (int i = 0; i < 8; i++) {
        digitalWrite(selectorPins[i], LOW);
    }
    delay(2); 

    // 2. Załączenie przekaźnika
    digitalWrite(selectorPins[currentChannel], HIGH);

    // 3. EEPROM
    EEPROM.update(EEPROM_ADDR_CHANNEL, currentChannel);

    // 4. Diody LED na HW-154
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

void setup() {
    Serial.begin(9600);
    sensors.begin();
    sensors.setWaitForConversion(false);

    // Inicjalizacja HW-154
    tm.displayBegin();
    tm.reset();
    tm.brightness(3); 

    // Sekwencja startowa
    tm.displayText("A-136 OK");
    for (int i = 0; i < 8; i++) {
        tm.setLED(i, 1);
        delay(60);
        tm.setLED(i, 0);
    }
    delay(400);

    // Konfiguracja chłodzenia
    for (int i = 0; i < 2; i++) {
        pinMode(pwmPins[i], OUTPUT);
        pinMode(alarmPins[i], OUTPUT);
        digitalWrite(alarmPins[i], LOW);

        if (fanType[i] == 1) {
            pinMode(tachoPins[i], INPUT_PULLUP);
        }
    }

    if (fanType[0] == 1) attachInterrupt(digitalPinToInterrupt(tachoPins[0]), countTacho0, FALLING);
    if (fanType[1] == 1) attachInterrupt(digitalPinToInterrupt(tachoPins[1]), countTacho1, FALLING);

    // Konfiguracja wyjść przekaźników
    for (int i = 0; i < 8; i++) {
        pinMode(selectorPins[i], OUTPUT);
        digitalWrite(selectorPins[i], LOW);
    }

    // Odczyt zapamiętanego kanału
    byte savedChannel = EEPROM.read(EEPROM_ADDR_CHANNEL);
    if (savedChannel > 7) savedChannel = 0;
    selectChannel(savedChannel);

    sensors.requestTemperatures();
    Serial.println("Sterownik A-136 gotowy do pracy.");
    Serial.println("-------------------------------------------------------");
}

void loop() {
    // 1. NATYCHMIASTOWA OBSŁUGA PRZYCISKÓW H-154
    uint8_t buttons = tm.readButtons();

    if (buttons != 0) {
        int pressedBit = -1;
        for (int i = 0; i < 8; i++) {
            if (buttons & (1 << i)) {
                pressedBit = i;
                break;
            }
        }

        if (pressedBit != -1 && pressedBit != currentChannel) {
            selectChannel(pressedBit);
            delay(200); // Filtrowanie drgań styków
        }
    }

    // 2. CHŁODZENIE, DIAGNOSTYKA SERIAL I EKRAN (Co 1000 ms)
    unsigned long currentMillis = millis();
    static bool showSensor2 = false;

    if (currentMillis - lastTempCheck >= tempInterval) {
        lastTempCheck = currentMillis;

        float t1 = sensors.getTempC(fan1Sensor);
        float t2 = sensors.getTempC(fan2Sensor);

        sensors.requestTemperatures();

        float temps[2] = {t1, t2};
        int pwmValues[2] = {0, 0};

        noInterrupts();
        int rpm[2];
        rpm[0] = (tachoCount[0] / 2) * 60;
        rpm[1] = (tachoCount[1] / 2) * 60;
        tachoCount[0] = 0;
        tachoCount[1] = 0;
        interrupts();

        // Przeliczenie sterowania i detekcja alarmów
        for (int i = 0; i < 2; i++) {
            if (fanEnabled[i]) {
                if (temps[i] < TEMP_START) {
                    pwmValues[i] = 0;
                } else {
                    float constrainedTemp = constrain(temps[i], TEMP_START, TEMP_MAX);
                    pwmValues[i] = map(constrainedTemp, TEMP_START, TEMP_MAX, PWM_MIN, 255);
                }
                analogWrite(pwmPins[i], pwmValues[i]);

                // Weryfikacja awarii: Wysterowany PWM > 0, ale obroty RPM = 0
                if (pwmValues[i] > 0 && rpm[i] == 0) {
                    digitalWrite(alarmPins[i], HIGH);
                    fanAlarmState[i] = true;
                } else {
                    digitalWrite(alarmPins[i], LOW);
                    fanAlarmState[i] = false;
                }
            }
        }

        // --- PEŁNY RAPORT DIAGNOSTYCZNY W SERIAL MONITORZE ---
        Serial.print("[STAT] ");
        Serial.print("CH:"); Serial.print(currentChannel + 1);
        Serial.print(" ("); Serial.print(channelNames[currentChannel]); Serial.print(") | ");
        
        // Wentylator 1
        Serial.print("F1: "); Serial.print(temps[0], 1); Serial.print("°C, ");
        Serial.print("PWM: "); Serial.print(map(pwmValues[0], 0, 255, 0, 100)); Serial.print("%, ");
        Serial.print("RPM: "); Serial.print(rpm[0]); 
        if (fanAlarmState[0]) Serial.print(" [ALARM!] "); else Serial.print(" [OK] ");
        
        Serial.print("| ");

        // Wentylator 2
        Serial.print("F2: "); Serial.print(temps[1], 1); Serial.print("°C, ");
        Serial.print("PWM: "); Serial.print(map(pwmValues[1], 0, 255, 0, 100)); Serial.print("%, ");
        Serial.print("RPM: "); Serial.print(rpm[1]); 
        if (fanAlarmState[1]) Serial.println(" [ALARM!]"); else Serial.println(" [OK]");

        // Odświeżenie ekranu LCD
        currentTempToDisplay = showSensor2 ? t2 : t1;
        updateDisplay();

        showSensor2 = !showSensor2;
    }
}
