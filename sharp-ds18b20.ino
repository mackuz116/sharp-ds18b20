/*
 =====================================================================================
           STEROWNIK WZMACNIACZA A-136 (CHŁODZENIE + SELEKTOR 8 CH + HW-154 + IR)
 =====================================================================================
 Hardware: Arduino Pro Mini (5V / 16MHz) + Moduł TM1638 (HW-154) + Odbiornik IR
 -------------------------------------------------------------------------------------
 PEŁNA MAPA POŁĄCZEŃ PINÓW (HARDWARE PINOUT):
 -------------------------------------------------------------------------------------
 Pin Arduino   | Funkcja / Moduł                 | Uwagi / Opis
 -------------+---------------------------------+-------------------------------------
 D0 (RX)      | Serial Monitor (Programowanie)  | Narzędzie diagnostyczne (9600 baud)
 D1 (TX)      | Serial Monitor (Programowanie)  | Narzędzie diagnostyczne (9600 baud)
 D2           | Wentylator 1 - TACHO (RPM)      | Przerwanie sprzętowe INT0
 D3           | Wentylator 2 - TACHO (RPM)      | Przerwanie sprzętowe INT1
 D4           | Selektor Przekaźników - CH1     | Wyjście cyfrowe Break-Before-Make
 D5           | Sygnalizacja Alarmu - Went. 1+2 | Zsumowane wyjście alarmowe (LED)
 D6           | Moduł HW-154 - DIO              | Linia danych TM1638 (Podpięta do D6)
 D7           | Selektor Przekaźników - CH2     | Wyjście cyfrowe Break-Before-Make
 D8           | Czujniki Temp. DS18B20 (1-Wire) | Magistrala OneWire (F1 i F2)
 D9           | Wentylator 1 - Sterowanie PWM   | Wyjście PWM (Timer 1)
 D10          | Wentylator 2 - Sterowanie PWM   | Wyjście PWM (Timer 1)
 D11          | Moduł HW-154 - STB (Strobe)     | Linia wyboru układu TM1638
 D12          | Moduł HW-154 - CLK (Clock)      | Linia zegarowa TM1638
 D13          | Odbiornik IR (DATA / OUT)       | Odbiornik IR (np. VS1838B)
 A0           | Selektor Przekaźników - CH3     | Wyjście cyfrowe GPIO
 A1           | Selektor Przekaźników - CH4     | Wyjście cyfrowe GPIO
 A2           | Selektor Przekaźników - CH5     | Wyjście cyfrowe GPIO
 A3           | Selektor Przekaźników - CH6     | Wyjście cyfrowe GPIO
 A4           | Selektor Przekaźników - CH7     | Wyjście cyfrowe GPIO
 A5           | Selektor Przekaźników - CH8     | Wyjście cyfrowe GPIO
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
//             DEFINICJE KODÓW PILOTA IR (PODMIEŃ WARTOŚCI NA SWOJE)
// ===================================================================================
const uint32_t IR_CODE_CH1  = 0x0C; // Przycisk 1 (CD)
const uint32_t IR_CODE_CH2  = 0x18; // Przycisk 2 (DAC)
const uint32_t IR_CODE_CH3  = 0x5E; // Przycisk 3 (TUBE)
const uint32_t IR_CODE_CH4  = 0x08; // Przycisk 4 (AUX1)
const uint32_t IR_CODE_CH5  = 0x1C; // Przycisk 5 (AUX2)
const uint32_t IR_CODE_CH6  = 0x5A; // Przycisk 6 (TAPE)
const uint32_t IR_CODE_CH7  = 0x42; // Przycisk 7 (PHON)
const uint32_t IR_CODE_CH8  = 0x52; // Przycisk 8 (TUNR)

const uint32_t IR_CODE_NEXT = 0x09; // Przycisk CH+ / Strzałka w prawo
const uint32_t IR_CODE_PREV = 0x15; // Przycisk CH- / Strzałka w lewo
// ===================================================================================

// Nazwy wejść wyświetlane na panelu (max 4 znaki)
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

const int EEPROM_ADDR_CHANNEL = 0;

// --- MODUŁ HW-154 ---
#define STROBE_TM 11
#define CLOCK_TM  12
#define DIO_TM    6   

TM1638plus tm(STROBE_TM, CLOCK_TM, DIO_TM, false);

// --- ODBIORNIK IR ---
#define IR_RECEIVE_PIN 13

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
const int COMMON_ALARM_PIN = 5;

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

    // 3. Zapis do nieulotnej pamięci EEPROM
    EEPROM.update(EEPROM_ADDR_CHANNEL, currentChannel);

    // 4. Załączenie odpowiedniej diody LED na HW-154
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

// --- OBSŁUGA KOMEND PILOTA IR ---
void handleIR() {
    if (IrReceiver.decode()) {
        // Ignorujemy powtórzenia przytrzymanego przycisku (Repeat Code)
        if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
            uint32_t irCommand = IrReceiver.decodedIRData.command;

            if (irCommand != 0) {
                Serial.print("[IR] Odebrano komendę HEX: 0x");
                Serial.println(irCommand, HEX);

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
        IrReceiver.resume(); // Odblokowanie odbiornika na kolejny sygnał
    }
}

void setup() {
    Serial.begin(9600);
    sensors.begin();
    sensors.setWaitForConversion(false);

    // Inicjalizacja odbiornika IR
    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

    // Inicjalizacja wyświetlacza HW-154
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

    // Konfiguracja wyjść PWM, TACHO oraz wspólnego Alarmu
    pinMode(COMMON_ALARM_PIN, OUTPUT);
    digitalWrite(COMMON_ALARM_PIN, LOW);

    for (int i = 0; i < 2; i++) {
        pinMode(pwmPins[i], OUTPUT);

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

    // Odczyt zapamiętanego kanału z EEPROM
    byte savedChannel = EEPROM.read(EEPROM_ADDR_CHANNEL);
    if (savedChannel > 7) savedChannel = 0;
    selectChannel(savedChannel);

    sensors.requestTemperatures();
    Serial.println("Sterownik A-136 gotowy do pracy (z obsługą IR).");
    Serial.println("-------------------------------------------------------");
}

void loop() {
    // 1. OBSŁUGA PILOTA IR (NATYCHMIASTOWA)
    handleIR();

    // 2. OBSŁUGA PRZYCISKÓW H-154 (S1-S8)
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
            delay(200); // Antydrabik
        }
    }

    // 3. CHŁODZENIE, DIAGNOSTYKA SERIAL I EKRAN (Co 1000 ms)
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

        // Przeliczenie sterowania i detekcja awarii
        for (int i = 0; i < 2; i++) {
            if (fanEnabled[i]) {
                if (temps[i] < TEMP_START) {
                    pwmValues[i] = 0;
                } else {
                    float constrainedTemp = constrain(temps[i], TEMP_START, TEMP_MAX);
                    pwmValues[i] = map(constrainedTemp, TEMP_START, TEMP_MAX, PWM_MIN, 255);
                }
                analogWrite(pwmPins[i], pwmValues[i]);

                // Weryfikacja awarii
                if (pwmValues[i] > 0 && rpm[i] == 0) {
                    fanAlarmState[i] = true;
                } else {
                    fanAlarmState[i] = false;
                }
            }
        }

        // Aktywacja wspólnej linii alarmu D5, jeśli którykolwiek wentylator zgłasza błąd
        if (fanAlarmState[0] || fanAlarmState[1]) {
            digitalWrite(COMMON_ALARM_PIN, HIGH);
        } else {
            digitalWrite(COMMON_ALARM_PIN, LOW);
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
