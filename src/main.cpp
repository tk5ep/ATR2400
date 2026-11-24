/*
ATR2400 controller board
by TK5EP
march 2025
For Arduino pro-mini 5V-16 MHz
    _  _____ ____  ____  _  _    ___   ___  
   / \|_   _|  _ \|___ \| || |  / _ \ / _ \ 
  / _ \ | | | |_) | __) | || |_| | | | | | |
 / ___ \| | |  _ < / __/|__   _| |_| | |_| |
/_/   \_\_| |_| \_\_____|  |_|  \___/ \___/ 
                                            
*/

#include <Arduino.h>
#include <DS18B20.h>
#include <TickerScheduler.h> //https://github.com/Toshik/TickerScheduler

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

//#define RXfreq 430.000
// The TX frequency determines the RX synthesizer frequency. If low band => supradyne, if high band => infradyne
// The duplex shift is determined by the TX synthesizer, 30.8 MHz for a 9.4 Mhz shift
// ex : TX 430.150 MHz, RX is 439.550 MHz VCO is 460.950 MHz (TX = 460.950 - 30.8 , RX = 460.950 - 21.4)
//      TX 439.950 MHz, RX is 430.150 MHz VCO is 408.750 MHz (TX = 408.750 + 30.8 , RX = 408.750 + 21.4)
// In simplex, the TX synthesizer has to be on 21.5 MHz and the RX synthesizer is shifted by 100 kHz during receive in order to avoid to receive the VCO

// comment out to verbose output
#define DEBUG
//#define DEBUG_SYNTHE  

// change these variables
double TXFREQ             = 430.175;
bool DUPLEX               = true;   // simplex=false duplex=true
uint16_t VREF =           1067;     // normaly 1100. Internal 1.1V reference, adjust this to read good voltage
#define TEMP_ENABLE                 // if a temperature sensor is connected
#define CW_ENABLE         1         // si une alarme CW doit être envoyée active = 1
#define BATT_LOW_LEVEL    12.0      // seuil d'alerte batterie (V)
#define TEMP_HI           26        // seuil déclenchement ventilo
#define TEMP_LO           25        //
#define TEMP_ALARM_LEVEL  28.0      // Seuil d'alerte surchauffe pour balise Morse (°C)
#if CW_ENABLE
    // Configuration CW (Morse)
    #define CW_FREQ_HZ              750UL           // Tonalité audio en Hz (ex: 800, 1000, 1750)
    #define CW_WPM                  25              // Vitesse Morse en mots par minute (WPM)// --- Messages CW personnalisables ---
    #define CW_BATTERY_MESSAGE "BAT"
    #define CW_TEMP_MESSAGE    "HOT"
#endif
// variables, don't change
bool TXisON               = false;
bool RXflag_prev          = false;
float Batt_voltage        = 13.80;
uint16_t  R1              = 27000;    // voltage divider R1 in Ohm
uint16_t  R2              = 2200;
float Temp                =  0.00;         // temperature
bool BATT_LOW_FLAG        = false;
bool TEMP_HIGH_FLAG       = false;    // Drapeau d'alerte température
// syntheziser
#define DELAY_uS          100         // delay for synthesizer
double RXfreq,RXVco,TXVco;
bool RXflag               = false;
bool TXflag               = 0;
uint8_t TXDELAY           = 10;       // PTT fall delay
#define PTT_DEBOUNCE_MS   30          // Anti-rebond PTT en millisecondes

// board pin definition
#define ONE_WIRE_BUS      2   // input  - DS18B20
#define PIN_FAN           3   // output - fan driver pin
#define PIN_PWR           12   // output - PA cmd HI/LOW
#define PIN_INFO_TX       4  // input  - pin 1 INFO EM platine CIRCULATEUR - open collector of PA filter board
#define PIN_SQ            5   // input  - pin B1 DP platine RECEPTEUR
#define PIN_RX_LOCK       6   // output - RX synthesizer lock
#define PIN_DATA          7   // input  - MC145156 DATA pin 12
#define PIN_CLOCK         8   // output - MC145156 CLOCK pin 11
#define PIN_ENB           9   // output - MC145156 ENB pin 13
#define PIN_CW_OUT        10  // output - PTT (INT capable pin)
#define PIN_PTT_IN        A0  // output - PTT (INT capable pin)
#define PIN_PTT_OUT       11  // output - PTT
#define PIN_PTT_IN_LED    13  // output - PTT LED
#define PIN_RSSI          A1  // input  - RSSI 
#define PIN_VOLTMETER     A3  // input  - Voltmeter pin

/*
ATR2400 boards pinout
RX SYNTH          TX SYNTH
========          ========
A1  +12V           PTT 
A2  +12V           GND
A3  GND            +12V 
A4  ENB            GND
B1  LOCK TX        TX AUDIO IN
B2  LOCK OUT       +12V
B3  DATA           LOCK OUT
B4  CLK            GND
*/


// Générateur CW
// ============================================================
// Sinusoïde 1000 Hz par PWM DDS sur Timer1 - ATmega328P 16 MHz
// Sortie : Pin 10  — filtrage RC simple
// ============================================================
#if CW_ENABLE
    #define CW_DOT_MS               (1200 / CW_WPM) // Durée d'un point en ms (standard PARIS : 1200 / WPM)
    #define PWM_FREQ_HZ  (F_CPU / 256UL)                 // 31250 Hz à 8 MHz, 62500 Hz à 16 MHz
    
    // Calcul automatique de PHASE_INC quelle que soit la fréquence F_CPU
    #define PHASE_INC    ((uint32_t)((4294967296ULL * CW_FREQ_HZ) / PWM_FREQ_HZ))
    
    // Adaptation automatique de la rampe (~4 ms) selon la cadence de l'interruption
    // 1 interruption = 256 cycles d'horloge -> à 8 MHz (32 µs) : 2 ticks, à 16 MHz (16 µs) : 4 ticks
    const uint8_t RAMP_UNIT_TICKS = (F_CPU / 4000000UL); 


    #define SINE_SIZE    256
    //const uint8_t sineTable[SINE_SIZE] = {
    const uint8_t sineTable[SINE_SIZE] PROGMEM = {
      128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
      176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,
      218,220,222,224,226,228,229,231,233,234,236,237,239,240,241,242,
      243,244,245,246,247,247,248,249,249,250,250,250,251,251,251,251,
      251,251,251,251,251,250,250,250,249,249,248,247,247,246,245,244,
      243,242,241,240,239,237,236,234,233,231,229,228,226,224,222,220,
      218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,
      176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,
      128,125,122,119,116,113,110,107,104,101, 98, 94, 91, 89, 86, 83,
      80, 77, 74, 71, 68, 66, 63, 60, 58, 55, 53, 50, 48, 45, 43, 41,
      38, 36, 34, 32, 30, 28, 27, 25, 23, 22, 20, 19, 17, 16, 15, 14,
      13, 12, 11, 10,  9,  9,  8,  7,  7,  6,  6,  6,  5,  5,  5,  5,
        5,  5,  5,  5,  5,  6,  6,  6,  7,  7,  8,  9,  9, 10, 11, 12,
      13, 14, 15, 16, 17, 19, 20, 22, 23, 25, 27, 28, 30, 32, 34, 36,
      38, 41, 43, 45, 48, 50, 53, 55, 58, 60, 63, 66, 68, 71, 74, 77,
      80, 83, 86, 89, 91, 94, 98,101,104,107,110,113,116,119,122,125
    };

    volatile uint32_t phaseAcc = 0;
    // --- Enveloppe de keying ---
    volatile int16_t  envLevel  = 0;    // 0..255 (amplitude courante)
    volatile int8_t   envStep   = 0;    // rampe : +n, -n ou 0
    volatile uint8_t  envTarget = 0;
    //const uint8_t RAMP_UNIT_TICKS = 4;     // temps pour monter/descendre d'un cran
    // Table de sinus 0..255 (offset 128, amplitude 124 pour éviter le clipping)
#endif


// declare functions
void setFrequency (double MHz, int dataPort, int clockPort, int enablePort);
void TXON();
void CheckTemperature();
void CheckBattery();
void FanON();
void FanOFF();
uint32_t getVoltage(uint8_t pin, uint16_t R1, uint16_t R2);
bool getSQ(int pin);
bool getTXinfo(int pin);

#if CW_ENABLE
  void keyDown();
  void keyUp();
  void waitEnv();
  void sendMorseChar(char ch);
  void sendMorse(const char* text);
#endif

// init
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
TickerScheduler ts(2);      // set Ticker to use n ticker


/*******************************
 _____                 _   _                 
|  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
|  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 

*******************************/



void TXON() {
  // si PTT actif et Synthé RX verouillé et TX pas en émission
  // on active la commande CER vers le synthé TX et PA
  
  // --- Anti-rebond non bloquant sur PIN_PTT_IN ---
  static bool pttStableState   = false;
  static bool pttLastReading   = false;
  static uint32_t lastDebounce = 0;

  bool currentReading = digitalRead(PIN_PTT_IN);
  // gestion de l'anti-rebond commande PTT
  // Si l'état brut change (rebond ou appui), on réinitialise le chronomètre
  if (currentReading != pttLastReading) {
    lastDebounce = millis();
    pttLastReading = currentReading;
  }
  // Si l'état est resté stable plus longtemps que PTT_DEBOUNCE_MS, on valide
  if ((millis() - lastDebounce) >= PTT_DEBOUNCE_MS) {
    pttStableState = currentReading;
  }

  // --- Logique d'émission utilisant pttStableState ---
  if (pttStableState == true && digitalRead(PIN_RX_LOCK) == true && TXflag == false) {
    //if (digitalRead(PIN_PTT_IN) ==1 && TXflag == false) {
      TXflag = true;
      // if we are in DUPLEX, no need to change the VCO frequency
      // only in simplex
      if (DUPLEX == false) {
        setFrequency (TXVco, PIN_DATA, PIN_CLOCK, PIN_ENB);
      }
      digitalWrite(PIN_PTT_OUT,HIGH);
      digitalWrite(PIN_PTT_IN_LED,HIGH);
      Serial.println(F("TX PTT ON"));
    }

    // nous sommes en émission
    if (pttStableState == false && TXflag == true) {
      TXflag = false;

      #if CW_ENABLE
      // --- Alerte batterie faible : ident B AVANT de couper l'émetteur ---
        if (BATT_LOW_FLAG) {
          // si CQ validée, envoi B en CW
            Serial.println(F("Low battery warning - sending CW alarm"));
            sendMorse(CW_BATTERY_MESSAGE);
        }
        // si la température est trop élevée
        if (TEMP_HIGH_FLAG) {
        // Si B vient d'être envoyé, on ajoute l'espace
        if (BATT_LOW_FLAG) {
          delay(9 * CW_DOT_MS);
        }
        Serial.println(F("High temp, sending CW alarm"));
        sendMorse(CW_TEMP_MESSAGE); // cw
      }
      #endif

      if (DUPLEX == false) {
        setFrequency (RXVco, PIN_DATA, PIN_CLOCK, PIN_ENB);
      }
      digitalWrite(PIN_PTT_OUT,LOW);
      digitalWrite(PIN_PTT_IN_LED,LOW);
      Serial.println(F("TX PTT OFF"));
    }
}

void setFrequency (double MHz, int dataPort, int clockPort, int enablePort)
{
  Serial.print(F("Set VCO frequency : "));
  Serial.println(MHz,3);

  unsigned long FK = MHz * 80;  // same as divided by 0.0125
  //unsigned long N = FK >> 6;    // same as FK/64 (prescaler divider factor)
  //unsigned long A = FK & 0x3F;  // same FK -(N*64)
  unsigned long N = FK /64;    // FK/64 (prescaler divider factor)
  unsigned long A = FK - (N*64);  // FK -(N*64)

  // ie: Rx on 430.000 MHz, PLL = 430.000 - 21.400 = 408.600 MHz (VCO FREQ)
  // FK = 408.600 * 80 = 32688
  // N = 32688/64 = 510
  // A = 32688 - ( 510 * 64) = 48

  // SW1 and SW2 are band switches not used in ATR2400, so set to 0
  // 7 bits A counter + 10 bits B counter + SW 2 + SW 1
  // A LSB A MSB - B LSB B MSB - SW2 - SW1
  
  unsigned long PLL;
  PLL = (N << 7) | A;   // shift left by 7 and logic OR with A
  unsigned long i;
  #ifdef DEBUG_SYNTHE
    Serial.print("Freq ");
    Serial.print(MHz,3);
    Serial.print(" N=");
    Serial.print(N);
    Serial.print(" A=");
    Serial.print(A);
    Serial.print(" PLL=");
    Serial.print(PLL);
    Serial.print(" - ");
  #endif

  //digitalWrite(clockPort, LOW);
  //digitalWrite(enablePort, HIGH);
  //delayMicroseconds (DELAY_uS);
  //digitalWrite(enablePort, LOW);
  //digitalWrite(clockPort, LOW);
  //delayMicroseconds (DELAY_uS);
  digitalWrite(enablePort, LOW);
  for (i=0; i<19; i++) {
    unsigned long sh = (((unsigned long)1) << (18-i));
    int bt = !!(PLL & sh);
    #ifdef DEBUG_SYNTHE
      Serial.print(bt ? "1" : "0");
    #endif
    delayMicroseconds (DELAY_uS);
    digitalWrite(dataPort, (bt ? HIGH : LOW));
    delayMicroseconds (DELAY_uS);
    digitalWrite(clockPort, HIGH);
    delayMicroseconds (DELAY_uS);
    digitalWrite(clockPort, LOW);
  }
  #ifdef DEBUG_SYNTHE
    Serial.println();
  #endif
  
  for (i=0; i<19; i++) {
    unsigned long sh = (((unsigned long)1) << (18-i));
    int bt = !!(PLL & sh);
    delayMicroseconds (DELAY_uS);
    digitalWrite(dataPort, (bt ? HIGH : LOW));
    delayMicroseconds (DELAY_uS);
    digitalWrite(clockPort, HIGH);
    delayMicroseconds (DELAY_uS);
    digitalWrite(clockPort, LOW);
  }
  delayMicroseconds (DELAY_uS);
  digitalWrite(enablePort, HIGH);
  delayMicroseconds (DELAY_uS);
  digitalWrite(enablePort, LOW);
}

// switch fan ON
void FanON() {
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0)); // déconnecte OC2B
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  #ifdef DEBUG
    Serial.println(F("Fan is ON"));
  #endif
}

// switch fan OFF
void FanOFF() {
  //OCR2B = 0;                 // rapport cyclique à zéro
  TCCR2A &= ~_BV(COM2B1);    // déconnecte OC2B du Timer2
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  #ifdef DEBUG
    Serial.println(F("Fan is OFF"));
  #endif
}


// read the DS18B20 and return temperature in C
// if temperature > 30C switch ON the cooling FAN
// switch it OFF with some hysteresis
void CheckTemperature() {
  sensor.requestTemperatures();

  // Attente obligatoire de la fin de conversion (non-bloquant possible avec millis)
  while (!sensor.isConversionComplete()) {
    delay(10);
  }
 
  Temp = sensor.getTempC();
  #ifdef DEBUG
    Serial.print("Temperature: ");
    Serial.println(Temp);
  #endif
  // gestion du ventilateur
  if( Temp >= TEMP_HI ) FanON();
  if( Temp <= TEMP_LO ) FanOFF();

    // Gestion de l'alerte surchauffe Morse (hystérésis de 2 °C)
  if (Temp >= TEMP_ALARM_LEVEL) {
    TEMP_HIGH_FLAG = true;
    #ifdef DEBUG
      Serial.println(F("Alerte temperature elevee"));
    #endif
  } 
  else if (Temp <= (TEMP_ALARM_LEVEL - 2.0)) {
    TEMP_HIGH_FLAG = false;
  }

}

/******************************************
Mesure la tension batterie
compare avec le seuil bas
si atteint, change un flag pour transmettre B en Morse en fin d'émission
*****************************************/
void CheckBattery() {
  const int N = 20;
  uint32_t sum = 0;
  for (int i = 0; i < N; i++) {
    sum += getVoltage(PIN_VOLTMETER, R1, R2);
    delay(2); // laisse le temps au signal de se stabiliser
  }
  Batt_voltage = (sum / (float)N) / 1000.00;
  
  // si batterie faible passage puissance basse
  if (Batt_voltage <= BATT_LOW_LEVEL) {
    digitalWrite(PIN_PWR,LOW);
    BATT_LOW_FLAG = true;
    #ifdef DEBUG
      Serial.println("Batterie faible");
    #endif
  }
  else if (Batt_voltage > (BATT_LOW_LEVEL + 0.5)) { // hysteresis tension batterie 0.5V
    digitalWrite(PIN_PWR,HIGH);
    BATT_LOW_FLAG = false;
  }
  #ifdef DEBUG
    Serial.print(F("Voltage: "));
    Serial.println(Batt_voltage, 2);
  #endif
}

/******************************************
 * getVoltage()
 * read ADC on pin
 * result in mV
 *****************************************/
uint32_t getVoltage(uint8_t pin,uint16_t R1,uint16_t R2) {
  analogReference(INTERNAL);
	uint32_t adc_value = 0;

  analogRead(pin);  // wake up the ADC and let it settle
  delay(50);
	for(uint8_t i=0; i<5; i++) adc_value = analogRead(pin); // make some measures and keep last one
  // DEBUG
  //Serial.println(adc_value);
	uint32_t volt = adc_value * VREF / 1024;  // convert adc value to mVolts refered to reference voltage. Adjust 1100 to you board
  volt = volt * (R1+R2)/R2;  // apply the voltage divider multiplication factor (R1+R2)/R2
  // DEBUG
  //Serial.println(volt);
  return volt;
}


/***************************
* read the SQUELCH state
***************************/
bool getSQ(int pin) {
  static bool dernierEtat = false;
  static bool premierAppel = true;

  bool etatActuel = digitalRead(pin);  // conversion automatique int -> bool

  if (premierAppel || etatActuel != dernierEtat) {
    Serial.print("SQ : ");
    Serial.println(etatActuel);
    dernierEtat = etatActuel;
    premierAppel = false;
  }

  return etatActuel;
}

// get TX info out from duplexer board
bool getTXinfo (int pin) {
  static bool dernierEtat = false;
  static bool premierAppel = true;
  bool etatActuel = digitalRead(pin);

  if (premierAppel || etatActuel != dernierEtat) {
    Serial.print("TX info : ");
    Serial.println(etatActuel);
    dernierEtat = etatActuel;
    premierAppel = false;
  } 
  return etatActuel; 
}

// CW gen
// --- ISR : sinusoïde modulée par l'enveloppe ---
#if CW_ENABLE
    ISR(TIMER1_OVF_vect) {
      phaseAcc += PHASE_INC;
      uint8_t s = pgm_read_byte(&sineTable[phaseAcc >> 24]);  // 0..255, off 128

      // Rampe d'enveloppe (incrément toutes les ~4 interruptions ≈ 64 µs)
    static uint8_t rampCnt = 0;
    if (envStep != 0 && ++rampCnt >= RAMP_UNIT_TICKS) {   // ← rampCnt utilisé ICI
        rampCnt = 0;
        envLevel += envStep;
        if (envStep > 0 && envLevel >= envTarget) { envLevel = envTarget; envStep = 0; }
        if (envStep < 0 && envLevel <= envTarget) { envLevel = envTarget; envStep = 0; }
      }
      // Application enveloppe : (s-128)*env/256 + 128
      int16_t v = ((int16_t)(s - 128) * envLevel) >> 8;
      OCR1B = 128 + v;
    }

    // --- Commande keying avec rampe ~4 ms ---
void keyDown() { envTarget = 255; envStep = +4; }
void keyUp()   { envTarget = 0;   envStep = -4; }

void waitEnv() {                        // attendre fin de rampe
  while (envStep != 0) {}
}

// --- Machine morse : lettre B ---
const uint16_t UNIT = 80;               // ms, ≈15 WPM

// Table Morse : chaque caractère est codé sur 1 octet
// Le bit de poids fort à '1' sert de marqueur de début, suivi des bits (0 = point, 1 = trait)
struct MorseCode {
  char c;
  uint8_t pattern;
};

const MorseCode morseTable[] PROGMEM = {
  {'A', 0b00000101}, {'B', 0b00011000}, {'C', 0b00011010}, {'D', 0b00001100},
  {'E', 0b00000010}, {'F', 0b00010010}, {'G', 0b00001110}, {'H', 0b00010000},
  {'I', 0b00000100}, {'J', 0b00010111}, {'K', 0b00001101}, {'L', 0b00010100},
  {'M', 0b00000111}, {'N', 0b00000110}, {'O', 0b00001111}, {'P', 0b00010110},
  {'Q', 0b00011101}, {'R', 0b00001010}, {'S', 0b00001000}, {'T', 0b00000011},
  {'U', 0b00001001}, {'V', 0b00010001}, {'W', 0b00001011}, {'X', 0b00011001},
  {'Y', 0b00011011}, {'Z', 0b00011100},
  {'0', 0b00111111}, {'1', 0b00101111}, {'2', 0b00100111}, {'3', 0b00100011},
  {'4', 0b00100001}, {'5', 0b00100000}, {'6', 0b00110000}, {'7', 0b00111000},
  {'8', 0b00111100}, {'9', 0b00111110}, {'/', 0b00110010}
};

void sendMorseChar(char ch) {
  OCR1B  = 128;          // Positionne le point milieu audio avant démarrage
  TIMSK1 = _BV(TOIE1);   // Active la génération sinusoïdale

  if (ch >= 'a' && ch <= 'z') ch -= 32; // Conversion automatique en majuscule

  for (uint8_t i = 0; i < sizeof(morseTable) / sizeof(MorseCode); i++) {
    char tableChar = pgm_read_byte(&morseTable[i].c);
    if (tableChar == ch) {
      uint8_t pattern = pgm_read_byte(&morseTable[i].pattern);
      
      // Recherche du bit marqueur de départ (le '1' le plus à gauche)
      uint8_t mask = 0x80;
      while ((pattern & mask) == 0 && mask > 0) {
        mask >>= 1;
      }
      mask >>= 1; // On passe au premier élément réel (point ou trait)

      // Émission des points et des traits
      while (mask > 0) {
        keyDown(); waitEnv();
        if (pattern & mask) {
          delay(3 * CW_DOT_MS); // Trait (dah)
        } else {
          delay(CW_DOT_MS);     // Point (dit)
        }
        keyUp(); waitEnv();

        mask >>= 1;
        if (mask > 0) {
          delay(CW_DOT_MS);     // Espace entre deux éléments d'une même lettre (1 point)
        }
      }
      return;
    }
  }
  TIMSK1 = 0;            // Coupe l'interruption à la fin du message
  OCR1B  = 0;            // Met la sortie D10 à 0 V
}

void sendMorse(const char* text) {
  for (uint16_t i = 0; text[i] != '\0'; i++) {
    if (text[i] == ' ') {
      delay(4 * CW_DOT_MS); // Complète les 3 points précédents pour faire 7 points entre deux mots
    } else {
      sendMorseChar(text[i]);
      // Si un caractère suit (et que ce n'est pas un espace), pause inter-lettres de 3 points
      if (text[i + 1] != '\0' && text[i + 1] != ' ') {
        delay(3 * CW_DOT_MS);
      }
    }
  }
}


#endif


/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup() {
  Serial.begin(115200);
    while (!Serial);
    Serial.print(F("ATR2400 controller by TK5EP v"));
    Serial.print(VERSION_MAJOR);
    Serial.print(".");
    Serial.print(VERSION_MINOR);
    Serial.print(F("."));
    Serial.println(VERSION_PATCH);
    Serial.print(F("Compile le "));
    Serial.println(BUILD_TIME);
    
    Serial.println(F("\nInit"));
    Serial.println(F("===="));

    Serial.print(F("TX frequency : "));
    Serial.println(TXFREQ,3);
    if (DUPLEX) {
      Serial.println(F("Duplex ON"));
    }
  
  // TX SYNTH PINS
  pinMode (PIN_ENB          , OUTPUT);
  pinMode (PIN_CLOCK        , OUTPUT);
  pinMode (PIN_DATA         , OUTPUT);
  pinMode (PIN_RX_LOCK      , INPUT);
  // RX
  pinMode (PIN_SQ           , INPUT);
    // TX
  pinMode (PIN_PTT_IN       , INPUT_PULLUP);
  pinMode (PIN_PTT_OUT      , OUTPUT);
  pinMode (PIN_PTT_IN_LED   , OUTPUT);
  pinMode (PIN_PWR          , OUTPUT);
  pinMode (PIN_INFO_TX      , INPUT_PULLUP);
  // 
  pinMode (PIN_FAN          , OUTPUT);
  pinMode (PIN_VOLTMETER    , INPUT);
  // CW
  pinMode(PIN_CW_OUT, OUTPUT);
  
  // pour PWM 25 kHz sur ventilateur
  // Timer2 en Fast PWM, 25 kHz, sortie sur D3
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  // non-inverting, Fast PWM
  TCCR2B = _BV(WGM22) | _BV(CS21);                 // prescaler /8
  OCR2A  = 79;                                     // 16 MHz / 8 / (79+1) = 25 kHz
  OCR2B  = 0;                                      // duty = 0 % (OFF)
  
  // si sonde de température
  #ifdef TEMP_ENABLE
    delay(100); // Laisse le temps à l'alimentation 5V de se stabiliser
    sensor.begin();
    sensor.setResolution(10);
    ts.add(0, 120000, [](void *) { CheckTemperature(); }, nullptr, true);  // add ticker read temp
  #endif
  ts.add(1, 300000, [](void *) { CheckBattery(); }, nullptr, true);  // add ticker check battery
  

  // set the synthesizer
  if (DUPLEX==0) {
    RXVco = TXFREQ + 21.400;
    TXVco = RXVco  + 0.100;
  }
  else {
    if (TXFREQ <= 430.600 && TXFREQ >= 430.000)      RXVco = TXFREQ + 30.800;
    else if (TXFREQ >= 439.400 && TXFREQ <= 440.000) RXVco = TXFREQ - 30.800;
    else Serial.println(F("Out of band!"));
    TXVco = RXVco;
  }
  
  // set the RX synthesizer
  setFrequency (RXVco, PIN_DATA, PIN_CLOCK, PIN_ENB);  

  // switch the fan OFF
  FanOFF();

  // set PA power level, sets PA pin Imax to high power
  digitalWrite(PIN_PWR,LOW);

  //Fast PWM 8 bits. Active la sortie matérielle OC1B (broche D10) mais interruption coupée au repos
  // sinon consommation de ressource trop importante
  #if CW_ENABLE
    TCCR1A = _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(WGM12)  | _BV(CS10);
    //TIMSK1 = _BV(TOIE1);
    TIMSK1 = 0; // Interruption désactivée hors émission CW
  #endif
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop() {
  #ifdef DEBUG_SYNTHE
    Serial.print(F("TX VCO:"));
    Serial.println(TXVco,3);
    Serial.print(F("RX VCO:"));
    Serial.println(RXVco,3);
    Serial.println();
  #endif

  // le squelch est-il ouvert?
  RXflag = getSQ(PIN_SQ);
  RXflag_prev = RXflag;
  
  // TEST lecture niveau INFO TX du filtre PA
  //Serial.println((getTXinfo(PIN_INFO_TX)));

 digitalWrite(PIN_PWR,HIGH);

  // check PTT state
  TXON();

    // run ticker with all measures
  // checks battery
  // checks temperature
  ts.update();
}
