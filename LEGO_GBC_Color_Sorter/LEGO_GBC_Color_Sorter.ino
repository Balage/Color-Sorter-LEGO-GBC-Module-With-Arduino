/*
    MIT License
    
    Copyright (c) 2022, Balazs Vecsey, www.vbstudio.hu
    
    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in the
    Software without restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
    Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
       
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
    AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
    WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


//
// TUNING
//
// CALIBRATION
#define CALIBRATION_SAMPLES 25

// SAMPLES
#define SAMPLE_COUNT 7

// TIMING
// Wait for the photo-resistor to respond (20-30ms response time)
#define WAIT_FOR_PHOTO_RESISTOR 40

// Time to wait after detecting a ball and before starting to take measurements
#define SERVO_BALL_SETTLE 200

// Time to go to base before turning to a lane
#define SERVO_OPENING_TIME 150

// Time spent on turning to a lane and staying there before going back to idle
#define SERVO_OPEN_DURATION 400

// SERVO DIRECTIONS [-7, 7]
#define SERVO_BASE -6
#define SERVO_STANDBY 0
#define SERVO_LANE_0 0
#define SERVO_LANE_1 2
#define SERVO_LANE_INVALID 5


//
// DEFINES
//
#include <EEPROM.h>
#include <SoftwareSerial.h>

// https://bitbucket.org/teckel12/arduino-toneac
// Pins  9 & 10 - ATmega328, ATmega128, ATmega640, ATmega8, Uno, Leonardo, etc.
// Pins 11 & 12 - ATmega2560/2561, ATmega1280/1281, Mega
// Pins 12 & 13 - ATmega1284P, ATmega644
// Pins 14 & 15 - Teensy 2.0
// Pins 25 & 26 - Teensy++ 2.0

#if defined (__AVR_ATmega32U4__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
    #define PWMT1AMASK DDB5
    #define PWMT1BMASK DDB6
    #define PWMT1DREG DDRB
    #define PWMT1PORT PORTB
#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__)
    #define PWMT1AMASK DDD4
    #define PWMT1BMASK DDD5
    #define PWMT1DREG DDRD
    #define PWMT1PORT PORTD
#else
    #define PWMT1AMASK DDB1
    #define PWMT1BMASK DDB2
    #define PWMT1DREG DDRB
    #define PWMT1PORT PORTB
#endif

// EEPROM ADDRESSES
#define EEPROM_SIZE (E2END + 1)
#define ADDR_CALIB 0
#define ADDR_SAMPLE(offset) (32 + (offset) * sizeof(SampleType))

// DEFINE I/O PINS
#define IO_MOTOR_1A 6
#define IO_MOTOR_2A 5
#define IO_LED_R A0
#define IO_LED_G A1
#define IO_LED_B A2
#define IO_SENSOR A3
#define IO_BLUETOOTH_TX 11
#define IO_BLUETOOTH_RX 3
#define IO_SW_LOCK 12

// Set up Bluetooth communication
SoftwareSerial BT(IO_BLUETOOTH_TX, IO_BLUETOOTH_RX);

// Timer0: pin 5, 6
// Timer1: pin 9, 10
// Timer2: pin 11, 3

// Musical notes
int const Notes[4][12] = {
    { 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247 },
    { 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494 },
    { 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988 },
    { 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976 },
};

#define NOTE_C 0
#define NOTE_CS 1
#define NOTE_D 2
#define NOTE_DS 3
#define NOTE_E 4
#define NOTE_F 5
#define NOTE_FS 6
#define NOTE_G 7
#define NOTE_GS 8
#define NOTE_A 9
#define NOTE_AS 10
#define NOTE_B 11

// States
#define LED_ON LOW
#define LED_OFF HIGH

#define LANE_1 0
#define LANE_2 1
#define LANE_INVALID 2


//
// HELPER MACROS
///////////////////////////////////////////////////////////////////////////////////
#define max3(a, b, c) (a > b ? (a > c ? a : c) : (b > c ? b : c))
#define min3(a, b, c) (a < b ? (a < c ? a : c) : (b < c ? b : c))
#define minmax(min, max, value) (value < min ? min : (value > max ? max : value))
#define strequal(a, b) (strcmp(a, b) == 0)


//
// BASE TYPES
///////////////////////////////////////////////////////////////////////////////////
struct hsl_int // size=4
{
    unsigned short hue;
    byte sat;
    byte lit;
};

struct rgb_flt
{
    float red;
    float gre;
    float blu;
};

struct SampleType // size=32
{
    byte Active;
    byte Lane;
    char Name[22];
    hsl_int Sample;
    hsl_int Spread;
};

//
// READ/WRITE PROTECTION
///////////////////////////////////////////////////////////////////////////////////
bool HasReadWriteRights()
{
    return digitalRead(IO_SW_LOCK) == LOW;
}

//
// EXTENSIONS
///////////////////////////////////////////////////////////////////////////////////
unsigned short AngleDistance(long a, long b)
{
    long diff = ((b - a + 32767) % 65535) - 32767;
    return (unsigned short)abs(diff < -32767 ? diff + 65535 : diff);
}

void BT_print_hex(byte value)
{
    if (value < 0x10) BT.print('0');
    BT.print(value, HEX);
}

void BT_print_hex(unsigned short value)
{
    BT_print_hex((byte)(value >> 8));
    BT_print_hex((byte)(value & 0xff));
}

void BT_print_hex(hsl_int value)
{
    BT_print_hex(value.hue);
    BT_print_hex(value.sat);
    BT_print_hex(value.lit);
}

void BT_print(hsl_int value)
{
    BT.print(F("HSL["));
    BT.print((float)value.hue / 65535.0f * 360.0f, 1);
    BT.print(F("°, "));
    BT.print((float)value.sat / 255.0f * 100.0f, 1);
    BT.print(F("%, "));
    BT.print((float)value.lit / 255.0f * 100.0f, 1);
    BT.print(F("%]"));
}

void BT_print(rgb_flt value)
{
    BT.print(F("RGB["));
    BT.print(value.red, 2);
    BT.print(F(", "));
    BT.print(value.gre, 2);
    BT.print(F(", "));
    BT.print(value.blu, 2);
    BT.print(F("]"));
}

void BT_printTime(unsigned long seconds)
{
    BT.print(seconds / 3600, DEC);
    BT.print(':');
    
    int num = (seconds % 3600) / 60;
    if (num < 10) BT.print('0');
    BT.print(num, DEC);
    BT.print(':');
    
    num = seconds % 60;
    if (num < 10) BT.print('0');
    BT.print(num, DEC);
}

// Halts execution until it recieves a character, or wait until timeout.
char BT_waitRead(unsigned long timeout = 0)
{
    if (timeout == 0)
    {
        while (!BT.available()) {}
        return BT.read();
    }
    else
    {
        auto end_time = millis() + timeout;
        while (!BT.available())
        {
            if (millis() >= end_time) return 0;
        }
        return BT.read();
    }
}

// Halts program until a linebreak encountered
bool BT_readLine(char* str, int str_size, unsigned long timeout = 100)
{
    int index = 0;
    char c;
    
    // Waits for first character indefinitely
    while (!BT.available()) {}
    
    auto timestamp = millis();
    
    while (true)
    {
        c = BT_waitRead(timeout);
        
        if (c == '\r')
        {
            BT.print(F("Invalid character! Use only LF (\\n) for new line.\n"));
            return false;
        }
        if (c == '\n')
        {
            *(str + index) = 0;
            return true;
        }
        if (c == 0)
        {
            BT.print(F("Timeout error! Send the whole line in one go.\n"));
            return false;
        }
        
        if (index < str_size - 1)
        {
            *(str + index) = c;
            index++;
        }
    }
}

byte BT_readLineInt8()
{
    char temp[4]; // max 255
    BT_readLine(temp, sizeof(temp));
    
    int number = 0;
    for (int i = 0; i < 3; i++)
    {
        char chr = temp[i];
        
        if (chr == 0)
            break;
        
        if (chr < '0' || '9' < chr)
            return 0;
        
        number *= 10;
        number += temp[i] - '0';
    }
    
    if (number > 255)
        return 255;
    
    return (byte)number;
}

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    {
        EEPROM.write(ee++, *p++);
    }
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    {
        *p++ = EEPROM.read(ee++);
    }
    return i;
}

bool askYesNo()
{
    char temp[2];
    BT_readLine(temp, sizeof(temp));
    return temp[0] == 'y' || temp[0] == 'Y';
}

//
// MUSIC PLAYER
///////////////////////////////////////////////////////////////////////////////////
void toneAC(unsigned long frequency) {
    PWMT1DREG |= _BV(PWMT1AMASK) | _BV(PWMT1BMASK); // Set timer 1 PWM pins to OUTPUT (because analogWrite does it too).
    
    uint8_t prescaler = _BV(CS10);                  // Try using prescaler 1 first.
    unsigned long top = F_CPU / frequency / 2 - 1;  // Calculate the top.
    if (top > 65535) {                              // If not in the range for prescaler 1, use prescaler 256 (122 Hz and lower @ 16 MHz).
        prescaler = _BV(CS12);                      // Set the 256 prescaler bit.
        top = top / 256 - 1;                        // Calculate the top using prescaler 256.
    }
    
    ICR1   = top;                                     // Set the top.
    if (TCNT1 > top) TCNT1 = top;                     // Counter over the top, put within range.
    TCCR1B = _BV(WGM13)  | prescaler;                 // Set PWM, phase and frequency corrected (top=ICR1) and prescaler.
    OCR1A  = OCR1B = top / 2;                         // Calculate & set the duty cycle.
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0); // Inverted/non-inverted mode (AC).
}

void noToneAC() {
    TCCR1B  = _BV(CS11);           // Default clock prescaler of 8.
    TCCR1A  = _BV(WGM10);          // Set to defaults so PWM can work like normal (PWM, phase corrected, 8bit).
    PWMT1PORT &= ~_BV(PWMT1AMASK); // Set timer 1 PWM pins to LOW.
    PWMT1PORT &= ~_BV(PWMT1BMASK); // Other timer 1 PWM pin also to LOW.
}

void playNote(int octave, int note, int duration)
{
    toneAC(Notes[octave][note]);
    delay(duration);
    noToneAC();
}

#define NOTE_LENGTH 125

void notePowerOn()
{
    playNote(2, NOTE_G, NOTE_LENGTH);
    playNote(3, NOTE_C, NOTE_LENGTH);
}

void noteMatch(int lane)
{
    switch (lane)
    {
        case LANE_1: playNote(2, NOTE_C, NOTE_LENGTH); break;
        case LANE_2: playNote(2, NOTE_E, NOTE_LENGTH); break;
        default: // LANE_INVALID
            playNote(2, NOTE_G, NOTE_LENGTH);
            playNote(2, NOTE_E, NOTE_LENGTH);
            break;
    }
}

#define noteDoomBase() \
    playNote(oct - 1, NOTE_E, spd / 2); delay(spd / 2); \
    playNote(oct - 1, NOTE_E, spd);

void noteDoom()
{
    int oct = 1;
    
    // Intro
    int spd = 64;
    
    playNote(oct, NOTE_B, spd);
    playNote(oct, NOTE_G, spd);
    playNote(oct, NOTE_E, spd);
    playNote(oct, NOTE_C, spd);
    
    playNote(oct, NOTE_E, spd);
    playNote(oct, NOTE_G, spd);
    playNote(oct, NOTE_B, spd);
    playNote(oct, NOTE_G, spd);
    
    playNote(oct, NOTE_B, spd);
    playNote(oct, NOTE_G, spd);
    playNote(oct, NOTE_E, spd);
    playNote(oct, NOTE_G, spd);
    
    playNote(oct, NOTE_B, spd);
    playNote(oct, NOTE_G, spd);
    playNote(oct, NOTE_B, spd);
    playNote(oct + 1, NOTE_E, spd);
    
    // Main
    spd = 128;
    
    noteDoomBase();
    playNote(oct, NOTE_E, spd);
    
    noteDoomBase();
    playNote(oct, NOTE_D, spd);
    
    noteDoomBase();
    playNote(oct, NOTE_C, spd);
    
    noteDoomBase();
    playNote(oct, NOTE_AS, spd);
    
    noteDoomBase();
    playNote(oct, NOTE_B, spd);
    playNote(oct, NOTE_C, spd);
    
    noteDoomBase();
    playNote(oct, NOTE_E, spd);
    
    noteDoomBase();
    playNote(oct, NOTE_D, spd);
    
    noteDoomBase();
    playNote(oct, NOTE_C, spd);
    
    noteDoomBase();
    playNote(oct - 1, NOTE_AS, 256);
}

//
// GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////
unsigned long _ballStatistics[SAMPLE_COUNT + 1];
bool _debug = false;
bool _binary = false;

byte _laneToServo[] = {
    SERVO_LANE_0,
    SERVO_LANE_1,
    SERVO_LANE_INVALID
};

const int _servoTestStops[] = {
    -7,
    SERVO_BASE,
    SERVO_LANE_0,
    SERVO_LANE_1,
    SERVO_LANE_INVALID,
    7,
    SERVO_LANE_INVALID,
    SERVO_LANE_1,
    SERVO_LANE_0,
    SERVO_BASE
};

// EEPROM cache
rgb_flt _sensorCorrection;

void setup()
{
    // Init pin modes
    pinMode(IO_MOTOR_1A, OUTPUT);
    pinMode(IO_MOTOR_2A, OUTPUT);
    pinMode(IO_LED_R, OUTPUT);
    pinMode(IO_LED_G, OUTPUT);
    pinMode(IO_LED_B, OUTPUT);
    pinMode(IO_SENSOR, INPUT);
    pinMode(IO_SW_LOCK, INPUT_PULLUP);
    
    // Set defaults
    SetAllLeds(LED_OFF);
    SetServo(SERVO_STANDBY);
    notePowerOn();
    
    // Read calibration value
    EEPROM_readAnything(ADDR_CALIB, _sensorCorrection);
    
    // Set up bluetooth
    BT.begin(38400);
    BT.print("Balazs Vecsey's GBC Color Sorter\n");
}

void TrainSamples(hsl_int &sample, hsl_int &spread, byte sampleCount)
{
    BT.print(F("Started training with "));
    BT.print(sampleCount, DEC);
    BT.print(F(" samples...\n"));

    double avg_hue_sin = 0.0;
    double avg_hue_cos = 0.0;
    double avg_sat = 0.0;
    double avg_lit = 0.0;

    long first_hue;
    long min_hue = 0xffff;
    int min_sat = 0xff;
    int min_lit = 0xff;
    long max_hue = -0xffff;
    int max_sat = 0;
    int max_lit = 0;
    
    // Read all colors
    for (int i = 0; i < sampleCount; ++i)
    {
        // Reset servo
        SetServo(SERVO_STANDBY);
        
        // Standby
        while (!DetectBall()) {}
        
        SetAllLeds(LED_OFF);
        delay(SERVO_BALL_SETTLE);
        
        // Read and save color
        auto hsl = ReadColor();
        
        // Average
        double hue_rad = (double)hsl.hue / 65535.0 * PI * 2.0;
        avg_hue_sin += sin(hue_rad);
        avg_hue_cos += cos(hue_rad);
        avg_sat += (double)hsl.sat;
        avg_lit += (double)hsl.lit;

        // Spread
        if (i == 0)
        {
            first_hue = hsl.hue;
        }
        long hue = hsl.hue > first_hue + 32767 ? hsl.hue - 65535 : hsl.hue;
        // Min
        if (hue < min_hue) min_hue = hue;
        if (hsl.sat < min_sat) min_sat = hsl.sat;
        if (hsl.lit < min_lit) min_lit = hsl.lit;
        // Max
        if (hue > max_hue) max_hue = hue;
        if (hsl.sat > max_sat) max_sat = hsl.sat;
        if (hsl.lit > max_lit) max_lit = hsl.lit;
        
        BT.print(i + 1, DEC);
        BT.print('/');
        BT.print(sampleCount, DEC);
        BT.print(F(": "));
        BT_print(hsl);  
        BT.print('\n');
        
        // Propagate ball to first lane
        SetServo(SERVO_BASE);
        delay(SERVO_OPENING_TIME);
        SetServo(SERVO_LANE_0);
        noteMatch(LANE_1);
        delay(SERVO_OPEN_DURATION - 125);
    }
    
    BT.print(F("Training completed\n"));
    
    // Average read
    avg_hue_sin /= (double)sampleCount;
    avg_hue_cos /= (double)sampleCount;
    sample = {
        (unsigned short)round(atan2(avg_hue_sin, avg_hue_cos) / (PI * 2.00) * 65535.0),
        (byte)round(avg_sat / (double)sampleCount),
        (byte)round(avg_lit / (double)sampleCount)
    };
    
    // Get spread (furthest value for each channel)
    spread = {
        (unsigned short)max(
            abs((long)sample.hue - min_hue),
            abs((long)sample.hue - max_hue)
        ),
        (byte)max(
            abs((int)sample.sat - min_sat),
            abs((int)sample.sat - max_sat)
        ),
        (byte)max(
            abs((int)sample.lit - min_lit),
            abs((int)sample.lit - max_lit)
        )
    };  
    
    // Print results
    BT.print(F("Average: "));
    BT_print(sample);
    BT.print('\n');
    
    BT.print(F("Spread ±: "));
    BT_print(spread);
    BT.print('\n');
}

void CalibrateSensor()
{
    BT.print(F("Provide "));
    BT.print(CALIBRATION_SAMPLES, DEC);
    BT.print(F(" bright white balls...\n"));
    
    rgb_flt average = { 0.0f, 0.0f, 0.0f };
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        // Reset servo
        SetServo(SERVO_STANDBY);
        
        // Standby
        while (!DetectBall()) {}
        
        SetAllLeds(LED_OFF);
        delay(SERVO_BALL_SETTLE);
        
        // Read color
        auto raw = ReadColorRaw();
        average.red += raw.red;
        average.gre += raw.gre;
        average.blu += raw.blu;
        
        BT.print(i + 1, DEC);
        BT.print('/');
        BT.print(CALIBRATION_SAMPLES, DEC);
        BT.print(F(": "));
        BT_print(raw);
        BT.print('\n');
        
        // Propagate ball to first lane
        SetServo(SERVO_BASE);
        delay(SERVO_OPENING_TIME);
        SetServo(SERVO_LANE_0);
        noteMatch(LANE_1);
        delay(SERVO_OPEN_DURATION - 125);
    }
    
    average.red /= (float)CALIBRATION_SAMPLES;
    average.gre /= (float)CALIBRATION_SAMPLES;
    average.blu /= (float)CALIBRATION_SAMPLES;
    
    float maxval = max3(average.red, average.gre, average.blu);
    float brightnessScale = (1 / maxval) * 0.98f;
    _sensorCorrection = {
        (maxval / average.red) * brightnessScale,
        (maxval / average.gre) * brightnessScale,
        (maxval / average.blu) * brightnessScale
    };
    
    // Save and update
    EEPROM_writeAnything(ADDR_CALIB, _sensorCorrection);
    
    // Print results
    BT.print(F("Average: "));
    BT_print(average);
    BT.print('\n');
    
    BT.print(F("New multiplier: "));
    BT_print(_sensorCorrection);
    BT.print('\n');
}

void loop()
{
    unsigned long seconds = millis() / 1000;
    
    if (BT.available())
    {
        char temp[9];
        
        if (!BT_readLine(temp, sizeof(temp)))
            return;
        
        bool canReadWrite = HasReadWriteRights();
        
        if (strequal(temp, "help"))
        {
            BT.print(F("GBC Color Sorting module\n"));
            BT.print(F("by Balazs Vecsey\n"));
            BT.print('\n');
            BT.print(F("  stat - Statistics since last power up\n"));
            BT.print(F("  samples - Display sample info\n"));
            BT.print('\n');
            if (canReadWrite)
            {
                BT.print(F("  calib - Calibrate sensor\n"));
                BT.print(F("  train - Train new sample\n"));
                BT.print(F("  remove - Remove single sample\n"));
                BT.print(F("  clrsamp - Clear all samples\n"));
                BT.print('\n');
                BT.print(F("  servo - Servo test\n"));
                BT.print(F("  music - Play a song\n"));
                BT.print('\n');
            }
            BT.print(F("  debug - Print extra debug data\n"));
            BT.print(F("  mem - Print debug data on EEPROM\n"));
            BT.print('\n');
        }
        else if (strequal(temp, "stat"))
        {
            BT.print(F("Module has been running for "));
            BT_printTime(seconds);
            BT.print('\n');
            
            double ballCount = 0.0;
            for (int i = 0; i <= SAMPLE_COUNT; i++)
            {
                ballCount += _ballStatistics[i];
            }
            BT.print(F("Throughput: "));
            BT.print(ballCount / ((double)seconds / 60.0), 2);
            BT.print(F(" bpm\n"));
            
            BT.print(F("#0 Invalid: "));
            BT.print(_ballStatistics[SAMPLE_COUNT], DEC);
            BT.print('\n');
            
            for (int i = 0; i < SAMPLE_COUNT; i++)
            {
                SampleType sample;
                EEPROM_readAnything(ADDR_SAMPLE(i), sample);
                
                if (sample.Active == 255)
                {
                    BT.print('#');
                    BT.print(i + 1);
                    BT.print(' ');
                    BT.print(sample.Name);
                    BT.print(F(": "));
                    BT.print(_ballStatistics[i], DEC);
                    BT.print('\n');
                }
            }
        }
        else if (canReadWrite && strequal(temp, "calib"))
        {
            BT.print(F("Current multiplier: "));
            BT_print(_sensorCorrection);
            BT.print('\n');
            
            BT.print(F("Do you want to recalibrate sensors? (y/n)\n"));
            if (askYesNo())
            {
                CalibrateSensor();
            }
            else
            {
                BT.print(F("Aborting...\n"));
            }
        }
        else if (canReadWrite && strequal(temp, "train"))
        {
            SampleType newSample;
            
            // Get sample index
            BT.print(F("Storage index for new sample (free: "));
            bool first = true;
            for (int i = 0; i < SAMPLE_COUNT; i++)
            {
                byte active = EEPROM.read(ADDR_SAMPLE(i));
                if (active != 255)
                {
                    if (!first) BT.print(", ");
                    first = false;
                    BT.print(i + 1);
                }
            }
            BT.print(F("):\n"));
            int index = BT_readLineInt8() - 1;
            
            if (index < 0 || SAMPLE_COUNT <= index)
            {
                BT.print(F("Invalid index! Aborting...\n"));
                return;
            }
            
            // Get lane index
            BT.print(F("Ouput lane (1: orange, 2: white):\n"));
            newSample.Lane = BT_readLineInt8() - 1;
            
            if (newSample.Lane != 0 && newSample.Lane != 1)
            {
                BT.print(F("Invalid lane index! Aborting...\n"));
                return;
            }
            
            // Get name
            BT.print(F("Sample name (max "));
            BT.print(sizeof(newSample.Name) - 1, DEC);
            BT.print(F(" chars):\n"));
            BT_readLine(newSample.Name, sizeof(newSample.Name));
            
            // Get training sample count
            BT.print(F("Training sample count (2 - 255):\n"));
            byte sampleCount = BT_readLineInt8();
            
            if (sampleCount < 2)
            {
                BT.print(F("Too few samples! Aborting...\n"));
                return;
            }
            
            // Train
            TrainSamples(newSample.Sample, newSample.Spread, sampleCount);
            newSample.Active = 255;
            _ballStatistics[index] = 0; // reset stats
            
            EEPROM_writeAnything(ADDR_SAMPLE(index), newSample);
        }
        else if (canReadWrite && strequal(temp, "remove"))
        {
            // Get sample index
            BT.print(F("Sample index ("));
            bool first = true;
            for (int i = 0; i < SAMPLE_COUNT; i++)
            {
                byte active = EEPROM.read(ADDR_SAMPLE(i));
                if (active == 255)
                {
                    if (!first) BT.print(", ");
                    first = false;
                    BT.print(i + 1);
                }
            }
            BT.print(F("):\n"));
            int index = BT_readLineInt8() - 1;
            
            if (index < 0 || SAMPLE_COUNT <= index)
            {
                BT.print(F("Invalid index! Aborting...\n"));
                return;
            }
            
            // Delete (set to active=0)
            EEPROM.write(ADDR_SAMPLE(index), 0);
            BT.print(F("Done\n"));
        }
        else if (canReadWrite && strequal(temp, "clrsamp"))
        {
            BT.print(F("Are you sure you want to delete all samples? (y/n)\n"));
            if (!askYesNo())
            {
                BT.print(F("Aborting...\n"));
                return;
            }
            
            BT.print(F("Deleting samples... "));
            for (int index = 0; index < SAMPLE_COUNT; index++)
            {
                // Set all to active=0
                EEPROM.write(ADDR_SAMPLE(index), 0);
            }
            
            BT.print(F("Done\n"));
        }
        else if (strequal(temp, "samples"))
        {
            int count = 0;
            for (int i = 0; i < SAMPLE_COUNT; i++)
            {
                SampleType sample;
                EEPROM_readAnything(ADDR_SAMPLE(i), sample);
                
                if (sample.Active == 255)
                {
                    BT.print('#');
                    BT.print(i + 1);
                    BT.print(F(": "));
                    BT.print(sample.Name);
                    BT.print('\n');
                    
                    BT.print(F("  "));
                    BT_print(sample.Sample);
                    BT.print('\n');
                    
                    BT.print(F("  ±"));
                    BT_print(sample.Spread);
                    BT.print('\n');
                    
                    count++;
                }
            }
            
            if (count == 0)
            {
                BT.print(F("There are no samples.\n"));
            }
        }
        else if (canReadWrite && strequal(temp, "music"))
        {
            noteDoom();
        }
        else if (strequal(temp, "mem"))
        {
            BT.print(F("EEPROM size: "));
            BT.print(EEPROM_SIZE);
            BT.print(F(" bytes\n"));
            
            BT.print(F("Calib data (size: 32; count: 1) @ "));
            BT.print(ADDR_CALIB, DEC);
            BT.print(F("-"));
            BT.print(ADDR_CALIB + 32, DEC);
            BT.print(F(" B\n"));
            
            BT.print(F("Samples (size: "));
            BT.print(sizeof(SampleType));
            BT.print(F("; count: "));
            BT.print(SAMPLE_COUNT, DEC);
            BT.print(F(") @ "));
            BT.print(ADDR_SAMPLE(0), DEC);
            BT.print(F("-"));
            BT.print(ADDR_SAMPLE(SAMPLE_COUNT) - 1, DEC);
            BT.print(F(" B\n"));
        }
        else if (strequal(temp, "debug"))
        {
            _debug = !_debug;
            BT.print(F("Debug print "));
            BT.print(_debug ? F("on") : F("off"));
            BT.print('\n');
        }
        else if (strequal(temp, "bin0"))
        {
            _binary = false;
        }
        else if (strequal(temp, "bin1"))
        {
            _binary = true;
        }
        else if (canReadWrite && strequal(temp, "servo"))
        {
            int index = 0;
            while (true)
            {
                SetServo(_servoTestStops[index]);
                index = (index + 1) % (sizeof(_servoTestStops) / sizeof(_servoTestStops[0]));
                delay(500);
                if (DetectBall()) return;
            }
        }
        else
        {
            BT.print(F("Invalid command.\nType 'help' for instructions.\n"));
        }
    }
    
    // Reset servo
    SetServo(SERVO_STANDBY);
    
    // Standby
    if (!DetectBall())
        return;
    
    SetAllLeds(LED_OFF);
    delay(SERVO_BALL_SETTLE);
    
    // Find match
    SampleType matchSample;
    int matchIndex = -1;
    int matchTries = 2;
    
    bool gotMatch = true;
    int servoDirection = 0;
    
    while (!DetectMatch(matchSample, matchIndex))
    {
        matchTries--;
        if (matchTries == 0)
        {
            servoDirection = SERVO_LANE_INVALID;
            gotMatch = false;
            break;
        }
    }
    
    // Found match
    if (gotMatch)
    {
        servoDirection = _laneToServo[matchSample.Lane & 1];
        _ballStatistics[matchIndex]++;
    }
    else
    {
        _ballStatistics[SAMPLE_COUNT]++; // Invalid
    }
    
    // Run to base
    SetServo(SERVO_BASE);
    delay(SERVO_OPENING_TIME);
    
    // Open up
    SetServo(servoDirection);
    if (gotMatch) noteMatch(matchSample.Lane & 1);
    else noteMatch(LANE_INVALID);
    delay(SERVO_OPEN_DURATION - 125);
}


//
// CONVERTER
///////////////////////////////////////////////////////////////////////////////////
// Expect RGB values in range of [0, 1]
struct hsl_int RgbToHsl(float red, float green, float blue)
{
    red = minmax(0.0f, 1.0f, red);
    green = minmax(0.0f, 1.0f, green);
    blue = minmax(0.0f, 1.0f, blue);
    
    // Get the biggest and smallest value
    auto max = max3(red, green, blue);
    auto min = min3(red, green, blue);

    // Calculate saturation and lightness
    auto val = max;
    auto sat = max > 0.0f ? (max - min) / max : 0.0f;
    auto lit = val - val * sat / 2.0;
    if (lit < 0.5f)
    {
        sat = sat / (2.0f - sat);
    }
    else
    {
        sat = (lit == 1.0f) ? 0.0f : (val - lit) / (1.0f - lit);
    }

    // Calculate HUE
    auto hue = 0.0f;
    if (min > 0.0f)
    {
        red = red / min - 1.0f;
        green = green / min - 1.0f;
        blue = blue / min - 1.0f;
    }
    min = min3(red, green, blue);
    
    if (red != green || green != blue)
    {
        if (min > 0.0f)
        {
            red /= min;
            green /= min;
            blue /= min;
        }
        max = max3(red, green, blue);
        
        if (max == red) {
            // between -60 and 60 degrees
            if (green > 0.0f) {
                hue = 60.0f * (green / red);
            } else {
                hue = 360.0f + (-60.0f * (blue / red));
            }
        } else if (max == green) {
            if (red > 0.0f) { // beween 60 and 120
                hue = 120.0f - ((red / green) * 60.0f);
            } else { // between 120 and 180
                hue = 120.0f + ((blue / green) * 60.0f);
            }
        } else if (max == blue) {
            if (green > 0.0f) { // between 180 and 240
                hue = 240.0f - ((green / blue) * 60.0f);
            } else { // between 240 and 300
                hue = 240.0f + ((red / blue) * 60.0f);
            }
        }
        
        // Normalize hue
        hue = fmod(hue + 360.0f, 360.0f);
    }
    
    return {
        (unsigned short)round(hue / 360.0f * 65535.0f),
        (byte)round(sat * 255.0f),
        (byte)round(lit * 255.0f)
    };
}

//
// SENSOR READING
///////////////////////////////////////////////////////////////////////////////////
float ReadSensor()
{
    // Get voltage level
    float voltageLevel = analogRead(IO_SENSOR) / 1023.0f;
    
    // Calculate resistor value
    float pullDown = 15000.0f;
    float resistance = (pullDown / voltageLevel) - pullDown;
    
    // Apply log scale (dark resistance is 1.0 MOhm)
    // Range is now approximately [0, 1], with 1 being dark
    float bright = log10(resistance) / 6.0f;
    
    // Invert and cap between [0, 1]
    return 1.0f - minmax(0.0f, 1.0f, bright);
}

void SetAllLeds(int value)
{
    digitalWrite(IO_LED_R, value);
    digitalWrite(IO_LED_G, value);
    digitalWrite(IO_LED_B, value);
}

float ReadSingleColor(int ledPin)
{
    digitalWrite(ledPin, LED_ON);
    delay(WAIT_FOR_PHOTO_RESISTOR);
    float reading = ReadSensor();
    digitalWrite(ledPin, LED_OFF);
    return reading;
}

struct rgb_flt ReadColorRaw()
{
    return {
        ReadSingleColor(IO_LED_R),
        ReadSingleColor(IO_LED_G),
        ReadSingleColor(IO_LED_B)
    };
}

struct hsl_int ReadColor()
{
    SetAllLeds(LED_OFF);
    
    auto raw = ReadColorRaw();
    
    // Scale reading based on calibration
    raw.red *= _sensorCorrection.red;
    raw.gre *= _sensorCorrection.gre;
    raw.blu *= _sensorCorrection.blu;
    
    // Gamma correction, make the values closer to linear
    raw.red = powf(raw.red, 7.0f);
    raw.gre = powf(raw.gre, 5.0f);
    raw.blu = powf(raw.blu, 4.0f);
    
    return RgbToHsl(raw.red, raw.gre, raw.blu);
}

bool DetectBall()
{
    SetAllLeds(LED_ON);
    delay(WAIT_FOR_PHOTO_RESISTOR); // 20-30ms response time
    return ReadSensor() > 0.3f;
}

#define MIN_SPREAD 0.0001f

bool DetectMatch(SampleType& matchSample, int &matchIndex)
{
    auto hsl = ReadColor();

    if (_debug && !_binary)
    {
        BT.print(F("Reading: "));
        BT_print(hsl);
        BT.print('\n');
    }
    
    float bestMatch = 0.0f;
    int bestMatchIndex = -1;
    SampleType preset;
    
    // Find closest color
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        EEPROM_readAnything(ADDR_SAMPLE(i), preset);
        
        if (preset.Active != 255)
        continue;
        
        hsl_int dist = {
            AngleDistance(preset.Sample.hue, hsl.hue),
            abs(preset.Sample.sat - hsl.sat),
            abs(preset.Sample.lit - hsl.lit)
        };
        
        // Threshold
        const float threshold = 1.5f;
        float thHue = max(MIN_SPREAD, (float)(preset.Spread.hue * threshold));
        float thSat = max(MIN_SPREAD, (float)(preset.Spread.sat * threshold));
        float thLit = max(MIN_SPREAD, (float)(preset.Spread.lit * threshold));
        
        float matchHue = (float)max(0.0f, thHue - dist.hue) / thHue;
        float matchSat = (float)max(0.0f, thSat - dist.sat) / thSat;
        float matchLit = (float)max(0.0f, thLit - dist.lit) / thLit;
        float match = matchHue * matchSat * matchLit;
        
        if (_debug && !_binary)
        {
            BT.print(F("Check #"));
            BT.print(i + 1, DEC);
            BT.print(F(" dist="));
            
            BT_print(dist);
            BT.print('\n');
            
            BT.print(F("  Match: h="));
            BT.print(matchHue, 2);
            BT.print(F("%; s="));
            BT.print(matchSat, 2);
            BT.print(F("%; v="));
            BT.print(matchLit, 2);
            BT.print(F("%\n"));
            
            BT.print(F("  Match total: "));
            BT.print(match, 2);
            BT.print('\n');
        }
        
        if (match > bestMatch)
        {
            bestMatch = match;
            bestMatchIndex = i;
        }
    }
        
    if (bestMatchIndex == -1)
    {
        if (_binary)
        {
            BT.print(F(">>N"));
            BT_print_hex(hsl);
            BT.print('\n');
        }
        else
        {
            BT.print(F("> INVALID: "));
            BT_print(hsl);
            BT.print('\n');
        }
        
        return false;
    }
    else
    {
        EEPROM_readAnything(ADDR_SAMPLE(bestMatchIndex), preset);
        
        if (_binary)
        {
            BT.print(F(">>P"));
            BT_print_hex(hsl);
            BT_print_hex((byte)round(bestMatch * 255.0f));
            BT.print(preset.Name);
            BT.print('\n');
        }
        else
        {
            BT.print(F("> "));
            BT.print(preset.Name);
            BT.print(' ');
            BT.print((int)round(bestMatch * 100.0f), DEC);
            BT.print(F("%\n"));
        }
        
        // Run to a position and back
        matchSample = preset;
        matchIndex = bestMatchIndex;
        return true;
    }
}

//
// SERVO
///////////////////////////////////////////////////////////////////////////////////
const int servoPwmValues[] = { 0, 67, 99, 131, 162, 194, 226, 255 };

void SetServo(int dir)
{
    if (dir == 0)
    {
        analogWrite(IO_MOTOR_1A, 255);
        analogWrite(IO_MOTOR_2A, 255);
    }
    else if (dir < 0)
    {
        if (dir < -7) dir = -7;
        analogWrite(IO_MOTOR_1A, 255 - servoPwmValues[-dir]);
        analogWrite(IO_MOTOR_2A, 255);
    }
    else
    {
        if (dir > 7) dir = 7;
        analogWrite(IO_MOTOR_1A, 255);
        analogWrite(IO_MOTOR_2A, 255 - servoPwmValues[dir]);
    }
}
