/**
 * @file lab3.ino
 * @author Christopher Good, Mike Adrien
 * @brief Source code for UML EECE5520 Lab 3
 * @version 1.0
 * @date 2021-04-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

////////////////////////////////////////////////////////////////////////
//// Includes //////////////////////////////////////////////////////////

// Built-in Arduino Libraries
#include <Arduino.h>
#include <Wire.h>

// ELEGOO libraries for HW modules
#include <DS3231.h>
#include <LiquidCrystal.h>
#include <IRremote.h>

////////////////////////////////////////////////////////////////////////
//// Pin constants /////////////////////////////////////////////////////

// L293D Chip Pins

constexpr uint8_t kL293DEnablePin = 5;
constexpr uint8_t kL293DDirectionAPin = 3;
constexpr uint8_t kL293DDirectionBPin = 4;


// LCD Display Pins

constexpr uint8_t kLcdRsPin = 7;
constexpr uint8_t kLcdEnablePin = 8;
// constexpr uint8_t kLcdData0Pin = 0;
// constexpr uint8_t kLcdData1Pin = 0;
// constexpr uint8_t kLcdData2Pin = 0;
// constexpr uint8_t kLcdData3Pin = 0;
constexpr uint8_t kLcdData4Pin = 9;
constexpr uint8_t kLcdData5Pin = 10;
constexpr uint8_t kLcdData6Pin = 11;
constexpr uint8_t kLcdData7Pin = 12;

// IR Receiver Pins

constexpr uint8_t kIrReceiverPin = 6;

// Button Pins

constexpr uint8_t kButtonPin = 2;

////////////////////////////////////////////////////////////////////////
//// IR Receiver Constants /////////////////////////////////////////////

    // Enumeration for all RF codes that can be received from the IR
    // remote control
    enum class IRCodes : uint32_t {
        POWER         = 0xFFA25D,
        FUNC_STOP     = 0xFFE21D,
        VOL_PLUS      = 0xFF629D,
        FAST_BACK     = 0xFF22DD,
        PAUSE         = 0xFF02FD,
        FAST_FORWARD  = 0xFFC23D,
        DOWN          = 0xFFE01F,
        VOL_MINUS     = 0xFFA857,
        UP            = 0xFF906F,
        EQ            = 0xFF9867,
        ST_REPT       = 0xFFB04F,
        ZERO          = 0xFF6897,
        ONE           = 0xFF30CF,
        TWO           = 0xFF18E7,
        THREE         = 0xFF7A85,
        FOUR          = 0xFF10EF,
        FIVE          = 0xFF38C7,
        SIX           = 0xFF5AA5,
        SEVEN         = 0xFF42BD,
        EIGHT         = 0xFF4AB5,
        NINE          = 0xFF52AD,
        UNKNOWN_CODE  = 0xFFFFFFFF
    };


////////////////////////////////////////////////////////////////////////
//// LCD Screen Constants //////////////////////////////////////////////

    // Number of character rows in LCD screen
    constexpr uint8_t kLcdRows = 2;

    // Number of character columns in LCD screen
    constexpr uint8_t kLcdColumns = 16;


////////////////////////////////////////////////////////////////////////
//// Real Time Clock (RTC) Constants ///////////////////////////////////

enum class FanSpeed : uint8_t {
    STOPPED,
    HALF,
    THREE_QUARTERS,
    FULL
};

static const char * const kFanSpeedStr[] = {
    "0   ",
    "1/2 ",
    "3/4 ",
    "Full"
};


////////////////////////////////////////////////////////////////////////
//// Timer Constants ///////////////////////////////////////////////////

// ATMega2560 has a 16MHz clock
constexpr unsigned long long kClockCycle = 16000000ULL;

// The frequency at which the timer is to initiate an interrupt
// (1 Hz = 1 second)
constexpr unsigned int kTimerInterruptFrequency = 1;

// The value at which to increment the timer clock at
// 1 = 16MHz increment
// 8 = 2MHz increment
// 64 = 250kHz increment
// 256 = 62.5kHz increment
// 1024 = 15.625kHz increment
constexpr unsigned int kTimerPrescalar = 1024;

// The value to input into the timer compare match register to get the
// timer to interrupt every interval
constexpr uint16_t kTimerMatchValue = kClockCycle 
        / (kTimerInterruptFrequency * kTimerPrescalar) - 1UL;

// The delay necessary to debounce the hardware switch
constexpr unsigned long kDebounceDelay = 50;


////////////////////////////////////////////////////////////////////////
//// Variables /////////////////////////////////////////////////////////

// IR Receiver module object
IRrecv ir_receiver(kIrReceiverPin);

// Variable to hold the IR code received from the IR module
decode_results ir_decode_results;

// LCD Screen module object
LiquidCrystal lcd(kLcdRsPin, kLcdEnablePin, kLcdData4Pin, kLcdData5Pin,
                  kLcdData6Pin, kLcdData7Pin);

// RTC module object
DS3231 real_time_clock;

// RTC date-time structure object
RTCDateTime date_time;

// The current speed at which the fan is currently rotating
FanSpeed current_fan_speed = FanSpeed::STOPPED;

// A flag that lets the time controlled fan motor start know that
// the remote stopped the fan for this iteration
bool ir_remote_used = false;

// The direction in which the fan is rotating
bool fan_direction_clockwise = true;

// Flag indicating fan speed or direction has been updated
bool fan_update = false;

// Flag indicating that LCD and time should be updated
bool update = false;

// Variable to hold line one string of LCD
char line_one[kLcdColumns + 1] = {};

// Variable to hold line two string of LCD
char line_two[kLcdColumns + 1] = {};


////////////////////////////////////////////////////////////////////////
//// Functions /////////////////////////////////////////////////////////

/**
 * @brief Translates the fan speed enumeration value into a PWM value
 * 
 * @param speed  The FanSpeed enumeration value
 * @return uint8_t - the PWM speed integer value
 */
uint8_t GetFanSpeed(FanSpeed speed){
    uint8_t speed_value;
    switch(speed){
        case FanSpeed::FULL: speed_value = 255; break;
        case FanSpeed::THREE_QUARTERS: speed_value = 192; break;
        case FanSpeed::HALF: speed_value = 128; break;
        case FanSpeed::STOPPED: speed_value = 0; break;
        default: speed_value = 0;
    }
    
    return speed_value;
}

/**
 * @brief This function increments the current fan speed to the next
 *        speed level or keeps it at the same level if it can go no
 *        higher
 * 
 */
void SpeedUpFan(){
    switch(current_fan_speed){
        case FanSpeed::STOPPED:{
            current_fan_speed = FanSpeed::HALF;
        } break;
        case FanSpeed::HALF:{
            current_fan_speed = FanSpeed::THREE_QUARTERS;
        } break;
        case FanSpeed::THREE_QUARTERS:
        case FanSpeed::FULL: {
            current_fan_speed = FanSpeed::FULL;
        } break;
    }

    fan_update = true;
}

/**
 * @brief This function decrements the current fan speed to the previous
 *        speed level or keeps it at the same level if it can go no
 *        lower
 * 
 */
void SlowDownFan(){
    switch(current_fan_speed){
        case FanSpeed::FULL:{
            current_fan_speed = FanSpeed::THREE_QUARTERS;
            
        } break;
        case FanSpeed::THREE_QUARTERS:{
            current_fan_speed = FanSpeed::HALF;
        } break;
        case FanSpeed::HALF:
        case FanSpeed::STOPPED: {
            current_fan_speed = FanSpeed::STOPPED;
        } break;
    }

    fan_update = true;
}

/**
 * @brief Sends fan motor turn direction and PWM speed of the fan
 * 
 */
void RunFan() {
    digitalWrite(kL293DDirectionAPin, fan_direction_clockwise);
    digitalWrite(kL293DDirectionBPin, !fan_direction_clockwise);
    analogWrite(kL293DEnablePin, GetFanSpeed(current_fan_speed));
}

/**
 * @brief This function gets the current date/time from the real time
 *        clock module and, depending on whether its at the beginning or
 *        end of a minute, turns on or off the fan
 * 
 */
void CheckRealTimeClock(){
    // Update clock, rpm, direction, etc. every second
    date_time = real_time_clock.getDateTime();

    // New minute, reset ir remote flag
    if(date_time.second == 0) ir_remote_used = false;

    if(date_time.second >= 0 
        && date_time.second < 30
        && current_fan_speed == FanSpeed::STOPPED
        && !ir_remote_used){
        current_fan_speed = FanSpeed::FULL;
        fan_update = true;
    }
    else if(date_time.second == 30 && !ir_remote_used){
        current_fan_speed = FanSpeed::STOPPED;
        fan_update = true;
    }
}

/**
 * @brief This function outputs the current time, fan speed, and fan
 *        direction to the LCD screen
 * 
 */
void UpdateLCD() {
    // Create LCD line 1
    snprintf(line_one, kLcdColumns+1, "[S: %s][D: %s]",
        kFanSpeedStr[static_cast<uint8_t>(current_fan_speed)],
        (fan_direction_clockwise) ? " C" : "CC");
    
    // Create LCD line 2
    snprintf(line_two, kLcdColumns+1, "%02d:%02d:%02d        ",
        date_time.hour, date_time.minute, date_time.second);

    // Print line 1 on LCD
    lcd.setCursor(0, 0);
    lcd.print(line_one);
    
    // Print line 2 on LCD
    lcd.setCursor(0, 1);
    lcd.print(line_two);
}

/**
 * @brief Checks the IR module to see if it received an IR code and
 *        translates the code into fan functions
 * 
 */
void CheckIR() {
    if(ir_receiver.decode(&ir_decode_results)){

        switch(ir_decode_results.value){
            case static_cast<long unsigned int>(IRCodes::UP): SpeedUpFan(); break;
            case static_cast<long unsigned int>(IRCodes::DOWN): SlowDownFan(); break;
            case static_cast<long unsigned int>(IRCodes::PAUSE): {

                current_fan_speed = FanSpeed::STOPPED;
                fan_update = true;

            } break;
            case static_cast<long unsigned int>(IRCodes::FAST_FORWARD): {

                fan_direction_clockwise = true;
                fan_update = true;

            } break;
            case static_cast<long unsigned int>(IRCodes::FAST_BACK): {
                
                fan_direction_clockwise = false;
                fan_update = true;

            } break;
        }

        ir_remote_used = true;
    }

    ir_receiver.resume();
}

/**
 * @brief Button interrupt function that checks if the interrut was
 *        caused by a button press or noise and if the button was
 *        pressed it will switch the current fan direction
 * 
 */
void CheckButton(){
    // A static variable (e.g. state persistent across function calls)
    // that holds the last time that the interrupt function was called
    static unsigned long last_interrupt_time = 0;

    // Time of the current interrupt
    unsigned long current_interrupt_time = millis();

    // Check to see if this interrupt was caused by noise or an actual
    // button press
    if((current_interrupt_time - last_interrupt_time) > kDebounceDelay){
        fan_direction_clockwise = !fan_direction_clockwise;
    }

    // Record current interrupt time for the next interrupt
    last_interrupt_time = current_interrupt_time;
}

//// Timer Setup and Interrupt Functions ///////////////////////////////

/**
 * @brief Set hardware timer 2 to interrupt at predetermined interval
 * 
 */
void SetupTimer(){

    // Disable interrupts
    cli();

    // Set all of timer/counter control register 1A to 0
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // | COM1A1| COM1A0| COM1B1| COM1B0| COM1C1| COM1C0| WGM11 | WGM10 |
    // *---------------------------------------------------------------*
    // COM1A1:0 - Compare Output Mode for Channel A
    // COM1B1:0 - Compare Output Mode for Channel B
    // COM1C1:0 - Compare Output Mode for Channel C
    // WGM11:0 - Waveform Generation Mode
    // pg. 154 of ATmega2560 datasheet
    TCCR1A = 0;

    // Set all of timer/counter control register 1B to 0
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // | ICNC1 | ICES1 |   -   | WGM13 | WGM12 | CS12  | CS11  | CS10  |
    // *---------------------------------------------------------------*
    // ICNC1 - Input Capture Noise Canceler
    // ICES1 - Input Capture Edge Select
    // WGM13:2 - Waveform Generation Mode
    // CS12:0 - Clock Select
    // pg. 156 of ATmega2560 datasheet
    TCCR1B = 0;

    // Initialize timer counter value  of timer 1 to 0
    TCNT1  = 0;

    // Set output compare match register 1A for predetermined increments
    OCR1A =  kTimerMatchValue;

    // turn on CTC mode
    TCCR1B |= (1 << WGM12);

    // Set CS10 and CS12 bits for 1024 prescaler
    // See pg. 157 of ATmega2560 datasheet
    TCCR1B |= (1 << CS12) | (1 << CS10);  

    // Allow interrupts
    sei();
}


/**
 * @brief Enables the interrupt for timer 1 and starts the counter at 0
 * 
 */
void EnableTimerInterrupts(){
    // Disable interrupts
    cli();

    // Initialize timer counter value  of timer 1 to 0
    TCNT1  = 0;

    // Enable timer compare interrupt through timer interupt mask 
    // register
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // |   -   |   -   | ICIE1 |   -   | OCIE1C| OCIE1B| OCIE1A| TOIE1 |
    // *---------------------------------------------------------------*
    // ICIE1 - Timer/Counter, Input Capture Interupt Enable
    // OCIE1C - Timer/Counter, Output Compare C Match Interrupt Enable
    // OCIE1B - Timer/Counter, Output Compare B Match Interrupt Enable
    // OCIE1A - Timer/Counter, Output Compare A Match Interrupt Enable
    // TOIE1 - Timer/Counter/ Overflow Interrupt Enable
    // pgs. 161 & 162 ATmega2560 datasheet
    TIMSK1 |= (1 << OCIE1A);

    // Allow interrupts
    sei();
}


/**
 * @brief Disables the interrupt for timer 1
 * 
 */
void DisableTimerInterrupts(){
    // Disable interrupts
    cli();

    // Disable timer compare interrupt through timer interupt mask 
    // register
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // |   -   |   -   | ICIE1 |   -   | OCIE1C| OCIE1B| OCIE1A| TOIE1 |
    // *---------------------------------------------------------------*
    TIMSK1 &= ~(OCIE1A);

    // Allow interrupts
    sei();
}


/**
 * @brief Construct a timer 1 interrupt handler that updates the real
 *        time clock, fan speed, and fan direction every second.
 * 
 */
ISR(TIMER1_COMPA_vect){
    update = true;
}


////////////////////////////////////////////////////////////////////////
//// Main Program //////////////////////////////////////////////////////

/**
 * @brief The setup function runs at the initialization of the arduino
 *        (either power on or when the reset button is pressed)
 * 
 */
void setup() {

    // Set up L293D pins for motor control
    pinMode(kL293DEnablePin, OUTPUT);
    pinMode(kL293DDirectionAPin, OUTPUT);
    pinMode(kL293DDirectionBPin, OUTPUT);

    // Set up number of columns and rows on the LCD screen
    lcd.begin(kLcdColumns, kLcdRows);

    // Output setup steps LCD start -> IR enable
    lcd.setCursor(0, 0);
    lcd.print("STARTED LCD!    ");
    lcd.setCursor(0, 1);
    lcd.print("ENABLING IR...  ");

    // Start the IR receiver module
    ir_receiver.enableIRIn();

    // Output setup steps IR enable -> RTC start
    lcd.setCursor(0, 0);
    lcd.print("ENABLED IR!     ");
    lcd.setCursor(0, 1);
    lcd.print("BEGINNING RTC...");

    // Initialize DS1307 RTC module
    real_time_clock.begin();

    // Output setup steps RTC start -> Date/Time setup
    lcd.setCursor(0, 0);
    lcd.print("BEGAN RTC!      ");
    lcd.setCursor(0, 1);
    lcd.print("SET DATE/TIME...");

    // Initializes the RTC module to the arduino sketch's compile time
    // and date
    real_time_clock.setDateTime(__DATE__, __TIME__);

    // Output setup steps Date/Time setup -> Get initial RTC vals
    lcd.setCursor(0, 0);
    lcd.print("DATE/TIME SET!  ");
    lcd.setCursor(0, 1);
    lcd.print("GET INIT RTC VAL");

    CheckRealTimeClock();

    // Output setup steps Get initial RTC vals -> Setup button interrupt
    lcd.setCursor(0, 0);
    lcd.print("RECEIVED RTC VAL");
    lcd.setCursor(0, 1);
    lcd.print("SET BUTTON INT..");

    // Set up button interrupt
    attachInterrupt(digitalPinToInterrupt(kButtonPin), CheckButton, RISING);

    // Output setup steps Setup button interrupt -> Setup timer interrupt
    lcd.setCursor(0, 0);
    lcd.print("BUTTON INT SET! ");
    lcd.setCursor(0, 1);
    lcd.print("SETUP TIMER INT.");

    // Setup Timer Interrupts
    SetupTimer();

    // Output setup steps Setup timer interrupt -> Enable timer interrupt
    lcd.setCursor(0, 0);
    lcd.print("TIMER INT READY!");
    lcd.setCursor(0, 1);
    lcd.print("ENABLE TIMER INT");

    // Enable timer interrupt
    EnableTimerInterrupts();

    // Output setup steps Enable timer interrupt -> Start main loop
    lcd.setCursor(0, 0);
    lcd.print("TIMER INT START!");
    lcd.setCursor(0, 1);
    lcd.print("STARTING LOOP...");
}


/**
 * @brief The main loop function that runs forever after the startup
 *        function
 * 
 */
void loop() {
    if(update){
        CheckRealTimeClock();
        CheckIR();
        if(fan_update)RunFan();
        UpdateLCD();
        update = false;
    } 
}