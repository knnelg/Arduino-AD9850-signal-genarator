#include <Wire.h>
#include <LiquidCrystal_I2C.h> // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library.git
#include <Rotary.h> // https://github.com/brianlow/Rotary.git

// AD9851 Signal generator board config. Set the pins we'll use to control the AD9850
const int AD9850_RESET = 5;
const int AD9850_DATA = 6;
const int AD9850_FQ_UD = 7;
const int AD9850_W_CLK = 8;

const long FREQUENCY_MIN = 0L;
const long FREQUENCY_MAX = 30000000L;
const long FREQUENCY_STEPS[] = {1, 10, 100, 500, 1000, 5000, 10000, 50000, 100000, 500000, 1000000, 5000000 }; 
const int FREQUENCY_STEPS_LENGTH = sizeof(FREQUENCY_STEPS)/sizeof(long);

volatile  long _frequency = 1000L;
volatile  long _currentFrequency = 0L;

volatile  long _frequencyStep = 10000L;
volatile  long _currentFrequencyStep = 0L;
volatile  int _frequencyStepIndex = 4;

// Construct an LCD instance
const int LCD_ADDRESS = 0x27; // Either use a scan sketch, or device documentation to determine this address
const int LCD_CHARS_PER_LINE = 16;
const int LCD_NUMBER_OF_LINES = 2;
LiquidCrystal_I2C _lcd(LCD_ADDRESS, LCD_CHARS_PER_LINE, LCD_NUMBER_OF_LINES);

// Construct a Rotary encoder using digital pins 2 (INTerrupt 0 - INT0) &
// 3 (INTerrupt1 - INT1)
// This will be used to change frequency
const int FREQUENCY_ENCODER_CLOCK = 2;
const int FREQUENCY_ENCODER_DATA = 3; 
const int FREQUENCY_ENCODER_SWITCH = 11; // State changes when encoder shaft is depressed
Rotary _frequencyRotaryEncoder = Rotary(FREQUENCY_ENCODER_CLOCK, FREQUENCY_ENCODER_DATA);

// Construct a Rotary encoder using digital pins 9 & 10 
const int FREQUENCY_STEP_ENCODER_CLOCK = 9;
const int FREQUENCY_STEP_ENCODER_DATA = 10; 
Rotary _frequencyStepRotaryEncoder = Rotary(FREQUENCY_STEP_ENCODER_DATA, FREQUENCY_STEP_ENCODER_CLOCK);


/////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine (ISR) that gets called whenever any of the pins in 'Pin Change
// Interrupt 2 vector' (PCINT2_vect, which is another way of saying the 'Pin Change
// Interrupt MaSK 2')
////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT2_vect) {
  // Ask the frequency encoder if there is a change
  unsigned char encoderDirection = _frequencyRotaryEncoder.process();

  // Update the desired frequency value accordingly
  // The DIR_CW & DIR_CCW are defined in Rotary.h
  switch (encoderDirection) {
    case DIR_CW : {
      if((_frequency + _frequencyStep) < FREQUENCY_MAX) {
          _frequency += _frequencyStep;
      }

      else {
        _frequency = FREQUENCY_MAX;
      }
      
      break;
    }
  
    case DIR_CCW : {
      // Prevent frequency going negative
      if((_frequency - _frequencyStep) >= FREQUENCY_MIN) {
        _frequency -= _frequencyStep;
      }

      else {
        _frequency = FREQUENCY_MIN;
      }
    }
    
  } // End switch

} // End ISR(PCINT2_vect)


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect) {
  // Ask the frequency step encoder if there is a change
  unsigned char encoderDirection = _frequencyStepRotaryEncoder.process();

  // Update the desired frequency value accordingly
  // The DIR_CW & DIR_CCW are defined in Rotary.h
  switch (encoderDirection) {
      case DIR_CW : {
          if(_frequencyStepIndex >= FREQUENCY_STEPS_LENGTH - 1) {
              _frequencyStepIndex = 0;
          }
    
          else {
              _frequencyStepIndex++;
          }
          
          break;
      }
      
      case DIR_CCW : {
          // Prevent frequency going negative
          if(_frequencyStepIndex <= 0) {
              _frequencyStepIndex = FREQUENCY_STEPS_LENGTH - 1;
          }
    
          else {
              _frequencyStepIndex--;
          }
      }
     
  } // End switch
             
  _frequencyStep = FREQUENCY_STEPS[_frequencyStepIndex];
 
  ///////////
  // Service the frequency select switch
  // When the switch is pressed, reset frequency to 1Khz
  //////////
  if (digitalRead(FREQUENCY_ENCODER_SWITCH) == 0) {
    _frequency = 1000;
  }

} // End ISR(PCINT1_vect)

///////////////////////////////////////////////////////////////////////////////
// Sends a high, immediately followed by a low, to the supplied digidal pin
///////////////////////////////////////////////////////////////////////////////
void pulseHigh(int pin) {
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}

///////////////////////////////////////////////////////////////////////////////
// Takes a Byte of data, serialises it, and transfers it to the AD9850 DATA pin
///////////////////////////////////////////////////////////////////////////////
void sendByteToAD9850(byte data) {
  // For each bit in the data byte...
  // - Mask off all but the least significant bit
  // - Send the masked HIGH/LOW value to the AD9850
  // - Rotate-right the data byte
  for (int dataIndex = 0; dataIndex < 8; dataIndex++, data >>= 1) {
    // Set the AD9850 'Data' bit to the state of the current bit
    digitalWrite(AD9850_DATA, data & 0x01);
    
    // Pulse the 'Write ClocK' pin to make the AD9850 read the DATA bit
    pulseHigh(AD9850_W_CLK);
  }
}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////
void sendFrequencyToAD9850(double frequency) {
  int32_t AD9850Frequency = frequency * 4294967295L / 125000000L;  // note 125 MHz clock on 9850
 
  for (int frequencyByte = 0; frequencyByte < 4; frequencyByte++, AD9850Frequency >>= 8) {
    sendByteToAD9850(AD9850Frequency & 0xFF);
  }
  
  sendByteToAD9850(0x00); // Final control byte, all 0 for 9850 chip
  
  pulseHigh(AD9850_FQ_UD); // Done!  Should see output
}

////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////
String formatFrequency(unsigned long frequency) {
    char displayFrequency[26];

    if(frequency >= 1000000) {
        dtostrf((frequency/1000000.0), -1, 6, displayFrequency);
        sprintf(displayFrequency, "%s Mhz", displayFrequency);
    }

    else if(frequency >= 1000) {
        dtostrf((frequency/1000.0), -1, 3, displayFrequency);
        sprintf(displayFrequency, "%s Khz", displayFrequency);
    }

    else {
        sprintf(displayFrequency, "%d Hz", frequency);
    }

    return String(displayFrequency);
    
} // End formatFrequency()

////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////
String formatSteps(unsigned long frequency) {
    char displaySteps[26];

    if(frequency >= 1000000) {
        dtostrf((frequency/1000000.0), -1, 0, displaySteps);
        sprintf(displaySteps, "%s Mhz", displaySteps);
    }

    else if(frequency >= 1000) {
        dtostrf((frequency/1000.0), -1, 0, displaySteps);
        sprintf(displaySteps, "%s Khz", displaySteps);
    }

    else {
        sprintf(displaySteps, "%d Hz", frequency);
    }

    return String(displaySteps);
    
} // End formatSteps()

///////////////////////////////////////////////////////////////////////////////
// One-off setup routine
///////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

  // Configure the GPIO pins we're using to talk to the AD9850 module.
  pinMode(AD9850_RESET, OUTPUT);
  pinMode(AD9850_DATA, OUTPUT);
  pinMode(AD9850_FQ_UD, OUTPUT);
  pinMode(AD9850_W_CLK, OUTPUT);
  
  // Enable the pullup resistor on the pin used to monitor the select switch on
  // the frequency rotary encoder 
  pinMode(FREQUENCY_ENCODER_SWITCH, INPUT_PULLUP);
  
  
  // Initialise the AD9850 module. 
  pulseHigh(AD9850_RESET);
  pulseHigh(AD9850_W_CLK);
  pulseHigh(AD9850_FQ_UD); // Eenable serial mode - Datasheet page 12 figure 10  

  /////////////////////////////////////////////////////////////////////////////
  // Initialize the LCD, turn on the backlight, and set the relevant interrupt
  // pins to be used for the serial (I2C) communications
  /////////////////////////////////////////////////////////////////////////////
  _lcd.begin();
  _lcd.backlight();
  
  /////////////////////////////////////////////////////////////////////////////
  // Initialize the serial (I2C) communications using digital pins 18 & 19
  /////////////////////////////////////////////////////////////////////////////
  // Enable the 'Pin Change Interupt Enable 2' (PCIE2) bit in the 'Pin Change
  // Interrupt Control Register' (PCICR) in order to detect interrupts
  // appearing on pins controlled by 'Pin Change MaSK Register 2' (PCMSK2) 
  PCICR |= (1 << PCIE2);
  PCICR |= (1 << PCIE0);
 
  // Set the 'Pin Change INTerrupt18 & 19' (PCINT18 & PCINT19) Pins in the
  // 'Pin Change MaSK2' register, to enable interrupts on digital pins 18
  // (Serial DAta - SDA) & 19 (Serial CLock - SLC)
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
 
  // Set the 'Pin Change INTerrupts 9 & 10' Pins in the
  // 'Pin Change MaSK1' register, to enable interrupts on digital pins 9
  // & 10
  PCMSK0 |= (1 << PCINT1) | (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  
  // Enable interupts
  sei();

} // End setup()

/////////////////////////////////////////////////////////////////////////////////////////////
// Main controller loop
////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // If desired frequency has changed...
  if(_frequency != _currentFrequency) {
    // Update the AD9850 genarator with the new desired frequency
    sendFrequencyToAD9850(_frequency);
    
    // Update global reference with new frequency
    _currentFrequency = _frequency;

    // Write the new frequency to the display
    _lcd.setCursor(0, 0);
    _lcd.print("F: " + formatFrequency(_frequency) + "              ");
  }
  
   // If the desired step size has changed...
  if(_frequencyStep != _currentFrequencyStep) {
    // Update global reference with new step size
    _currentFrequencyStep = _frequencyStep;

    // Write the new step size  to the display
    _lcd.setCursor(0, 1);
    _lcd.print("Step: " + formatSteps(_frequencyStep) + "              ");
  }

}
