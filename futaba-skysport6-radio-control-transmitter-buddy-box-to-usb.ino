// Mark B Jones 2023 - I have adapted the code from W.D. Cossey's github repo below.
// This is a Ardunio Nano Every sketch - using a ATmega4809, Little Endian (Least Significant Byte first in a Word)
// Limited to 4ms resolution due to 16MHz CPU and micros measuring only to 4ms.
// The Timer2 library does not work with Nano Every due to the timer registers/capabilities being different.
// Cannot see why the debug mode works but the non-debug mode results in no data being picked up in vJoySerialFeeder.
// Setup for a 6 channel Futaba radio transmitter.
// See MBJ comments for explanations

/*
 Name:    ppm_to_ibus_serial.ino
 Author:  Original author: wdcossey 5/31/2018
*/
// https://github.com/wdcossey/ppm-to-ibus-serial


// Start 27-12-201 01:17:05 and finish 
//     Short bad frames 195, long bad frames 33, total frames bad=228: total frames 52840   (0.4%)
// Start 27-12-01 16:31:16 to 16:58:38
//     Short bad frames 3, long bad frames 0 - total frames 82000 = 0.0037%

/*
// occasional: could be due to writing out serial whilst in Interrupt Service Routine.
// this happened when using delayMicroseconds due to watchdog timers
Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1). 
01:37:29.936 -> 
01:37:29.936 -> Core  1 register dump:
01:37:29.936 -> PC      : 0x4008ae6e  PS      : 0x00060e34  A0      : 0x80089f8a  A1      : 0x3ffbed60  
01:37:29.936 -> A2      : 0x3ffb8a00  A3      : 0x3ffb8890  A4      : 0x00000004  A5      : 0xb33fffff  
01:37:29.936 -> A6      : 0x00000001  A7      : 0x00000001  A8      : 0x3ffb8890  A9      : 0x3ffb8890  
01:37:29.936 -> A10     : 0x00000018  A11     : 0x00000018  A12     : 0x3ffc0c2c  A13     : 0xb33fffff  
01:37:29.936 -> A14     : 0x00000001  A15     : 0x00000001  SAR     : 0x00000004  EXCCAUSE: 0x00000006  
01:37:29.936 -> EXCVADDR: 0x00000000  LBEG    : 0x400857b0  LEND    : 0x400857ba  LCOUNT  : 0x00000000  
01:37:29.973 -> Core  1 was running in ISR context:
01:37:29.973 -> EPC1    : 0x400def57  EPC2    : 0x00000000  EPC3    : 0x00000000  EPC4    : 0x4008ae6e
01:37:29.973 -> 
01:37:29.973 -> 
01:37:29.973 -> Backtrace:0x4008ae6b:0x3ffbed600x40089f87:0x3ffbed80 0x40088c6a:0x3ffbeda0 0x400d75a5:0x3ffbede0 0x400d64f5:0x3ffbee00 0x400f279a:0x3ffbee20 0x400d1532:0x3ffbee40 0x400d3f55:0x3ffbeef0 0x40084425:0x3ffbef10 0x40088b0a:0x3ffb2690 0x400d9121:0x3ffb26d0 0x400d994d:0x3ffb2720 0x400d75b2:0x3ffb2750 0x400d64f5:0x3ffb2770 0x400d67eb:0x3ffb2790 0x400d17c6:0x3ffb27b0 0x400d18b6:0x3ffb2800 0x400d6ed8:0x3ffb2820 
01:37:30.020 -> 
01:37:30.020 -> 
01:37:30.020 -> Core  0 register dump:
01:37:30.020 -> PC      : 0x4008b04e  PS      : 0x00060034  A0      : 0x80089821  A1      : 0x3ffbe870  
01:37:30.020 -> A2      : 0x3ffbef28  A3      : 0x3ffbe88c  A4      : 0x00060023  A5      : 0xb33fffff  
01:37:30.020 -> A6      : 0x0000cdcd  A7      : 0x0000abab  A8      : 0x0000abab  A9      : 0x0000abab  
01:37:30.020 -> A10     : 0x00000000  A11     : 0x3ffc0a20  A12     : 0x3ffc0a24  A13     : 0x80000020  
01:37:30.067 -> A14     : 0x00000007  A15     : 0x00000020  SAR     : 0x00000011  EXCCAUSE: 0x00000006  
01:37:30.067 -> EXCVADDR: 0x00000000  LBEG    : 0x00000000  LEND    : 0x00000000  LCOUNT  : 0x00000000  
01:37:30.067 -> 
01:37:30.067 -> 
01:37:30.067 -> Backtrace:0x4008b04b:0x3ffbe8700x4008981e:0x3ffbe8b0 0x4008afd7:0x3ffbe8e0 0x40087b7d:0x3ffbe900 0x4008442d:0x3ffbe910 0x400f2aa7:0x3ffbd3a0 0x400ddf66:0x3ffbd3c0 0x40089040:0x3ffbd3e0 
01:37:30.067 -> 
*/
//#include "soc/rtc_wdt.h"

#include <M5StickCPlus.h>

#include "esp_attr.h" // for IRAM_ATTR definition

// See here for usage of IRAM_ATTR and DRAM_ATTR
// https://docs.espressif.com/projects/esp-idf/en/v4.2.1/esp32/api-guides/general-notes.html

void IRAM_ATTR processPulse();    // need forward defintion due to the IRAM_ATTR flag - ESP specific

DRAM_ATTR const bool debugMode=false;
DRAM_ATTR const bool verboseDebug=false;    // enable this for full pulse output, otherwise only short and long bad frames will be reported.
DRAM_ATTR const int debugStatePeriod=10;

uint8_t writeSerialIteration=0;

bool dataReady=false;

#define IBUS_FRAME_LENGTH 0x20                                  // iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_COMMAND40 0x40                                     // Command is always 0x40
#define IBUS_MAXCHANNELS 14                                     // iBus has a maximum of 14 channels

#define IBUS_DEFAULT_VALUE (uint16_t)1500

//#define PPM_LOW_SEPARATOR (uint16_t)2100                        // Anything higher than this value is considered to be the pulse separator

#define PPM_LOW_SEPARATOR (uint16_t)4000                        // Anything higher than this value is considered to be the pulse separator

#define PPM_INVALID_LOW (uint16_t)800

#define PPM_MAX_CHANNELS 10                                     // Should be no more than 10

volatile uint32_t frameTotal=(uint32_t)0UL;
volatile uint32_t shortBadFrameTotal=(uint32_t)0UL;
volatile uint32_t longBadFrameTotal=(uint32_t)0UL;
volatile uint32_t badFrames=(uint32_t)0UL;
uint16_t loopDelay=(uint16_t)10;

volatile bool dumpShortBad=false;
volatile bool dumpLongBad=false;
volatile bool dumpISROutOfSync=false;
volatile bool dumpChannelCount=false;

volatile uint16_t ppm_pulse_data[PPM_MAX_CHANNELS] = { 0 };   // 
volatile uint16_t ppm_channel_data[PPM_MAX_CHANNELS] = { 0 }; // 
volatile uint8_t ppm_pulse_index = 0;
volatile uint8_t ppm_channel_count = 0;

enum syncStates {
  CALC_CHANNELS, // Channel calculation
  PROCESS_PULSE, // Process pulse data
};

volatile enum syncStates syncState = CALC_CHANNELS;

byte serial_buffer[IBUS_FRAME_LENGTH] = { 0 };
int buffer_index = 0;


// Next step - measure the max voltage on output of transmitter and then use a voltage divider circuit to get voltage down to 3V max.
DRAM_ATTR const byte futabaInPin=26;  // Pin G26 on the M5 Stick C Plus

/* VOLTAGE DIVIDER NEEDED
 *  
 *  Scope measured 4.4V square wave. Needs to be max of 3.3.
 *  Assume level could be as high as 5V - need to take this down to 3V. 
 *  Voltage divider is R1 = 200, R2 = 150. Where R1 connects to +ve input, tap is taken between R1 and R2, and R2 is connected to Ground.
 *  https://www.allaboutcircuits.com/tools/voltage-divider-calculator/
 *  Vout = Vin x (R2 / R1 + R2)
 *  Standard values: (R1=20kOhm, R2=14kOhm) or (R1=200kOhm, R2=160kOhm
 *  https://www.eleccircuit.com/how-to-basic-use-resistor/
 */

// the setup function runs once when you press reset or power the board
void setup() {

    delay(500); // ensure all initialisation done (ie frame totals set to zero - some bad initialisation was happening);

    bool LCDEnable=true;
    M5.begin(LCDEnable);

    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);

  Serial.begin(115200); // interrupts moved to below serial port open
  delay(50);    // give time for serial to startup. (even though it does this already in M5.begin();

  checkFrameTotals();

  M5.Lcd.printf("Serial Open\n");
  delay(100);    // Allow LCD to write before doing time sensitive work.

//  frameTotal=shortBadFrameTotal=longBadFrameTotal=badFrames=(uint32_t)0;
  dumpShortBad=dumpLongBad=dumpISROutOfSync=dumpChannelCount=false;

  frameTotal=(uint32_t)0UL;
  shortBadFrameTotal=(uint32_t)0UL;
  longBadFrameTotal=(uint32_t)0UL;
  badFrames=(uint32_t)0UL;
  
  pinMode(futabaInPin,INPUT);

  delay(500); // needed for some reason the frame total ints are not being initialised.

  checkFrameTotals();
  
  attachInterrupt(digitalPinToInterrupt(futabaInPin), processPulse, RISING);
}

// esp32 memory model: https://docs.espressif.com/projects/esp-idf/en/v4.2.1/esp32/api-guides/general-notes.html


void checkFrameTotals()
{
  if (debugMode)
  {
    if (shortBadFrameTotal > 0 || longBadFrameTotal > 0)
    {
       Serial.print("Bad Short ");
       Serial.print(shortBadFrameTotal);
       Serial.print(" Bad Long ");
       Serial.print(longBadFrameTotal);
       Serial.print("... Resetting to 0\n");
       shortBadFrameTotal=(uint32_t)0;
       longBadFrameTotal=(uint32_t)0;     
    }
    else
    {
      Serial.print("Short and Long Frame Counts 0\n");
    }
  }
}

void WriteSerial()
{
  uint16_t ibus_checksum = ((uint16_t)0xFFFF);

  switch (syncState) {
  case CALC_CHANNELS:
    // do nothing until channels have been calculated
    break;
  case PROCESS_PULSE:

    if (debugMode)
    {
        if (verboseDebug)
        {
          if (writeSerialIteration > debugStatePeriod)
          {
            for (int i = 0; i < min(ppm_channel_count, IBUS_MAXCHANNELS); i++) 
            {
              Serial.print(ppm_channel_data[i]);
              Serial.print(", ");
            }
            Serial.print("\n");
            
            writeSerialIteration=0;
          }
          else
          {
            writeSerialIteration++;
          }
        }
        if (frameTotal%1000 == 0)
          Serial.print("Total Frames: " + String(frameTotal) + String("\n"));
    }
    else
    {
      buffer_index = 0;
  
      // Write the IBus buffer length
      serial_buffer[buffer_index++] = (byte)IBUS_FRAME_LENGTH;
      // Write the IBus Command 0x40
      serial_buffer[buffer_index++] = (byte)IBUS_COMMAND40;
  
      // Write the IBus channel data to the buffer
      for (int i = 0; i < min(ppm_channel_count, IBUS_MAXCHANNELS); i++) {
        serial_buffer[buffer_index++] = (byte)(ppm_channel_data[i] & 0xFF);
        serial_buffer[buffer_index++] = (byte)((ppm_channel_data[i] >> 8) & 0xFF);
      }
  
      // Fill the remaining buffer channels with the default value
      if (ppm_channel_count < IBUS_MAXCHANNELS) {
        for (int i = 0; i < IBUS_MAXCHANNELS - ppm_channel_count; i++) {
          serial_buffer[buffer_index++] = (byte)(IBUS_DEFAULT_VALUE & 0xFF);
          serial_buffer[buffer_index++] = (byte)((IBUS_DEFAULT_VALUE >> 8) & 0xFF);
        }
      }
  
      // Calculate the IBus checksum
      for (int i = 0; i < buffer_index; i++) {
        ibus_checksum -= (uint16_t)serial_buffer[i];
      }
  
      // Write the IBus checksum to the buffer
      serial_buffer[buffer_index++] = (byte)(ibus_checksum & 0xFF);
      serial_buffer[buffer_index++] = (byte)((ibus_checksum >> 8) & 0xFF);

      Serial.write(serial_buffer, buffer_index);
    }

    buffer_index = 0;
    
    break;
   default:
    break;
  }

}

DRAM_ATTR const int skySportChannelCount=7;
DRAM_ATTR const int channel5_SkySport_index = 4;
DRAM_ATTR const int channel5_FutabaComp_index = 4;

DRAM_ATTR const int futabaComputer8Chan=8;
DRAM_ATTR const int channel5_comp=4;
DRAM_ATTR const int channel7_comp=5;

// the loop function runs over and over again until power down or reset
void loop() {

  // Write the IBus data to the Serial Port
  if (dataReady)
  {
    if (ppm_channel_count == skySportChannelCount)
    {
      if (ppm_channel_data[channel5_SkySport_index] < 1100)
      {
        // Stop switch jitter on low value
        ppm_channel_data[channel5_SkySport_index] = 800;
      }
      else
      {
        // Stop switch jitter on high value
        ppm_channel_data[channel5_SkySport_index] = 2100;
      }
    }
    else if (ppm_channel_count == futabaComputer8Chan)
    {      
      if (ppm_channel_data[channel5_FutabaComp_index] < 1100)
      {
        // Stop switch jitter on low value
        ppm_channel_data[channel5_FutabaComp_index] = 800;
      }
      else
      {
        // Stop switch jitter on high value
        ppm_channel_data[channel5_FutabaComp_index] = 2100;
      }
    }

    dataReady=false;    // added this so that only writes once a complete set of channels has been recorded.
    WriteSerial();
  }
  else
  {

 // https://stackoverflow.com/questions/51750377/how-to-disable-interrupt-watchdog-in-esp32-or-increase-isr-time-limit
    if (syncState==PROCESS_PULSE)   // loopDelay starts at =6 whilst calculating channels
      loopDelay=1;    // much more jitter when this is set to 10

    // note difference in implementation between delay and delaymicroseconds. delay allows background tasks to complete
    // whereas delayMicroseconds is a busy loop.
    // C:\Projects\arduino_app\portable\packages\m5stack\hardware\esp32\2.0.0\cores\esp32\esp32-hal-misc.c
//    delayMicroseconds(loopDelay*1000);  // this doesn't allow any background tasks to get done, hence the core dumps
    delay(loopDelay); // this allows any background tasks to get done - it still caused a wdt timeout though

    // using the delay function prevents need to call these functions, which didn't 
    // seem to be too reliable stopping 100% issues with delayMicroseconds
//     rtc_wdt_feed();    // see C:\Projects\arduino_app\portable\packages\m5stack\hardware\esp32\2.0.0\tools\sdk\esp32\include\esp_hw_support\include\soc\rtc_wdt.h
 //    feedLoopWDT();     // see C:\Projects\arduino_app\portable\packages\m5stack\hardware\esp32\2.0.0\cores\esp32\esp32-hal-misc.c

  }


    if (debugMode && dumpShortBad)
    {
      badFrames=longBadFrameTotal+shortBadFrameTotal;
//      Serial.print("Short no. " + String(shortBadFrameTotal) + String(" where ") +String(badFrames) + (" bad frames of ") + String(frameTotal) + String("\n"));
  //    Serial.flush();

      Serial.print("Short no. ");
      Serial.print(shortBadFrameTotal);
      Serial.print(" where ");
      Serial.print(badFrames);
      Serial.print(" bad frames of ");
      Serial.print(frameTotal);
      Serial.print("\n");

      dumpShortBad=false;
    }

    if (debugMode && dumpLongBad)
    {
      badFrames=longBadFrameTotal+shortBadFrameTotal;
//      Serial.print("Long no. " + String(longBadFrameTotal) + String(" where ") +String(badFrames) + (" bad frames of ") + String(frameTotal) + String("\n"));
//      Serial.flush();

      Serial.print("Long no. ");
      Serial.print(longBadFrameTotal);
      Serial.print(" where ");
      Serial.print(badFrames);
      Serial.print(" bad frames of ");
      Serial.print(frameTotal);
      Serial.print("\n");

      dumpLongBad=false;
    }

    if (dumpISROutOfSync)
    {
      if (debugMode)
      {
        Serial.println("ISR Out of Sync");
        Serial.flush();
      }
      M5.Lcd.printf("ISR Out of Sync\n");
      dumpISROutOfSync=false;
    }
    
    if (dumpChannelCount)
    {
      if (debugMode)
      {
        Serial.println("ISR Channel Count: " + ((String)ppm_channel_count));
        Serial.flush();
      }
      M5.Lcd.printf("Channels: %d\n",ppm_channel_count);
      dumpChannelCount=false;

      checkFrameTotals();
    }
    
    // delay(20) is really 19750 uS
    // delay(10) is really 9600 to 10100 uS
    // delay(5) is really 4500 to 4600 uS
    // delay(1) is really 500 to 1000 uS
    // delayMicroSeconds(10000) is really 10001 to 10002 uS
    // delayMicroSeconds(1000) is really 1000 to 1002 uS
    // delayMicroSeconds(50) is really 50 to 52 uS
    // delayMicroSeconds(10) is really 10 to 12 uS
    // delayMicroSeconds(5) is really 5 to 7 uS
    

  
}

// https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
// use IRAM to make sure processPulse is placed in internal memory which is much faster than flash.
// more interrupt guidance: https://stackoverflow.com/questions/54651158/esp-32-interrupt-issue
void IRAM_ATTR processPulse() {
    uint16_t currentPulseWidth = esp_timer_get_time(); // ESP32 number of uS since program started
    
    ppm_pulse_data[ppm_pulse_index] = currentPulseWidth;

    if (ppm_pulse_index==0)
      ++frameTotal;
      
    switch (syncState) {
    case CALC_CHANNELS:
      if (ppm_pulse_index > 0 &&
        (ppm_pulse_data[ppm_pulse_index] - ppm_pulse_data[ppm_pulse_index - 1] > PPM_LOW_SEPARATOR))
      {
        int prevChannelCount = ppm_channel_count;
        ppm_channel_count = ppm_pulse_index;

        // Clear the usless data
        ppm_pulse_data[PPM_MAX_CHANNELS] = { 0 };

        // Typically a PPM signal shouldn't carry more than 10 pulses
        // 9 channels and a separator
        if (ppm_channel_count > 0 &&
          ppm_channel_count <= PPM_MAX_CHANNELS &&
          ppm_channel_count == prevChannelCount) {

          // We should have a valid channel count.
          // Update pulse 0 with the current value (as it was cleared above).
          ppm_pulse_data[0] = currentPulseWidth;

          // Set the pulse index to 1.
          // We have the first pulse, no need to calculate it again
          ppm_pulse_index = 1;

          // Update the sync state to process pulse data
          syncState = PROCESS_PULSE;

          dumpChannelCount=true;

          return;
        }
        
        dumpISROutOfSync=true;

        // We are out of sync, start again.
        ppm_pulse_index = 0;

        return;
      }
      break;
    case PROCESS_PULSE:
      // Pulse index must be > 0 to run the calculation
      if (ppm_pulse_index > 0) {

        uint16_t next_width=0, prev_width=0;

        next_width = ppm_pulse_data[ppm_pulse_index] - ppm_pulse_data[ppm_pulse_index - 1];
        prev_width = ppm_channel_data[ppm_pulse_index - 1];

        if (next_width < PPM_INVALID_LOW)
        {
           if (!dumpShortBad)
          {
              ++shortBadFrameTotal;
              dumpShortBad=true; // avoiding writing to serial in interrupt routine.
            }
                      
            // a pulse shorter than anything expected has been detected - ignore it, do not increment
            // ppm_pulse_index so the bad value gets overwritten with the next pulse.
            return;          
        }
        else if (next_width > PPM_LOW_SEPARATOR)
        {
            if (!dumpLongBad)
            {
              ++longBadFrameTotal;
              dumpLongBad=true; // avoiding writing to serial in interrupt routine.
            }

            // the long sync pulse has been detected - throw it away, take the micros and use it 
            // as a new start point and start at zero again.
            ppm_pulse_data[0]=ppm_pulse_data[ppm_pulse_index];
            ppm_pulse_index=1;
            return;
        }
                          
        ppm_channel_data[ppm_pulse_index - 1] = next_width;

        if (ppm_pulse_index == ppm_channel_count)
        {
          ppm_pulse_index = 0;
          dataReady=true;
          return;
        }
      }
      break;

    default:
      break;
    }

    ppm_pulse_index++;

}

////////////////////////////////////////////////////////////////////////////////////

void debouncePWMRead()
{
    // M5 Stick C Plus only - as the 240MHz processor is over 10x faster than 16MHz Uno I have added
    // a busy wait before processing the next interrupt as I believe there was a bouncing issue with the waveform
    // not being a clean rising or falling edge from the transmitter. This isn't an issue on the Uno as it's processing
    // interrupts over 10x slower.
    // Not true below!
    // This delay debounces the interrupt reading. Without this there is a lot more jitter and the synchronisation
    // goes out easily.
//    volatile long delayVCount=25; // 1.15uS     (100 gives forcing resync occasionally
  //  volatile long delayVCount=100;  // 100 is better than 25.
//    volatile long delayVCount=1000;  // 100 is better than 25. 100,000 completely breaks it
    volatile long delayVCount=2;  // 100 is better than 25. 100,000 completely breaks it
    while (--delayVCount); // 1e8=4608829 uS, 10000=465 uS, 100 = 4.6uS
    // 1 cycle at 16MhZ = 62.5ns
    // 1 cycle at 240Mhz = 4.2nS

/*
    long delayCount=100;  
    while (--delayVCount)
    {
       __asm__ ("nop\n"); // 10000==254 uS, 100000=2515 uS, 100000=25142 uS
    }
*/
}
