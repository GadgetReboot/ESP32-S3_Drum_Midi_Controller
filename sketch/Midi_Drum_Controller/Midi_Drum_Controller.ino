// Piezo Trigger using MCP3008 ADC
//
// Platform : ESP32-S3 (Arduino core for ESP32)
// Hardware : One or Two MCP3008 8-channel SPI ADCs
//            Choose "Board" ESP32S3 Dev Module in Tools menu.
// Function : Continuously reads analog inputs as fast as possible.
//            Sends Midi note on/off messages over USB when inputs are triggered
//            Provides info messages on serial terminal
//
//            Enable USB CDC on boot in Tools menu to see serial over USB
//            Disable to see serial on Uart0 TX pin
//            Choose USB Mode "USB-OTG (TinyUSB)" in Tools menu for Midi support
//            Choose Upload Mode "USB-OTG CDC (TinyUSB)" in Tools menu
//
//   MCP3008 Wiring (both chips share the same MOSI, MISO, SCK and have different /CS pins):
//
//   ESP32-S3 Pin  │  MCP3008 (Chip 1)  │  MCP3008 (Chip 2)
//   ──────────────┼────────────────────┼──────────────────
//   GPIO 11       │  CLK               │  CLK
//   GPIO 12       │  DOUT (MISO)       │  DOUT (MISO)
//   GPIO 13       │  DIN  (MOSI)       │  DIN  (MOSI)
//   GPIO 10       │  /CS               │  —
//   GPIO  9       │  —                 │  /CS
//   3.3 V         │  VDD, VREF         │  VDD, VREF
//   GND           │  AGND, DGND        │  AGND, DGND
//
// Uses Adafruit TinyUSB Library v3.7.7
//      Board file: ESP32 by Espressif v3.3.8
//
// Tested with: Arduino IDE v2.3.8

#include "midi_percussion_notes.h"  // list of midi percussion instruments and note numbers
#include <SPI.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#define VSPI_SCLK 11
#define VSPI_MISO 12
#define VSPI_MOSI 13

#define CS_CHIP1 10
#define CS_CHIP2 9

// SPI Clk should be max 2.34MHz for 3.3V power 
// or 3.6MHz for 5V power on MCP3008 (from datasheet)
#define SPI_CLK_SPEED 2340000
//#define SPI_CLK_SPEED 3600000

// how many ADC inputs are being scanned
// system reads starting at chip 1 ch0..7 then chip 2 ch0..7 in that order
// eg. if CHANNELS_IN_USE = 4 the system will look at chip 1 ch0 thru ch3
const uint8_t CHANNELS_IN_USE = 16;

#define ADC_VREF 3.3  // Voltage Reference of the MCP3008 (5.0 or 3.3 volts expected)

// --- TRIGGER TUNING CONSTANTS ---
#define PEAK_WINDOW_MS 2     // 2ms harvesting window to collect trigger peaks
#define BLANKING_TIME_MS 20  // lockout period before accepting a new trigger input once triggered
#define X_TALK_WINDOW_MS 2   // filter window for channel crosstalk rejection
#define X_TALK_RATIO 0.25
#define MIDI_CHANNEL 10              //  percussion midi channel 10
#define MAX_CHANNELS 16              //  total of 16 channels available across 2 chips to trigger midi notes
const uint16_t MIN_THRESHOLD = 50;   //  Soft hit floor
const uint16_t MAX_THRESHOLD = 700;  //  Hard hit limit

const uint8_t drumNotes[MAX_CHANNELS] = {  // midi note numbers assigned to adc input channels. defined in midi_percussion_notes.h
  BassDrum1,
  AcousticSnare,
  HighTom,
  LowTom,
  LowFloorTom,
  OpenHiHat,
  ClosedHiHat,
  CrashCymbal1,
  48,
  49,
  51,
  53,
  55,
  57,
  59,
  60
};

volatile bool noteReadyToSend[MAX_CHANNELS] = { false };  // a trigger event is ready to send a midi note message
volatile uint8_t outVelocity[MAX_CHANNELS] = { 0 };

volatile uint32_t channelHitCounter[MAX_CHANNELS] = { 0 };  // track trigger events for debug messages
volatile uint16_t channelMaxPeakReport[MAX_CHANNELS] = { 0 };

bool noteActive[MAX_CHANNELS] = { false };   // whether a certain channel note has been triggered
uint32_t noteEndTime[MAX_CHANNELS] = { 0 };  // how long to wait until note should be turned back off so it's not hanging

uint16_t currentReading[MAX_CHANNELS] = { 0 };   // adc sample being read
uint32_t lastTriggerTime[MAX_CHANNELS] = { 0 };  // track last time a ch was triggered for use in trigger lockout delays

uint16_t harvestedPeak[MAX_CHANNELS] = { 0 };    // store the highest ch peak detected in a trigger event
uint32_t windowStartTime[MAX_CHANNELS] = { 0 };  // track when a trigger event begins so peak harvesting can occur for the correct duration
bool isHarvesting[MAX_CHANNELS] = { false };     // track whether a trigger has occurred and peak harvesting is in progress

volatile uint32_t groupTriggerTime[MAX_CHANNELS] = { 0 };  // crosstalk filter - check to see if triggered events occur in close proximity to other events
volatile uint16_t groupTriggerPeak[MAX_CHANNELS] = { 0 };  // and how their peaks compare (a weak hit right next to a strong hit is probably crosstalk pickup)

enum VelocityCurveMode {  // options to scale adc trigger levels to midi note velocity in different ways
  CURVE_LINEAR,
  CURVE_SCURVE,
  CURVE_LOGARITHMIC,
  CURVE_EXPONENTIAL
};

const VelocityCurveMode ACTIVE_CURVE = CURVE_LOGARITHMIC;

volatile uint32_t globalLoopCounter = 0;
TaskHandle_t AnalogScanTaskHandle = NULL;

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// the strength of a drum hit is scaled against selectable curves so that
// the midi note velocity (drum hit intensity) maps the trigger strength to the note volume/feel
// eg. make sure light hits translate as light sounds and heavy hits map to loud playback properly
// this may need to be tweaked based on the physical piezo assembly and resulting adc readings across the range
uint8_t calculateCurveVelocity(uint16_t rawPeak, VelocityCurveMode mode) {

  // for readings outside of expected signal range, clamp to min or max thresholds
  if (rawPeak <= MIN_THRESHOLD) return 1;
  if (rawPeak >= MAX_THRESHOLD) return 127;

  // normalize x from 0.0 to 1.0
  float x = (float)(rawPeak - MIN_THRESHOLD) / (MAX_THRESHOLD - MIN_THRESHOLD);
  float y = x;

  // apply scaling curve profile
  switch (mode) {
    case CURVE_LINEAR:
      y = x;
      break;
    case CURVE_SCURVE:
      y = x * x * (3.0f - 2.0f * x);
      break;
    case CURVE_LOGARITHMIC:
      y = sqrtf(x);
      break;
    case CURVE_EXPONENTIAL:
      y = x * x;
      break;
  }

  // scale output to standard MIDI velocity
  float velocityOut = y * 126.0f + 1.0f;
  return (uint8_t)constrain(velocityOut, 1, 127);
}

// read a sample from a specified adc MCP3008 chip and channel
inline uint16_t readMCP3008_FastArduino(uint8_t chipSelect, uint8_t channel) {
  uint8_t commandMSB = 0x01;
  uint8_t commandLSB = (0x08 | channel) << 4;

  digitalWrite(chipSelect, LOW);
  SPI.transfer(commandMSB);
  uint8_t highByte = SPI.transfer(commandLSB);
  uint8_t lowByte = SPI.transfer(0x00);
  digitalWrite(chipSelect, HIGH);

  return ((highByte & 0x03) << 8) | lowByte;
}

// scan all adc inputs and look for trigger events
void AnalogScanTask(void* pvParameters) {
  uint32_t yieldCounter = 0;

  for (;;) {
    uint32_t now = millis();

    // read new samples for all adc channels
    SPI.beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
    for (uint8_t i = 0; i < CHANNELS_IN_USE; i++) {
      if (i < 8) currentReading[i] = readMCP3008_FastArduino(CS_CHIP1, i);
      else currentReading[i] = readMCP3008_FastArduino(CS_CHIP2, i - 8);
    }
    SPI.endTransaction();

    // process each channel's new sample
    for (uint8_t i = 0; i < CHANNELS_IN_USE; i++) {
      uint16_t val = currentReading[i];

      if (isHarvesting[i]) {  // if a ch has been triggered and a peak is being harvested, check if current sample is a new max peak
        if (val > harvestedPeak[i]) {
          harvestedPeak[i] = val;
        }

        if (now - windowStartTime[i] >= PEAK_WINDOW_MS) {  // after peak harvesting is finished for a triggered ch (peak window time lapsed)...
          isHarvesting[i] = false;

          // filter for crosstalk false triggers
          bool isCrosstalk = false;
          for (uint8_t neighbor = 0; neighbor < CHANNELS_IN_USE; neighbor++) {
            if (neighbor == i) continue;
            if (now - groupTriggerTime[neighbor] <= X_TALK_WINDOW_MS) {              // if another ch has recently triggered, check if current ch trigger may be crosstalk
              if (harvestedPeak[i] < (groupTriggerPeak[neighbor] * X_TALK_RATIO)) {  // interpret as a crosstalk false trigger if below a threshold of another ch's recent peak
                isCrosstalk = true;
                break;
              }
            }
          }

          if (!isCrosstalk) {
            // if no crosstalk is detected while harvesting peaks for this ch,
            // update crosstalk tracking variables for trigger times and peaks recorded
            // so future comparisons can be made against established valid readings
            // while checking if a new sample may be crosstalk from recent triggers
            groupTriggerTime[i] = now;
            groupTriggerPeak[i] = harvestedPeak[i];

            outVelocity[i] = calculateCurveVelocity(harvestedPeak[i], ACTIVE_CURVE);  // set midi velocity (how hard was the triggered hit) based on scaling chosen
            noteReadyToSend[i] = true;                                                // a midi note will be sent for the triggered ch

            // track trigger events for debug reporting
            channelHitCounter[i]++;
            if (harvestedPeak[i] > channelMaxPeakReport[i]) {
              channelMaxPeakReport[i] = harvestedPeak[i];
            }
          }
          lastTriggerTime[i] = now;
        }
      } else if (now - lastTriggerTime[i] >= BLANKING_TIME_MS) {  // if not peak harvesting, see if a trigger is beginning now
        if (val > MIN_THRESHOLD) {
          isHarvesting[i] = true;
          windowStartTime[i] = now;
          harvestedPeak[i] = val;
        }
      }
    }

    globalLoopCounter++;
    yieldCounter++;
    if (yieldCounter >= 500) {
      yieldCounter = 0;
      taskYIELD();
    }
  }
}

void setup() {
  TinyUSBDevice.setID(0x2341, 0x0058);
  usb_midi.setStringDescriptor("ESP32-S3 Drum Trigger Module");
  MIDI.begin(MIDI_CHANNEL_OMNI);

  Serial.begin(115200);

  // init SPI bus
  pinMode(CS_CHIP1, OUTPUT);
  pinMode(CS_CHIP2, OUTPUT);
  digitalWrite(CS_CHIP1, HIGH);
  digitalWrite(CS_CHIP2, HIGH);

  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, -1);
  delay(10);

  xTaskCreatePinnedToCore(AnalogScanTask, "AnalogScan", 4096, NULL, 1, &AnalogScanTaskHandle, 1);
}

void loop() {
  uint32_t currentMillis = millis();
  static uint32_t lastReportTime = 0;

  // print debug messages every second
  if (currentMillis - lastReportTime >= 1000) {
    uint32_t scans = globalLoopCounter;
    globalLoopCounter = 0;
    lastReportTime = currentMillis;

    float sweepTimeUs = 1000000.0 / scans;
    float channelSpacingUs = sweepTimeUs / CHANNELS_IN_USE;

    Serial.println("\n=================== DIAGNOSTIC REPORT ===================");
    Serial.printf("Active Channels Polled: %u / 16\n", CHANNELS_IN_USE);
    Serial.print("SPI Clock: ");
    Serial.println(SPI_CLK_SPEED);
    Serial.printf("[SYSTEM ENGINE SPEED] Full-Kit Scans Per Second: %u Hz\n", scans);
    Serial.printf("[TIMING] Full Kit Sweep Loop Time: %.2f us\n", sweepTimeUs);
    Serial.printf("[TIMING] Time Spacing Between Individual Channel Readings: %.2f us\n", channelSpacingUs);
    Serial.println("-------------------------------------------------------");
    Serial.println("Channel Hit Tracking Breakdown (Past 1 Second Interval):");

    bool hitsRecorded = false;
    for (uint8_t i = 0; i < CHANNELS_IN_USE; i++) {
      if (channelHitCounter[i] > 0) {
        uint16_t rawAdc = channelMaxPeakReport[i];
        float calculatedVolts = ((float)rawAdc / 1023.0f) * ADC_VREF;

        Serial.printf(" -> CH %2u | Hits: %3u | Max Raw Peak: %4u / 1023 | Calculated Voltage: %.2f V\n",
                      i, channelHitCounter[i], rawAdc, calculatedVolts);
        hitsRecorded = true;
      }
      channelHitCounter[i] = 0;
      channelMaxPeakReport[i] = 0;
    }
    if (!hitsRecorded) Serial.println("   [No pads broken threshold limits this second interval]");
    Serial.println("=========================================================\n");
  }

  // send a midi note-on message if a channel has been triggered
  for (int i = 0; i < CHANNELS_IN_USE; i++) {
    if (noteReadyToSend[i]) {
      noteReadyToSend[i] = false;

      if (noteActive[i]) {
        MIDI.sendNoteOff(drumNotes[i], 0, MIDI_CHANNEL);  // if a note-on is being sent, cancel any existing note-on state to avoid multiple note-ons
      }
      // Temporarily only send midi messages for the first 8 ch 
      // because there is no second ADC chip on the breadboard and 
      // those last 8 inputs will read as max voltage and keep triggering
      if (i<8) MIDI.sendNoteOn(drumNotes[i], outVelocity[i], MIDI_CHANNEL);
      noteActive[i] = true;
      noteEndTime[i] = currentMillis + 25;
    }
  }

  // if a midi note has been on for a certain duration, turn it off
  for (int i = 0; i < CHANNELS_IN_USE; i++) {
    if (noteActive[i] && (currentMillis >= noteEndTime[i])) {
      MIDI.sendNoteOff(drumNotes[i], 0, MIDI_CHANNEL);
      noteActive[i] = false;
    }
  }
}
