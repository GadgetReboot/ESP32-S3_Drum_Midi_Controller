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
//
// ---------------------------------------------------------------------------
// TRIGGER ALGORITHM
//
// 1. ENVELOPE / DYNAMIC THRESHOLD
//    Every channel keeps a running "envelope" of its recent ADC level: instant
//    attack, a short hold, then exponential decay (ENV_DECAY_TAU_US). The
//    trigger threshold is always just above the envelope:
//        threshold = max(MIN_THRESHOLD, envelope*(1+RETRIG_RATIO) + RETRIG_ABS_MARGIN)
//    Ringing of a previous hit stays below it, a genuinely new strike rises
//    above it, so quiet buzz-roll strokes / ghost notes get through.
//
// 2. HIT-STRENGTH-SCALED MASK (rebound guard)
//    After a hit is reported, the channel is masked for a time that grows with
//    the hit's strength (MASK_MIN_US for soft hits up to MASK_MAX_US for the
//    hardest). While masked, a new trigger must additionally exceed
//    a multiple of the previous hit's peak that starts at MASK_OVERRIDE_RATIO and
//    relaxes linearly to MASK_OVERRIDE_RATIO_END by the end of the mask (ring is
//    strongest right after a hit and fades, so early ring is blocked strictly while
//    a similar-strength stroke later in the mask can still pass; a louder follow-up
//    always passes). Hard hits ring/rebound longer than soft ones, so this stops
//    multiple note-ons on a hard hit without slowing down soft buzz strokes.
//
// 2b. OPTIONAL SECOND MASK TIER (MASK2_ENABLED)
//    Only for very hard hits (peak >= MASK2_MIN_PEAK). After the main mask ends,
//    a further window (up to MASK2_MAX_US) requires a new hit to exceed
//    MASK2_RATIO x the previous peak. This blocks late rebounds/plate bounce
//    after a hard hit while letting a comparably strong follow-up through.
//    Medium and soft hits never get a second tier, so buzz rolls are untouched.
//    Set MASK2_ENABLED to 0 and the sketch behaves exactly as without it.
//
// 3. PRE-HIT OFFSET
//    The envelope value just before a trigger is saved. Velocity is computed
//    from  net = peak - PRE_HIT_OFFSET_WEIGHT * preHit  so a soft hit riding on
//    a decaying loud hit is scaled by how far it actually rose.
// ---------------------------------------------------------------------------

#include "midi_percussion_notes.h"  // list of midi percussion instruments and note numbers
#include <SPI.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "soc/soc.h"       // REG_WRITE
#include "soc/gpio_reg.h"  // GPIO set/clear registers used for fast chip-select toggling

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
const uint8_t CHANNELS_IN_USE = 8;

#define ADC_VREF 3.3  // Voltage Reference of the MCP3008 (5.0 or 3.3 volts expected)

// --- DEBUG OUTPUT SWITCHES ---
#define DEBUG_HITS 1        // 1 = print one line per detected hit (peak, pre-hit offset, net, velocity, gap since previous hit)
#define SHOW_IDLE_NOISE 1   // 1 = include per-channel idle noise levels in the once-per-second report
#define SHOW_SECOND_REPORT 1  // 1 = print the once-per-second diagnostic report at all

// Blocked-hit debug: reports the strongest hit-like signal that was REJECTED after a hit, and what rejected it.
// Printed once BLOCK_EPISODE_US after that hit (or when the next real hit arrives), as up to TWO lines per hit:
//   EARLY = inside the same-strike guard (the first MIN_RETRIGGER_US of the strike): the second hump / ring of the
//           same strike. Almost always present, and usually not interesting.
//   LATE  = after that: ring, rebound, or a genuine follow-up stroke (buzz bounce, flam) that was held back by the
//           envelope threshold or a mask. This is the line to read when strokes seem to be missing.
// Use "after hit" ms and the % of the needed level to tell ring from a real second stroke (a flam stroke typically
// lands 10-40 ms after the first at a high %).
// Has no effect on triggering, only on what is printed.
#define DEBUG_BLOCKED 1
#define DEBUG_BLOCKED_MIN_PEAK 80       // ignore blocked signals below this raw level
#define DEBUG_BLOCKED_MIN_RATIO 0.30f   // ignore blocked signals below this fraction of the previous hit's peak
#define BLOCK_EPISODE_US 100000         // how long after a hit the strongest blocked signal is collected

// Raw waveform capture (for offline analysis of how a single hit and a flam actually look).
// Records one channel's raw samples plus the envelope and threshold the sketch was using, from a little before a
// fresh strike to CAPTURE_POST_MS after it (so a flam's second stroke is inside the recording), then prints them
// as CSV lines starting with "CAP," once the recording is complete. Triggering and MIDI are NOT affected.
// Copy the lines from CAP_BEGIN to CAP_END out of the serial monitor to a text file and plot/share them.
// flags column (add the values): 1 = trigger started, 2 = note sent, 4 = main mask active, 8 = second tier active,
//                                16 = rejected as crosstalk, 32 = fresh strike (channel was quiet before it),
//                                64 = rejected by the same-strike guard (it would otherwise have triggered)
#define CAPTURE_MODE 0
#define CAPTURE_CHANNEL 1             // which ADC channel to record (0-15)
#define CAPTURE_PRE_MS 5              // milliseconds recorded before the trigger
#define CAPTURE_POST_MS 80            // milliseconds recorded after the trigger
#define CAPTURE_MAX_PREHIT 60         // a capture only starts when the channel's level before the hit was below this
#define CAPTURE_BUF_SAMPLES 3000      // buffer size in samples (12 bytes each). Must cover PRE+POST ms at your scan rate

// --- TRIGGER TUNING CONSTANTS ---
#define PEAK_WINDOW_US 3000    // window after threshold crossing in which the hit's peak is harvested (microseconds)
                               // also the delay before the note is sent. Must be long enough to reach the strike's main hump
                               // (about 2-4 ms on the author's rubber pad); 2000 was too short, 5000 raised the softest velocity
#define MIN_RETRIGGER_US 8000  // same-strike guard: a new hit is rejected if it starts less than this long after the START
                               // of the previous strike, whatever its level. A strike's second hump (3-6 ms later) can be
                               // 15-60% larger than the first and would otherwise trigger a second note. 0 = off.
                               // Only affects strokes closer than this together (a flam is normally 10-40 ms).
#define X_TALK_WINDOW_US 2000  // filter window for channel crosstalk rejection (microseconds)
#define X_TALK_RATIO 0.25f

// Envelope follower / dynamic threshold
#define ENV_HOLD_US 3000           // envelope is held at its peak this long before decay starts
#define ENV_DECAY_TAU_US 12000.0f  // envelope decay time constant. Smaller = easier buzz rolls/ghost notes,
                                   // larger = better double-trigger rejection
#define RETRIG_RATIO 0.08f         // threshold sits this fraction above the envelope...
#define RETRIG_ABS_MARGIN 12.0f    // ...plus this many ADC counts

// Hit-strength-scaled mask. Mask length interpolates between MIN and MAX with the hit's peak
// (MIN_THRESHOLD -> MASK_MIN_US ... MAX_THRESHOLD -> MASK_MAX_US).
#define MASK_MIN_US 3000          // mask after a very soft hit
#define MASK_MAX_US 40000         // mask after the hardest hit. Raise if hard hits still double-trigger
#define MASK_OVERRIDE_RATIO 0.95f     // during the mask a new hit must exceed (last hit peak x ratio) to pass.
                                      // This is the ratio right after the hit; it relaxes linearly to the END value
                                      // below by the time the mask ends. ~0.95-1.0 right after a hit blocks the strong
                                      // early ring; much lower (0.7-0.8) lets plate ring through as extra note-ons
#define MASK_OVERRIDE_RATIO_END 0.80f // ratio at the end of the mask. Set equal to MASK_OVERRIDE_RATIO for a fixed ratio
                                      // (exactly the previous behaviour)

// Optional second mask tier for very hard hits only (see header). 0 = disabled, identical to no tier.
#define MASK2_ENABLED 1
#define MASK2_MIN_PEAK 400        // hits with a raw peak below this get no second tier at all
#define MASK2_MAX_US 30000        // extra window after the main mask for a full-strength hit
                                  // (scales from 0 at MASK2_MIN_PEAK up to this at MAX_THRESHOLD)
#define MASK2_RATIO 0.5f          // during the second tier a new hit must exceed (last hit peak x this)

// Velocity offset compensation
// 0.0 = ignore pre-hit level, 1.0 = subtract the full envelope level.
#define PRE_HIT_OFFSET_WEIGHT 0.2f

// Idle noise measurement: samples are only counted when a channel has been quiet this long
#define NOISE_SETTLE_US 150000

#define MIDI_CHANNEL 10              //  percussion midi channel 10
#define MAX_CHANNELS 16              //  total of 16 channels available across 2 chips to trigger midi notes
const uint16_t MIN_THRESHOLD = 30;   //  Trigger floor: lowest ADC level that can start a hit
const uint16_t MAX_THRESHOLD = 700;  //  Hard hit limit: net peak that maps to velocity 127
                                     //  (set it just above your hardest real hit)

// --- VELOCITY SCALING ---
// net peak <= VEL_SCALE_LOW_PEAK  -> MIN_VELOCITY
// net peak >= MAX_THRESHOLD       -> 127
// in between follows the selected curve.
// To make soft hits quieter: raise VEL_SCALE_LOW_PEAK, raise VELOCITY_CURVE_EXPONENT
// (1.0 = linear, >1 = even more reserved at the soft end, <1 = boosts soft hits), or lower MIN_VELOCITY.
#define VEL_SCALE_LOW_PEAK 170  // set this just below the raw peak of your lightest intentional tap (see the HIT lines)
#define MIN_VELOCITY 8
#define VELOCITY_CURVE_EXPONENT 1.2f  // used by CURVE_POWER

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

// A detected hit travels from the scan task to the MIDI sender through this queue,
// so back-to-back hits can never overwrite each other.
struct HitEvent {
  uint8_t channel;
  uint8_t velocity;
  uint16_t peak;     // raw harvested peak
  uint16_t preHit;   // envelope level just before the hit
  int16_t net;       // peak minus weighted pre-hit offset
  uint16_t gapMs;    // ms since previous reported hit on this channel (capped)
};
QueueHandle_t hitQueue = NULL;

// debug reporting (written by scan task, read/cleared by loop)
volatile uint32_t channelHitCounter[MAX_CHANNELS] = { 0 };
volatile uint16_t channelMaxPeakReport[MAX_CHANNELS] = { 0 };
volatile uint16_t channelLastPreHit[MAX_CHANNELS] = { 0 };
volatile uint8_t channelLastVelocity[MAX_CHANNELS] = { 0 };
volatile uint32_t droppedCrosstalk[MAX_CHANNELS] = { 0 };
volatile uint32_t droppedQueueFull = 0;

// idle noise statistics (below-threshold samples while the channel has been quiet)
volatile uint16_t noiseMax[MAX_CHANNELS] = { 0 };
volatile uint32_t noiseSum[MAX_CHANNELS] = { 0 };
volatile uint32_t noiseCount[MAX_CHANNELS] = { 0 };

bool noteActive[MAX_CHANNELS] = { false };   // whether a certain channel note has been triggered
uint32_t noteEndTime[MAX_CHANNELS] = { 0 };  // how long to wait until note should be turned back off so it's not hanging

uint16_t currentReading[MAX_CHANNELS] = { 0 };  // adc sample being read

// per-channel envelope tracker
float envelope[MAX_CHANNELS] = { 0 };           // decaying peak-hold of the channel's ADC level
uint32_t envHoldUntilUs[MAX_CHANNELS] = { 0 };  // time until which envelope is held before it begins to decay

// per-channel mask state
uint32_t maskUntilUs[MAX_CHANNELS] = { 0 };    // mask end time after the last hit
uint32_t mask2UntilUs[MAX_CHANNELS] = { 0 };   // end of the optional second tier (== maskUntilUs when unused)
uint32_t maskStartUs[MAX_CHANNELS] = { 0 };    // when the main mask began (for the relaxing override ratio)
float maskRatioSlope[MAX_CHANNELS] = { 0 };    // change in override ratio per microsecond over this hit's mask
float lastHitPeak[MAX_CHANNELS] = { 0 };       // peak of the last hit (used for mask override test)
uint32_t lastHitEndUs[MAX_CHANNELS] = { 0 };   // when the last hit window finished (for idle noise settle)
uint32_t lastReportedHitUs[MAX_CHANNELS] = { 0 };  // when the last note was reported (for gap debug)

// per-channel hit capture
bool isHarvesting[MAX_CHANNELS] = { false };     // trigger has occurred and peak harvesting is in progress
uint16_t harvestedPeak[MAX_CHANNELS] = { 0 };    // highest sample seen in this trigger event
float preHitLevel[MAX_CHANNELS] = { 0 };         // envelope level right before this trigger began (the "offset")
uint32_t windowStartUs[MAX_CHANNELS] = { 0 };    // when the trigger began

volatile uint32_t groupTriggerTimeUs[MAX_CHANNELS] = { 0 };  // crosstalk filter - check to see if triggered events occur in close proximity to other events
volatile uint16_t groupTriggerPeak[MAX_CHANNELS] = { 0 };    // and how their peaks compare (a weak hit right next to a strong hit is probably crosstalk pickup)

enum VelocityCurveMode {  // options to scale adc trigger levels to midi note velocity in different ways
  CURVE_LINEAR,
  CURVE_SCURVE,
  CURVE_LOGARITHMIC,
  CURVE_EXPONENTIAL,
  CURVE_POWER  // y = x^VELOCITY_CURVE_EXPONENT, adjustable
};

const VelocityCurveMode ACTIVE_CURVE = CURVE_POWER;

volatile uint32_t globalLoopCounter = 0;
volatile uint32_t maxSweepGapUs = 0;  // longest gap between two sweeps in the last report interval (stalls show up here)
TaskHandle_t AnalogScanTaskHandle = NULL;

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// the strength of a drum hit is scaled against selectable curves so that
// the midi note velocity (drum hit intensity) maps the trigger strength to the note volume/feel
// netPeak is the hit's peak AFTER the pre-hit offset has been subtracted
uint8_t calculateCurveVelocity(float netPeak, VelocityCurveMode mode) {

  // for readings outside of expected signal range, clamp to min or max
  if (netPeak <= VEL_SCALE_LOW_PEAK) return MIN_VELOCITY;
  if (netPeak >= MAX_THRESHOLD) return 127;

  // normalize x from 0.0 to 1.0
  float x = (netPeak - VEL_SCALE_LOW_PEAK) / (float)(MAX_THRESHOLD - VEL_SCALE_LOW_PEAK);
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
    case CURVE_POWER:
      y = powf(x, VELOCITY_CURVE_EXPONENT);
      break;
  }

  // scale output between MIN_VELOCITY and 127
  float velocityOut = MIN_VELOCITY + y * (127.0f - MIN_VELOCITY);
  if (velocityOut < (float)MIN_VELOCITY) velocityOut = (float)MIN_VELOCITY;
  if (velocityOut > 127.0f) velocityOut = 127.0f;
  return (uint8_t)(velocityOut + 0.5f);
}

// dynamic trigger threshold: always just above the channel's current decaying envelope
inline float dynamicThreshold(float env) {
  float t = env * (1.0f + RETRIG_RATIO) + RETRIG_ABS_MARGIN;
  return (t > (float)MIN_THRESHOLD) ? t : (float)MIN_THRESHOLD;
}

// mask duration after a hit: longer for harder hits
inline uint32_t maskDurationUs(float peak) {
  float frac = (peak - (float)MIN_THRESHOLD) / (float)(MAX_THRESHOLD - MIN_THRESHOLD);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  return (uint32_t)(MASK_MIN_US + frac * (float)(MASK_MAX_US - MASK_MIN_US));
}

// --- blocked-hit debug support (strongest rejected signal per hit) ---
struct BlockedEvent {
  uint8_t channel;
  uint8_t limiter;     // 0 = envelope threshold, 1 = main mask, 2 = second-tier mask, 3 = same-strike guard
  uint8_t band;        // 0 = EARLY (inside the same-strike guard time), 1 = LATE (after it)
  uint16_t peak;       // strongest rejected sample
  uint16_t needed;     // threshold it had to beat at that moment
  uint16_t lastPeak;   // peak of the previous reported hit
  uint16_t sinceMs;    // ms between the previous hit and the blocked sample
};
QueueHandle_t blockedQueue = NULL;
// one set per band: [0] = EARLY, [1] = LATE
uint16_t blockedPeak[2][MAX_CHANNELS] = { { 0 }, { 0 } };
uint16_t blockedNeeded[2][MAX_CHANNELS] = { { 0 }, { 0 } };
uint16_t blockedSinceMs[2][MAX_CHANNELS] = { { 0 }, { 0 } };
uint8_t blockedLimiter[2][MAX_CHANNELS] = { { 0 }, { 0 } };

inline bool hasBlocked(uint8_t i) {
  return blockedPeak[0][i] > 0 || blockedPeak[1][i] > 0;
}

// send the collected blocked signals for a channel (if any) and clear them
inline void emitBlocked(uint8_t i) {
  for (uint8_t b = 0; b < 2; b++) {
    if (blockedPeak[b][i] == 0) continue;
    BlockedEvent be;
    be.channel = i;
    be.limiter = blockedLimiter[b][i];
    be.band = b;
    be.peak = blockedPeak[b][i];
    be.needed = blockedNeeded[b][i];
    be.lastPeak = (uint16_t)lastHitPeak[i];
    be.sinceMs = blockedSinceMs[b][i];
    xQueueSend(blockedQueue, &be, 0);  // dropped silently if full: debug only
    blockedPeak[b][i] = 0;
  }
}

// --- raw waveform capture support (see CAPTURE_MODE) ---
#define CAP_F_TRIGGER 1
#define CAP_F_NOTE 2
#define CAP_F_MASK1 4
#define CAP_F_MASK2 8
#define CAP_F_XTALK 16
#define CAP_F_FRESH 32
#define CAP_F_SAMESTRIKE 64

#if CAPTURE_MODE
struct CaptureSample {
  uint32_t tUs;
  uint16_t val;
  uint16_t env;
  uint16_t thr;
  uint8_t flags;
};
static CaptureSample capBuf[CAPTURE_BUF_SAMPLES];
volatile uint8_t capState = 0;  // 0 = armed (rolling buffer), 1 = collecting after trigger, 2 = ready to print, 3 = printing
uint32_t capHead = 0;           // next write index
uint32_t capCount = 0;          // valid samples in the buffer
uint32_t capTrigTimeUs = 0;
uint32_t capPrintIdx = 0;
uint32_t capPrintLeft = 0;

// called by the scan task once per sweep for the capture channel
inline void captureSample(uint32_t nowUs, uint16_t val, float env, float thr, uint8_t flags) {
  if (capState >= 2) return;  // frozen while the finished recording is being printed
  CaptureSample& s = capBuf[capHead];
  s.tUs = nowUs;
  s.val = val;
  s.env = (uint16_t)env;
  s.thr = (uint16_t)(thr > 65535.0f ? 65535.0f : thr);
  s.flags = flags;
  if (capState == 0 && (flags & CAP_F_TRIGGER) && (flags & CAP_F_FRESH)) {
    capState = 1;
    capTrigTimeUs = nowUs;
  }
  capHead++;
  if (capHead >= CAPTURE_BUF_SAMPLES) capHead = 0;
  if (capCount < CAPTURE_BUF_SAMPLES) capCount++;
  if (capState == 1 && (uint32_t)(nowUs - capTrigTimeUs) >= (uint32_t)CAPTURE_POST_MS * 1000UL) capState = 2;
}

// called from loop(): prints a finished recording a few lines at a time, without ever blocking on serial output
inline void capturePrintStep() {
  if (capState == 2) {
    uint32_t idx = (capHead + CAPTURE_BUF_SAMPLES - capCount) % CAPTURE_BUF_SAMPLES;  // oldest sample
    uint32_t left = capCount;
    const int32_t preUs = -(int32_t)((uint32_t)CAPTURE_PRE_MS * 1000UL);
    while (left > 0 && (int32_t)(capBuf[idx].tUs - capTrigTimeUs) < preUs) {  // skip samples older than the pre-trigger window
      idx = (idx + 1) % CAPTURE_BUF_SAMPLES;
      left--;
    }
    capPrintIdx = idx;
    capPrintLeft = left;
    Serial.printf("CAP_BEGIN ch=%u samples=%lu pre_ms=%u post_ms=%u\n", (unsigned)CAPTURE_CHANNEL,
                  (unsigned long)left, (unsigned)CAPTURE_PRE_MS, (unsigned)CAPTURE_POST_MS);
    Serial.println("CAP_COLS t_us,raw,env,thr,flags");
    capState = 3;
    return;
  }
  if (capState != 3) return;

  uint8_t lines = 0;
  while (capPrintLeft > 0 && lines < 32 && Serial.availableForWrite() >= 48) {
    const CaptureSample& s = capBuf[capPrintIdx];
    Serial.printf("CAP,%ld,%u,%u,%u,%u\n", (long)(int32_t)(s.tUs - capTrigTimeUs), (unsigned)s.val,
                  (unsigned)s.env, (unsigned)s.thr, (unsigned)s.flags);
    capPrintIdx = (capPrintIdx + 1) % CAPTURE_BUF_SAMPLES;
    capPrintLeft--;
    lines++;
  }
  if (capPrintLeft == 0) {
    Serial.println("CAP_END");
    capCount = 0;
    capState = 0;  // re-arm
  }
}
#endif

inline bool captureIsPrinting() {
#if CAPTURE_MODE
  return capState == 3;
#else
  return false;
#endif
}

// extra second-tier mask length after the main mask: zero unless the hit was very hard
inline uint32_t mask2DurationUs(float peak) {
  float frac = (peak - (float)MASK2_MIN_PEAK) / (float)(MAX_THRESHOLD - MASK2_MIN_PEAK);
  if (frac <= 0.0f) return 0;
  if (frac > 1.0f) frac = 1.0f;
  return (uint32_t)(frac * (float)MASK2_MAX_US);
}

// read a sample from a specified adc MCP3008 chip and channel
// Reduced per-read overhead compared with the original version (same SPI hardware, same 3-byte exchange):
//  - the 3 bytes go out in ONE transferBytes() call instead of three separate SPI.transfer() calls
//  - chip select is toggled with a single GPIO register write instead of digitalWrite()
// NOTE: the chip-select pins must be GPIO 0-31 for the single-register fast path (GPIO 9 and 10 are).
// This function must only be used inside an SPI.beginTransaction()/endTransaction() pair.
inline uint16_t readMCP3008_FastArduino(uint8_t chipSelect, uint8_t channel) {
  const uint32_t csMask = 1UL << chipSelect;
  const uint8_t tx[3] = { 0x01, (uint8_t)((0x08 | channel) << 4), 0x00 };
  uint8_t rx[3];

  REG_WRITE(GPIO_OUT_W1TC_REG, csMask);  // CS low
  SPI.transferBytes(tx, rx, 3);
  REG_WRITE(GPIO_OUT_W1TS_REG, csMask);  // CS high

  return ((rx[1] & 0x03) << 8) | rx[2];
}

// scan all adc inputs and look for trigger events
void AnalogScanTask(void* pvParameters) {
  uint32_t yieldCounter = 0;
  uint32_t lastSweepUs = micros();

  for (;;) {
    uint32_t nowUs = micros();

    // time since previous sweep -> per-sweep envelope decay multiplier (independent of scan speed)
    uint32_t dtUs = nowUs - lastSweepUs;
    lastSweepUs = nowUs;
    if (dtUs > maxSweepGapUs) maxSweepGapUs = dtUs;  // longest time between sweeps (reported once a second)
    // exp(-x) via a short series for the normal tiny per-sweep step (faster than expf), exact expf if a sweep was delayed
    float x = (float)dtUs / ENV_DECAY_TAU_US;
    float decayMul = (x < 0.05f) ? (1.0f - x + 0.5f * x * x) : expf(-x);

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
      float env = envelope[i];  // envelope level BEFORE this sample is folded in
      float thrUsed = 0.0f;     // threshold applied to this sample (0 while a hit is being harvested), for the capture
      uint8_t capFlags = 0;     // event markers for the capture (see CAP_F_*)

      if (isHarvesting[i]) {
        // a hit is in progress: keep the largest sample seen during the peak window
        if (val > harvestedPeak[i]) {
          harvestedPeak[i] = val;
        }

        if ((uint32_t)(nowUs - windowStartUs[i]) >= PEAK_WINDOW_US) {  // peak window finished
          isHarvesting[i] = false;

          // filter for crosstalk false triggers
          bool isCrosstalk = false;
          for (uint8_t neighbor = 0; neighbor < CHANNELS_IN_USE; neighbor++) {
            if (neighbor == i) continue;
            if ((uint32_t)(nowUs - groupTriggerTimeUs[neighbor]) <= X_TALK_WINDOW_US) {            // another ch recently triggered
              if ((float)harvestedPeak[i] < ((float)groupTriggerPeak[neighbor] * X_TALK_RATIO)) {  // and this one is far weaker -> pickup
                isCrosstalk = true;
                break;
              }
            }
          }

          if (!isCrosstalk) {
            groupTriggerTimeUs[i] = nowUs;
            groupTriggerPeak[i] = harvestedPeak[i];

            // true strength of this hit = peak minus where the signal already was just before the hit
            float netPeak = (float)harvestedPeak[i] - PRE_HIT_OFFSET_WEIGHT * preHitLevel[i];
            uint8_t vel = calculateCurveVelocity(netPeak, ACTIVE_CURVE);

            uint32_t gapMs = (nowUs - lastReportedHitUs[i]) / 1000UL;
            if (gapMs > 65535UL) gapMs = 65535UL;
            lastReportedHitUs[i] = nowUs;

            HitEvent ev;
            ev.channel = i;
            ev.velocity = vel;
            ev.peak = harvestedPeak[i];
            ev.preHit = (uint16_t)preHitLevel[i];
            ev.net = (int16_t)netPeak;
            ev.gapMs = (uint16_t)gapMs;
            if (xQueueSend(hitQueue, &ev, 0) != pdTRUE) droppedQueueFull++;
            capFlags |= CAP_F_NOTE;

            // track trigger events for debug reporting
            channelHitCounter[i]++;
            channelLastPreHit[i] = (uint16_t)preHitLevel[i];
            channelLastVelocity[i] = vel;
            if (harvestedPeak[i] > channelMaxPeakReport[i]) {
              channelMaxPeakReport[i] = harvestedPeak[i];
            }
          } else {
            droppedCrosstalk[i]++;
            capFlags |= CAP_F_XTALK;
          }

          // mask scales with the strength of the hit (hard hits ring/rebound longer)
          lastHitPeak[i] = (float)harvestedPeak[i];
          uint32_t maskLenUs = maskDurationUs((float)harvestedPeak[i]);
          maskStartUs[i] = nowUs;
          maskRatioSlope[i] = (MASK_OVERRIDE_RATIO_END - MASK_OVERRIDE_RATIO) / (float)maskLenUs;
          maskUntilUs[i] = nowUs + maskLenUs;
#if MASK2_ENABLED
          mask2UntilUs[i] = maskUntilUs[i] + mask2DurationUs((float)harvestedPeak[i]);
#else
          mask2UntilUs[i] = maskUntilUs[i];
#endif
          lastHitEndUs[i] = nowUs;
        }
      } else {
        // idle: a new hit begins when the sample rises above the dynamic threshold
        float thr = dynamicThreshold(env);
        uint8_t limiter = 0;  // what is setting the threshold: 0 envelope, 1 main mask, 2 second-tier mask

#if DEBUG_BLOCKED
        // after the collection window, report the strongest rejected signals from the last hit
        if (hasBlocked(i) && (uint32_t)(nowUs - lastHitEndUs[i]) > BLOCK_EPISODE_US) emitBlocked(i);
#endif

        // same-strike guard: still inside the first MIN_RETRIGGER_US of the previous strike (measured from its start)
        const bool sameStrike = (MIN_RETRIGGER_US > 0) && ((uint32_t)(nowUs - windowStartUs[i]) < (uint32_t)MIN_RETRIGGER_US);

        // while masked after a previous hit, a new hit must also beat that hit's peak
        if ((int32_t)(nowUs - maskUntilUs[i]) < 0) {
          // override ratio relaxes linearly from MASK_OVERRIDE_RATIO to MASK_OVERRIDE_RATIO_END over the mask
          float ratio = MASK_OVERRIDE_RATIO + maskRatioSlope[i] * (float)(uint32_t)(nowUs - maskStartUs[i]);
          float maskThr = lastHitPeak[i] * ratio;
          if (maskThr > thr) {
            thr = maskThr;
            limiter = 1;
          }
        }
#if MASK2_ENABLED
        else if ((int32_t)(nowUs - mask2UntilUs[i]) < 0) {
          // second tier (very hard hits only): late rebound must clear a fraction of the last peak
          float mask2Thr = lastHitPeak[i] * MASK2_RATIO;
          if (mask2Thr > thr) {
            thr = mask2Thr;
            limiter = 2;
          }
        }
#endif

        thrUsed = thr;
        if (limiter == 1) capFlags |= CAP_F_MASK1;
        else if (limiter == 2) capFlags |= CAP_F_MASK2;

        if (sameStrike && (float)val > thr) capFlags |= CAP_F_SAMESTRIKE;

        if ((float)val > thr && !sameStrike) {
#if DEBUG_BLOCKED
          emitBlocked(i);  // a real hit is starting: report whatever was rejected before it
#endif
          isHarvesting[i] = true;
          windowStartUs[i] = nowUs;
          harvestedPeak[i] = val;
          preHitLevel[i] = env;  // remember where the signal was just before this hit
          capFlags |= CAP_F_TRIGGER;
          if (env < (float)CAPTURE_MAX_PREHIT) capFlags |= CAP_F_FRESH;
        } else {
#if DEBUG_BLOCKED
          // a hit-like sample was rejected: remember the strongest one since the last hit
          const uint8_t band = sameStrike ? 0 : 1;  // 0 = EARLY (same strike), 1 = LATE
          if (val >= DEBUG_BLOCKED_MIN_PEAK && val > MIN_THRESHOLD && (float)val >= lastHitPeak[i] * DEBUG_BLOCKED_MIN_RATIO
              && (uint32_t)(nowUs - lastHitEndUs[i]) <= BLOCK_EPISODE_US && val > blockedPeak[band][i]) {
            blockedPeak[band][i] = val;
            blockedNeeded[band][i] = (uint16_t)thr;
            blockedLimiter[band][i] = (sameStrike && (float)val > thr) ? 3 : limiter;  // 3 = it beat the threshold but the guard rejected it
            uint32_t sm = (nowUs - lastHitEndUs[i]) / 1000UL;
            blockedSinceMs[band][i] = (uint16_t)(sm > 65535UL ? 65535UL : sm);
          }
#endif
          if ((uint32_t)(nowUs - lastHitEndUs[i]) > NOISE_SETTLE_US) {
            // channel is quiet and settled: record idle noise statistics
            if (val > noiseMax[i]) noiseMax[i] = val;
            noiseSum[i] += val;
            noiseCount[i]++;
          }
        }
      }

      // update the envelope with this sample: instant attack, hold, then exponential decay
      if ((float)val >= env) {
        env = (float)val;
        envHoldUntilUs[i] = nowUs + ENV_HOLD_US;
      } else if ((int32_t)(nowUs - envHoldUntilUs[i]) >= 0) {
        env *= decayMul;
        if (env < 1.0f) env = 0.0f;
      }
      envelope[i] = env;

#if CAPTURE_MODE
      if (i == CAPTURE_CHANNEL) captureSample(nowUs, val, env, thrUsed, capFlags);
#endif
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

  hitQueue = xQueueCreate(64, sizeof(HitEvent));
  blockedQueue = xQueueCreate(16, sizeof(BlockedEvent));

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

#if SHOW_SECOND_REPORT
  // print debug messages every second
  if (currentMillis - lastReportTime >= 1000 && !captureIsPrinting()) {  // report waits while a capture is being printed
    uint32_t scans = globalLoopCounter;
    globalLoopCounter = 0;
    lastReportTime = currentMillis;
    if (scans == 0) scans = 1;

    float sweepTimeUs = 1000000.0 / scans;
    float channelSpacingUs = sweepTimeUs / CHANNELS_IN_USE;

    Serial.println("\n=================== DIAGNOSTIC REPORT ===================");
    Serial.printf("Active Channels Polled: %u / 16\n", CHANNELS_IN_USE);
    Serial.print("SPI Clock: ");
    Serial.println(SPI_CLK_SPEED);
    Serial.printf("[SYSTEM ENGINE SPEED] Full-Kit Scans Per Second: %u Hz\n", scans);
    Serial.printf("[TIMING] Full Kit Sweep Loop Time: %.2f us\n", sweepTimeUs);
    Serial.printf("[TIMING] Time Spacing Between Individual Channel Readings: %.2f us\n", channelSpacingUs);
    uint32_t worstGapUs = maxSweepGapUs;
    maxSweepGapUs = 0;
    Serial.printf("[TIMING] Longest Gap Between Sweeps: %lu us (should stay close to the sweep time; ~1000+ means the scan task is being interrupted)\n",
                  (unsigned long)worstGapUs);
    Serial.printf("[QUEUE] Events dropped (queue full): %u\n", (unsigned)droppedQueueFull);
    Serial.println("-------------------------------------------------------");
    Serial.println("Channel Hit Tracking Breakdown (Past 1 Second Interval):");

    bool hitsRecorded = false;
    for (uint8_t i = 0; i < CHANNELS_IN_USE; i++) {
      if (channelHitCounter[i] > 0 || droppedCrosstalk[i] > 0) {
        uint16_t rawAdc = channelMaxPeakReport[i];
        float calculatedVolts = ((float)rawAdc / 1023.0f) * ADC_VREF;

        Serial.printf(" -> CH %2u | Hits: %3u | Max Peak: %4u | %.2f V | Last PreHit Offset: %4u | Last Vel: %3u | Xtalk rejected: %u\n",
                      i, (unsigned)channelHitCounter[i], rawAdc, calculatedVolts,
                      channelLastPreHit[i], channelLastVelocity[i], (unsigned)droppedCrosstalk[i]);
        hitsRecorded = true;
      }
      channelHitCounter[i] = 0;
      channelMaxPeakReport[i] = 0;
      droppedCrosstalk[i] = 0;
    }
    if (!hitsRecorded) Serial.println("   [No pads broke threshold limits this second interval]");

#if SHOW_IDLE_NOISE
    Serial.printf("Idle Noise (quiet samples only, trigger floor = %u counts):\n", (unsigned)MIN_THRESHOLD);
    for (uint8_t i = 0; i < CHANNELS_IN_USE; i++) {
      uint32_t cnt = noiseCount[i];
      uint16_t mx = noiseMax[i];
      uint32_t sm = noiseSum[i];
      noiseMax[i] = 0;
      noiseSum[i] = 0;
      noiseCount[i] = 0;
      if (cnt > 0) {
        Serial.printf(" -> CH %2u | Noise Max: %3u | Noise Mean: %5.1f | Samples: %lu\n",
                      i, mx, (float)sm / (float)cnt, (unsigned long)cnt);
      } else {
        Serial.printf(" -> CH %2u | (no quiet samples - pad was active)\n", i);
      }
    }
#endif
    Serial.println("=========================================================\n");
  }
#endif

  // send a midi note-on message for every hit the scan task has queued
  HitEvent ev;
  while (xQueueReceive(hitQueue, &ev, 0) == pdTRUE) {
    uint8_t i = ev.channel;

    if (noteActive[i]) {
      MIDI.sendNoteOff(drumNotes[i], 0, MIDI_CHANNEL);  // cancel any existing note-on state to avoid stacked note-ons
    }

    MIDI.sendNoteOn(drumNotes[i], ev.velocity, MIDI_CHANNEL);
    noteActive[i] = true;
    noteEndTime[i] = currentMillis + 25;

#if DEBUG_HITS
    // printed after the MIDI message is sent so debug output never delays the note
    Serial.printf("HIT ch%2u | peak %4u | pre %4u | net %4d | vel %3u | gap %5u ms\n",
                  (unsigned)i, (unsigned)ev.peak, (unsigned)ev.preHit, (int)ev.net,
                  (unsigned)ev.velocity, (unsigned)ev.gapMs);
#endif
  }

#if DEBUG_BLOCKED
  // report hit-like signals that were rejected (printed after MIDI work so it never delays a note)
  BlockedEvent be;
  while (xQueueReceive(blockedQueue, &be, 0) == pdTRUE) {
    const char* by = (be.limiter == 1) ? "MASK1" : (be.limiter == 2) ? "MASK2" : (be.limiter == 3) ? "SAMESTRIKE" : "ENVELOPE";
    unsigned pct = be.needed ? ((unsigned)be.peak * 100U) / be.needed : 0;
    Serial.printf("BLOCKED ch%2u %-5s | peak %4u | needed %4u (%3u%%) | by %-10s | last hit peak %4u | %4u ms after hit\n",
                  (unsigned)be.channel, be.band == 0 ? "EARLY" : "LATE", (unsigned)be.peak, (unsigned)be.needed, pct, by,
                  (unsigned)be.lastPeak, (unsigned)be.sinceMs);
  }
#endif

#if CAPTURE_MODE
  capturePrintStep();  // prints a finished waveform recording in small non-blocking pieces
#endif

  // if a midi note has been on for a certain duration, turn it off
  for (int i = 0; i < CHANNELS_IN_USE; i++) {
    if (noteActive[i] && (currentMillis >= noteEndTime[i])) {
      MIDI.sendNoteOff(drumNotes[i], 0, MIDI_CHANNEL);
      noteActive[i] = false;
    }
  }

  // IMPORTANT: let the scan task have the CPU.
  // The Arduino loop task and AnalogScan run on the same core at the same priority, and loop() never blocks, so
  // FreeRTOS time-slices them 1 ms / 1 ms: the capture showed the ADC being sampled only ~45% of the time
  // (about 1 ms of samples, then a ~1.1 ms blind gap, repeating). Sleeping one tick here hands the whole core to the
  // scan task; loop() still runs about once per millisecond, which is plenty for MIDI output and serial reports.
  vTaskDelay(1);
}
