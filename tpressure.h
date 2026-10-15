// tpressure: stylus pressure for the 3DS touch screen.
//
// The touch controller can measure the pressure channels Z1 and Z2, but the OS makes it report only X and Y.
// While the pen is down, this module switches the report to include Z1 and Z2 and turns them into a pressure
// from 0 to 1. Z1 and Z2 replace the Circle Pad samples, so the Circle Pad reads wrong during a touch.
// On an Old 3DS the OS does not see the pen lift in the Z layout, so tp_sample switches back by itself.
//
// Usage, once per frame after hidScanInput():
//
//     char message[64];
//     bool hasPressure = tp_open(message, sizeof message);
//     ...
//     tp_update(hidKeysHeld() & KEY_TOUCH);
//     float pressure = tp_pressure();
//     ...
//     tp_close();
//
// Needs Luma3DS for access to the cdc:CHK service. Without it, and in emulators, tp_open fails.
#pragma once
#include <3ds.h>
#include <stdbool.h>
#include <stddef.h>

// Raw samples kept for tp_hist_get.
#define TP_HIST 300

// Each report holds this many conversions of X, Y, Z1 and Z2 (12 bit each).
#define TP_CONVERSIONS 5

typedef struct {
	u16 x[TP_CONVERSIONS];
	u16 y[TP_CONVERSIONS];
	u16 z1[TP_CONVERSIONS];
	u16 z2[TP_CONVERSIONS];
	bool down;
} TpSample;

// Opens the touch controller session. `message` (may be NULL) says how it worked or why not.
// While open, the OS touch mode is also restored when the app is suspended, sleeps or exits.
bool tp_open(char *message, size_t messageSize);

// Restores the OS touch mode and closes the session.
void tp_close(void);

bool tp_ready(void);

// Call once per frame. Same as tp_pen(touching) followed by tp_sample().
void tp_update(bool touching);

// Switches the report to the Z layout while the pen is down, and back to the OS layout when it lifts.
void tp_pen(bool touching);

// Reads the touch report once.
void tp_sample(void);

// Smoothed pressure from 0 (light) to 1 (hard). -1 until a touch has a valid reading.
float tp_pressure(void);

// Touch resistance, median of the last three samples. Lower means harder. -1 if there is none.
float tp_resistance(void);

// The resistance range that pressure is mapped over: R at the hardest and at the lightest press.
// The defaults (900 and 1250) come from one New 3DS. Every unit and stylus differs, so calibrate.
void tp_range_set(float hard, float light);

void tp_range_get(float *hard, float *light);

// Pressure from 0 to 1 for a resistance, linear in conductance (1/R) over the current range.
float tp_map(float resistance);

// Sets the range from about 5 s of tp_resistance() values, taken while the user presses from light to hard.
// Uses the 3rd and 97th percentile. Takes 20 to 4096 samples. Returns false and keeps the range when
// the count is outside that or the samples hardly differ.
bool tp_range_from_samples(const float *resistances, int count);

// Number of samples kept, up to TP_HIST. Only valid samples taken during a touch are kept.
int tp_hist_n(void);

// A kept sample. Index 0 is the oldest.
const TpSample *tp_hist_get(int index);

// Failed reads and mode writes since tp_open.
u32 tp_errors(void);

// Reports dropped since tp_open: not yet in the Z layout after the switch, or no contact.
u32 tp_rejected(void);

// True when the touch data stopped changing in the Z layout. The OS layout then stays until tp_close,
// so there is no pressure.
bool tp_stalled(void);

// Diagnosis: reads any codec page (size up to 64 bytes, else returns -1).
// It does not change the module's state.
Result tp_read(u8 page, u8 reg, void *out, size_t size);

// Diagnosis: writes the touch mode register (0x53 = OS layout, 0x57 = X, Y, Z1, Z2).
// It does not change the module's state.
Result tp_write_mode(u8 mode);
