#include "tpressure.h"
#include <stdio.h>
#include <string.h>

// Touch controller registers (TSC2117-class codec, reached through cdc:CHK).
#define REPORT_PAGE 0xFB // the touch data FIFO
#define REPORT_REGISTER 1
#define REPORT_WORDS 26 // 5 X, 5 Y, 5 Z1, 5 Z2 in the Z layout, then Circle Pad samples
#define BYTES_PER_WORD 2
#define REPORT_SIZE (REPORT_WORDS * BYTES_PER_WORD)
#define MODE_PAGE 0x67 // 3DS touch and Circle Pad control page
#define MODE_REGISTER 0x25 // TSC[67h:25h]: the touch scan mode
#define MODE_SCAN_MASK 0x3C // bits 5-2 select what the scan measures
#define MODE_OS 0x53 // X, Y, then Circle Pad: the OS default
#define MODE_Z 0x57 // X, Y, Z1, Z2, then Circle Pad

// Report layout: each channel is TP_CONVERSIONS 12-bit big-endian words.
#define CHANNEL_BYTES (TP_CONVERSIONS * BYTES_PER_WORD)
#define REPORT_X_OFFSET 0
#define REPORT_Y_OFFSET (1 * CHANNEL_BYTES)
#define REPORT_Z1_OFFSET (2 * CHANNEL_BYTES)
#define REPORT_Z2_OFFSET (3 * CHANNEL_BYTES)
#define SAMPLE_MASK 0xFFF
#define PEN_UP_FLAG 0x10 // in the first byte of the report

// After a mode write the first report may still have the old layout.
#define SKIP_REPORTS 2
// Samples into a touch before pressure is trusted, while the contact settles.
#define SETTLE_SAMPLES 2
#define TOUCH_DATA_SIZE (4 * CHANNEL_BYTES) // X, Y, Z1 and Z2

// On an Old 3DS the OS does not see the pen lift while the report is in the Z layout. So the Z layout ends
// here too: after this many reports in a row without contact, or this many reads with unchanged touch data.
#define LIFTED_REPORTS 6
#define STALE_REPORTS 20

// A report in the Z layout has Z2 well above Z1 (Z2/Z1 about 1.9 to 5.8). A report still in the OS layout has
// Circle Pad samples there (ratio about 1.07), which would read as the hardest press. Such reports are rejected.
#define Z_MIN_RATIO 1.3f
#define Z1_MIN_SUM 40 // sum of the Z1 conversions: no contact below this

// Pressure from resistance.
#define DEFAULT_RANGE_HARD 900.0f
#define DEFAULT_RANGE_LIGHT 1250.0f
#define FALLBACK_PRESSURE 0.5f // returned when the range or the resistance is invalid
#define ADC_FULL_SCALE 4096.0f // 12-bit conversions
#define RESISTANCE_SCALE 1000.0f // R in thousandths of the X plate resistance
// The smoothing rate goes from 0.25 at the lightest press to 0.75 at the hardest.
#define SMOOTHING_RATE_LIGHT 0.25f
#define SMOOTHING_RATE_PER_PRESSURE 0.5f

// Range calibration.
#define RANGE_MIN_SAMPLES 20
#define RANGE_MAX_SAMPLES 4096
#define RANGE_HARD_PERCENTILE 0.03f
#define RANGE_LIGHT_PERCENTILE 0.97f
#define RANGE_MIN_SPREAD 1.08f // light must be at least 8 % above hard

// Nothing restores the mode once the session is closed, so tp_close tries more than once.
#define RESTORE_ATTEMPTS 2
#define MAX_READ_SIZE 64

// Luma3DS custom SVC numbers, as strings for the inline asm.
#define SVC_CONTROL_SERVICE "0xB0"
#define SVC_COPY_HANDLE "0xB1"
#define SVC_CONTROL_PROCESS "0xB3"

// svcGetSystemInfo and svcGetProcessInfo types.
#define SYSTEM_INFO_LUMA 0x10000 // Luma3DS: parameter 0 gives its version
#define SYSTEM_INFO_EMULATOR 0x20000 // Citra and its forks: parameter 0 gives 1
#define PROCESS_INFO_NAME 0x10000 // Luma3DS: the 8-byte process name

// Copying a cdc:CHK session from a system process.
#define MAX_PROCESSES 64
#define MAX_PROCESS_HANDLES 512
#define PROCESS_NAME_LENGTH 8
#define SERVICE_NAME_SIZE 16
#define CDC_SERVICE_NAME "cdc:CHK"
#define RESULT_NO_SESSION (-3)

// Operations of the Luma3DS ControlService and ControlProcess SVCs.
typedef enum {
	SERVICEOP_STEAL_CLIENT_SESSION = 0,
	SERVICEOP_GET_NAME = 1
} ServiceOp;

typedef enum {
	PROCESSOP_GET_ALL_HANDLES = 0
} ProcessOp;

__attribute__((naked)) static Result luma_control_service(ServiceOp op, void *argument, Handle handle)
{
	__asm__ volatile("svc " SVC_CONTROL_SERVICE "\n\t"
	                 "bx lr");
}

// The SVC returns the new handle in r1, so the out pointer is kept on the stack across it.
__attribute__((naked)) static Result luma_copy_handle(Handle *out, Handle outProcess, Handle in,
                                                      Handle inProcess)
{
	__asm__ volatile("str r0, [sp, #-4]!\n\t"
	                 "svc " SVC_COPY_HANDLE "\n\t"
	                 "ldr r2, [sp], #4\n\t"
	                 "str r1, [r2]\n\t"
	                 "bx lr");
}

__attribute__((naked)) static Result luma_control_process(Handle process, ProcessOp op, u32 argument1,
                                                          u32 argument2)
{
	__asm__ volatile("svc " SVC_CONTROL_PROCESS "\n\t"
	                 "bx lr");
}

static bool g_open;
static bool g_sessionCopied;
static bool g_sessionFromSrv;
static Handle g_copiedSession;
static bool g_zModeOn; // set only while the pen is down
static bool g_stalled; // the touch data stopped changing in the Z layout: it is not used again
static aptHookCookie g_aptHook;
static int g_reportsToSkip;
static int g_touchSamples;
static int g_liftedReports; // reports in a row without contact
static int g_staleReports; // reads in a row with unchanged touch data
static u8 g_lastTouchData[TOUCH_DATA_SIZE];
static float g_rangeHard = DEFAULT_RANGE_HARD;
static float g_rangeLight = DEFAULT_RANGE_LIGHT;
static float g_pressure; // smoothed
static float g_resistance; // median of the last three samples
static u32 g_errorCount;
static u32 g_rejectedCount;
static TpSample g_history[TP_HIST];
static int g_historyCount;
static int g_historyNext;

// Finds a cdc:CHK session in a system process and copies it into g_copiedSession.
static Result copy_cdc_session(void)
{
	u32 processIds[MAX_PROCESSES];
	s32 processCount = 0;
	Result result = svcGetProcessList(&processCount, processIds, MAX_PROCESSES);
	if (R_FAILED(result)) {
		return result;
	}
	static u32 theirHandles[MAX_PROCESS_HANDLES];
	for (s32 i = 0; i < processCount; i++) {
		Handle process;
		if (R_FAILED(svcOpenProcess(&process, processIds[i]))) {
			continue;
		}
		s64 nameValue = 0;
		char name[PROCESS_NAME_LENGTH + 1] = "?";
		if (R_SUCCEEDED(svcGetProcessInfo(&nameValue, process, PROCESS_INFO_NAME))) {
			memcpy(name, &nameValue, PROCESS_NAME_LENGTH);
			name[PROCESS_NAME_LENGTH] = 0;
		}
		bool codecClient =
		        !strcmp(name, "hid") || !strcmp(name, "codec") || !strcmp(name, "ir") || !strcmp(name, "mcu");
		if (!codecClient) {
			svcCloseHandle(process);
			continue;
		}
		// Returns the number of handles, or an error below 0.
		s32 handleCount = luma_control_process(process, PROCESSOP_GET_ALL_HANDLES, (u32)theirHandles, 0);
		for (s32 j = 0; j < handleCount && j < MAX_PROCESS_HANDLES && !g_copiedSession; j++) {
			Handle copy = 0;
			if (R_FAILED(luma_copy_handle(&copy, CUR_PROCESS_HANDLE, theirHandles[j], process))) {
				continue;
			}
			char serviceName[SERVICE_NAME_SIZE] = "";
			Result nameResult = luma_control_service(SERVICEOP_GET_NAME, serviceName, copy);
			if (R_SUCCEEDED(nameResult) && !strcmp(serviceName, CDC_SERVICE_NAME)) {
				g_copiedSession = copy; // ends the loop
				continue;
			}
			svcCloseHandle(copy);
		}
		svcCloseHandle(process);
		if (g_copiedSession) {
			return 0;
		}
	}
	return RESULT_NO_SESSION;
}

// Citra and the emulators built on it answer 1. A real 3DS answers 0 or an error.
static bool in_emulator(void)
{
	s64 isEmulator = 0;
	Result result = svcGetSystemInfo(&isEmulator, SYSTEM_INFO_EMULATOR, 0);
	return R_SUCCEEDED(result) && isEmulator;
}

// The custom SVCs used by copy_cdc_session exist only under Luma3DS.
static bool has_luma(void)
{
	s64 version = 0;
	Result result = svcGetSystemInfo(&version, SYSTEM_INFO_LUMA, 0);
	return R_SUCCEEDED(result) && version;
}

// If the write fails, g_zModeOn stays set so the next call tries again.
static void restore_os_mode(void)
{
	g_touchSamples = 0;
	if (R_SUCCEEDED(tp_write_mode(MODE_OS))) {
		g_zModeOn = false;
	} else {
		g_errorCount++;
	}
}

// HOME Menu and applets read the Circle Pad from the same report, so the OS layout comes back whenever the app
// stops running.
static void on_apt_event(APT_HookType event, void *param)
{
	if (g_zModeOn && (event == APTHOOK_ONSUSPEND || event == APTHOOK_ONSLEEP || event == APTHOOK_ONEXIT)) {
		restore_os_mode();
	}
}

static void set_message(char *message, size_t messageSize, const char *text, Result result)
{
	if (!message || !messageSize) {
		return;
	}
	if (result) {
		snprintf(message, messageSize, "%s (%08lX)", text, (unsigned long)result);
	} else {
		snprintf(message, messageSize, "%s", text);
	}
}

bool tp_open(char *message, size_t messageSize)
{
	if (g_open) {
		set_message(message, messageSize, "already open", 0);
		return true;
	}
	if (in_emulator()) {
		set_message(message, messageSize, "running in an emulator", 0);
		return false;
	}
	// Fail instead of waiting when the service has no free session.
	srvSetBlockingPolicy(true);
	Result result = cdcChkInit();
	srvSetBlockingPolicy(false);
	if (R_SUCCEEDED(result)) {
		g_sessionFromSrv = true;
		set_message(message, messageSize, "cdc:CHK from srv", 0);
	} else {
		if (!has_luma()) {
			set_message(message, messageSize, "no cdc:CHK without Luma3DS", result);
			return false;
		}
		if (R_FAILED(copy_cdc_session())) {
			set_message(message, messageSize, "no cdc:CHK from srv or a system process", result);
			return false;
		}
		// libctru's cdc:CHK calls use the handle cdcChkInit would have opened, so point it at the copy.
		*cdcChkGetSessionHandle() = g_copiedSession;
		g_sessionCopied = true;
		set_message(message, messageSize, "cdc:CHK copied from a system process", 0);
	}
	g_open = true;
	g_zModeOn = false;
	g_stalled = false;
	g_historyNext = 0;
	g_historyCount = 0;
	g_rejectedCount = 0;
	g_errorCount = 0;

	// An app that quit mid-touch leaves the chip out of the OS scan, and nothing else puts it back.
	u8 mode = 0;
	Result readResult = CDCCHK_ReadRegisters2(MODE_PAGE, MODE_REGISTER, &mode, sizeof mode);
	bool outOfOsScan = R_SUCCEEDED(readResult) && (mode & MODE_SCAN_MASK) != (MODE_OS & MODE_SCAN_MASK);
	if (outOfOsScan && R_FAILED(tp_write_mode(MODE_OS))) {
		g_errorCount++;
	}
	aptHook(&g_aptHook, on_apt_event, NULL);
	return true;
}

void tp_close(void)
{
	if (!g_open) {
		return;
	}
	aptUnhook(&g_aptHook);
	for (int attempt = 0; attempt < RESTORE_ATTEMPTS && g_zModeOn; attempt++) {
		restore_os_mode();
	}
	g_zModeOn = false;
	if (g_sessionCopied) {
		svcCloseHandle(g_copiedSession);
		*cdcChkGetSessionHandle() = 0;
		g_copiedSession = 0;
		g_sessionCopied = false;
	} else if (g_sessionFromSrv) {
		cdcChkExit();
	}
	g_sessionFromSrv = false;
	g_open = false;
}

bool tp_ready(void)
{
	return g_open;
}

// Uses the 16 MHz interface (ReadRegisters2): the 4 MHz one reads the report page as zeros.
Result tp_read(u8 page, u8 reg, void *out, size_t size)
{
	if (!g_open || size > MAX_READ_SIZE) {
		return -1;
	}
	return CDCCHK_ReadRegisters2(page, reg, out, size);
}

Result tp_write_mode(u8 mode)
{
	if (!g_open) {
		return -1;
	}
	return CDCCHK_WriteRegisters2(MODE_PAGE, MODE_REGISTER, &mode, sizeof mode);
}

int tp_hist_n(void)
{
	return g_historyCount;
}

const TpSample *tp_hist_get(int index)
{
	int oldest = g_historyNext - g_historyCount;
	// Adding 2 * TP_HIST keeps the value positive for the modulo.
	int slot = (oldest + index + TP_HIST * 2) % TP_HIST;
	return &g_history[slot];
}

u32 tp_errors(void)
{
	return g_errorCount;
}

u32 tp_rejected(void)
{
	return g_rejectedCount;
}

bool tp_stalled(void)
{
	return g_stalled;
}

void tp_range_set(float hard, float light)
{
	if (light > hard && hard > 0) {
		g_rangeHard = hard;
		g_rangeLight = light;
	}
}

void tp_range_get(float *hard, float *light)
{
	if (hard) {
		*hard = g_rangeHard;
	}
	if (light) {
		*light = g_rangeLight;
	}
}

// Resistance falls roughly as 1/force, so the map is linear in conductance. A map linear in R would crowd
// everything at the hard end.
float tp_map(float resistance)
{
	if (g_rangeLight <= g_rangeHard || g_rangeHard <= 0 || resistance <= 0) {
		return FALLBACK_PRESSURE;
	}
	float conductance = 1.0f / resistance;
	float hardConductance = 1.0f / g_rangeHard;
	float lightConductance = 1.0f / g_rangeLight;
	float pressure = (conductance - lightConductance) / (hardConductance - lightConductance);
	if (pressure < 0) {
		return 0;
	}
	if (pressure > 1) {
		return 1;
	}
	return pressure;
}

bool tp_range_from_samples(const float *resistances, int count)
{
	if (count < RANGE_MIN_SAMPLES || count > RANGE_MAX_SAMPLES) {
		return false;
	}
	static float sorted[RANGE_MAX_SAMPLES];
	memcpy(sorted, resistances, (size_t)count * sizeof(float));
	// Insertion sort.
	for (int i = 1; i < count; i++) {
		float value = sorted[i];
		int j = i - 1;
		while (j >= 0 && sorted[j] > value) {
			sorted[j + 1] = sorted[j];
			j--;
		}
		sorted[j + 1] = value;
	}
	int hardIndex = (int)(count * RANGE_HARD_PERCENTILE);
	int lightIndex = (int)(count * RANGE_LIGHT_PERCENTILE);
	float hard = sorted[hardIndex];
	float light = sorted[lightIndex];
	if (hard <= 0 || light < hard * RANGE_MIN_SPREAD) {
		return false;
	}
	tp_range_set(hard, light);
	return true;
}

// Touch resistance of a sample with TI's formula R = X / 4096 * (Z2 / Z1 - 1). Lower means harder.
static float touch_resistance(const TpSample *sample)
{
	float xSum = 0;
	float z1Sum = 0;
	float z2Sum = 0;
	for (int i = 0; i < TP_CONVERSIONS; i++) {
		xSum += sample->x[i];
		z1Sum += sample->z1[i];
		z2Sum += sample->z2[i];
	}
	// Z1 below one code per conversion: no contact.
	if (z1Sum < TP_CONVERSIONS) {
		return -1;
	}
	float x = xSum / TP_CONVERSIONS;
	float z2OverZ1 = z2Sum / z1Sum;
	return x / ADC_FULL_SCALE * (z2OverZ1 - 1.0f) * RESISTANCE_SCALE;
}

static float median_of_three(float a, float b, float c)
{
	if (a > b) {
		if (b > c) {
			return b;
		}
		if (a > c) {
			return c;
		}
		return a;
	}
	if (a > c) {
		return a;
	}
	if (b > c) {
		return c;
	}
	return b;
}

// Slow for light presses, which are noisy, and quick for hard ones.
static float smooth_pressure(float previous, float target)
{
	float rate = SMOOTHING_RATE_LIGHT + SMOOTHING_RATE_PER_PRESSURE * previous;
	return previous + (target - previous) * rate;
}

static void update_pressure(const TpSample *sample)
{
	float resistance = touch_resistance(sample);
	if (resistance < 0) {
		return;
	}
	// Light touches are noisy (R jumps about 18 % of the range there, 5 % at hard presses),
	// so the median of the last three drives everything.
	static float recent[3];
	bool firstOfTouch = g_touchSamples <= SETTLE_SAMPLES + 1;
	if (firstOfTouch) {
		recent[2] = resistance;
		recent[1] = resistance;
		recent[0] = resistance;
	} else {
		recent[0] = recent[1];
		recent[1] = recent[2];
		recent[2] = resistance;
	}
	g_resistance = median_of_three(recent[0], recent[1], recent[2]);
	float pressure = tp_map(g_resistance);
	g_pressure = firstOfTouch ? pressure : smooth_pressure(g_pressure, pressure);
}

static bool has_reading(void)
{
	return g_open && g_zModeOn && g_touchSamples > SETTLE_SAMPLES;
}

float tp_pressure(void)
{
	return has_reading() ? g_pressure : -1;
}

float tp_resistance(void)
{
	return has_reading() ? g_resistance : -1;
}

void tp_pen(bool touching)
{
	if (!g_open) {
		return;
	}
	if (touching && !g_zModeOn && !g_stalled) {
		if (R_SUCCEEDED(tp_write_mode(MODE_Z))) {
			g_zModeOn = true;
			g_reportsToSkip = SKIP_REPORTS;
			g_touchSamples = 0;
			g_liftedReports = 0;
			g_staleReports = 0;
			memset(g_lastTouchData, 0, sizeof g_lastTouchData);
		} else {
			g_errorCount++;
		}
	} else if (!touching && g_zModeOn) {
		restore_os_mode();
	}
}

// One 12-bit sample, stored big endian.
static u16 read_report_word(const u8 *report, int offset)
{
	int high = report[offset] << 8;
	int low = report[offset + 1];
	return (u16)((high | low) & SAMPLE_MASK);
}

// True once the touch data has not changed for STALE_REPORTS reads: the chip stopped scanning.
static bool scan_stopped(const u8 *report)
{
	if (memcmp(report, g_lastTouchData, TOUCH_DATA_SIZE)) {
		memcpy(g_lastTouchData, report, TOUCH_DATA_SIZE);
		g_staleReports = 0;
		return false;
	}
	g_staleReports++;
	return g_staleReports >= STALE_REPORTS;
}

// True once LIFTED_REPORTS reports in a row had no contact.
static bool pen_lifted(bool contact)
{
	g_liftedReports = contact ? 0 : g_liftedReports + 1;
	return g_liftedReports >= LIFTED_REPORTS;
}

void tp_sample(void)
{
	// With the pen up the report has no Z data.
	if (!g_open || !g_zModeOn) {
		return;
	}
	u8 report[REPORT_SIZE];
	if (R_FAILED(CDCCHK_ReadRegisters2(REPORT_PAGE, REPORT_REGISTER, report, REPORT_SIZE))) {
		g_errorCount++;
		return;
	}
	if (g_reportsToSkip > 0) {
		g_reportsToSkip--;
		return;
	}
	if (scan_stopped(report)) {
		g_stalled = true;
		restore_os_mode();
		return;
	}
	TpSample sample;
	float z1Sum = 0;
	float z2Sum = 0;
	for (int i = 0; i < TP_CONVERSIONS; i++) {
		int wordOffset = i * BYTES_PER_WORD;
		sample.x[i] = read_report_word(report, REPORT_X_OFFSET + wordOffset);
		sample.y[i] = read_report_word(report, REPORT_Y_OFFSET + wordOffset);
		sample.z1[i] = read_report_word(report, REPORT_Z1_OFFSET + wordOffset);
		sample.z2[i] = read_report_word(report, REPORT_Z2_OFFSET + wordOffset);
		z1Sum += sample.z1[i];
		z2Sum += sample.z2[i];
	}
	sample.down = !(report[0] & PEN_UP_FLAG);
	if (pen_lifted(sample.down && z1Sum >= Z1_MIN_SUM)) {
		restore_os_mode();
		return;
	}
	// A stale OS-layout report, or no contact yet.
	if (sample.down && (z1Sum < Z1_MIN_SUM || z2Sum < z1Sum * Z_MIN_RATIO)) {
		g_rejectedCount++;
		return;
	}
	g_history[g_historyNext] = sample;
	g_historyNext = (g_historyNext + 1) % TP_HIST;
	if (g_historyCount < TP_HIST) {
		g_historyCount++;
	}
	if (sample.down) {
		g_touchSamples++;
		if (g_touchSamples > SETTLE_SAMPLES) {
			update_pressure(&sample);
		}
	} else {
		g_touchSamples = 0;
	}
}

void tp_update(bool touching)
{
	tp_pen(touching);
	tp_sample();
}
