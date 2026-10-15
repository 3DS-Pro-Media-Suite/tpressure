// tpressure example: a live pressure bar. Press the touch screen. X starts a calibration: press from your
// lightest to your hardest for about 5 s. START quits.
#include <3ds.h>
#include <stdio.h>
#include "tpressure.h"

#define MESSAGE_SIZE 64
#define CALIBRATION_SAMPLES 300
#define BAR_WIDTH 40

// Calibration state: the number of samples so far while calibrating, otherwise one of these.
#define CALIBRATION_OFF (-1)
#define CALIBRATION_DONE (-2)
#define CALIBRATION_FAILED (-3)

// Moves the console cursor to row 8, column 1, below the header.
#define STATUS_POSITION "\x1b[8;1H"

int main(void)
{
	gfxInitDefault();
	consoleInit(GFX_TOP, NULL);
	char message[MESSAGE_SIZE];
	bool hasPressure = tp_open(message, sizeof message);
	const char *result = hasPressure ? "pressure" : "NO pressure";
	printf("tpressure example\n%s: %s\nSTART quits, X calibrates\n\n", result, message);

	static float calibrationSamples[CALIBRATION_SAMPLES];
	int calibrationCount = CALIBRATION_OFF;
	while (aptMainLoop()) {
		hidScanInput();
		u32 pressedKeys = hidKeysDown();
		if (pressedKeys & KEY_START) {
			break;
		}
		bool touching = (hidKeysHeld() & KEY_TOUCH) != 0;
		tp_update(touching);

		float pressure = tp_pressure();
		float resistance = tp_resistance();
		if (pressedKeys & KEY_X) {
			calibrationCount = 0;
		}
		if (calibrationCount >= 0 && touching && resistance > 0) {
			calibrationCount++;
			calibrationSamples[calibrationCount - 1] = resistance;
			if (calibrationCount == CALIBRATION_SAMPLES) {
				bool rangeSet = tp_range_from_samples(calibrationSamples, CALIBRATION_SAMPLES);
				calibrationCount = rangeSet ? CALIBRATION_DONE : CALIBRATION_FAILED;
			}
		}

		// The trailing spaces clear what the previous frame printed.
		printf(STATUS_POSITION);
		if (calibrationCount >= 0) {
			printf("calibrating: %3d%%          \n", calibrationCount * 100 / CALIBRATION_SAMPLES);
		} else {
			const char *status;
			if (calibrationCount != CALIBRATION_DONE) {
				status = calibrationCount == CALIBRATION_FAILED ? "too little difference"
				                                                : "press the touch screen";
			} else {
				status = "range set from your presses";
			}
			printf("%s                \n", status);
		}
		if (pressure >= 0) {
			printf("pressure %.2f  R %.0f   \n[", pressure, resistance);
			int filled = (int)(pressure * BAR_WIDTH);
			for (int i = 0; i < BAR_WIDTH; i++) {
				putchar(i < filled ? '#' : ' ');
			}
			printf("]\n");
		} else {
			printf("-                           \n                                          \n\n");
		}
		gfxFlushBuffers();
		gfxSwapBuffers();
		gspWaitForVBlank();
	}
	tp_close();
	gfxExit();
	return 0;
}
