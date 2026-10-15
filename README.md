# tpressure

A small C module that reads stylus pressure on the 3DS.

## Usage

```c
#include "tpressure.h"

char message[64];
bool hasPressure = tp_open(message, sizeof message); // false: no pressure, plain touch still works

while (aptMainLoop()) {
    hidScanInput();
    tp_update(hidKeysHeld() & KEY_TOUCH); // once per frame
    float pressure = tp_pressure();       // 0 light to 1 hard, -1 while there is no reading
    // Read the position with hidTouchRead() as usual.
}
tp_close(); // restores the OS touch mode
```

`tp_open` writes into `message` how it got the `cdc:CHK` session, or why it failed.

Calibrate the pressure range for each 3DS.
Collect `tp_resistance()` for about 5 s while the user presses from light to hard.
Skip the -1 values.
Pass 20 to 4096 of these values to `tp_range_from_samples()`.
`tp_range_from_samples()` returns false and keeps the old range if the presses hardly differ.
To keep the range, save the two values from `tp_range_get`.
Restore the range at the next start with `tp_range_set`.

## Add it to your project

The module needs only libctru.
Add the repository as a git submodule:

```sh
git submodule add https://github.com/3DS-Pro-Media-Suite/tpressure tpressure
```

Add the folder to your Makefile:

```make
SOURCES  := source tpressure
INCLUDES := source tpressure
```

## Build the example

You need devkitPro with `3ds-dev`.

```sh
git clone https://github.com/3DS-Pro-Media-Suite/tpressure
cd tpressure/example
make        # tpexample.3dsx
```

The example shows a live pressure bar.
To calibrate, press X and press the touch screen from light to hard for about 5 s.
Press START to quit.

## How it works

A TSC2117-class codec chip reads the touch screen.
Apps reach the chip through the `cdc:CHK` service.
The chip can measure X, Y, Z1 and Z2. Z1 and Z2 are the pressure channels.
The OS sets the chip to report X, Y and the Circle Pad, without Z1 and Z2.

The scan mode is register `0x25` in codec page `0x67`. The touch report is in codec page `0xFB`.

| Value | Report |
|---|---|
| `0x53` (OS default) | 5 X, 5 Y, then Circle Pad samples |
| `0x57` | 5 X, 5 Y, 5 Z1, 5 Z2, then part of the Circle Pad samples |

In `0x57` mode, Z1 and Z2 take the place of Circle Pad samples.
The module sets `0x57` while the pen is down and `0x53` when it lifts.
The Circle Pad works between strokes, not during one.

The module turns Z1 and Z2 into a touch resistance with TI's formula:

```
R = 1000 * X/4096 * (Z2/Z1 - 1)
```

Lower R means a harder press. Each new R goes through three steps:

1. The median of the last three samples removes spikes.
2. R is mapped to 0..1 over the range you set. The map is linear in 1/R, because R falls roughly as 1/force.
3. A filter smooths the result. The filter is slow for light presses and quick for hard ones.

## Limits

* Needs Luma3DS. `tp_open` first asks `srv` for `cdc:CHK`.
  If `srv` refuses, `tp_open` copies a session from a system process with Luma's custom SVCs.
* Does not work in emulators. `tp_open` fails there.
* The default range (hard 900, light 1250) comes from one New 3DS. Each unit and stylus differs.
* The same press gives a different R in different places on the screen.
* About 10 pressure levels are reliable:

  | | Levels | Why |
  |---|---|---|
  | ADC | 4096 (12 bit) | The converter resolution. |
  | Without noise | about 210 to 400 | The useful R range (about 915 to 1250) divided by the R change per ADC code. |
  | Measured | about 10 reliable, about 19 barely apart | Noise: about 7 R units at hard presses, about 30 (estimated) at light ones. |

  Noise sets the limit, not the ADC. Use pressure for a few clear steps, not fine control.
* Tested on a New 3DS with the stylus. On an Old 3DS there is no pressure yet; plain touch works.
* On an Old 3DS the OS does not see the pen lift while the chip reports pressure.
  So `tp_sample` switches back to the OS touch mode itself once the chip reports no contact.
  If the chip's data stops changing, `tp_stalled()` turns true and pressure stays off until `tp_close`.
* The module restores the OS touch mode on `tp_close`, HOME, sleep and exit.
* If the app crashes during a touch, the Circle Pad reads wrong. The next `tp_open` or a reboot fixes it.
* `tp_hist_*`, `tp_errors`, `tp_rejected`, `tp_read` and `tp_write_mode` are for diagnosis only.

## License

MIT. See `LICENSE`.
