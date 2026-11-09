# HELIO: a drone synth for the M-VAVE FM-1

A module for [fm1-dock](https://github.com/joakimeriksson/fm1-dock), inspired by the Elta Music Solar 42
(not affiliated with Elta Music or M-VAVE).

- 6 drone voices × 4 oscillators (one per knob), free microtonal tuning, slow random "analog" drift
- sine → saw morph per drone, slow swell and fade
- drones 1–3 → filter A (left), drones 4–6 → filter B (right): resonant, saturating low-pass filters,
  swept in opposite directions by one slow LFO
- an LFO on every oscillator (24 in all), each swelling its oscillator's level at its own speed
- a big hall reverb: 8-line feedback delay network with diffusion and slow modulation, decay up to 60 s
- the sound is saved with SAVE and comes back at the next start

## Controls

| Control | Does |
|---|---|
| white keys **F3 G3 A3 B3 C4 D4** | drone 1–6 on/off (and selects it) |
| keys **F4 … G5** | root note of the selected drone (two octaves lower) |
| **OCT− / OCT+** | selected drone one octave down / up |
| **SELECT** knob | selected drone's root, one semitone per click |
| **ALGORITHM** knob | selected drone's waveform, sine → saw |
| **PRESETS** knob | selected drone's volume |
| **KNOB 1–4** | tune oscillator 1–4, 5 cents per click (hold **SEL**: 1 semitone) |
| **EDIT** page: KNOB 1–4 | level of oscillator 1–4 |
| **FX** page: KNOB 1–4 | reverb: decay (0.5–60 s), size, tone (dark → bright), mix |
| **GLO** page: KNOB 1–4 | filter A cutoff, A resonance, B cutoff, B resonance |
| **LFO** page, 1st tap: KNOB 1–4 | LFO speed of oscillator 1–4 (0.01–5 Hz): its level breathes |
| **LFO** page, 2nd tap: KNOB 1–4 | depth of those 4 LFOs |
| **LFO** page, 3rd tap: KNOB 1–4 | filter LFO speed, filter LFO depth, drift amount, filter drive (4th tap: back) |
| **ENV** page: KNOB 1–2 | selected drone's swell time, fade time |
| **PLAY** | all drones off / back on |
| **SAVE** | keep the sound |
| hold **HOME** 1.5 s | back to fm1-dock's module list |

Pages: press EDIT, FX, GLO, LFO or ENV once to open the page (its button lights up), press it again to go
back to the oscillators. Every value is shown as a number and a bar.

MIDI: notes C3–F3 (48–53) switch drones 1–6, CC 123 turns all off.
The bottom line of the screen shows how much of the chip's time the sound uses (CPU %).

## Building

Push to GitHub: the workflow in `.github/workflows/build.yml` builds `helio.dock` on GitHub's servers and
renders `helio_test.wav` from the same code (Actions → the latest run → Artifacts → helio).

Locally, inside an fm1-dock checkout with the JieLi toolchain:

    cp -r helio modules/helio && python3 tools/hbmod.py modules/helio

## Licence

GPL-3.0-only, as fm1-dock.
