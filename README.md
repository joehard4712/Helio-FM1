# HELIO: a drone synth for the M-VAVE FM-1

A module for [fm1-dock](https://github.com/joakimeriksson/fm1-dock), inspired by the Elta Music Solar 42
(not affiliated with Elta Music or M-VAVE).

- 6 drone voices × 4 oscillators (one per knob), free microtonal tuning, slow random "analog" drift
- sine → saw morph per drone, slow swell and fade
- drones 1–3 → filter A (left), drones 4–6 → filter B (right): resonant, saturating low-pass filters,
  swept in opposite directions by one slow LFO
- an LFO on every oscillator (24 in all), each swelling its oscillator's level at its own speed
- 2 melodic voices: 2 oscillators each (saw → square, detuned against each other), noise, their own
  resonant filter with an envelope, attack-decay-sustain-release, glide
- a 5-step sequencer that plays voice 1, recorded from the keys
- noise and sample & hold (free running, or a new random value on every sequencer step) on the voices'
  cutoff and pitch
- an echo on the voices (up to 1 s, tape-like, stereo)
- a big hall reverb on everything: 8-line feedback delay network with diffusion and slow modulation,
  decay up to 60 s
- **SWEET** (the ALGORITHM knob), one per drone: morphs the drone's 4 oscillators into a pure-tuned chord;
  drone 1 chooses it, drones 2–6 follow in a progression
- each drone has its own colour, so one look tells you which drone you are editing
- the sound is saved with SAVE and comes back at the next start (sounds saved by older HELIOs still load)

## SWEET: chords

Every drone has its own SWEET: turn **ALGORITHM** and it sets the selected drone's. At 0 % the
drone's 4 oscillators sound exactly as you tuned them; turning it up morphs them into a chord, and at
100 % they play it. Every chord is tuned pure (just ratios from its root: 5/4, 3/2, 15/8 …), so it
does not beat or wobble.

- **Drone 1 chooses the chord.** HELIO looks at how its 4 oscillators are tuned and finds the nearest
  chord (major 7, 7, 6, add 9, sus 4, power, minor 7, minor 6), each oscillator moving to the chord
  tone nearest to it, in its own octave.
- **Drones 2–6 follow** with the next chords of a progression in drone 1's key (major or minor, from
  drone 1's chord), each moving its oscillators to the nearest notes of its chord.
- **SEL + ALGORITHM** chooses the progression (drones 1 → 6, from a C major 7 on drone 1):

| Progression | Chords |
|---|---|
| DREAM | C maj7 · F maj7 · A m7 · D m7 · G7 · E m7 |
| POP | C maj7 · G7 · A m7 · F maj7 · C maj7 · G7 |
| DARK | i · VI · III · VII · iv · i (made for a minor drone 1) |
| STILL | the same chord six times, in six voicings |
| FIFTHS | round the circle: C · F · B♭ · E♭ · A♭ · D♭, all like drone 1's chord |

On the start screen the title shows the drone's chord (`1 CMAJ7`) once its SWEET is up, each
oscillator shows which chord tone it goes to (R, 3, 5, 7, 9 …, white when it is there), and a white
line under the title shows how far SWEET is turned up.

## Drone colours

1 red · 2 orange · 3 yellow · 4 green · 5 blue · 6 violet. The start screen is drawn in the selected
drone's colour, the six boxes at the top right are the six drones in theirs (filled while they sound,
the selected one framed in white), and on the selected drone's pages (EDIT, LFO 1/4 and 2/4, ENV) the
bars are in its colour too, under the page's own coloured title.

## The screen

Big letters, one colour per page on black. Each page is a title bar and four rows, one per knob: the
knob's number, the name, the value and a long bar. The start screen (in the selected drone's colour) shows the selected drone
and its note, the six drones as boxes (filled while they sound) and the four oscillators' tuning, with
their levels as bars.

| Page | Colour |
|---|---|
| start screen (drone) | the drone's colour |
| EDIT | white |
| FX | cyan |
| GLO | magenta |
| LFO | green |
| ENV | orange |
| ARP (voices) | violet |
| SEQ | yellow (red while recording) |

## Controls

| Control | Does |
|---|---|
| white keys **F3 G3 A3 B3 C4 D4** | drone 1–6 on/off (and selects it) |
| keys **F4 … G5** | play the 2 voices (two notes at once) |
| hold **SEL** + key F4 … G5 | root note of the selected drone (two octaves lower) |
| **OCT− / OCT+** | selected drone one octave down / up |
| **SELECT** knob | selected drone's root, one semitone per click |
| **ALGORITHM** knob | selected drone's **SWEET**: 0 % as tuned → 100 % its chord |
| hold **SEL** + **ALGORITHM** | the chord progression (DREAM, POP, DARK, STILL, FIFTHS) |
| **HOME**, a short press | out of any page, straight back to the selected drone |
| **PRESETS** knob | selected drone's volume |
| **KNOB 1–4** | tune oscillator 1–4, 5 cents per click (hold **SEL**: 1 semitone) |
| **PLAY** | sequencer start / stop |
| hold **SEL** + **PLAY** | all drones off / back on |
| **REC** | record the 5 steps: play up to 5 keys, press SEL for a rest; REC again ends early |
| **SAVE** | keep the sound |
| hold **HOME** 1.5 s | back to fm1-dock's module list |

Pages: press a page button to open it (its light comes on). Pages with more than one part show
"1/3" in the corner: press the same button again for the next part. After the last part it closes.
A short press on HOME leaves any page at once and goes back to the selected drone.

| Page | KNOB 1 | KNOB 2 | KNOB 3 | KNOB 4 |
|---|---|---|---|---|
| **EDIT** | level osc 1 | level osc 2 | level osc 3 | level osc 4 |
| **FX** 1/2 reverb | decay 0.5–60 s | size | tone | mix |
| **FX** 2/2 echo | time 20–1000 ms | repeats | tone | mix |
| **GLO** | filter A cutoff | A resonance | filter B cutoff | B resonance |
| **LFO** 1/4 | LFO speed osc 1 | osc 2 | osc 3 | osc 4 |
| **LFO** 2/4 | LFO depth osc 1 | osc 2 | osc 3 | osc 4 |
| **LFO** 3/4 | filter LFO speed | filter LFO depth | drift | drive |
| **LFO** 4/4 noise + S&H | S&H rate (lowest: SEQ) | to voice cutoff | to voice pitch (semitones) | noise |
| **ENV** (selected drone) | swell | fade | wave | volume |
| **ARP** 1/3 voice | wave saw → square | beat (detune) | cutoff | resonance |
| **ARP** 2/3 voice env | attack | decay | sustain | release |
| **ARP** 3/3 voice play | filter envelope | glide | octave | volume |
| **SEQ** 1/2 | rate (BPM) | gate | steps 1–5 | key (transpose) |
| **SEQ** 2/2 steps | step 1 | step 2 | step 3 | step 4 (PRESETS: step 5) |

On the steps page: turn to change a step's note; hold SEL and turn left for a rest, right for the note.
While the sequencer runs it plays voice 1 and the keys play voice 2, so you can play over it.

MIDI: notes C3–F3 (48–53) switch drones 1–6, all other notes play the voices, CC 123 turns all off.
Hold SEL on the start screen to see how much of the chip's time the sound uses (CPU %).

## Building

Push to GitHub: the workflow in `.github/workflows/build.yml` builds `helio.dock` on GitHub's servers and
renders `helio_test.wav` from the same code (Actions → the latest run → Artifacts → helio).
`test/screens.c` draws every page with the firmware's fonts (`screens/*.png`).

Locally, inside an fm1-dock checkout with the JieLi toolchain:

    cp -r helio modules/helio && python3 tools/hbmod.py modules/helio

## Licence

GPL-3.0-only, as fm1-dock.
