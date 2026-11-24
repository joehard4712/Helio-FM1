/* SPDX-License-Identifier: GPL-3.0-only */
/* HELIO: a drone synth for fm1-dock, inspired by the Elta Music Solar 42 (not affiliated).
 *
 *   6 drone voices x 4 oscillators (one oscillator per knob), free microtonal tuning,
 *   slow random "analog" drift, sine->saw morph, slow swell/fade.
 *   Drones 1-3 -> filter A (left-ish), drones 4-6 -> filter B (right-ish): resonant 2-pole
 *   low-pass with saturation, swept in opposite directions by one slow LFO.
 *   2 melodic voices (2 oscillators saw..square, noise, own filter + envelope, glide),
 *   played from the upper keys or by a 5-step sequencer; noise + sample & hold; an echo on
 *   the voices. Then a big hall reverb on everything: 8-line feedback delay network, up to 60 s.
 *
 * Panel (HB_F_PANEL: all keys are the module's; HOME held 1.5 s alone opens the module list)
 *   white keys F3 G3 A3 B3 C4 D4   latch drone 1..6 on/off, and select it for editing
 *   keys F4 .. G5                  play the 2 voices (SEL held: set the selected drone's root)
 *   OCT- / OCT+                    selected drone's root -/+ one octave
 *   SELECT / PRESETS               selected drone's root (semitones) / volume
 *   ALGORITHM                      the selected drone's SWEET: 0 % free tuning .. 100 % its 4 oscillators pulled onto the
 *                                  nearest sweet spot: a just ratio to the drone's root (3/2, 5/4, 7/4..),
 *                                  or a hair off it so the two beat in time with the sequencer's tempo
 *   KNOB1..4                       tune oscillator 1..4 (5 cents a click; SEL held: 1 semitone)
 *   HOME (a short press)           out of any page, back to the selected drone
 *   Each drone has its colour (1 red, 2 orange, 3 yellow, 4 green, 5 blue, 6 violet): the start
 *   screen, and the bars on the pages of the selected drone, are drawn in it.
 *   Pages: press a page button to open it, press it again for its next page; after the last
 *   page it closes. The open page's light is on, the other page buttons glow.
 *   EDIT  levels of oscillator 1..4
 *   FX    1 reverb: decay, size, tone, mix      2 echo: time, repeats, tone, mix
 *   GLO   filter A cutoff, A resonance, B cutoff, B resonance
 *   LFO   1 oscillator LFO speeds  2 their depths  3 filter LFO, depth, drift, drive
 *         4 noise + sample & hold: rate (lowest: on each sequencer step), to cutoff, to pitch, noise
 *   ENV   selected drone: swell, fade, wave, volume
 *   ARP   voices  1 wave, beat (detune), cutoff, resonance  2 attack, decay, sustain, release
 *                 3 filter envelope, glide, octave, volume
 *   SEQ   1 rate (BPM), gate, steps (1..5), key (transpose)
 *         2 the 5 steps: KNOB1..4 steps 1..4, PRESETS step 5; SEL + turn left: rest, right: note
 *   PLAY                           sequencer start / stop (it plays voice 1; the keys play voice 2)
 *   SEL + PLAY                     all drones off / back on
 *   REC                            record the steps: play up to 5 keys (SEL: a rest); REC ends early
 *   SAVE                           keep the sound (it is loaded at the next start)
 * MIDI: notes 48..53 (C3..F3) toggle drones 1..6, other notes play the voices; CC 123 all off.
 *
 * Floats with the chip's FPU (module.cfg: -mcpu=r3 -mfprev1 -ffp-contract=off); no libm. */
#include "hb_module.h"

static const hb_host_t *H;

#define SR 44100.0f
#define ND 6u                       /* drones */
#define NO 4u                       /* oscillators per drone */
#define SUB 16u                     /* frames per control step (LFO, filter coefficients) */

/* ------------------------------------------------------------------ small maths, no libm */

typedef union { float f; uint32_t u; } fu_t;

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

static float exp2f_(float x)                    /* 2^x, ~1e-6 relative, for |x| < 60 */
{
    fu_t s;
    int32_t n;
    float a, p;
    x = clampf(x, -60.0f, 60.0f);
    n = (int32_t)(x + (x < 0.0f ? -0.5f : 0.5f));
    a = (x - (float)n) * 0.69314718f;
    p = 1.0f + a * (1.0f + a * (0.5f + a * (0.16666667f + a * (0.041666668f + a * (0.0083333338f + a * 0.0013888889f)))));
    s.u = (uint32_t)(n + 127) << 23;
    return p * s.f;
}

static float log2f_(float x)                    /* log2(x), x > 0, ~1e-5 */
{
    fu_t u;
    int32_t e;
    float m, z, q;
    if (x < 1e-30f) x = 1e-30f;
    u.f = x;
    e = (int32_t)(u.u >> 23) - 127;
    u.u = (u.u & 0x7FFFFFu) | 0x3F800000u;
    m = u.f;
    if (m > 1.41421356f) { m *= 0.5f; e++; }
    z = (m - 1.0f) / (m + 1.0f);
    q = z * z;
    return (float)e + 2.8853901f * z * (1.0f + q * (0.33333333f + q * (0.2f + q * (0.14285714f + q * 0.11111111f))));
}

static float sin_q(float a)                     /* sin(a) for 0 <= a <= pi/2 */
{
    float q = a * a;
    return a * (1.0f + q * (-0.16666667f + q * (0.0083333338f + q * (-0.00019841270f + q * 2.7557319e-6f))));
}

static float tan_w(float w)                     /* tan(w), 0 <= w < pi/2 */
{
    float c = sin_q(1.5707964f - w);
    return sin_q(w) / (c < 1e-6f ? 1e-6f : c);
}

static float soft_clip(float x)                 /* a tanh-like curve */
{
    float x2;
    if (x > 3.0f) return 1.0f;
    if (x < -3.0f) return -1.0f;
    x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

static float sine_ph(uint32_t ph)               /* one cycle over 2^32, -1..1 (refined parabola) */
{
    float x = (float)((int32_t)ph >> 8) * (1.0f / 8388608.0f);
    float ax = x < 0.0f ? -x : x;
    float y = 4.0f * x * (1.0f - ax);
    float ay = y < 0.0f ? -y : y;
    return y + 0.225f * (y * ay - y);
}

static uint32_t rng = 0x2545F491u;
static float rand01(void)
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return (float)(int32_t)(rng >> 8) * (1.0f / 16777216.0f);
}

/* ------------------------------------------------------------------ the sound (shared with the main loop) */

typedef struct {
    volatile float tune;            /* semitones from the drone's root */
    volatile float level;           /* 0..1 */
    volatile float lrate;           /* its own LFO: Hz (0.01..5) */
    volatile float ldepth;          /* 0..1: how deep it pulls the level down */
    volatile float sweet_t;         /* the nearest sweet spot, semitones (main loop) */
    uint8_t ji, beat;               /* which ratio; 1: the sweet spot beats in tempo */
    float drift, drift_target;      /* audio side only */
    float lph, lgain;               /* audio side: LFO phase 0..1, the gain it gave last block */
    uint32_t ph;
} osc_t;

typedef struct {
    osc_t o[NO];
    volatile float root;            /* MIDI note */
    volatile float wave;            /* 0 sine .. 1 saw */
    volatile float vol;             /* 0..1 */
    volatile float att_s, rel_s;    /* seconds */
    volatile int32_t gate;          /* 1 on (latched) */
    volatile float sweet;           /* its SWEET knob, 0..1 */
    float env;                      /* audio side only */
} drone_t;

typedef struct {
    volatile float cutoff;          /* Hz */
    volatile float res;             /* 0..1 */
    float ic1, ic2, g, k, a1, a2, a3;
} filt_t;

static drone_t dr[ND];
static filt_t fl[2];
static volatile float lfo_rate = 0.05f, lfo_depth = 1.5f, drift_amt = 0.08f, drive = 1.8f;
static float lfo_ph;
static volatile uint32_t load_pct;              /* render time, % of a block */

static void filt_coef(filt_t *f, float hz)
{
    float g = tan_w(3.1415927f * clampf(hz, 20.0f, 18000.0f) / SR);
    float k = 2.0f - 1.95f * clampf(f->res, 0.0f, 1.0f);   /* low k: high Q */
    f->g = g; f->k = k;
    f->a1 = 1.0f / (1.0f + g * (g + k));
    f->a2 = g * f->a1;
    f->a3 = g * f->a2;
}

static float filt_tick(filt_t *f, float x, float drv)
{
    float v1, v2, v3;
    x = soft_clip(x * drv);
    v3 = x - f->ic2;
    v1 = f->a1 * f->ic1 + f->a2 * v3;
    v2 = f->ic2 + f->a2 * f->ic1 + f->a3 * v3;
    f->ic1 = soft_clip(2.0f * v1 - f->ic1);     /* saturated band state: the rough resonance */
    f->ic2 = 2.0f * v2 - f->ic2;
    return v2;
}

/* ------------------------------------------------------------------ the reverb: a big hall
 * Pre-delay -> 4 series allpass diffusers -> an 8-line feedback delay network (orthonormal
 * Hadamard mixing, a low-pass in every line, slow modulation of every line's length against
 * metallic ringing). Decay is set as a time (T60, up to 60 s); the input is scaled with the
 * loop gain so long tails do not build up louder than short ones. About 190 KB of float memory. */

#define RV_N 8u
#define RV_MOD 24u                                  /* max modulation, samples */
static const uint16_t RV_LEN[RV_N] = {3011, 3527, 4129, 4813, 5501, 6143, 6959, 7907};   /* at SIZE 100 % */
static const uint16_t RV_AP[4] = {211, 157, 563, 409};
static const float RV_APG[4] = {0.75f, 0.75f, 0.625f, 0.625f};
#define RV_PRE 3000u                                /* max pre-delay, 68 ms */
#define RV_LINE_TOTAL (3011u + 3527u + 4129u + 4813u + 5501u + 6143u + 6959u + 7907u + RV_N * (RV_MOD + 4u))
#define RV_AP_TOTAL (211u + 157u + 563u + 409u)

static float rv_line[RV_LINE_TOTAL], rv_ap[RV_AP_TOTAL], rv_pre[RV_PRE];
static uint32_t rv_base[RV_N], rv_cap[RV_N], rv_w[RV_N], rv_apbase[4], rv_api[4], rv_prew;
static float rv_lp[RV_N], rv_ph[RV_N];
static volatile float rv_decay = 12.0f, rv_size = 0.85f, rv_tone = 0.5f, rv_mix = 0.35f;

static float sqrt_(float x)                         /* x >= 0 */
{
    fu_t u;
    uint32_t i;
    float y;
    if (x <= 0.0f) return 0.0f;
    u.f = x;
    u.u = (u.u >> 1) + 0x1FC00000u;
    y = u.f;
    for (i = 0; i < 3u; i++) y = 0.5f * (y + x / y);
    return y;
}

static void rv_init(void)
{
    uint32_t i, at = 0;
    for (i = 0; i < RV_N; i++) {
        rv_base[i] = at;
        rv_cap[i] = RV_LEN[i] + RV_MOD + 4u;
        at += rv_cap[i];
        rv_w[i] = 0;
        rv_lp[i] = 0.0f;
        rv_ph[i] = (float)i * 0.137f;               /* spread the modulation phases */
    }
    for (i = 0; i < RV_LINE_TOTAL; i++) rv_line[i] = 0.0f;
    for (i = 0, at = 0; i < 4u; i++) { rv_apbase[i] = at; rv_api[i] = 0; at += RV_AP[i]; }
    for (i = 0; i < RV_AP_TOTAL; i++) rv_ap[i] = 0.0f;
    for (i = 0; i < RV_PRE; i++) rv_pre[i] = 0.0f;
    rv_prew = 0;
}

/* in: dry stereo, n frames (interleaved floats); adds the wet signal into it */
static void rv_process(float *lr, uint32_t n)
{
    float g[RV_N], dlen[RV_N], dmp, mix = rv_mix, inj, gsum = 0.0f, size = clampf(rv_size, 0.5f, 1.0f);
    float t60 = clampf(rv_decay, 0.3f, 60.0f);
    uint32_t i, k, pre = (uint32_t)(size * (float)(RV_PRE - 1u));
    static uint32_t tick;
    if (mix <= 0.0f) return;

    dmp = 1.0f - exp2f_(-6.2831853f * 1000.0f * exp2f_(clampf(rv_tone, 0.0f, 1.0f) * 4.0f) / SR * 1.442695f);
    for (k = 0; k < RV_N; k++) {
        float len = (float)RV_LEN[k] * size, ph = rv_ph[k];
        g[k] = exp2f_(-9.9657843f * len / (t60 * SR));          /* 10^(-3 len / (T60 fs)) */
        gsum += g[k] * g[k];
        ph += (0.07f + 0.023f * (float)k) * (float)n / SR;       /* 0.07 .. 0.23 Hz */
        if (ph >= 1.0f) ph -= 1.0f;
        rv_ph[k] = ph;
        dlen[k] = len + (float)(RV_MOD / 2u) * (1.0f + sine_ph((uint32_t)(int32_t)(ph * 4294967296.0f - 2147483648.0f) + 0x80000000u));
    }
    inj = 0.35f * sqrt_(1.0f - gsum * (1.0f / (float)RV_N));     /* equal loudness at any decay */

    for (i = 0; i < n; i++) {
        float x = (lr[2u * i] + lr[2u * i + 1u]) * 0.5f, y[RV_N], a, b, c, d2, wl, wr;
        uint32_t r;
        /* pre-delay */
        rv_pre[rv_prew] = x;
        r = rv_prew + RV_PRE - pre; if (r >= RV_PRE) r -= RV_PRE;
        x = rv_pre[r];
        if (++rv_prew >= RV_PRE) rv_prew = 0;
        /* diffusion: 4 allpasses */
        for (k = 0; k < 4u; k++) {
            float *buf = &rv_ap[rv_apbase[k]], dd = buf[rv_api[k]], w = x - RV_APG[k] * dd;
            buf[rv_api[k]] = w;
            x = dd + RV_APG[k] * w;
            if (++rv_api[k] >= RV_AP[k]) rv_api[k] = 0;
        }
        x *= inj;
        x += (tick++ & 1u) ? 1e-15f : -1e-15f;                    /* keeps the tail out of denormals */
        /* read the 8 lines (interpolated), damp */
        for (k = 0; k < RV_N; k++) {
            float *buf = &rv_line[rv_base[k]], pos = (float)rv_w[k] - dlen[k], fr, s0, s1;
            int32_t ip;
            if (pos < 0.0f) pos += (float)rv_cap[k];
            ip = (int32_t)pos;
            fr = pos - (float)ip;
            s0 = buf[ip];
            s1 = buf[(uint32_t)ip + 1u < rv_cap[k] ? ip + 1 : 0];
            s0 += fr * (s1 - s0);
            rv_lp[k] += (s0 - rv_lp[k]) * dmp;
            y[k] = rv_lp[k];
        }
        /* the stereo outputs: two orthogonal sign patterns */
        wl = y[0] - y[1] + y[2] - y[3] + y[4] - y[5] + y[6] - y[7];
        wr = y[0] + y[1] - y[2] - y[3] + y[4] + y[5] - y[6] - y[7];
        /* feedback: gains, then the 8x8 Hadamard (3 butterfly stages), normalised */
        for (k = 0; k < RV_N; k++) y[k] *= g[k];
        for (k = 0; k < RV_N; k += 2u) { a = y[k]; b = y[k + 1u]; y[k] = a + b; y[k + 1u] = a - b; }
        for (k = 0; k < RV_N; k += 4u) {
            a = y[k]; b = y[k + 1u]; c = y[k + 2u]; d2 = y[k + 3u];
            y[k] = a + c; y[k + 1u] = b + d2; y[k + 2u] = a - c; y[k + 3u] = b - d2;
        }
        for (k = 0; k < 4u; k++) { a = y[k]; b = y[k + 4u]; y[k] = a + b; y[k + 4u] = a - b; }
        for (k = 0; k < RV_N; k++) {
            float *buf = &rv_line[rv_base[k]];
            buf[rv_w[k]] = y[k] * 0.35355339f + ((k & 1u) ? -x : x);
            if (++rv_w[k] >= rv_cap[k]) rv_w[k] = 0;
        }
        lr[2u * i] += wl * 0.5f * mix;
        lr[2u * i + 1u] += wr * 0.5f * mix;
    }
}

static int32_t roundi(float x) { return (int32_t)(x + (x < 0.0f ? -0.5f : 0.5f)); }

/* ------------------------------------------------------------------ the two melodic voices */

#define NV 2u
typedef struct {
    int32_t key;                    /* the note it plays (MIDI), -1 none */
    uint32_t stage, age;            /* 0 silent, 1 attack, 2 decay/sustain, 3 release */
    float pitch, target, env;
    uint32_t ph[2];
    filt_t f;
} voice_t;
static voice_t vo[NV];
static uint32_t v_age;
static volatile float v_wave = 0.0f, v_beat = 0.08f, v_cut = 900.0f, v_res = 0.45f;   /* beat: semitones */
static volatile float v_att = 0.01f, v_dec = 0.4f, v_sus = 0.6f, v_rel = 0.8f;       /* seconds, 0..1 */
static volatile float v_env = 3.0f, v_glide = 0.05f, v_vol = 0.6f;                   /* octaves, s, 0..1 */
static volatile int32_t v_oct;
/* noise + sample & hold */
static volatile float noise_lv = 0.0f, sh_rate = 0.0f, sh_cut = 1.0f, sh_pitch = 0.0f;  /* rate 0: each step */
static volatile float sh_val;
static float sh_ph;
/* the sequencer: 5 steps */
#define NS 5u
static volatile uint8_t sq_note[NS] = {48, 55, 51, 58, 60}, sq_rest;
static volatile float sq_bpm = 96.0f, sq_gate = 0.5f;
static volatile int32_t sq_len = 5, sq_key = 0;
static volatile uint32_t sq_run, sq_pos;
static float sq_ph;
static uint32_t sq_was, sq_on;
/* the echo: on the voices, a tape-like line at half the rate (1 s), two taps for the stereo */
#define DL_LEN 22050u
static int16_t dl_buf[DL_LEN];
static uint32_t dl_w, dl_half;
static float dl_t, dl_lp, dl_acc, dl_o[4];
static volatile float dl_time = 0.375f, dl_feed = 0.45f, dl_tone = 0.5f, dl_mix = 0.35f;

/* notes from the main loop to the audio side (the voices are only touched there) */
static struct { uint8_t on, note; } vq[32];
static volatile uint32_t vq_w, vq_r, v_panic;
static void vpush(uint32_t on, uint32_t note)
{
    uint32_t w = vq_w;
    if (w - vq_r >= 32u) return;
    vq[w & 31u].on = (uint8_t)on;
    vq[w & 31u].note = (uint8_t)note;
    __asm__ volatile ("" ::: "memory");
    vq_w = w + 1u;
}

static void voice_on(uint32_t i, int32_t note)
{
    voice_t *v = &vo[i];
    v->key = note;
    v->target = (float)note;
    if (v->stage == 0u) { v->pitch = v->target; v->env = 0.0f; }
    v->stage = 1u;
    v->age = ++v_age;
}

static void voice_off(uint32_t i) { if (vo[i].stage && vo[i].stage < 3u) vo[i].stage = 3u; }

static void key_note(uint32_t on, int32_t note)  /* a key or MIDI note: voice 2 alone while the sequencer runs */
{
    uint32_t i, best = NV;
    if (on) {
        if (sq_run) best = 1u;
        else {
            for (i = 0; i < NV; i++)                    /* a free voice, the one free longest */
                if ((vo[i].stage == 0u || vo[i].stage == 3u) && (best == NV || vo[i].age < vo[best].age)) best = i;
            if (best == NV) best = vo[0].age < vo[1].age ? 0u : 1u;   /* both held: the older one */
        }
        voice_on(best, note);
    } else {
        for (i = 0; i < NV; i++)
            if (vo[i].key == note && !(sq_run && i == 0u)) voice_off(i);
    }
}

static float blep(float t, float dt, float idt)  /* polyBLEP correction for a step at t = 0 */
{
    float u;
    if (t < dt) { u = t * idt; return u + u - u * u - 1.0f; }
    if (t > 1.0f - dt) { u = (t - 1.0f) * idt; return u * u + u + u + 1.0f; }
    return 0.0f;
}

static float ph01(uint32_t ph) { return (float)(int32_t)(ph >> 8) * (1.0f / 16777216.0f); }

static void seq_block(uint32_t n)               /* steps, gate, the S&H; on the audio side */
{
    uint32_t len = (uint32_t)(sq_len < 1 ? 1 : sq_len > (int32_t)NS ? (int32_t)NS : sq_len);
    if (sq_run && !sq_was) { sq_ph = 1.0f; sq_pos = len - 1u; }
    if (!sq_run && sq_was) { voice_off(0); sq_on = 0; }
    sq_was = sq_run;
    if (sq_run) {
        sq_ph += clampf(sq_bpm, 20.0f, 300.0f) * (1.0f / 60.0f) * (float)n / SR;
        if (sq_ph >= 1.0f) {
            sq_ph -= 1.0f;
            if (sq_ph >= 1.0f) sq_ph = 0.0f;
            sq_pos = sq_pos + 1u >= len ? 0u : sq_pos + 1u;
            if (sh_rate <= 0.0f) sh_val = rand01() * 2.0f - 1.0f;
            if (sq_rest & (1u << sq_pos)) { voice_off(0); sq_on = 0; }
            else { voice_on(0, (int32_t)sq_note[sq_pos] + sq_key + 12 * v_oct); sq_on = 1; }
        } else if (sq_on && sq_ph >= sq_gate && sq_gate < 0.999f) { voice_off(0); sq_on = 0; }
    }
    if (sh_rate > 0.0f) {
        sh_ph += sh_rate * (float)n / SR;
        if (sh_ph >= 1.0f) { sh_ph -= (float)(int32_t)sh_ph; sh_val = rand01() * 2.0f - 1.0f; }
    }
}

static void voices(float *vb, uint32_t n)        /* adds the two voices (mono) into vb */
{
    uint32_t vi, i, j, k;
    float ainc = 1.0f / (clampf(v_att, 0.002f, 10.0f) * SR);
    float dcf = 1.0f - exp2f_(-9.9657843f / (clampf(v_dec, 0.005f, 10.0f) * SR));
    float rcf = 1.0f - exp2f_(-9.9657843f / (clampf(v_rel, 0.005f, 10.0f) * SR));
    float gk = v_glide < 0.002f ? 1.0f : 1.0f - exp2f_(-1.442695f * (float)n / (v_glide * SR));
    float sus = clampf(v_sus, 0.0f, 1.0f), w = clampf(v_wave, 0.0f, 1.0f), nz = noise_lv, g = v_vol * 0.5f;
    float shv = sh_val, shp = (float)roundi(shv * sh_pitch), envamt = v_env;
    for (vi = 0; vi < NV; vi++) {
        voice_t *v = &vo[vi];
        uint32_t inc[2];
        float dt[2], idt[2], env = v->env;
        if (!v->stage) continue;
        v->pitch += (v->target - v->pitch) * gk;
        for (k = 0; k < 2u; k++) {
            float hz = 440.0f * exp2f_((v->pitch + shp + (k ? v_beat : 0.0f) - 69.0f) * (1.0f / 12.0f));
            hz = clampf(hz, 8.0f, 12000.0f);
            inc[k] = (uint32_t)(int32_t)(hz * (4294967296.0f / SR));
            dt[k] = ph01(inc[k]);
            idt[k] = 1.0f / dt[k];
        }
        v->f.res = v_res;
        for (i = 0; i < n; i += SUB) {
            filt_coef(&v->f, v_cut * exp2f_(env * envamt + shv * sh_cut));
            for (j = i; j < i + SUB && j < n; j++) {
                float x = 0.0f;
                if (v->stage == 1u) { env += ainc; if (env >= 1.0f) { env = 1.0f; v->stage = 2u; } }
                else if (v->stage == 2u) env += (sus - env) * dcf;
                else if (v->stage == 3u) { env -= env * rcf; if (env < 1e-4f) { env = 0.0f; v->stage = 0u; } }
                for (k = 0; k < 2u; k++) {
                    uint32_t ph = v->ph[k];
                    float t = ph01(ph), t2 = ph01(ph + 0x80000000u);
                    float saw = 2.0f * t - 1.0f - blep(t, dt[k], idt[k]);
                    float sq = saw - (2.0f * t2 - 1.0f - blep(t2, dt[k], idt[k]));
                    x += saw + w * (sq - saw);
                    v->ph[k] = ph + inc[k];
                }
                x = x * 0.5f + nz * rand01();
                vb[j] += filt_tick(&v->f, x * 0.7f, 1.4f) * env * g;
            }
        }
        v->env = env;
    }
}

static float dl_read(float d)
{
    float pos = (float)dl_w - d, fr;
    int32_t ip;
    uint32_t i1;
    if (pos < 0.0f) pos += (float)DL_LEN;
    ip = (int32_t)pos;
    fr = pos - (float)ip;
    i1 = (uint32_t)ip + 1u >= DL_LEN ? 0u : (uint32_t)ip + 1u;
    return ((float)dl_buf[ip] + fr * (float)(dl_buf[i1] - dl_buf[ip])) * (1.0f / 30000.0f);
}

static void echo(const float *vb, float *lr, uint32_t n)   /* the voices, dry and echoed, into lr */
{
    float tgt = clampf(dl_time, 0.02f, 1.0f) * (SR * 0.5f), fb = clampf(dl_feed, 0.0f, 0.95f);
    float tc = 0.08f + 0.9f * clampf(dl_tone, 0.0f, 1.0f), mix = dl_mix;
    uint32_t i;
    if (tgt > (float)(DL_LEN - 4u)) tgt = (float)(DL_LEN - 4u);
    if (dl_t <= 0.0f) dl_t = tgt;
    dl_t += (tgt - dl_t) * 0.08f;                               /* a time change glides, like tape */
    for (i = 0; i < n; i++) {
        float x = vb[i], ol, orr;
        dl_acc += x;
        if (dl_half) {                                          /* every second frame: one step of the line */
            float a = dl_read(dl_t), b = dl_read(dl_t * 0.75f);
            dl_lp += (a - dl_lp) * tc;
            dl_buf[dl_w] = (int16_t)(soft_clip(dl_acc * 0.5f + dl_lp * fb) * 30000.0f);
            if (++dl_w >= DL_LEN) dl_w = 0;
            dl_acc = 0.0f;
            dl_o[0] = dl_o[1]; dl_o[1] = a;
            dl_o[2] = dl_o[3]; dl_o[3] = b;
            ol = a; orr = b;
        } else {
            ol = 0.5f * (dl_o[0] + dl_o[1]); orr = 0.5f * (dl_o[2] + dl_o[3]);
        }
        dl_half ^= 1u;
        lr[2u * i] += 0.8f * x + ol * mix;
        lr[2u * i + 1u] += 0.8f * x + orr * mix;
    }
}

static void render(int32_t *out, uint32_t n)
{
    static float bus[2][256], lr[512], vb[256];
    uint32_t t0 = H->ticks(), d, o, i, j;
    float drv = drive;
    if (n > 256u) n = 256u;

    while (vq_r != vq_w) {                                  /* notes from the keys */
        uint32_t r = vq_r;
        key_note(vq[r & 31u].on, vq[r & 31u].note);
        vq_r = r + 1u;
    }
    if (v_panic) { v_panic = 0; voice_off(0); voice_off(1); }
    seq_block(n);

    for (i = 0; i < n; i++) bus[0][i] = bus[1][i] = vb[i] = 0.0f;

    for (d = 0; d < ND; d++) {
        drone_t *v = &dr[d];
        float target = v->gate ? 1.0f : 0.0f;
        float step = 1.0f / (clampf(v->gate ? v->att_s : v->rel_s, 0.05f, 60.0f) * SR);
        float env = v->env, w = v->wave, g = v->vol * 0.25f;
        float sw = v->sweet, da = drift_amt * (1.0f - 0.8f * sw);   /* sweet spots want steady drift */
        float *b = bus[d < 3u ? 0 : 1];
        uint32_t inc[NO];
        float lev[NO], dlev[NO], dts[NO], idt[NO];
        if (env <= 0.0f && !v->gate) continue;              /* silent: costs nothing */

        for (o = 0; o < NO; o++) {                          /* pitch once a block */
            osc_t *s = &v->o[o];
            float hz;
            if (rand01() < 0.02f) s->drift_target = (rand01() * 2.0f - 1.0f) * da;
            s->drift += (s->drift_target - s->drift) * 0.003f;
            hz = 440.0f * exp2f_((v->root + s->tune + sw * (s->sweet_t - s->tune) + s->drift - 69.0f) * (1.0f / 12.0f));
            hz = clampf(hz, 1.0f, 8000.0f);
            inc[o] = (uint32_t)(int32_t)(hz * (4294967296.0f / SR));
            {                                               /* the oscillator's LFO: level, ramped over the block */
                float ph2 = s->lph + s->lrate * (float)n / SR, gnew;
                if (ph2 >= 1.0f) ph2 -= (float)(int32_t)ph2;
                s->lph = ph2;
                gnew = 1.0f - clampf(s->ldepth, 0.0f, 1.0f) *
                       (0.5f + 0.5f * sine_ph((uint32_t)(int32_t)(ph2 * 4294967296.0f - 2147483648.0f) + 0x80000000u));
                lev[o] = s->level * s->lgain;
                dlev[o] = (s->level * gnew - lev[o]) / (float)n;
                s->lgain = gnew;
            }
            dts[o] = ph01(inc[o]);
            idt[o] = 1.0f / dts[o];
        }
        for (i = 0; i < n; i++) {
            float x = 0.0f;
            if (env < target) { env += step; if (env > target) env = target; }
            else if (env > target) { env -= step; if (env < target) env = target; }
            for (o = 0; o < NO; o++) {
                uint32_t ph = v->o[o].ph;
                float t = ph01(ph + 0x80000000u);
                float si = sine_ph(ph), saw = 2.0f * t - 1.0f - blep(t, dts[o], idt[o]);
                x += lev[o] * (si + w * (saw - si));
                lev[o] += dlev[o];
                v->o[o].ph = ph + inc[o];
            }
            b[i] += x * g * env;
        }
        v->env = env;
    }

    for (i = 0; i < n; i += SUB) {                          /* filters, LFO every SUB frames */
        float lfo = sine_ph((uint32_t)(int32_t)(lfo_ph * 4294967296.0f - 2147483648.0f) + 0x80000000u);
        float m = exp2f_(lfo * lfo_depth);
        lfo_ph += lfo_rate * (float)SUB / SR;
        if (lfo_ph >= 1.0f) lfo_ph -= 1.0f;
        filt_coef(&fl[0], fl[0].cutoff * m);
        filt_coef(&fl[1], fl[1].cutoff / m);
        for (j = i; j < i + SUB && j < n; j++) {
            float a = filt_tick(&fl[0], bus[0][j], drv);
            float c = filt_tick(&fl[1], bus[1][j], drv);
            lr[2u * j] = (0.7f * a + 0.3f * c) * 1.1f;
            lr[2u * j + 1u] = (0.3f * a + 0.7f * c) * 1.1f;
        }
    }
    voices(vb, n);
    echo(vb, lr, n);                                        /* the voices and their echo */
    rv_process(lr, n);                                      /* the hall, added to everything */
    for (j = 0; j < 2u * n; j++) {
        int32_t s = out[j] + (int32_t)(soft_clip(lr[j]) * 30000.0f);
        out[j] = s > 32767 ? 32767 : s < -32767 ? -32767 : s;
    }
    load_pct = (H->ticks() - t0) * 100u / (5805u * 24u);
}

/* ------------------------------------------------------------------ sweet spots (main loop)
 * For every oscillator: the nearest just ratio to its drone's root (in any octave), and around it the
 * tunings a hair off that beat 1/4, 1/2, 1 or 2 times a sequencer step: whichever is nearest is its
 * sweet spot. SWEET pulls the oscillator that far towards it (render). */

static const float JI[13] = {0.0f, 1.1173129f, 2.0391000f, 3.1564129f, 3.8631371f, 4.9804500f, 5.8251344f,
                             7.0195500f, 8.1368629f, 8.8435871f, 9.6882590f, 10.882687f, 12.0f};
static const char *const JI_NAME[13] = {"1/1", "16/15", "9/8", "6/5", "5/4", "4/3", "7/5", "3/2", "8/5", "5/3",
                                        "7/4", "15/8", "2/1"};

static void sweet_update(void)
{
    static const float BEAT[4] = {0.25f, 0.5f, 1.0f, 2.0f};
    uint32_t d, o, k, b;
    float step_hz = clampf(sq_bpm, 20.0f, 300.0f) * (1.0f / 60.0f);
    for (d = 0; d < ND; d++)
        for (o = 0; o < NO; o++) {
            osc_t *s = &dr[d].o[o];
            float t = s->tune, fr, base, best, bd, f;
            int32_t oc = (int32_t)(t * (1.0f / 12.0f));
            uint32_t ji = 0, beat = 0;
            if ((float)oc * 12.0f > t) oc--;                    /* floor */
            fr = t - 12.0f * (float)oc;
            for (k = 1; k < 13u; k++)
                if ((fr - JI[k]) * (fr - JI[k]) < (fr - JI[ji]) * (fr - JI[ji])) ji = k;
            if (ji == 12u) { ji = 0; oc++; }                  /* the octave above: 1/1 again */
            base = 12.0f * (float)oc + JI[ji];
            best = base;
            bd = t > base ? t - base : base - t;
            f = 440.0f * exp2f_((dr[d].root + base - 69.0f) * (1.0f / 12.0f));
            for (b = 0; b < 8u; b++) {                          /* beating in time, either side */
                float r = BEAT[b >> 1] * step_hz / f, c, e;
                if (r > 0.5f) continue;
                c = base + 12.0f * log2f_((b & 1u) ? 1.0f - r : 1.0f + r);
                e = t > c ? t - c : c - t;
                if (c - base < 0.5f && base - c < 0.5f && e < bd) { bd = e; best = c; beat = 1; }
            }
            s->sweet_t = best;
            s->ji = (uint8_t)ji;
            s->beat = (uint8_t)beat;
        }
}

/* ------------------------------------------------------------------ the sound as saved (main loop) */

typedef struct {
    uint32_t magic;
    int16_t cents[ND][NO];          /* tune in cents */
    uint8_t level[ND][NO];          /* 0..100 */
    uint8_t root[ND], wave[ND], vol[ND], att[ND], rel[ND];   /* note, %, %, 0.1 s, 0.1 s */
    uint16_t cut[2];                /* Hz */
    uint8_t res[2];                 /* % */
    uint8_t lfo_rate, lfo_depth, drift, drv;   /* 0.01 Hz, 0.1 oct, cent, 0.1 */
    uint8_t rv_dec, rv_siz, rv_ton, rv_mx;      /* reverb (v2): 0.25 s, %, %, % */
    uint16_t lrate[ND][NO];         /* oscillator LFOs (v3): 0.001 Hz */
    uint8_t ldepth[ND][NO];         /* % */
    /* v4: the voices, sequencer, S&H, echo */
    uint16_t v_cut, v_att, v_dec, v_rel, v_glide, sq_bpm, dl_time, sh_rate;   /* Hz, ms, ms, ms, ms, BPM, ms, 0.01 Hz */
    uint8_t v_wave, v_beat, v_res, v_sus, v_env, v_vol;          /* %, cent, %, %, 0.1 oct, % */
    int8_t v_oct, sq_key;
    uint8_t sq_gate, sq_len, sq_rest, sq_note[NS];
    uint8_t sh_cut, sh_pitch, noise, dl_feed, dl_tone, dl_mix;   /* 0.1 oct, semitone, %, %, %, % */
    uint8_t v4_pad[2];              /* (a v4 sound was 232 bytes long) */
    uint8_t sweet[ND];              /* v6: % per drone (v5: one for all, 236 bytes) */
} saved_t;
#define MAGIC 0x48454C36u           /* "HEL6" */
#define MAGIC_V5 0x48454C35u        /* "HEL5": one SWEET for all the drones */
#define SAVED_V5 236u
#define MAGIC_V4 0x48454C34u        /* "HEL4": before SWEET */
#define SAVED_V4 ((uint32_t)__builtin_offsetof(saved_t, sweet))
#define MAGIC_V3 0x48454C33u        /* "HEL3": before the voices */
#define SAVED_V3 ((uint32_t)__builtin_offsetof(saved_t, v_cut))
#define MAGIC_V2 0x48454C32u        /* "HEL2": before the oscillator LFOs */
#define SAVED_V2 ((uint32_t)__builtin_offsetof(saved_t, lrate))
#define MAGIC_V1 0x48454C31u        /* "HEL1": a sound saved before the reverb, without its last 4 bytes */
#define SAVED_V1 ((uint32_t)__builtin_offsetof(saved_t, rv_dec))
typedef char saved_fits[sizeof(saved_t) <= 240u ? 1 : -1];
typedef char saved_v4[__builtin_offsetof(saved_t, sweet) == 232u ? 1 : -1];

static void voice_defaults(void)
{
    static const uint8_t STEP[NS] = {48, 55, 51, 58, 60};
    uint32_t i;
    v_wave = 0.0f; v_beat = 0.08f; v_cut = 900.0f; v_res = 0.45f;
    v_att = 0.01f; v_dec = 0.4f; v_sus = 0.6f; v_rel = 0.8f;
    v_env = 3.0f; v_glide = 0.05f; v_vol = 0.6f; v_oct = 0;
    noise_lv = 0.0f; sh_rate = 0.0f; sh_cut = 1.0f; sh_pitch = 0.0f;
    sq_bpm = 96.0f; sq_gate = 0.5f; sq_len = 5; sq_key = 0; sq_rest = 0;
    for (i = 0; i < NS; i++) sq_note[i] = STEP[i];
    dl_time = 0.375f; dl_feed = 0.45f; dl_tone = 0.5f; dl_mix = 0.35f;
}

static void defaults(void)
{
    static const uint8_t ROOT[ND] = {36, 43, 48, 31, 55, 60};
    static const float TUNE[NO] = {0.0f, 0.07f, 7.02f, 11.9f};
    static const float LRATE[NO] = {0.05f, 0.07f, 0.11f, 0.13f};
    uint32_t d, o;
    for (d = 0; d < ND; d++) {
        dr[d].root = ROOT[d];
        dr[d].wave = (d & 1u) ? 0.7f : 0.3f;
        dr[d].vol = 0.5f;
        dr[d].att_s = 4.0f;
        dr[d].rel_s = 6.0f;
        for (o = 0; o < NO; o++) {
            dr[d].o[o].tune = TUNE[o] + (rand01() - 0.5f) * 0.1f;
            dr[d].o[o].level = o ? 0.6f : 0.9f;
            dr[d].o[o].lrate = LRATE[o] * (1.0f + 0.1f * (float)d);   /* all a little different */
            dr[d].o[o].ldepth = 0.0f;                               /* off until turned up */
        }
    }
    fl[0].cutoff = 700.0f; fl[1].cutoff = 1200.0f;
    fl[0].res = fl[1].res = 0.6f;
    lfo_rate = 0.05f; lfo_depth = 1.5f; drift_amt = 0.08f; drive = 1.8f;
    rv_decay = 12.0f; rv_size = 0.85f; rv_tone = 0.5f; rv_mix = 0.35f;
    voice_defaults();
    for (d = 0; d < ND; d++) dr[d].sweet = 0.0f;
}

static uint8_t pc(float x) { return (uint8_t)roundi(clampf(x, 0.0f, 1.0f) * 100.0f); }
static uint16_t ms16(float s) { return (uint16_t)roundi(clampf(s, 0.0f, 60.0f) * 1000.0f); }

static void save_sound(void)
{
    saved_t s;
    uint32_t d, o;
    for (d = 0; d < sizeof s; d++) ((uint8_t *)&s)[d] = 0;
    s.magic = MAGIC;
    for (d = 0; d < ND; d++) {
        for (o = 0; o < NO; o++) {
            s.cents[d][o] = (int16_t)roundi(dr[d].o[o].tune * 100.0f);
            s.level[d][o] = (uint8_t)roundi(dr[d].o[o].level * 100.0f);
            s.lrate[d][o] = (uint16_t)roundi(dr[d].o[o].lrate * 1000.0f);
            s.ldepth[d][o] = (uint8_t)roundi(dr[d].o[o].ldepth * 100.0f);
        }
        s.root[d] = (uint8_t)roundi(dr[d].root);
        s.wave[d] = (uint8_t)roundi(dr[d].wave * 100.0f);
        s.vol[d] = (uint8_t)roundi(dr[d].vol * 100.0f);
        s.att[d] = (uint8_t)roundi(dr[d].att_s * 10.0f);
        s.rel[d] = (uint8_t)roundi(dr[d].rel_s * 10.0f);
    }
    for (d = 0; d < 2u; d++) {
        s.cut[d] = (uint16_t)roundi(fl[d].cutoff);
        s.res[d] = (uint8_t)roundi(fl[d].res * 100.0f);
    }
    s.lfo_rate = (uint8_t)roundi(lfo_rate * 100.0f);
    s.lfo_depth = (uint8_t)roundi(lfo_depth * 10.0f);
    s.drift = (uint8_t)roundi(drift_amt * 100.0f);
    s.drv = (uint8_t)roundi(drive * 10.0f);
    s.rv_dec = (uint8_t)roundi(rv_decay * 4.0f);
    s.rv_siz = (uint8_t)roundi(rv_size * 100.0f);
    s.rv_ton = (uint8_t)roundi(rv_tone * 100.0f);
    s.rv_mx = (uint8_t)roundi(rv_mix * 100.0f);
    s.v_cut = (uint16_t)roundi(v_cut);
    s.v_att = ms16(v_att); s.v_dec = ms16(v_dec); s.v_rel = ms16(v_rel); s.v_glide = ms16(v_glide);
    s.sq_bpm = (uint16_t)roundi(sq_bpm);
    s.dl_time = ms16(dl_time);
    s.sh_rate = (uint16_t)roundi(sh_rate * 100.0f);
    s.v_wave = pc(v_wave); s.v_beat = (uint8_t)roundi(v_beat * 100.0f); s.v_res = pc(v_res); s.v_sus = pc(v_sus);
    s.v_env = (uint8_t)roundi(v_env * 10.0f); s.v_vol = pc(v_vol);
    s.v_oct = (int8_t)v_oct; s.sq_key = (int8_t)sq_key;
    s.sq_gate = pc(sq_gate); s.sq_len = (uint8_t)sq_len; s.sq_rest = sq_rest;
    for (d = 0; d < NS; d++) s.sq_note[d] = sq_note[d];
    s.sh_cut = (uint8_t)roundi(sh_cut * 10.0f); s.sh_pitch = (uint8_t)roundi(sh_pitch); s.noise = pc(noise_lv);
    s.dl_feed = pc(dl_feed); s.dl_tone = pc(dl_tone); s.dl_mix = pc(dl_mix);
    for (d = 0; d < ND; d++) s.sweet[d] = pc(dr[d].sweet);
    H->save(&s, sizeof s);
}

static void load_sound(void)
{
    saved_t s;
    uint32_t d, o, ver = 0;
    if (H->load(&s, sizeof s) == 0 && s.magic == MAGIC) ver = 6;
    else if (H->load(&s, SAVED_V5) == 0 && s.magic == MAGIC_V5) ver = 5;
    else if (H->load(&s, SAVED_V4) == 0 && s.magic == MAGIC_V4) ver = 4;
    else if (H->load(&s, SAVED_V3) == 0 && s.magic == MAGIC_V3) ver = 3;
    else if (H->load(&s, SAVED_V2) == 0 && s.magic == MAGIC_V2) ver = 2;
    else if (H->load(&s, SAVED_V1) == 0 && s.magic == MAGIC_V1) ver = 1;
    if (!ver) return;                                       /* nothing saved: keep the defaults */
    if (ver >= 2u) {
        rv_decay = (float)s.rv_dec * 0.25f;
        rv_size = (float)s.rv_siz * 0.01f;
        rv_tone = (float)s.rv_ton * 0.01f;
        rv_mix = (float)s.rv_mx * 0.01f;
    }
    for (d = 0; d < ND; d++) {
        for (o = 0; o < NO; o++) {
            dr[d].o[o].tune = (float)s.cents[d][o] * 0.01f;
            dr[d].o[o].level = (float)s.level[d][o] * 0.01f;
            if (ver >= 3u) {
                dr[d].o[o].lrate = clampf((float)s.lrate[d][o] * 0.001f, 0.01f, 5.0f);
                dr[d].o[o].ldepth = (float)s.ldepth[d][o] * 0.01f;
            }
        }
        dr[d].root = (float)s.root[d];
        dr[d].wave = (float)s.wave[d] * 0.01f;
        dr[d].vol = (float)s.vol[d] * 0.01f;
        dr[d].att_s = (float)s.att[d] * 0.1f;
        dr[d].rel_s = (float)s.rel[d] * 0.1f;
    }
    for (d = 0; d < 2u; d++) {
        fl[d].cutoff = (float)s.cut[d];
        fl[d].res = (float)s.res[d] * 0.01f;
    }
    lfo_rate = (float)s.lfo_rate * 0.01f;
    lfo_depth = (float)s.lfo_depth * 0.1f;
    drift_amt = (float)s.drift * 0.01f;
    drive = (float)s.drv * 0.1f;
    if (ver < 4u) return;                                   /* an older sound: the voices keep their defaults */
    v_cut = clampf((float)s.v_cut, 30.0f, 12000.0f);
    v_att = (float)s.v_att * 0.001f; v_dec = (float)s.v_dec * 0.001f; v_rel = (float)s.v_rel * 0.001f;
    v_glide = (float)s.v_glide * 0.001f;
    sq_bpm = clampf((float)s.sq_bpm, 20.0f, 300.0f);
    dl_time = clampf((float)s.dl_time * 0.001f, 0.02f, 1.0f);
    sh_rate = (float)s.sh_rate * 0.01f;
    v_wave = (float)s.v_wave * 0.01f; v_beat = (float)s.v_beat * 0.01f; v_res = (float)s.v_res * 0.01f;
    v_sus = (float)s.v_sus * 0.01f; v_env = (float)s.v_env * 0.1f; v_vol = (float)s.v_vol * 0.01f;
    v_oct = s.v_oct; sq_key = s.sq_key;
    sq_gate = (float)s.sq_gate * 0.01f;
    sq_len = s.sq_len < 1u ? 1 : s.sq_len > NS ? (int32_t)NS : (int32_t)s.sq_len;
    sq_rest = s.sq_rest;
    for (d = 0; d < NS; d++) sq_note[d] = s.sq_note[d];
    sh_cut = (float)s.sh_cut * 0.1f; sh_pitch = (float)s.sh_pitch; noise_lv = (float)s.noise * 0.01f;
    dl_feed = (float)s.dl_feed * 0.01f; dl_tone = (float)s.dl_tone * 0.01f; dl_mix = (float)s.dl_mix * 0.01f;
    for (d = 0; d < ND; d++)
        if (ver >= 5u) dr[d].sweet = (float)s.sweet[ver >= 6u ? d : 0] * 0.01f;
}

/* ------------------------------------------------------------------ the panel (main loop) */

static const uint8_t DRONE_KEY[ND] = {0, 2, 4, 6, 7, 9};   /* F3 G3 A3 B3 C4 D4 */
static uint32_t sel, held, page, lsub, dirty = 1, saved_mask, msg_until;   /* page: 0 or the open page's button */
static uint32_t home_ms;                        /* when HOME went down */
static uint32_t rec, rec_n;                     /* recording the steps: on, steps so far */
static uint8_t key_held[27];                    /* the note each upper key started (0: none) */
#define PAGE_BTNS ((1u << HB_BTN_EDIT) | (1u << HB_BTN_FX) | (1u << HB_BTN_GLO) | (1u << HB_BTN_LFO) | \
                   (1u << HB_BTN_ENV) | (1u << HB_BTN_ARP) | (1u << HB_BTN_SEQ))
#define SEL_HELD (held & (1u << HB_BTN_SEL))
static const char *msg = "";

static uint32_t nsub(uint32_t pg)               /* how many pages a page button has */
{
    return pg == HB_BTN_FX ? 2u : pg == HB_BTN_LFO ? 4u : pg == HB_BTN_ARP ? 3u : pg == HB_BTN_SEQ ? 2u : 1u;
}

static void leds(void)
{
    uint32_t d;
    for (d = 0; d < ND; d++)
        H->led(HB_LED_KEY(DRONE_KEY[d]), dr[d].gate ? 1u : (d == sel ? 2u : 0u));
    for (d = 0; d < 14u; d++)
        if (PAGE_BTNS & (1u << d)) H->led(d, page == d ? 1u : 2u);   /* open page lit, the others glow */
    H->led(HB_BTN_PLAY, sq_run ? 1u : 2u);
    H->led(HB_BTN_REC, rec ? 1u : 2u);
}

static void toggle(uint32_t d)
{
    if (d >= ND) return;
    dr[d].gate = !dr[d].gate;
    sel = d;
    dirty = 1;
    leds();
}

static char *put_s(char *p, const char *s);
static char *put_u(char *p, uint32_t v);
static void say(const char *s) { msg = s; msg_until = H->ms() + 1500u; dirty = 1; }

static void rec_step(int32_t note)              /* note < 0: a rest */
{
    if (note < 0) sq_rest = (uint8_t)(sq_rest | (1u << rec_n));
    else { sq_note[rec_n] = (uint8_t)note; sq_rest = (uint8_t)(sq_rest & ~(1u << rec_n)); }
    if (++rec_n >= NS) { rec = 0; sq_len = (int32_t)NS; say("5 STEPS"); leds(); }
    dirty = 1;
}

static float expstep(float x, int32_t c, float per2, float lo, float hi)   /* per2 clicks: x2 */
{
    return clampf(x * exp2f_((float)c / per2), lo, hi);
}

static void knob(uint32_t k, int32_t c)          /* k 0..3, c clicks */
{
    drone_t *v = &dr[sel];
    float f = (float)c;
    if (page == HB_BTN_EDIT) {
        v->o[k].level = clampf(v->o[k].level + f * 0.02f, 0.0f, 1.0f);
    } else if (page == HB_BTN_FX && lsub == 0) {            /* the reverb */
        if (k == 0) rv_decay = expstep(rv_decay, c, 8.0f, 0.5f, 60.0f);
        else if (k == 1) rv_size = clampf(rv_size + f * 0.02f, 0.5f, 1.0f);
        else if (k == 2) rv_tone = clampf(rv_tone + f * 0.02f, 0.0f, 1.0f);
        else rv_mix = clampf(rv_mix + f * 0.02f, 0.0f, 1.0f);
    } else if (page == HB_BTN_FX) {                         /* the echo */
        if (k == 0) dl_time = expstep(dl_time, c, 12.0f, 0.02f, 1.0f);
        else if (k == 1) dl_feed = clampf(dl_feed + f * 0.02f, 0.0f, 0.95f);
        else if (k == 2) dl_tone = clampf(dl_tone + f * 0.02f, 0.0f, 1.0f);
        else dl_mix = clampf(dl_mix + f * 0.02f, 0.0f, 1.0f);
    } else if (page == HB_BTN_GLO) {                        /* the filters */
        filt_t *x = &fl[k >> 1];
        if (k & 1u) x->res = clampf(x->res + f * 0.01f, 0.0f, 1.0f);
        else x->cutoff = expstep(x->cutoff, c, 24.0f, 30.0f, 16000.0f);   /* quarter tones */
    } else if (page == HB_BTN_LFO && lsub == 0) {           /* the oscillators' LFO speeds */
        v->o[k].lrate = expstep(v->o[k].lrate, c, 6.0f, 0.01f, 5.0f);
    } else if (page == HB_BTN_LFO && lsub == 1) {           /* their depths */
        v->o[k].ldepth = clampf(v->o[k].ldepth + f * 0.02f, 0.0f, 1.0f);
    } else if (page == HB_BTN_LFO && lsub == 2) {
        if (k == 0) lfo_rate = clampf(lfo_rate + f * 0.01f, 0.0f, 2.0f);
        else if (k == 1) lfo_depth = clampf(lfo_depth + f * 0.1f, 0.0f, 4.0f);
        else if (k == 2) drift_amt = clampf(drift_amt + f * 0.01f, 0.0f, 1.0f);
        else drive = clampf(drive + f * 0.1f, 0.5f, 8.0f);
    } else if (page == HB_BTN_LFO) {                        /* noise + sample & hold */
        if (k == 0) {
            float r = sh_rate;
            if (r <= 0.0f) r = c > 0 ? 0.05f : 0.0f;
            else { r = r * exp2f_(f / 6.0f); if (r < 0.05f) r = 0.0f; }
            sh_rate = clampf(r, 0.0f, 20.0f);
        } else if (k == 1) sh_cut = clampf(sh_cut + f * 0.1f, 0.0f, 4.0f);
        else if (k == 2) sh_pitch = clampf(sh_pitch + f, 0.0f, 24.0f);
        else noise_lv = clampf(noise_lv + f * 0.02f, 0.0f, 1.0f);
    } else if (page == HB_BTN_ENV) {                        /* the selected drone */
        if (k == 0) v->att_s = clampf(v->att_s + f * 0.1f, 0.1f, 25.0f);
        else if (k == 1) v->rel_s = clampf(v->rel_s + f * 0.1f, 0.1f, 25.0f);
        else if (k == 2) v->wave = clampf(v->wave + f * 0.05f, 0.0f, 1.0f);
        else v->vol = clampf(v->vol + f * 0.02f, 0.0f, 1.0f);
    } else if (page == HB_BTN_ARP && lsub == 0) {           /* the voices: sound */
        if (k == 0) v_wave = clampf(v_wave + f * 0.02f, 0.0f, 1.0f);
        else if (k == 1) v_beat = clampf(v_beat + f * 0.01f, 0.0f, 0.5f);
        else if (k == 2) v_cut = expstep(v_cut, c, 24.0f, 30.0f, 12000.0f);
        else v_res = clampf(v_res + f * 0.01f, 0.0f, 1.0f);
    } else if (page == HB_BTN_ARP && lsub == 1) {           /* their envelope */
        if (k == 0) v_att = expstep(v_att, c, 6.0f, 0.002f, 5.0f);
        else if (k == 1) v_dec = expstep(v_dec, c, 6.0f, 0.01f, 10.0f);
        else if (k == 2) v_sus = clampf(v_sus + f * 0.02f, 0.0f, 1.0f);
        else v_rel = expstep(v_rel, c, 6.0f, 0.01f, 10.0f);
    } else if (page == HB_BTN_ARP) {
        if (k == 0) v_env = clampf(v_env + f * 0.1f, 0.0f, 6.0f);
        else if (k == 1) {
            float g = v_glide;
            if (g <= 0.0f) g = c > 0 ? 0.01f : 0.0f;
            else { g = g * exp2f_(f / 6.0f); if (g < 0.01f) g = 0.0f; }
            v_glide = clampf(g, 0.0f, 2.0f);
        } else if (k == 2) { int32_t o = v_oct + c; v_oct = o < -3 ? -3 : o > 3 ? 3 : o; }
        else v_vol = clampf(v_vol + f * 0.02f, 0.0f, 1.0f);
    } else if (page == HB_BTN_SEQ && lsub == 0) {           /* the sequencer */
        if (k == 0) sq_bpm = clampf(sq_bpm + f, 20.0f, 300.0f);
        else if (k == 1) sq_gate = clampf(sq_gate + f * 0.02f, 0.06f, 1.0f);
        else if (k == 2) { int32_t l = sq_len + c; sq_len = l < 1 ? 1 : l > (int32_t)NS ? (int32_t)NS : l; }
        else { int32_t t = sq_key + c; sq_key = t < -12 ? -12 : t > 12 ? 12 : t; }
    } else if (page == HB_BTN_SEQ) {                        /* one step */
        if (SEL_HELD) {
            if (c < 0) sq_rest = (uint8_t)(sq_rest | (1u << k));
            else if (c > 0) sq_rest = (uint8_t)(sq_rest & ~(1u << k));
        } else {
            int32_t nn = (int32_t)sq_note[k] + c;
            sq_note[k] = (uint8_t)(nn < 24 ? 24 : nn > 96 ? 96 : nn);
            sq_rest = (uint8_t)(sq_rest & ~(1u << k));
        }
    } else {
        float st = SEL_HELD ? 1.0f : 0.05f;
        v->o[k].tune = clampf(v->o[k].tune + f * st, -24.0f, 24.0f);
    }
    dirty = 1;
}

static void ui(uint32_t ev, int32_t arg)
{
    uint32_t id = HB_EV_ID(ev), d;
    drone_t *v = &dr[sel];
    switch (HB_EV_KIND(ev)) {
    case HB_EV_KEY_DOWN:
        for (d = 0; d < ND; d++)
            if (DRONE_KEY[d] == id) { toggle(d); return; }
        if (id >= 12u && id <= 26u) {                       /* F4..G5 */
            int32_t nn = 65 + (int32_t)(id - 12u);
            if (SEL_HELD && !rec) { v->root = (float)(nn - 24); dirty = 1; break; }   /* the drone's root */
            nn += 12 * v_oct;
            if (nn < 12) nn = 12;
            if (nn > 120) nn = 120;
            if (key_held[id]) vpush(0, key_held[id]);
            key_held[id] = (uint8_t)nn;
            vpush(1, (uint32_t)nn);
            if (rec) rec_step(nn - 12 * v_oct);             /* steps are kept without the octave */
        }
        break;
    case HB_EV_KEY_UP:
        if (id <= 26u && key_held[id]) { vpush(0, key_held[id]); key_held[id] = 0; }
        break;
    case HB_EV_BTN_DOWN:
        if (id < 32u) held |= 1u << id;
        if (id == HB_BTN_HOME) home_ms = H->ms();
        if (id == HB_BTN_OCT_DOWN) { v->root = clampf(v->root - 12.0f, 12.0f, 96.0f); dirty = 1; }
        else if (id == HB_BTN_OCT_UP) { v->root = clampf(v->root + 12.0f, 12.0f, 96.0f); dirty = 1; }
        else if (id == HB_BTN_SEL && rec) rec_step(-1);
        else if (id == HB_BTN_PLAY && SEL_HELD) {           /* all drones off / back on */
            uint32_t on = 0;
            for (d = 0; d < ND; d++) on |= (uint32_t)(dr[d].gate != 0) << d;
            if (on) { saved_mask = on; for (d = 0; d < ND; d++) dr[d].gate = 0; say("DRONES OFF"); }
            else { for (d = 0; d < ND; d++) dr[d].gate = (int32_t)((saved_mask >> d) & 1u); say("DRONES ON"); }
            leds();
        } else if (id == HB_BTN_PLAY) {
            sq_run = !sq_run;
            say(sq_run ? "SEQ PLAY" : "SEQ STOP");
            leds();
        } else if (id == HB_BTN_REC) {
            if (rec) { rec = 0; if (rec_n) sq_len = (int32_t)rec_n; say("REC DONE"); }
            else { rec = 1; rec_n = 0; page = HB_BTN_SEQ; lsub = 1; say("PLAY 5 KEYS"); }
            leds();
        } else if (id == HB_BTN_SAVE) { save_sound(); say("SAVED"); }
        else if (id < 14u && (PAGE_BTNS & (1u << id))) {   /* open the page, the next page, or close */
            if (page == id) { if (++lsub >= nsub(id)) { lsub = 0; page = 0; } }
            else { page = id; lsub = 0; }
            leds();
            dirty = 1;
        } else dirty = 1;
        break;
    case HB_EV_BTN_UP:
        if (id < 32u) held &= ~(1u << id);
        if (id == HB_BTN_HOME && H->ms() - home_ms < 1000u && page) {   /* a short HOME: back to the drone */
            page = 0;
            lsub = 0;
            leds();
        }
        dirty = 1;
        break;
    case HB_EV_ENC:
        if (id >= HB_ENC_KNOB1 && id <= HB_ENC_KNOB4) { knob(id - HB_ENC_KNOB1, arg); sweet_update(); }
        else if (id == HB_ENC_PRESETS && page == HB_BTN_SEQ && lsub == 1u) knob(4, arg);   /* step 5 */
        else if (id == HB_ENC_SELECT) v->root = clampf(v->root + (float)arg, 12.0f, 96.0f);
        else if (id == HB_ENC_ALGORITHM) {                  /* SWEET */
            static char sm[16];
            sweet_update();
            v->sweet = clampf(v->sweet + (float)arg * 0.02f, 0.0f, 1.0f);
            put_s(put_u(put_s(sm, "SWEET "), (uint32_t)roundi(v->sweet * 100.0f)), "%");
            say(sm);
        }
        else if (id == HB_ENC_PRESETS) v->vol = clampf(v->vol + (float)arg * 0.02f, 0.0f, 1.0f);
        dirty = 1;
        break;
    default:
        break;
    }
}

static void note(uint32_t ch, uint32_t key, uint32_t vel)
{
    (void)ch;
    if (key >= 48u && key < 48u + ND) { if (vel) toggle(key - 48u); }
    else if (key >= 12u && key <= 120u) vpush(vel != 0u, key);
}

static void cc(uint32_t ch, uint32_t num, uint32_t val)
{
    uint32_t d;
    (void)ch; (void)val;
    if (num == 123u) {
        for (d = 0; d < ND; d++) dr[d].gate = 0;
        v_panic = 1;
        sq_run = 0;
        leds();
        dirty = 1;
    }
}

/* ------------------------------------------------------------------ the screen (main loop)
 * A big list: a title bar in the page's colour, then 4 rows, one per knob: its number, its name
 * and its value in the big letters (32 px), and a long bar. Drawn in three strips (a canvas is
 * at most 96 rows): y 24..102 the title and row 1, 102..194 rows 2 and 3, 194..240 row 4. */

static char *put_s(char *p, const char *s) { while (*s) *p++ = *s++; *p = 0; return p; }
static char *put_u(char *p, uint32_t v)
{
    char t[10];
    uint32_t n = 0;
    do { t[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (n) *p++ = t[--n];
    *p = 0;
    return p;
}
static char *put_i(char *p, int32_t v)          /* with its sign: "+3", "-1", "0" */
{
    if (v > 0) *p++ = '+';
    if (v < 0) { *p++ = '-'; v = -v; }
    return put_u(p, (uint32_t)v);
}
static const char *const NOTE_NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
static char *put_note(char *p, int32_t n)
{
    if (n < 0) n = 0;
    p = put_s(p, NOTE_NAME[(uint32_t)n % 12u]);
    return put_u(p, (uint32_t)(n / 12 - 1));
}
static char *put_cents(char *p, float semis)     /* +7.02 st -> "+7.02" */
{
    int32_t c = roundi(semis * 100.0f), a = c < 0 ? -c : c;
    *p++ = c < 0 ? '-' : '+';
    p = put_u(p, (uint32_t)(a / 100));
    *p++ = '.';
    *p++ = (char)('0' + a / 10 % 10);
    *p++ = (char)('0' + a % 10);
    *p = 0;
    return p;
}
static char *put_fix(char *p, float v, uint32_t dec)   /* v with dec decimals (0..2) */
{
    uint32_t sc = dec == 2u ? 100u : dec == 1u ? 10u : 1u, a;
    int32_t r = roundi(v * (float)sc);
    if (r < 0) { *p++ = '-'; r = -r; }
    a = (uint32_t)r;
    p = put_u(p, a / sc);
    if (dec) {
        *p++ = '.';
        if (dec == 2u) *p++ = (char)('0' + a / 10u % 10u);
        *p++ = (char)('0' + a % 10u);
        *p = 0;
    }
    return p;
}
static int32_t slen(const char *s) { int32_t n = 0; while (s[n]) n++; return n; }

#define C_BLK  HB_RGB(0, 0, 0)
#define C_WHT  HB_RGB(255, 255, 255)
#define C_UNIT HB_RGB(190, 190, 190)
#define C_TRK  HB_RGB(55, 55, 55)
#define C_DIM  HB_RGB(90, 90, 90)
#define C_SUN  HB_RGB(255, 170, 40)
#define C_REC  HB_RGB(255, 50, 50)
#define FW_S 8                                  /* the fonts are fixed width: 8 and 16 px */
#define FW_L 16

static uint16_t page_col(uint32_t pg)
{
    switch (pg) {
    case HB_BTN_EDIT: return HB_RGB(255, 255, 255);
    case HB_BTN_FX:   return HB_RGB(0, 215, 255);
    case HB_BTN_GLO:  return HB_RGB(255, 80, 215);
    case HB_BTN_LFO:  return HB_RGB(90, 255, 90);
    case HB_BTN_ENV:  return HB_RGB(255, 120, 60);
    case HB_BTN_ARP:  return HB_RGB(175, 135, 255);
    case HB_BTN_SEQ:  return HB_RGB(255, 225, 0);
    default:          return C_SUN;
    }
}

static uint16_t drone_col(uint32_t d)          /* each drone's colour: a rainbow, 1 red .. 6 violet */
{
    static const uint16_t C[ND] = {HB_RGB(255, 70, 70), HB_RGB(255, 150, 30), HB_RGB(255, 230, 40),
                                   HB_RGB(60, 230, 90), HB_RGB(60, 170, 255), HB_RGB(200, 110, 255)};
    return C[d < ND ? d : 0];
}

typedef struct {
    const char *name, *unit;
    char val[12];
    float bar;                                  /* 0..1 */
    uint16_t ucol;                              /* the unit's colour (0: grey) */
} row_t;

static int32_t cy, ch;                          /* the strip being drawn: its top (screen y) and height */
static void R(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t c)   /* a rectangle, clipped to the strip */
{
    int32_t y0 = y < cy ? cy : y, y1 = y + h > cy + ch ? cy + ch : y + h;
    if (w > 0 && y1 > y0) H->cv_rect(x, y0 - cy, w, y1 - y0, c);
}
static void T(int32_t x, int32_t y, uint32_t big, const char *s, uint16_t c)   /* text, if it is in the strip */
{
    if (y >= cy && y + (big ? 32 : 16) <= cy + ch) H->cv_text(x, y - cy, big, s, c);
}

static float lg(float x, float lo, float oct) { return log2f_(x / lo) / oct; }   /* a bar on a log scale */

static void set_row(row_t *r, const char *name, float v, uint32_t dec, const char *unit, float bar)
{
    r->name = name;
    r->unit = unit;
    put_fix(r->val, v, dec);
    r->bar = bar;
    r->ucol = 0;
}

/* the open page's title, its colour and rows; returns the number of rows */
static uint32_t page_rows(row_t *r, const char **title)
{
    drone_t *v = &dr[sel];
    uint32_t o;
    static const char *const OSC[NO] = {"OSC1", "OSC2", "OSC3", "OSC4"};
    if (page == HB_BTN_EDIT) {
        *title = "LEVELS";
        for (o = 0; o < NO; o++) set_row(&r[o], OSC[o], v->o[o].level * 100.0f, 0, "%", v->o[o].level);
    } else if (page == HB_BTN_FX && lsub == 0) {
        *title = "REVERB";
        set_row(&r[0], "DECAY", rv_decay, rv_decay < 10.0f ? 1 : 0, "S", lg(rv_decay, 0.5f, 6.9069f));
        set_row(&r[1], "SIZE", rv_size * 100.0f, 0, "%", (rv_size - 0.5f) * 2.0f);
        set_row(&r[2], "TONE", rv_tone * 100.0f, 0, "%", rv_tone);
        set_row(&r[3], "MIX", rv_mix * 100.0f, 0, "%", rv_mix);
    } else if (page == HB_BTN_FX) {
        *title = "ECHO";
        set_row(&r[0], "TIME", dl_time * 1000.0f, 0, "MS", lg(dl_time, 0.02f, 5.6439f));
        set_row(&r[1], "FEED", dl_feed * 100.0f, 0, "%", dl_feed / 0.95f);
        set_row(&r[2], "TONE", dl_tone * 100.0f, 0, "%", dl_tone);
        set_row(&r[3], "MIX", dl_mix * 100.0f, 0, "%", dl_mix);
    } else if (page == HB_BTN_GLO) {
        *title = "FILTERS";
        set_row(&r[0], "CUT A", fl[0].cutoff, 0, "HZ", lg(fl[0].cutoff, 30.0f, 9.0589f));
        set_row(&r[1], "RES A", fl[0].res * 100.0f, 0, "%", fl[0].res);
        set_row(&r[2], "CUT B", fl[1].cutoff, 0, "HZ", lg(fl[1].cutoff, 30.0f, 9.0589f));
        set_row(&r[3], "RES B", fl[1].res * 100.0f, 0, "%", fl[1].res);
    } else if (page == HB_BTN_LFO && lsub < 2u) {
        *title = lsub ? "LFO DEPTH" : "LFO SPEED";
        for (o = 0; o < NO; o++) {
            if (lsub == 0) set_row(&r[o], OSC[o], v->o[o].lrate, 2, "HZ", lg(v->o[o].lrate, 0.01f, 8.9658f));
            else set_row(&r[o], OSC[o], v->o[o].ldepth * 100.0f, 0, "%", v->o[o].ldepth);
        }
    } else if (page == HB_BTN_LFO && lsub == 2u) {
        *title = "FILTER LFO";
        set_row(&r[0], "RATE", lfo_rate, 2, "HZ", lfo_rate * 0.5f);
        set_row(&r[1], "DEPTH", lfo_depth, 1, "OCT", lfo_depth * 0.25f);
        set_row(&r[2], "DRIFT", drift_amt * 100.0f, 0, "CT", drift_amt);
        set_row(&r[3], "DRIVE", drive, 1, "", (drive - 0.5f) * (1.0f / 7.5f));
    } else if (page == HB_BTN_LFO) {
        *title = "NOISE S+H";
        if (sh_rate <= 0.0f) { r[0].name = "RATE"; r[0].unit = ""; put_s(r[0].val, "SEQ"); r[0].bar = 0.0f; }
        else set_row(&r[0], "RATE", sh_rate, sh_rate < 10.0f ? 2 : 1, "HZ", lg(sh_rate, 0.05f, 8.6439f));
        set_row(&r[1], "CUT", sh_cut, 1, "OCT", sh_cut * 0.25f);
        set_row(&r[2], "PITCH", sh_pitch, 0, "ST", sh_pitch * (1.0f / 24.0f));
        set_row(&r[3], "NOISE", noise_lv * 100.0f, 0, "%", noise_lv);
    } else if (page == HB_BTN_ENV) {
        static char t[12];
        put_u(put_s(t, "DRONE "), sel + 1u);
        *title = t;
        set_row(&r[0], "SWELL", v->att_s, 1, "S", lg(v->att_s, 0.1f, 7.9658f));
        set_row(&r[1], "FADE", v->rel_s, 1, "S", lg(v->rel_s, 0.1f, 7.9658f));
        set_row(&r[2], "WAVE", v->wave * 100.0f, 0, "%", v->wave);
        set_row(&r[3], "VOL", v->vol * 100.0f, 0, "%", v->vol);
    } else if (page == HB_BTN_ARP && lsub == 0) {
        *title = "VOICE";
        set_row(&r[0], "WAVE", v_wave * 100.0f, 0, "%", v_wave);
        set_row(&r[1], "BEAT", v_beat * 100.0f, 0, "CT", v_beat * 2.0f);
        set_row(&r[2], "CUT", v_cut, 0, "HZ", lg(v_cut, 30.0f, 8.6439f));
        set_row(&r[3], "RES", v_res * 100.0f, 0, "%", v_res);
    } else if (page == HB_BTN_ARP && lsub == 1) {
        *title = "VOICE ENV";
        set_row(&r[0], "ATT", v_att * 1000.0f, 0, "MS", lg(v_att, 0.002f, 11.2877f));
        set_row(&r[1], "DEC", v_dec * 1000.0f, 0, "MS", lg(v_dec, 0.01f, 9.9658f));
        set_row(&r[2], "SUS", v_sus * 100.0f, 0, "%", v_sus);
        set_row(&r[3], "REL", v_rel * 1000.0f, 0, "MS", lg(v_rel, 0.01f, 9.9658f));
        for (o = 0; o < NO; o += 3u) {                      /* long times in seconds */
            float s = o ? v_rel : v_att;
            if (s >= 1.0f) { put_fix(r[o].val, s, 1); r[o].unit = "S"; }
        }
        if (v_dec >= 1.0f) { put_fix(r[1].val, v_dec, 1); r[1].unit = "S"; }
    } else if (page == HB_BTN_ARP) {
        *title = "VOICE PLAY";
        set_row(&r[0], "ENV", v_env, 1, "OCT", v_env * (1.0f / 6.0f));
        if (v_glide <= 0.0f) { r[1].name = "GLIDE"; r[1].unit = ""; put_s(r[1].val, "OFF"); r[1].bar = 0.0f; }
        else set_row(&r[1], "GLIDE", v_glide * 1000.0f, 0, "MS", lg(v_glide, 0.01f, 7.6439f));
        r[2].name = "OCT"; r[2].unit = ""; put_i(r[2].val, v_oct); r[2].bar = (float)(v_oct + 3) * (1.0f / 6.0f);
        set_row(&r[3], "VOL", v_vol * 100.0f, 0, "%", v_vol);
    } else {
        *title = "SEQUENCER";
        set_row(&r[0], "RATE", sq_bpm, 0, "BPM", (sq_bpm - 20.0f) * (1.0f / 280.0f));
        set_row(&r[1], "GATE", sq_gate * 100.0f, 0, "%", sq_gate);
        set_row(&r[2], "STEPS", (float)sq_len, 0, "", (float)sq_len * (1.0f / (float)NS));
        r[3].name = "KEY"; r[3].unit = "ST"; put_i(r[3].val, sq_key); r[3].bar = (float)(sq_key + 12) * (1.0f / 24.0f);
    }
    return 4;
}

static void draw_row(uint32_t k, const row_t *r, uint16_t col)
{
    int32_t y = 56 + 46 * (int32_t)k, uw = slen(r->unit) * FW_S, vw = slen(r->val) * FW_L;
    char num[2];
    int32_t w = roundi(clampf(r->bar, 0.0f, 1.0f) * 232.0f);
    num[0] = (char)('1' + k); num[1] = 0;
    T(4, y, 1, num, col);
    T(26, y, 1, r->name, C_WHT);
    T(236 - uw - (uw ? 4 : 0) - vw, y, 1, r->val, C_WHT);
    if (uw) T(236 - uw, y + 14, 0, r->unit, r->ucol ? r->ucol : C_UNIT);
    R(4, y + 35, 232, 8, C_TRK);
    R(4, y + 35, w, 8, col);
}

static void draw_title(const char *t, const char *sub, uint16_t col)
{
    R(0, 24, 240, 32, col);
    T(6, 24, 1, t, C_BLK);
    if (sub && *sub) T(236 - slen(sub) * FW_L, 24, 1, sub, C_BLK);
}

static void draw_home(void)                     /* the selected drone: its oscillators, and the six drones */
{
    drone_t *v = &dr[sel];
    char b[16], *p;
    uint32_t d, o;
    uint16_t dc = drone_col(sel);
    if (*msg) p = put_s(b, msg);
    else if (SEL_HELD) { p = put_s(b, "CPU "); p = put_u(p, load_pct); put_s(p, "%"); }
    else { p = put_s(b, "D"); p = put_u(p, sel + 1u); p = put_s(p, " "); put_note(p, roundi(v->root)); }
    T(4, 24, 1, b, dc);
    if (v->sweet > 0.0f) R(4, 53, roundi(v->sweet * 116.0f), 3, C_WHT);   /* how sweet: a line under the name */
    if (!*msg)
        for (d = 0; d < ND; d++) {                          /* the six drones, each in its colour */
            int32_t x = 128 + 18 * (int32_t)d, e = roundi(dr[d].env * 20.0f);
            uint16_t c = drone_col(d);
            R(x, 27, 16, 28, d == sel ? C_WHT : C_BLK);
            R(x + 2, 29, 12, 24, c);
            R(x + 4, 31, 8, 20, C_BLK);
            if (e > 0) R(x + 4, 51 - e, 8, e, dr[d].gate ? c : C_DIM);
        }
    for (o = 0; o < NO; o++) {
        osc_t *s = &v->o[o];
        float t = s->tune + v->sweet * (s->sweet_t - s->tune), off = t - s->sweet_t;   /* what sounds */
        row_t r;
        static char u[NO][8];
        r.name = "OSC";
        r.unit = "ST";
        r.ucol = 0;
        if (v->sweet > 0.0f) {                              /* the sweet spot's ratio; white when it is on it */
            put_s(u[o], JI_NAME[s->ji]);
            if (s->beat) put_s(u[o] + slen(u[o]), "~");
            r.unit = u[o];
            r.ucol = off < 0.03f && off > -0.03f ? C_WHT : C_DIM;
        }
        put_cents(r.val, t);
        r.bar = s->level * s->lgain;
        draw_row(o, &r, dc);
    }
}

static void draw_steps(uint16_t col)            /* the 5 steps, as columns */
{
    static const char *const KN[NS] = {"K1", "K2", "K3", "K4", "PRE"};
    char b[24], *p;
    uint32_t i;
    for (i = 0; i < NS; i++) {
        int32_t x = 48 * (int32_t)i, nn = sq_note[i], h;
        uint32_t on = i < (uint32_t)sq_len, play = sq_run && i == sq_pos, rest = (sq_rest >> i) & 1u;
        uint16_t tc = play ? C_BLK : !on ? C_DIM : C_WHT, bc = play ? C_BLK : on ? col : C_DIM;
        if (play) R(x + 1, 58, 46, 136, col);
        if (rec && i == rec_n) R(x + 1, 58, 46, 4, C_REC);
        p = put_s(b, rest ? "--" : NOTE_NAME[(uint32_t)nn % 12u]);
        T(x + (48 - (int32_t)(p - b) * FW_L) / 2, 62, 1, b, rest && !play ? C_DIM : tc);
        if (!rest) {
            put_u(b, (uint32_t)(nn / 12 - 1));
            T(x + 16, 106, 1, b, tc);
        }
        R(x + 14, 144, 20, 46, play ? HB_RGB(90, 80, 0) : C_TRK);
        h = rest ? 0 : roundi((float)(nn - 24) * (46.0f / 72.0f));
        if (h < 2 && !rest) h = 2;
        R(x + 14, 190 - h, 20, h, bc);
        T(x + (48 - slen(KN[i]) * FW_S) / 2, 200, 0, KN[i], C_UNIT);
    }
    if (rec) { p = put_s(b, "REC: PLAY STEP "); put_u(p, rec_n + 1u); }
    else put_s(b, "SEL+TURN: REST");
    T((240 - slen(b) * FW_S) / 2, 220, 0, b, rec ? C_REC : C_UNIT);
}

static void draw(uint32_t full)
{
    static uint32_t last_ms, last_load, last_pos, last_sw;
    static const int32_t SY[3] = {24, 102, 194}, SH[3] = {78, 92, 46};
    row_t rows[4];
    const char *title = "";
    char sub[8], *p;
    uint32_t s, k, n = 0, home = page == 0u, steps = page == HB_BTN_SEQ && lsub == 1u;
    uint16_t col = page_col(page);
    if (full) dirty = 1;
    if (msg_until && H->ms() > msg_until) { msg_until = 0; msg = ""; dirty = 1; }
    if (home && SEL_HELD && H->ms() - last_ms > 500u && load_pct != last_load) { last_load = load_pct; dirty = 1; }
    if (sq_pos != last_pos) { last_pos = sq_pos; if (steps) dirty = 1; }
    if (home && H->ms() - last_ms > 100u) {
        for (k = 0; k < NO; k++) if (dr[sel].o[k].ldepth > 0.0f) dirty = 1;   /* the level bars breathe */
        for (k = 0; k < ND; k++) if (dr[k].env > 0.0f && dr[k].env < 1.0f) dirty = 1;   /* swelling, fading */
    }
    if (H->ms() - last_sw > 100u) { last_sw = H->ms(); sweet_update(); if (home && dr[sel].sweet > 0.0f) dirty = 1; }
    if (!dirty || H->ms() - last_ms < 30u) return;
    dirty = 0;
    last_ms = H->ms();

    if (!home && !steps) n = page_rows(rows, &title);
    if (steps) title = rec ? "RECORD" : "STEPS";
    sub[0] = 0;
    if (nsub(page) > 1u) { p = put_u(sub, lsub + 1u); p = put_s(p, "/"); put_u(p, nsub(page)); }
    if (*msg && !home) { title = msg; sub[0] = 0; }

    for (s = 0; s < 3u; s++) {
        cy = SY[s]; ch = SH[s];
        H->cv_begin(240, ch, C_BLK);
        if (home) draw_home();
        else {
            draw_title(title, sub, rec && steps && !*msg ? C_REC : col);
            if (steps) draw_steps(col);
            else for (k = 0; k < n; k++)                    /* the selected drone's pages: in its colour */
                draw_row(k, &rows[k], page == HB_BTN_EDIT || page == HB_BTN_ENV || (page == HB_BTN_LFO && lsub < 2u)
                                      ? drone_col(sel) : col);
        }
        H->cv_blit(0, cy);
    }
}

/* ------------------------------------------------------------------ the descriptor */

static int init(const hb_host_t *h)
{
    uint32_t d, o;
    H = h;
    rng ^= H->rand() | 1u;
    defaults();
    load_sound();
    for (d = 0; d < ND; d++) {
        dr[d].gate = 0;
        dr[d].env = 0.0f;
        for (o = 0; o < NO; o++) {
            dr[d].o[o].ph = H->rand();
            dr[d].o[o].lph = (float)(H->rand() >> 8) * (1.0f / 16777216.0f);
            dr[d].o[o].lgain = 1.0f;
        }
    }
    for (d = 0; d < NV; d++) {
        vo[d].stage = 0; vo[d].key = -1; vo[d].env = 0.0f; vo[d].age = 0;
        vo[d].f.ic1 = vo[d].f.ic2 = 0.0f;
        vo[d].ph[0] = H->rand(); vo[d].ph[1] = H->rand();
    }
    for (d = 0; d < DL_LEN; d++) dl_buf[d] = 0;
    dl_w = 0; dl_half = 0; dl_t = 0.0f; dl_lp = 0.0f; dl_acc = 0.0f;
    for (d = 0; d < 4u; d++) dl_o[d] = 0.0f;
    for (d = 0; d < 27u; d++) key_held[d] = 0;
    vq_r = vq_w = 0; v_panic = 0;
    sq_run = 0; sq_was = 0; sq_on = 0; sq_pos = 0; sh_val = 0.0f; sh_ph = 0.0f;
    rec = 0; rec_n = 0;
    fl[0].ic1 = fl[0].ic2 = fl[1].ic1 = fl[1].ic2 = 0.0f;
    rv_init();
    sweet_update();
    sel = 0; held = 0; page = 0; lsub = 0; dirty = 1; msg = ""; msg_until = 0;
    leds();
    H->log("helio: ready");
    return 0;
}

static void release(void)
{
    uint32_t d;
    for (d = 0; d < ND; d++) { dr[d].gate = 0; dr[d].env = 0.0f; }
    for (d = 0; d < NV; d++) { vo[d].stage = 0; vo[d].env = 0.0f; }
    sq_run = 0;
    rec = 0;
    held = 0;
    page = 0;
    lsub = 0;
}

HB_DESC = {HB_ABI, "HELIO", init, release, note, cc, render, ui, draw, 0, 0, HB_F_PANEL, 0, 0};
