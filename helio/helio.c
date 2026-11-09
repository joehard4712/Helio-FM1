/* SPDX-License-Identifier: GPL-3.0-only */
/* HELIO: a drone synth for fm1-dock, inspired by the Elta Music Solar 42 (not affiliated).
 *
 *   6 drone voices x 4 oscillators (one oscillator per knob), free microtonal tuning,
 *   slow random "analog" drift, sine->saw morph, slow swell/fade.
 *   Drones 1-3 -> filter A (left-ish), drones 4-6 -> filter B (right-ish): resonant 2-pole
 *   low-pass with saturation, swept in opposite directions by one slow LFO.
 *
 * Panel (HB_F_PANEL: all keys are the module's; HOME held 1.5 s alone opens the module list)
 *   white keys F3 G3 A3 B3 C4 D4   latch drone 1..6 on/off, and select it for editing
 *   keys F4 .. G5                  set the selected drone's root note (two octaves lower)
 *   OCT- / OCT+                    selected drone's root -/+ one octave
 *   SELECT                         selected drone's root -/+ one semitone
 *   ALGORITHM                      selected drone's waveform (sine .. saw)
 *   PRESETS                        selected drone's volume
 *   KNOB1..4                       tune oscillator 1..4 (5 cents a click; SEL held: 1 semitone)
 *   EDIT held + KNOB1..4           level of oscillator 1..4
 *   FX held   + KNOB1..4           filter A cutoff, A resonance, B cutoff, B resonance
 *   LFO held  + KNOB1..4           LFO rate, LFO depth, drift amount, filter drive
 *   ENV held  + KNOB1..2           selected drone's swell (attack) and fade (release) time
 *   PLAY                           all drones off / back on
 *   SAVE                           keep the sound (it is loaded at the next start)
 * MIDI: notes 48..53 (C3..F3) toggle drones 1..6; CC 123 all off.
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
    float drift, drift_target;      /* audio side only */
    uint32_t ph;
} osc_t;

typedef struct {
    osc_t o[NO];
    volatile float root;            /* MIDI note */
    volatile float wave;            /* 0 sine .. 1 saw */
    volatile float vol;             /* 0..1 */
    volatile float att_s, rel_s;    /* seconds */
    volatile int32_t gate;          /* 1 on (latched) */
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

static void render(int32_t *out, uint32_t n)
{
    static float bus[2][256];
    uint32_t t0 = H->ticks(), d, o, i, j;
    float da = drift_amt, drv = drive;
    if (n > 256u) n = 256u;

    for (i = 0; i < n; i++) bus[0][i] = bus[1][i] = 0.0f;

    for (d = 0; d < ND; d++) {
        drone_t *v = &dr[d];
        float target = v->gate ? 1.0f : 0.0f;
        float step = 1.0f / (clampf(v->gate ? v->att_s : v->rel_s, 0.05f, 60.0f) * SR);
        float env = v->env, w = v->wave, g = v->vol * 0.25f;
        float *b = bus[d < 3u ? 0 : 1];
        uint32_t inc[NO];
        float lev[NO], dts[NO], idt[NO];
        if (env <= 0.0f && !v->gate) continue;              /* silent: costs nothing */

        for (o = 0; o < NO; o++) {                          /* pitch once a block */
            osc_t *s = &v->o[o];
            float hz;
            if (rand01() < 0.02f) s->drift_target = (rand01() * 2.0f - 1.0f) * da;
            s->drift += (s->drift_target - s->drift) * 0.003f;
            hz = 440.0f * exp2f_((v->root + s->tune + s->drift - 69.0f) * (1.0f / 12.0f));
            hz = clampf(hz, 1.0f, 8000.0f);
            inc[o] = (uint32_t)(int32_t)(hz * (4294967296.0f / SR));
            lev[o] = s->level;
            dts[o] = (float)(int32_t)(inc[o] >> 8) * (1.0f / 16777216.0f);
            idt[o] = 1.0f / dts[o];
        }
        for (i = 0; i < n; i++) {
            float x = 0.0f;
            if (env < target) { env += step; if (env > target) env = target; }
            else if (env > target) { env -= step; if (env < target) env = target; }
            for (o = 0; o < NO; o++) {
                uint32_t ph = v->o[o].ph;
                float dt = dts[o];
                float t = (float)(int32_t)((ph + 0x80000000u) >> 8) * (1.0f / 16777216.0f);
                float si = sine_ph(ph), saw = 2.0f * t - 1.0f;
                if (t < dt) { float u = t * idt[o]; saw -= u + u - u * u - 1.0f; }             /* polyBLEP */
                else if (t > 1.0f - dt) { float u = (t - 1.0f) * idt[o]; saw -= u * u + u + u + 1.0f; }
                x += lev[o] * (si + w * (saw - si));
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
            float l = soft_clip((0.7f * a + 0.3f * c) * 1.1f);
            float r = soft_clip((0.3f * a + 0.7f * c) * 1.1f);
            int32_t L = out[2u * j] + (int32_t)(l * 30000.0f);
            int32_t R = out[2u * j + 1u] + (int32_t)(r * 30000.0f);
            out[2u * j] = L > 32767 ? 32767 : L < -32767 ? -32767 : L;
            out[2u * j + 1u] = R > 32767 ? 32767 : R < -32767 ? -32767 : R;
        }
    }
    load_pct = (H->ticks() - t0) * 100u / (5805u * 24u);
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
} saved_t;
#define MAGIC 0x48454C31u           /* "HEL1" */

static void defaults(void)
{
    static const uint8_t ROOT[ND] = {36, 43, 48, 31, 55, 60};
    static const float TUNE[NO] = {0.0f, 0.07f, 7.02f, 11.9f};
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
        }
    }
    fl[0].cutoff = 700.0f; fl[1].cutoff = 1200.0f;
    fl[0].res = fl[1].res = 0.6f;
    lfo_rate = 0.05f; lfo_depth = 1.5f; drift_amt = 0.08f; drive = 1.8f;
}

static int32_t roundi(float x) { return (int32_t)(x + (x < 0.0f ? -0.5f : 0.5f)); }

static void save_sound(void)
{
    saved_t s;
    uint32_t d, o;
    s.magic = MAGIC;
    for (d = 0; d < ND; d++) {
        for (o = 0; o < NO; o++) {
            s.cents[d][o] = (int16_t)roundi(dr[d].o[o].tune * 100.0f);
            s.level[d][o] = (uint8_t)roundi(dr[d].o[o].level * 100.0f);
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
    H->save(&s, sizeof s);
}

static void load_sound(void)
{
    saved_t s;
    uint32_t d, o;
    if (H->load(&s, sizeof s) != 0 || s.magic != MAGIC) return;   /* keep the defaults */
    for (d = 0; d < ND; d++) {
        for (o = 0; o < NO; o++) {
            dr[d].o[o].tune = (float)s.cents[d][o] * 0.01f;
            dr[d].o[o].level = (float)s.level[d][o] * 0.01f;
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
}

/* ------------------------------------------------------------------ the panel (main loop) */

static const uint8_t DRONE_KEY[ND] = {0, 2, 4, 6, 7, 9};   /* F3 G3 A3 B3 C4 D4 */
static uint32_t sel, held, dirty = 1, saved_mask, msg_until;
static const char *msg = "";

static void leds(void)
{
    uint32_t d;
    for (d = 0; d < ND; d++)
        H->led(HB_LED_KEY(DRONE_KEY[d]), dr[d].gate ? 1u : (d == sel ? 2u : 0u));
}

static void toggle(uint32_t d)
{
    if (d >= ND) return;
    dr[d].gate = !dr[d].gate;
    sel = d;
    dirty = 1;
    leds();
}

static void say(const char *s) { msg = s; msg_until = H->ms() + 1500u; dirty = 1; }

static void knob(uint32_t k, int32_t c)          /* k 0..3, c clicks */
{
    drone_t *v = &dr[sel];
    float f = (float)c;
    if (held & (1u << HB_BTN_EDIT)) {
        v->o[k].level = clampf(v->o[k].level + f * 0.02f, 0.0f, 1.0f);
    } else if (held & (1u << HB_BTN_FX)) {
        filt_t *x = &fl[k >> 1];
        if (k & 1u) x->res = clampf(x->res + f * 0.01f, 0.0f, 1.0f);
        else x->cutoff = clampf(x->cutoff * exp2f_(f * (1.0f / 24.0f)), 30.0f, 16000.0f);   /* quarter tones */
    } else if (held & (1u << HB_BTN_LFO)) {
        if (k == 0) lfo_rate = clampf(lfo_rate + f * 0.01f, 0.0f, 2.0f);
        else if (k == 1) lfo_depth = clampf(lfo_depth + f * 0.1f, 0.0f, 4.0f);
        else if (k == 2) drift_amt = clampf(drift_amt + f * 0.01f, 0.0f, 1.0f);
        else drive = clampf(drive + f * 0.1f, 0.5f, 8.0f);
    } else if (held & (1u << HB_BTN_ENV)) {
        if (k == 0) v->att_s = clampf(v->att_s + f * 0.1f, 0.1f, 25.0f);
        else if (k == 1) v->rel_s = clampf(v->rel_s + f * 0.1f, 0.1f, 25.0f);
    } else {
        float st = (held & (1u << HB_BTN_SEL)) ? 1.0f : 0.05f;
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
        if (id >= 12u && id <= 26u) {                       /* F4..G5: the selected drone's root */
            v->root = (float)(65u + (id - 12u) - 24u);
            dirty = 1;
        }
        break;
    case HB_EV_BTN_DOWN:
        if (id < 32u) held |= 1u << id;
        if (id == HB_BTN_OCT_DOWN) { v->root = clampf(v->root - 12.0f, 12.0f, 96.0f); dirty = 1; }
        else if (id == HB_BTN_OCT_UP) { v->root = clampf(v->root + 12.0f, 12.0f, 96.0f); dirty = 1; }
        else if (id == HB_BTN_PLAY) {
            uint32_t on = 0;
            for (d = 0; d < ND; d++) on |= (uint32_t)(dr[d].gate != 0) << d;
            if (on) { saved_mask = on; for (d = 0; d < ND; d++) dr[d].gate = 0; say("ALL OFF"); }
            else { for (d = 0; d < ND; d++) dr[d].gate = (int32_t)((saved_mask >> d) & 1u); say("BACK ON"); }
            leds();
        } else if (id == HB_BTN_SAVE) { save_sound(); say("SAVED"); }
        else dirty = 1;                                     /* a held mode: show its labels */
        break;
    case HB_EV_BTN_UP:
        if (id < 32u) held &= ~(1u << id);
        dirty = 1;
        break;
    case HB_EV_ENC:
        if (id >= HB_ENC_KNOB1 && id <= HB_ENC_KNOB4) knob(id - HB_ENC_KNOB1, arg);
        else if (id == HB_ENC_SELECT) v->root = clampf(v->root + (float)arg, 12.0f, 96.0f);
        else if (id == HB_ENC_ALGORITHM) v->wave = clampf(v->wave + (float)arg * 0.05f, 0.0f, 1.0f);
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
    if (vel && key >= 48u && key < 48u + ND) toggle(key - 48u);
}

static void cc(uint32_t ch, uint32_t num, uint32_t val)
{
    uint32_t d;
    (void)ch; (void)val;
    if (num == 123u) { for (d = 0; d < ND; d++) dr[d].gate = 0; leds(); dirty = 1; }
}

/* ------------------------------------------------------------------ the screen (main loop) */

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
static char *put_note(char *p, int32_t n)
{
    static const char *const NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    p = put_s(p, NAME[(uint32_t)n % 12u]);
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

#define C_DIM HB_RGB(70, 70, 80)
#define C_SUN HB_RGB(255, 170, 40)
#define C_TXT HB_RGB(220, 220, 220)
#define C_HINT HB_RGB(140, 120, 255)

static void draw(uint32_t full)
{
    static uint32_t last_ms, last_load;
    char b[48], *p;
    uint32_t d, o;
    drone_t *v = &dr[sel];
    if (full) dirty = 1;
    if (msg_until && H->ms() > msg_until) { msg_until = 0; msg = ""; dirty = 1; }
    if (H->ms() - last_ms > 500u && load_pct != last_load) { last_load = load_pct; dirty = 1; }
    if (!dirty || H->ms() - last_ms < 30u) return;
    dirty = 0;
    last_ms = H->ms();

    /* the six drones and the selected one's header: y 24..120 */
    H->cv_begin(240, 96, 0);
    for (d = 0; d < ND; d++) {
        int32_t x = 6 + (int32_t)d * 39, y = 6;
        uint16_t c = dr[d].gate ? C_SUN : C_DIM;
        int32_t e = (int32_t)(dr[d].env * 30.0f);
        H->cv_rect(x, y, 33, 34, d == sel ? C_TXT : 0);
        H->cv_rect(x + 2, y + 2, 29, 30, 0);
        if (e > 0) H->cv_rect(x + 2, y + 32 - e, 29, e, dr[d].gate ? HB_RGB(120, 70, 10) : HB_RGB(40, 40, 50));
        b[0] = (char)('1' + d); b[1] = 0;
        H->cv_text(x + 12, y + 9, 0, b, c);
    }
    p = put_s(b, "DRONE ");
    p = put_u(p, sel + 1u);
    p = put_s(p, "  ");
    p = put_note(p, roundi(v->root));
    H->cv_text(6, 50, 0, b, C_SUN);
    p = put_s(b, "WAVE ");
    p = put_u(p, (uint32_t)roundi(v->wave * 100.0f));
    p = put_s(p, "%  VOL ");
    p = put_u(p, (uint32_t)roundi(v->vol * 100.0f));
    p = put_s(p, "%");
    H->cv_text(6, 72, 0, b, C_TXT);
    H->cv_blit(0, 24);

    /* the four oscillators, or the held page: y 120..216 */
    H->cv_begin(240, 96, 0);
    if (held & ((1u << HB_BTN_FX) | (1u << HB_BTN_LFO) | (1u << HB_BTN_ENV))) {
        const char *L[4];
        int32_t V[4];
        uint32_t n = 4;
        if (held & (1u << HB_BTN_FX)) {
            L[0] = "A CUT HZ"; V[0] = roundi(fl[0].cutoff); L[1] = "A RES %"; V[1] = roundi(fl[0].res * 100.0f);
            L[2] = "B CUT HZ"; V[2] = roundi(fl[1].cutoff); L[3] = "B RES %"; V[3] = roundi(fl[1].res * 100.0f);
        } else if (held & (1u << HB_BTN_LFO)) {
            L[0] = "LFO .01HZ"; V[0] = roundi(lfo_rate * 100.0f); L[1] = "DEPTH .1OCT"; V[1] = roundi(lfo_depth * 10.0f);
            L[2] = "DRIFT CENT"; V[2] = roundi(drift_amt * 100.0f); L[3] = "DRIVE x.1"; V[3] = roundi(drive * 10.0f);
        } else {
            L[0] = "SWELL .1S"; V[0] = roundi(v->att_s * 10.0f); L[1] = "FADE .1S"; V[1] = roundi(v->rel_s * 10.0f);
            n = 2;
        }
        for (o = 0; o < n; o++) {
            H->cv_text(6, 4 + (int32_t)o * 22, 0, L[o], C_HINT);
            put_u(b, (uint32_t)V[o]);
            H->cv_text(150, 4 + (int32_t)o * 22, 0, b, C_TXT);
        }
    } else {
        for (o = 0; o < NO; o++) {
            int32_t y = 4 + (int32_t)o * 22, w = roundi(v->o[o].level * 70.0f);
            p = put_s(b, "K");
            p = put_u(p, o + 1u);
            p = put_s(p, " ");
            put_cents(p, v->o[o].tune);
            H->cv_text(6, y, 0, b, C_TXT);
            H->cv_rect(160, y + 4, 72, 10, C_DIM);
            if (w > 0) H->cv_rect(161, y + 5, w, 8, (held & (1u << HB_BTN_EDIT)) ? C_HINT : C_SUN);
        }
    }
    H->cv_blit(0, 120);

    /* the hint line: y 216..240 */
    p = b;
    if (*msg) p = put_s(p, msg);
    else if (held & (1u << HB_BTN_EDIT)) p = put_s(p, "EDIT: KNOBS = LEVELS");
    else if (held & (1u << HB_BTN_SEL)) p = put_s(p, "SEL: KNOBS = SEMITONES");
    else { p = put_s(p, "CPU "); p = put_u(p, load_pct); p = put_s(p, "%"); }
    H->text(0, 218, 240, 0, b, *msg ? C_SUN : C_DIM, 1);
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
        for (o = 0; o < NO; o++) dr[d].o[o].ph = H->rand();
    }
    fl[0].ic1 = fl[0].ic2 = fl[1].ic1 = fl[1].ic2 = 0.0f;
    sel = 0; held = 0; dirty = 1;
    leds();
    H->log("helio: ready");
    return 0;
}

static void release(void)
{
    uint32_t d;
    for (d = 0; d < ND; d++) { dr[d].gate = 0; dr[d].env = 0.0f; }
    held = 0;
}

HB_DESC = {HB_ABI, "HELIO", init, release, note, cc, render, ui, draw, 0, 0, HB_F_PANEL, 0, 0};
