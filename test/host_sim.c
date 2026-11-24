/* host_sim.c: a fake fm1-dock host to run modules/helio/helio.c on a computer.
 * Build: cc -O2 -Wall -I<fm1-dock>/firmware/dock -Ihelio -o host_sim host_sim.c -lm
 * Renders a scripted performance to helio_test.wav (stereo) and checks for silence, NaN, clipping. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#define HB_DESC const hb_module_t hb_module
#include "helio.c"

static uint32_t now_ms;
static uint8_t store[256];
static uint32_t store_n;
static int text_calls, blit_calls;

static uint32_t h_ms(void) { return now_ms; }
static uint32_t h_ticks(void) { return (uint32_t)clock(); }
static void h_log(const char *s) { printf("log: %s\n", s); }
static void h_log_u32(const char *s, uint32_t v) { printf("log: %s %08x\n", s, v); }
static void h_fill(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t c) { (void)x; (void)y; (void)w; (void)h; (void)c; }
static void h_text(int32_t x, int32_t y, int32_t w, uint32_t big, const char *s, uint16_t fg, uint32_t c)
{ (void)x; (void)w; (void)big; (void)fg; (void)c; text_calls++; if (y >= 200) printf("  [line] %s\n", s); }
static void h_cvb(int32_t w, int32_t h, uint16_t bg) { (void)w; (void)h; (void)bg; }
static void h_cvr(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t c) { (void)x; (void)y; (void)w; (void)h; (void)c; }
static void h_cvl(int32_t a, int32_t b, int32_t c, int32_t d, uint16_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; }
static void h_cvp(int32_t x, int32_t y, uint16_t c) { (void)x; (void)y; (void)c; }
static void h_cvt(int32_t x, int32_t y, uint32_t big, const char *s, uint16_t fg) { (void)x; (void)big; (void)fg; text_calls++; if (y == 50) printf("  [hdr] %s\n", s); }
static void h_cvblit(int32_t x, int32_t y) { (void)x; (void)y; blit_calls++; }
static void h_led(uint32_t id, uint32_t on) { (void)id; (void)on; }
static uint32_t h_rand(void) { static uint32_t r = 12345; r = r * 1664525u + 1013904223u; return r; }
static int h_load(void *p, uint32_t n) { if (store_n != n) return -1; memcpy(p, store, n); return 0; }
static void h_save(const void *p, uint32_t n) { memcpy(store, p, n); store_n = n; }
static void h_midi(uint32_t a, uint32_t b, uint32_t c) { (void)a; (void)b; (void)c; }
static uint32_t h_master(void) { return 4096; }
static int32_t h_tw(uint32_t big, const char *s) { return (int32_t)strlen(s) * (big ? 16 : 8); }

static hb_host_t host = {
    .abi = HB_ABI, .size = sizeof(hb_host_t), .ms = h_ms, .ticks = h_ticks, .log = h_log, .log_u32 = h_log_u32,
    .fill = h_fill, .text = h_text, .cv_begin = h_cvb, .cv_rect = h_cvr, .cv_line = h_cvl, .cv_pixel = h_cvp,
    .cv_text = h_cvt, .cv_blit = h_cvblit, .led = h_led, .rand = h_rand, .load = h_load, .save = h_save,
    .midi_send = h_midi, .master = h_master, .text_w = h_tw,
};

#define BLOCK 256
#define SECS 60
static void key(uint32_t k) { hb_module.ui(HB_EV_KEY_DOWN | k, 0); hb_module.ui(HB_EV_KEY_UP | k, 0); }
static void btn(uint32_t b, int down) { hb_module.ui((down ? HB_EV_BTN_DOWN : HB_EV_BTN_UP) | b, 0); }
static void enc(uint32_t e, int32_t c) { hb_module.ui(HB_EV_ENC | e, c); }
static void tap(uint32_t b) { btn(b, 1); btn(b, 0); }   /* a page button: press opens, press again closes */
static void kdown(uint32_t k) { hb_module.ui(HB_EV_KEY_DOWN | k, 0); }
static void kup(uint32_t k) { hb_module.ui(HB_EV_KEY_UP | k, 0); }
static void sel_and(void (*f)(uint32_t), uint32_t a) { btn(HB_BTN_SEL, 1); f(a); btn(HB_BTN_SEL, 0); }

int main(void)
{
    static int32_t out[2 * BLOCK];
    FILE *f = fopen("helio_test.wav", "wb");
    uint32_t total = 44100 * SECS, pos, blk = 0, clips = 0, nans = 0;
    double sum2 = 0; int32_t peak = 0;
    uint8_t hdr[44] = "RIFF....WAVEfmt ";
    uint32_t v;
    clock_t worst = 0;

    if (hb_module.init(&host) != 0) return 1;
    hb_module.draw(1);

    /* the WAV header, stereo 16-bit */
    v = 36 + total * 4; memcpy(hdr + 4, &v, 4);
    v = 16; memcpy(hdr + 16, &v, 4);
    hdr[20] = 1; hdr[22] = 2;
    v = 44100; memcpy(hdr + 24, &v, 4);
    v = 44100 * 4; memcpy(hdr + 28, &v, 4);
    hdr[32] = 4; hdr[34] = 16;
    memcpy(hdr + 36, "data", 4);
    v = total * 4; memcpy(hdr + 40, &v, 4);
    fwrite(hdr, 1, 44, f);

    for (pos = 0; pos < total; pos += BLOCK, blk++) {
        float t = (float)pos / 44100.0f;
        uint32_t i;
        clock_t c0;
        now_ms = (uint32_t)(t * 1000.0f);
#define AT(s) (blk == (uint32_t)((s) * 44100.0f / BLOCK))
        if (AT(0.0f)) key(0);                       /* drone 1 on (F3) */
        if (AT(0.5f)) { tap(HB_BTN_FX); enc(HB_ENC_KNOB1, 8); enc(HB_ENC_KNOB4, 10); tap(HB_BTN_FX); tap(HB_BTN_FX); }  /* reverb 24 s, mix 55 % */
        if (AT(5.0f)) key(6);                       /* drone 4 on (B3) */
        if (AT(9.0f)) key(2);                       /* drone 2 on (G3) */
        if (AT(12.0f)) { tap(HB_BTN_GLO); enc(HB_ENC_KNOB1, 20); enc(HB_ENC_KNOB2, 25); tap(HB_BTN_GLO); }
        if (t >= 16.0f && t < 24.0f && blk % 20 == 0) enc(HB_ENC_KNOB4, 1);   /* slide osc 4 of drone 2 */
        if (AT(25.0f)) enc(HB_ENC_ALGORITHM, 6);    /* drone 2 brighter */
        if (AT(27.0f)) key(7);                      /* drone 5 on (C4) */
        if (AT(29.0f)) sel_and(key, 19);            /* drone 5's root from the keys (SEL held) */
        if (AT(30.0f)) { tap(HB_BTN_EDIT); enc(HB_ENC_KNOB3, -20); tap(HB_BTN_EDIT); }
        if (AT(32.0f)) { btn(HB_BTN_SAVE, 1); btn(HB_BTN_SAVE, 0); }
        if (AT(36.0f)) sel_and(tap, HB_BTN_PLAY);   /* SEL + PLAY: all drones off, they fade */
        if (AT(37.0f)) kdown(14);                   /* the voices from the upper keys: G4 */
        if (AT(37.5f)) kdown(18);                   /* B4 with it: duophonic */
        if (AT(38.5f)) { kup(14); kup(18); }
        if (AT(40.0f)) { tap(HB_BTN_LFO); tap(HB_BTN_LFO); tap(HB_BTN_LFO); tap(HB_BTN_LFO);   /* noise + S&H */
                         enc(HB_ENC_KNOB3, 7); enc(HB_ENC_KNOB4, 10); tap(HB_BTN_LFO);
                         tap(HB_BTN_PLAY); }       /* the sequencer: voice 1 */
        if (AT(46.0f)) kdown(19);                   /* a key over it: voice 2 */
        if (AT(47.0f)) kup(19);
        if (AT(52.0f)) tap(HB_BTN_PLAY);            /* stop: the echo rings out */
        if (blk % 8 == 0) hb_module.draw(0);

        memset(out, 0, sizeof out);
        c0 = clock();
        hb_module.render(out, BLOCK);
        if (clock() - c0 > worst) worst = clock() - c0;
        for (i = 0; i < 2 * BLOCK; i++) {
            int16_t s = (int16_t)out[i];
            int32_t a = out[i] < 0 ? -out[i] : out[i];
            if (a >= 32767) clips++;
            if (a > peak) peak = a;
            sum2 += (double)out[i] * out[i];
            fwrite(&s, 2, 1, f);
        }
    }
    fclose(f);
    for (uint32_t d = 0; d < ND; d++)
        for (uint32_t o = 0; o < NO; o++)
            if (dr[d].o[o].tune != dr[d].o[o].tune) nans++;

    /* reload the saved sound into a fresh init: the round trip */
    float before = dr[4].root;
    hb_module.init(&host);
    printf("save/load: root of drone 5 %.0f -> %.0f, saved %u bytes (sizeof saved_t %u)\n", before, dr[4].root, store_n, (uint32_t)sizeof(saved_t));
    printf("peak %.2f, rms %.3f, clipped samples %u, NaN %u, worst block %.0f us on this computer\n",
           peak / 32768.0, sqrt(sum2 / (2.0 * total)) / 32768.0, clips, nans, (double)worst * 1e6 / CLOCKS_PER_SEC);
    printf("text calls %d, blits %d\n", text_calls, blit_calls);

    /* stress: the longest, biggest reverb at full mix, all six drones, 120 s; then 60 s of tail */
    {
        uint32_t d2, b2, nan2 = 0;
        double worst_us = 0, rms_on = 0, rms_tail_early = 0, rms_tail_late = 0;
        int32_t pk = 0;
        hb_module.init(&host);
        tap(HB_BTN_FX);
        enc(HB_ENC_KNOB1, 100); enc(HB_ENC_KNOB2, 50); enc(HB_ENC_KNOB4, 100);
        tap(HB_BTN_FX); tap(HB_BTN_FX);
        printf("stress: decay %.1f s, size %.2f, mix %.2f, page after closing %u\n", rv_decay, rv_size, rv_mix, page);
        tap(HB_BTN_GLO); tap(HB_BTN_FX);
        printf("pages: GLO then FX -> page %u (FX is %u); ", page, (uint32_t)HB_BTN_FX);
        hb_module.draw(1);
        tap(HB_BTN_FX); printf("FX again -> sub %u", lsub);
        tap(HB_BTN_FX); printf(" -> page %u\n", page);
        tap(HB_BTN_LFO); printf("LFO taps: page %u sub %u", page, lsub);
        tap(HB_BTN_LFO); printf(" -> sub %u", lsub);
        tap(HB_BTN_LFO); printf(" -> sub %u", lsub);
        tap(HB_BTN_LFO); printf(" -> sub %u", lsub);
        tap(HB_BTN_LFO); printf(" -> page %u\n", page);
        for (d2 = 0; d2 < ND; d2++) hb_module.ui(HB_EV_KEY_DOWN | DRONE_KEY[d2], 0);
        dl_feed = 0.95f; dl_mix = 1.0f; dl_time = 0.05f; v_res = 1.0f; noise_lv = 1.0f; sh_rate = 8.0f; sh_cut = 4.0f;
        sq_bpm = 300.0f; tap(HB_BTN_PLAY); kdown(26);       /* both voices too, at their worst */
        for (b2 = 0; b2 < (uint32_t)(180.0 * 44100 / BLOCK); b2++) {
            double t2 = b2 * (double)BLOCK / 44100.0, e = 0;
            clock_t c1;
            uint32_t i;
            if (b2 == (uint32_t)(120.0 * 44100 / BLOCK)) { sel_and(tap, HB_BTN_PLAY); tap(HB_BTN_PLAY); kup(26); }
            memset(out, 0, sizeof out);
            c1 = clock();
            hb_module.render(out, BLOCK);
            if ((double)(clock() - c1) * 1e6 / CLOCKS_PER_SEC > worst_us) worst_us = (double)(clock() - c1) * 1e6 / CLOCKS_PER_SEC;
            for (i = 0; i < 2 * BLOCK; i++) {
                int32_t a = out[i] < 0 ? -out[i] : out[i];
                if (a > pk) pk = a;
                e += (double)out[i] * out[i];
            }
            e = sqrt(e / (2.0 * BLOCK)) / 32768.0;
            if (t2 > 100 && t2 < 120) rms_on = rms_on > e ? rms_on : e;
            if (t2 > 135 && t2 < 136) rms_tail_early = e;
            if (t2 > 179) rms_tail_late = e;
            for (i = 0; i < 8; i++) if (rv_lp[i] != rv_lp[i]) nan2++;
        }
        printf("stress: peak %.2f, rms while playing %.3f, tail 15 s after off %.4f, 59 s after off %.5f, NaN %u, "
               "worst block %.0f us on this computer\n", pk / 32768.0, rms_on, rms_tail_early, rms_tail_late, nan2, worst_us);
    }

    /* oscillator LFOs: drone 1 alone, osc 1 at 0.5 Hz full depth; the level should rise and fall twice a second... */
    {
        uint32_t b3, i, d3;
        double mn = 1e9, mx = 0;
        hb_module.init(&host);
        tap(HB_BTN_FX); enc(HB_ENC_KNOB4, -100); tap(HB_BTN_FX); tap(HB_BTN_FX);   /* reverb off: see the dry level */
        for (d3 = 1; d3 < NO; d3++) dr[0].o[d3].level = 0.0f;                 /* only oscillator 1 */
        tap(HB_BTN_LFO);                                                        /* speeds */
        dr[0].o[0].lrate = 0.5f;
        tap(HB_BTN_LFO); enc(HB_ENC_KNOB1, 50);                                 /* depth 100 % */
        tap(HB_BTN_LFO); tap(HB_BTN_LFO); tap(HB_BTN_LFO);                      /* back */
        dr[0].att_s = 0.1f;
        hb_module.ui(HB_EV_KEY_DOWN | DRONE_KEY[0], 0);
        for (b3 = 0; b3 < (uint32_t)(8.0 * 44100 / BLOCK); b3++) {
            double e = 0;
            memset(out, 0, sizeof out);
            hb_module.render(out, BLOCK);
            for (i = 0; i < 2 * BLOCK; i++) e += (double)out[i] * out[i];
            e = sqrt(e / (2.0 * BLOCK)) / 32768.0;
            if (b3 * BLOCK > 2 * 44100) { if (e < mn) mn = e; if (e > mx) mx = e; }
        }
        printf("osc LFO: depth %.0f %%, block rms min %.4f max %.4f\n", dr[0].o[0].ldepth * 100.0f, mn, mx);
        /* save and load the v3 sound */
        dr[0].o[2].lrate = 1.23f; dr[0].o[2].ldepth = 0.4f;
        save_sound(); dr[0].o[2].lrate = 0.05f; dr[0].o[2].ldepth = 0.0f;
        load_sound();
        printf("save/load v3: rate %.3f depth %.2f (saved %u bytes)\n", dr[0].o[2].lrate, dr[0].o[2].ldepth, store_n);
        /* an older v2 sound still loads */
        store_n = (uint32_t)SAVED_V2; { uint32_t m = MAGIC_V2; memcpy(store, &m, 4); }
        dr[0].root = 0.0f; load_sound();
        printf("v2 sound loads: root %.0f\n", dr[0].root);
    }
    /* voices, the sequencer, recording, the S&H */
    {
        uint32_t b4, i, nz = 0, steps_seen = 0, last = 99;
        hb_module.init(&host);
        kdown(12); hb_module.render(out, BLOCK);
        printf("voices: key F4 -> voice note %d stage %u; ", vo[0].key, vo[0].stage);
        kdown(16); hb_module.render(out, BLOCK);
        printf("A4 -> voice 2 note %d; ", vo[1].key);
        kup(12); kup(16); hb_module.render(out, BLOCK);
        printf("released: stages %u %u\n", vo[0].stage, vo[1].stage);
        tap(HB_BTN_REC);
        kdown(12); kup(12); kdown(14); kup(14); btn(HB_BTN_SEL, 1); btn(HB_BTN_SEL, 0); kdown(19); kup(19);
        tap(HB_BTN_REC);
        printf("record: notes %u %u %u(rest %u) %u, length %d, page %u sub %u\n", sq_note[0], sq_note[1], sq_note[2],
               (sq_rest >> 2) & 1u, sq_note[3], sq_len, page, lsub);
        tap(HB_BTN_PLAY);
        for (b4 = 0; b4 < (uint32_t)(8.0 * 44100 / BLOCK); b4++) {
            memset(out, 0, sizeof out);
            hb_module.render(out, BLOCK);
            for (i = 0; i < 2 * BLOCK; i++) if (out[i]) { nz++; break; }
            if (sq_pos != last) { last = sq_pos; steps_seen++; }
        }
        printf("sequencer: %u steps in 8 s at %.0f BPM, sounding blocks %u, PLAY led on: %u\n", steps_seen, sq_bpm, nz, sq_run);
        tap(HB_BTN_PLAY);
        /* v4 save and load, then an old v3 sound */
        v_cut = 2345.0f; sq_note[4] = 77; dl_time = 0.6f; v_oct = -1; sh_rate = 3.0f;
        save_sound(); voice_defaults(); load_sound();
        printf("save/load v4: cut %.0f step5 %u echo %.3f oct %d S&H %.2f (%u bytes)\n", v_cut, sq_note[4], dl_time, v_oct, sh_rate, store_n);
        store_n = (uint32_t)SAVED_V3; { uint32_t m = MAGIC_V3; memcpy(store, &m, 4); }
        dr[0].root = 0.0f; v_cut = 1.0f; load_sound();
        printf("v3 sound loads: root %.0f, voice cut stays %.0f\n", dr[0].root, v_cut);
        /* SWEET: osc 3 at +6.90 st goes to 3/2 (+7.02); osc 2 a hair off unison may beat in tempo */
        dr[0].o[2].tune = 6.90f; dr[0].o[1].tune = 0.12f; dr[0].root = 36.0f; sq_bpm = 120.0f;
        enc(HB_ENC_ALGORITHM, 50);
        printf("sweet %.0f %%: osc3 %.2f -> %.3f (%s%s), osc2 %.2f -> %.3f (%s%s)\n", sweet * 100.0f,
               dr[0].o[2].tune, dr[0].o[2].sweet_t, JI_NAME[dr[0].o[2].ji], dr[0].o[2].beat ? " beating" : "",
               dr[0].o[1].tune, dr[0].o[1].sweet_t, JI_NAME[dr[0].o[1].ji], dr[0].o[1].beat ? " beating" : "");
        {   /* the beat: osc 2 against 1/1 at C2 (65.4 Hz) */
            float f0 = 440.0f * powf(2.0f, (36.0f - 69.0f) / 12.0f), f1 = f0 * powf(2.0f, dr[0].o[1].sweet_t / 12.0f);
            printf("  beat %.3f Hz = %.2f steps at 120 BPM\n", f1 - f0, (f1 - f0) / 2.0f);
        }
        save_sound(); sweet = 0.0f; load_sound();
        printf("save/load v5: sweet %.0f %% (%u bytes)\n", sweet * 100.0f, store_n);
        store_n = (uint32_t)SAVED_V4; { uint32_t m = MAGIC_V4; memcpy(store, &m, 4); }
        sweet = 0.3f; v_cut = 1.0f; load_sound();
        printf("v4 sound loads: voice cut %.0f, sweet stays %.0f %%\n", v_cut, sweet * 100.0f);
        hb_module.cc(0, 123, 0); hb_module.render(out, BLOCK);
        printf("CC 123: voices releasing %u %u, seq %u\n", vo[0].stage, vo[1].stage, sq_run);
    }
    return 0;
}
