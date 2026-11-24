/* screens.c: draws HELIO's pages with the firmware's own fonts into PPM pictures (screens/NAME.ppm).
 * Build: cc -O2 -Idock/firmware/dock -Idock/firmware/gen -Ihelio -o screens test/screens.c -lm */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define HB_DESC const hb_module_t hb_module
#include "helio.c"

typedef struct {
    uint8_t h, pad, first, last;
    const uint8_t *adv, *bw;
    const uint16_t *off;
    const uint8_t *data;
} felucca_font_t;
#include "felucca_font.h"

static uint16_t fb[240 * 240], cv[240 * 96];
static int32_t cw, chh;
static uint32_t now_ms;

static void cv_pset(int32_t x, int32_t y, uint16_t c) { if (x >= 0 && y >= 0 && x < cw && y < chh) cv[y * cw + x] = c; }
static void h_cvb(int32_t w, int32_t h, uint16_t bg) { int32_t i; cw = w; chh = h; for (i = 0; i < w * h; i++) cv[i] = bg; }
static void h_cvr(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t c)
{ int32_t i, j; for (j = y; j < y + h; j++) for (i = x; i < x + w; i++) cv_pset(i, j, c); }
static void h_cvt(int32_t x, int32_t y, uint32_t big, const char *s, uint16_t c)
{
    const felucca_font_t *f = big ? &FONT_L : &FONT_S;
    for (; *s; s++) {
        uint32_t ch = (uint8_t)*s, gi, gx, gy, w, bpr;
        const uint8_t *gd;
        if (ch >= 'a' && ch <= 'z' && f->last < 'a') ch -= 32u;
        if (ch < f->first || ch > f->last) ch = '?';
        gi = ch - f->first;
        w = f->bw[gi]; bpr = (w + 7u) / 8u; gd = f->data + f->off[gi];
        for (gy = 0; gy < f->h; gy++)
            for (gx = 0; gx < w; gx++)
                if (gd[gy * bpr + gx / 8u] >> (7u - (gx & 7u)) & 1u) cv_pset(x - f->pad + (int32_t)gx, y + (int32_t)gy, c);
        x += f->adv[gi];
    }
}
static void h_cvblit(int32_t x, int32_t y)
{ int32_t i, j; for (j = 0; j < chh; j++) for (i = 0; i < cw; i++) if (y + j < 240 && x + i < 240) fb[(y + j) * 240 + x + i] = cv[j * cw + i]; }
static void h_fill(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t c)
{ int32_t i, j; for (j = y; j < y + h; j++) for (i = x; i < x + w; i++) fb[j * 240 + i] = c; }
static void h_text(int32_t x, int32_t y, int32_t w, uint32_t big, const char *s, uint16_t fg, uint32_t c)
{ (void)x; (void)y; (void)w; (void)big; (void)s; (void)fg; (void)c; }
static void h_cvl(int32_t a, int32_t b, int32_t c, int32_t d, uint16_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; }
static uint32_t h_ms(void) { return now_ms; }
static uint32_t h_ticks(void) { return 0; }
static void h_log(const char *s) { (void)s; }
static void h_log_u32(const char *s, uint32_t v) { (void)s; (void)v; }
static void h_led(uint32_t id, uint32_t on) { (void)id; (void)on; }
static uint32_t h_rand(void) { static uint32_t r = 1; r = r * 1664525u + 1013904223u; return r; }
static int h_load(void *p, uint32_t n) { (void)p; (void)n; return -1; }
static void h_save(const void *p, uint32_t n) { (void)p; (void)n; }
static int32_t h_tw(uint32_t big, const char *s) { return (int32_t)strlen(s) * (big ? 16 : 8); }

static hb_host_t host = {
    .abi = HB_ABI, .size = sizeof(hb_host_t), .ms = h_ms, .ticks = h_ticks, .log = h_log, .log_u32 = h_log_u32,
    .fill = h_fill, .text = h_text, .cv_begin = h_cvb, .cv_rect = h_cvr, .cv_line = h_cvl, .cv_text = h_cvt,
    .cv_blit = h_cvblit, .led = h_led, .rand = h_rand, .load = h_load, .save = h_save, .text_w = h_tw,
};

static void shot(const char *name)
{
    char path[64];
    FILE *f;
    int32_t i;
    now_ms += 2000;                             /* any message is over */
    hb_module.draw(1);
    snprintf(path, sizeof path, "screens/%s.ppm", name);
    f = fopen(path, "wb");
    fprintf(f, "P6 240 240 255\n");
    for (i = 0; i < 240 * 240; i++) {
        uint16_t c = fb[i];
        uint8_t rgb[3] = {(uint8_t)((c >> 11) << 3), (uint8_t)(((c >> 5) & 63) << 2), (uint8_t)((c & 31) << 3)};
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}
static void tap(uint32_t b) { hb_module.ui(HB_EV_BTN_DOWN | b, 0); hb_module.ui(HB_EV_BTN_UP | b, 0); }

int main(void)
{
    static int32_t out[512];
    uint32_t i;
    hb_module.init(&host);
    h_fill(0, 0, 240, 24, HB_RGB(30, 30, 30));          /* the host's line */
    hb_module.ui(HB_EV_KEY_DOWN | 0, 0); hb_module.ui(HB_EV_KEY_DOWN | 6, 0);
    for (i = 0; i < 400; i++) hb_module.render(out, 256);
    shot("0_home");
    hb_module.ui(HB_EV_ENC | HB_ENC_ALGORITHM, 35);   /* SWEET 70 % */
    for (i = 0; i < 40; i++) hb_module.render(out, 256);
    shot("0b_home_sweet");
    tap(HB_BTN_EDIT); shot("1_levels"); tap(HB_BTN_EDIT);
    tap(HB_BTN_FX); shot("2_reverb"); tap(HB_BTN_FX); shot("3_echo"); tap(HB_BTN_FX);
    tap(HB_BTN_GLO); shot("4_filters"); tap(HB_BTN_GLO);
    tap(HB_BTN_LFO); shot("5_lfo_speed"); tap(HB_BTN_LFO); shot("6_lfo_depth"); tap(HB_BTN_LFO); shot("7_filter_lfo");
    tap(HB_BTN_LFO); shot("8_noise_sh"); tap(HB_BTN_LFO);
    tap(HB_BTN_ENV); shot("9_drone"); tap(HB_BTN_ENV);
    tap(HB_BTN_ARP); shot("a_voice"); tap(HB_BTN_ARP); shot("b_voice_env"); tap(HB_BTN_ARP); shot("c_voice_play"); tap(HB_BTN_ARP);
    tap(HB_BTN_SEQ); shot("d_sequencer"); tap(HB_BTN_SEQ);
    sq_rest = 1u << 3; sq_len = 4; tap(HB_BTN_PLAY);
    for (i = 0; i < 300; i++) hb_module.render(out, 256);
    shot("e_steps");
    tap(HB_BTN_REC); shot("f_record");
    return 0;
}
