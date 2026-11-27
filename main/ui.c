// TREELINE OLED interface: C port of the "TREELINE · OLED interface" mockup (app.js).
// Same drawing engine, fonts, pages, animations and one-button logic, function by function.
// Time T is counted in 1/30 s ticks like the mockup; it follows the real clock, so the
// animations keep their speed even if a frame takes longer.
#pragma GCC diagnostic ignored "-Wformat-truncation"   // screen strings are clipped on purpose
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "ui.h"
#include "ui_fonts.h"

#define W UI_W
#define H UI_H
#define TPS 30.0f
#define NPAGES 6
#define AUTO_SLEEP_S 600          // screen sleeps after 10 min without input

// ---------------------------------------------------------------- framebuffer engine
static uint8_t fb[W * H], bufA[W * H], bufB[W * H], prevFrame[W * H], tmpBuf[W * H];
static uint8_t dissolveTh[W * H];  // hash(i, 99) * 255, precomputed
static uint8_t *cur = fb;
static float clipR[4] = { 0, 0, W, H };
static float T = 0, DT = 1;
static uint32_t nowMs;
static ui_data_t S;
void ui_do_ping(void);

enum { F46, F57, F610 };
typedef struct { const int8_t (*g)[16]; const uint8_t *codes; int n, a, d; } font_t;
static const font_t FONTS[3] = {
    { f4x6_g, f4x6_codes, F4X6_N, F4X6_A, F4X6_D },
    { f5x7_g, f5x7_codes, F5X7_N, F5X7_A, F5X7_D },
    { f6x10_g, f6x10_codes, F6X10_N, F6X10_A, F6X10_D },
};
static int16_t glyphIdx[3][256];

static void setTarget(uint8_t *b) { cur = b; }
static void clip(float x0, float y0, float x1, float y1)
{
    clipR[0] = fmaxf(0, x0); clipR[1] = fmaxf(0, y0); clipR[2] = fminf(W, x1); clipR[3] = fminf(H, y1);
}
static void noclip(void) { clipR[0] = 0; clipR[1] = 0; clipR[2] = W; clipR[3] = H; }
static inline int rnd(float v) { return (int)floorf(v + 0.5f); }   // JS Math.round
static inline void pset(float xf, float yf, int v)
{
    int x = rnd(xf), y = rnd(yf);
    if (x < clipR[0] || y < clipR[1] || x >= clipR[2] || y >= clipR[3]) return;
    int i = y * W + x;
    cur[i] = v == 2 ? cur[i] ^ 1 : (uint8_t)v;
}
static void clearBuf(void) { memset(cur, 0, W * H); }
static void fill(float x, float y, float w, float h, int v)
{
    for (int j = 0; j < h; j++) for (int i = 0; i < w; i++) pset(x + i, y + j, v);
}
static void hl(float x0, float x1, float y, int v) { for (float x = x0; x <= x1; x++) pset(x, y, v); }
static void vl(float x, float y0, float y1, int v) { for (float y = y0; y <= y1; y++) pset(x, y, v); }
static void rect(float x, float y, float w, float h, int v)
{
    hl(x, x + w - 1, y, v); hl(x, x + w - 1, y + h - 1, v); vl(x, y, y + h - 1, v); vl(x + w - 1, y, y + h - 1, v);
}
static void dottedH(float x0, float x1, float y, float ph, int v)
{
    for (float x = x0; x <= x1; x++) if (fmodf(x + ph, 2) == 0) pset(x, y, v);
}
static void line(float fx0, float fy0, float fx1, float fy1, int v)
{
    int x0 = rnd(fx0), y0 = rnd(fy0), x1 = rnd(fx1), y1 = rnd(fy1);
    int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, e = dx + dy;
    for (;;) {
        pset(x0, y0, v);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * e;
        if (e2 >= dy) { e += dy; x0 += sx; }
        if (e2 <= dx) { e += dx; y0 += sy; }
    }
}
static void circ(float cx, float cy, float rf, int v, int dither)
{
    int r = rnd(rf);
    if (r <= 0) { pset(cx, cy, v); return; }
    int x = r, y = 0, d = 1 - r, k = 0;
    while (x >= y) {
        const int pts[8][2] = { { x, y }, { y, x }, { -y, x }, { -x, y }, { -x, -y }, { -y, -x }, { y, -x }, { x, -y } };
        for (int i = 0; i < 8; i++) if (!dither || (k++ % dither == 0)) pset(cx + pts[i][0], cy + pts[i][1], v);
        y++;
        if (d < 0) d += 2 * y + 1; else { x--; d += 2 * (y - x) + 1; }
    }
}
static void disc(float cx, float cy, float r, int v)
{
    for (float y = -r; y <= r; y++) for (float x = -r; x <= r; x++)
        if (x * x + y * y <= r * r + r * 0.6f) pset(cx + x, cy + y, v);
}

// text with the real u8g2 / X11 bitmap fonts; (x, y) = top-left of the character cell
static const int8_t *glyph(int f, unsigned char ch)
{
    int i = glyphIdx[f][ch];
    if (i < 0) i = glyphIdx[f]['?'];
    return FONTS[f].g[i];
}
static float text(const char *s, float x, float y, int f, int v, int scale)
{
    float base = y + FONTS[f].a * scale;
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        const int8_t *g = glyph(f, *p);
        int dw = g[0], w = g[1], h = g[2], xo = g[3], yo = g[4];
        for (int r = 0; r < h; r++) {
            int row = (uint8_t)g[5 + r];
            for (int col = 0; col < w; col++) if ((row >> (w - 1 - col)) & 1) {
                float px = x + (xo + col) * scale, py = base - (yo + h - r) * scale;
                if (scale == 1) pset(px, py, v); else fill(px, py, scale, scale, v);
            }
        }
        x += dw * scale;
    }
    return x;
}
#define T4(s, x, y) text((s), (x), (y), F46, 1, 1)
static float tw(const char *s, int f, int scale)
{
    float w = 0;
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) w += glyph(f, *p)[0];
    return w * scale;
}
static int lh(int f) { return FONTS[f].a + FONTS[f].d; }
static float textC(const char *s, float cx, float y, int f, int v, int sc) { return text(s, rnd(cx - tw(s, f, sc) / 2), y, f, v, sc); }
static float textR(const char *s, float rx, float y, int f, int v) { return text(s, rx - tw(s, f, 1), y, f, v, 1); }
static void invBar(const char *label, float y, const char *right)
{
    fill(0, y, W, 7, 1); text(label, 2, y + 1, F46, 0, 1);
    if (right && *right) textR(right, W - 2, y + 1, F46, 0);
}

// ---------------------------------------------------------------- animation helpers
static float clampf(float v, float a, float b) { return v < a ? a : v > b ? b : v; }
static float clamp01(float v) { return clampf(v, 0, 1); }
static float lerp(float a, float b, float t) { return a + (b - a) * t; }
static float easeOut(float t) { return 1 - powf(1 - clamp01(t), 3); }
static float easeInOut(float t)
{
    t = clamp01(t);
    return t < 0.5f ? 4 * t * t * t : 1 - powf(-2 * t + 2, 3) / 2;
}
static float easeBounce(float t)
{
    t = clamp01(t);
    const float n = 7.5625f, d = 2.75f;
    if (t < 1 / d) return n * t * t;
    if (t < 2 / d) { t -= 1.5f / d; return n * t * t + 0.75f; }
    if (t < 2.5f / d) { t -= 2.25f / d; return n * t * t + 0.9375f; }
    t -= 2.625f / d; return n * t * t + 0.984375f;
}
static const char GLITCH[] = "#$%&*+=<>/\\|01?@~";
static float hashf(float a, float b)
{
    uint32_t h = ((uint32_t)(int32_t)floorf(a) * 374761393u + (uint32_t)(int32_t)floorf(b) * 668265263u) ^ 0x5bd1e995u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (float)((h ^ (h >> 16)) >> 8) / 16777216.0f;
}
static char *sbuf(void) { static char pool[24][128]; static unsigned k; return pool[k++ % 24]; }
// decode effect: characters scramble, then lock in left-to-right
static const char *decode(const char *s, float p, int seed)
{
    if (p >= 1) return s;
    char *o = sbuf();
    int n = (int)strlen(s);
    if (n > 127) n = 127;
    for (int i = 0; i < n; i++) {
        char c = s[i];
        if (c == ' ' || i < (p * 1.25f - 0.25f) * n) o[i] = c;
        else if (i < p * 1.25f * n + 2) o[i] = GLITCH[(int)floorf(hashf(i + seed, floorf(T / 2)) * 17)];
        else o[i] = ' ';
    }
    o[n] = 0;
    return o;
}
static const char *typed(const char *s, float n)
{
    char *o = sbuf();
    int k = (int)floorf(n), len = (int)strlen(s);
    if (k < 0) k = 0;
    if (k > len) k = len;
    if (k > 127) k = 127;
    memcpy(o, s, k); o[k] = 0;
    return o;
}
static bool blink(float period) { return ((long)floorf(T / (period / 2))) % 2 == 0; }
static const char *padStart(const char *s, int n)
{
    char *o = sbuf();
    int len = (int)strlen(s), pad = n - len;
    if (pad < 0) pad = 0;
    memset(o, ' ', pad); strcpy(o + pad, s);
    return o;
}

// odometer: digits roll vertically when a value changes
typedef struct { bool used; char cur[16], prev[16]; float t; } roll_t;
static roll_t rolls[8];
static float rollText(int key, const char *s, float x, float y, int f, const char *resetFrom)
{
    roll_t *r = &rolls[key];
    int n = (int)strlen(s);
    if (!r->used || resetFrom) {
        r->used = true;
        snprintf(r->cur, sizeof(r->cur), "%s", s);
        snprintf(r->prev, sizeof(r->prev), "%s", resetFrom ? resetFrom : s);
        r->t = resetFrom ? 0 : 1;
    }
    if (strcmp(r->cur, s) != 0) {
        const char *p = padStart(r->cur, n);
        int pl = (int)strlen(p);
        snprintf(r->prev, sizeof(r->prev), "%s", p + (pl - n));
        snprintf(r->cur, sizeof(r->cur), "%s", s);
        r->t = 0;
    }
    r->t = fminf(1.6f, r->t + DT / 9);
    int hgt = lh(f), fw = glyph(f, '0')[0];
    float saved[4]; memcpy(saved, clipR, sizeof(saved));
    float xx = x;
    int pn = (int)strlen(r->prev);
    for (int i = 0; i < n; i++) {
        char a[2] = { i < pn ? r->prev[i] : ' ', 0 }, b[2] = { s[i], 0 };
        float ti = easeOut(clamp01(r->t - i * 0.07f));
        clip(xx, y, xx + fw, y + hgt);
        if (a[0] == b[0] || ti >= 1) text(b, xx, y, f, 1, 1);
        else { int off = rnd(ti * hgt); text(a, xx, y - off, f, 1, 1); text(b, xx, y + hgt - off, f, 1, 1); }
        memcpy(clipR, saved, sizeof(saved));
        xx += fw;
    }
    return xx;
}

// ---------------------------------------------------------------- ASCII "shader" effects
static const char RAMP[] = " .:-~=+*XM#";
static char rampChar(float v)
{
    int i = rnd(clamp01(v) * 10);
    return RAMP[i < 0 ? 0 : i > 10 ? 10 : i];
}
typedef float (*field_fn)(float x, float y, const float *a);
static void asciiField(field_fn f, const float *arg, float x0, float y0, float x1, float y1, int cw, int ch)
{
    for (float y = y0; y + 4 <= y1; y += ch) for (float x = x0; x + 3 <= x1; x += cw) {
        char c[2] = { rampChar(f(x + 1.5f, y + 2.5f, arg)), 0 };
        if (c[0] != ' ') T4(c, x, y);
    }
}
static void textOutlined(const char *s, float x, float y, int f, int sc)
{
    const int d[8][2] = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 }, { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 } };
    for (int i = 0; i < 8; i++) text(s, x + d[i][0], y + d[i][1], f, 0, sc);
    text(s, x, y, f, 1, sc);
}
// ASCII ripple field (radio waves) used by the incoming-message animation
static float rippleF(float x, float y, const float *a)
{
    float r = hypotf(x - a[1], (y - a[2]) * 1.25f);
    float w = powf(fmaxf(0, cosf(r * 0.32f - a[0] * 0.45f)), 4) * expf(-r / 85);
    return r < 10 ? 0 : clamp01(w * 1.15f);
}
static void ripple(float t, float cx, float cy, float x0, float x1)
{
    const float a[3] = { t, cx, cy };
    asciiField(rippleF, a, x0, 0, x1, H, 4, 5);
}
// "ASCII wave" transition: a band of density characters sweeps across
static void asciiWave(const uint8_t *bufOld, const uint8_t *bufNew, float p)
{
    float pos = lerp(-0.35f, 1.35f, p), band = 0.22f;
    for (int cy = 8; cy < H; cy += 5) for (int cx = 0; cx < W; cx += 4) {
        float d = ((float)cx / W + (cy - 8) / (float)H * 0.35f) - pos;
        const uint8_t *src = d < -band ? bufNew : d > band ? bufOld : NULL;
        for (int y = cy; y < (H < cy + 5 ? H : cy + 5); y++) for (int x = cx; x < cx + 4; x++) fb[y * W + x] = src ? src[y * W + x] : 0;
        if (!src) {
            float v = 1 - fabsf(d) / band;
            char c[2] = { rampChar(v * v * 1.15f), 0 };
            if (c[0] != ' ') { setTarget(fb); T4(c, cx, cy); }
        }
    }
}

// ASCII forest: moon, stars, three parallax layers of pines, drifting fog, fireflies
static void treeClear(float cx, float top, int h, float half)
{
    for (int y = 0; y < h; y++) { float w = rnd(half * (y + 1) / h); hl(cx - w, cx + w, top + y, 0); }
}
static void forest(float t, float speed)
{
    for (int i = 0; i < 18; i++) {
        float x = hashf(i, 11) * 128, y = 1 + hashf(i, 12) * 22, tw_ = sinf(t * 0.1f + i * 2.1f);
        if (tw_ > 0.85f) T4("+", x - 1, y - 2); else if (tw_ > -0.3f) pset(x, y, 1);
    }
    disc(116, 8, 6, 1); disc(113, 6, 5, 0);
    {   // far layer: small dithered pixel pines
        float sp = 0.12f, gap = 9, off = t * sp * speed;
        for (int i = (int)floorf(off / gap) - 1; i < (int)floorf(off / gap) + 16; i++) {
            float x = i * gap - off + hashf(i, 21) * 5;
            int h = 9 + (int)floorf(hashf(i, 22) * 6), base = 44;
            for (int y = 0; y < h; y++) {
                int w = (int)floorf((y + 1) * 0.45f);
                for (int dx = -w; dx <= w; dx++) if (abs(dx) == w || ((dx + y) % 3 + 3) % 3 == 0) pset(x + dx, base - h + y, 1);
            }
        }
    }
    // fog band drifting the other way
    for (int y = 42; y < 48; y++) for (int x = 0; x < W; x++) {
        long ph = (long)floorf(x + t * 0.25f * speed + y * 3);
        if (ph % 7 == 0 && (y % 2 == 0) && sinf((x + t * 0.4f) * 0.07f + y) > 0.1f) pset(x, y, 1);
    }
    {   // mid layer: ASCII pines (4x6 font)
        static const char *art[4] = { "  ^  ", " /^\\ ", "/^^^\\", "  |  " };
        float sp = 0.38f, gap = 31, off = t * sp * speed;
        for (int i = (int)floorf(off / gap) - 1; i < (int)floorf(off / gap) + 8; i++) {
            float x = rnd(i * gap - off + hashf(i, 31) * 8), top = 30 + floorf(hashf(i, 32) * 4);
            treeClear(x + 10, top, 19, 11);
            for (int k = 0; k < 4; k++) T4(art[k], x, top + k * 6);
        }
    }
    // ground line with scrolling grass
    float gs = t * 0.9f * speed;
    hl(0, 127, 58, 1);
    for (int x = -8; x < W + 8; x += 5) {
        float wx = floorf((x + gs) / 5), sx = rnd(x - fmodf(gs, 5));
        if (hashf(wx, 51) > 0.55f) T4(hashf(wx, 52) > 0.5f ? "," : "\"", sx, 57);
    }
    {   // near layer: big pines passing in front
        float sp = 0.9f, gap = 70, off = t * sp * speed;
        for (int i = (int)floorf(off / gap) - 1; i < (int)floorf(off / gap) + 3; i++) {
            float x = rnd(i * gap - off + hashf(i, 41) * 30), top = 23, cx = x + 17;
            treeClear(cx, top - 1, 31, 19);
            for (int tier = 0; tier < 3; tier++) {
                float ty = top + tier * 8, hw = 7 + tier * 4; int h = 13;
                for (int y = 0; y < h; y++) { float w = rnd(hw * (y + 1) / h); hl(cx - w, cx + w, ty + y, 1); }
            }
            for (int r = 0; r < 4; r++) for (int c = -2; c <= 2; c++)
                if (abs(c) <= r * 0.7f + 0.5f) text("^", cx + c * 5 - 2 + (r % 2) * 2, top + 4 + r * 6, F46, 0, 1);
            fill(cx - 2, top + 30, 5, 5, 1);
        }
    }
    // fireflies
    for (int i = 0; i < 6; i++) {
        float x = 64 + sinf(t * 0.021f * (1 + i * 0.13f) + i * 1.9f) * 58, y = 40 + sinf(t * 0.033f * (1 + i * 0.07f) + i * 2.7f) * 12;
        float b = sinf(t * 0.15f + i * 1.3f);
        if (b > 0.6f) { pset(x, y, 1); pset(x - 1, y, 1); pset(x + 1, y, 1); pset(x, y - 1, 1); pset(x, y + 1, 1); }
        else if (b > -0.2f) pset(x, y, 1);
    }
}

// ---------------------------------------------------------------- pages
enum { P_HOME, P_INBOX, P_SENSORS, P_WEATHER, P_LINK, P_SYS };
static const char *TITLE = "TREELINE";
static float BOOT_INTRO_unused;
static int pageIdx = 0;
static float entered[NPAGES];
static struct { bool on; int from, to, fromIdx, toIdx, dir; float t0; } trans;

static const char *fmtAge(int s)
{
    char *o = sbuf();
    if (s < 0) strcpy(o, "--");
    else if (s < 60) sprintf(o, "%ds", s);
    else if (s < 3600) sprintf(o, "%dm", s / 60);
    else sprintf(o, "%dh", s / 3600);
    return o;
}
static const char *fmtUp(uint32_t s)
{
    char *o = sbuf();
    sprintf(o, "%02lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60), (unsigned long)(s % 60));
    return o;
}
static void lower(char *d, const char *s) { while (*s) *d++ = (char)tolower((unsigned char)*s++); *d = 0; }

static const char *BOLT[7] = { "...XX", "..XX.", ".XXXX", "XXXX.", ".XX..", "XX...", "X...." };
static void bolt(float x, float y, int v)
{
    for (int j = 0; j < 7; j++) for (int i = 0; i < 5; i++) if (BOLT[j][i] == 'X') pset(x + i, y + j, v);
}
static float chgT = NAN;
static float pingRxT = NAN;          // a ping from the peer was answered (toast like chgToast)
static char pingRxFrom[8];
static void statusBar(float dotPos)
{
    float ex = T4(S.me, 0, 0) + 1;
    if (ex < 17) ex = 17;
    if (S.linkUp) { static const char *ani[4] = { " ", ")", "))", ")))" }; T4(ani[(int)floorf(T / 5) % 4], ex, 0); }
    else if (blink(20)) T4("x", ex, 0);
    int n = NPAGES; float x0 = 64 - (n - 1) * 3;
    for (int i = 0; i < n; i++) pset(x0 + i * 6, 3, 1);
    float ax = rnd(x0 + dotPos * 6); fill(ax - 1, 2, 3, 3, 1);
    if (S.unread > 0 && blink(24)) { char b[16]; snprintf(b, sizeof(b), "*%d", S.unread); T4(b, ex + 15 > 32 ? ex + 15 : 32, 0); }
    rect(99, 1, 14, 5, 1); fill(113, 2, 1, 3, 1);
    int batt = S.batt_ok ? S.batt : 0;
    float fw = rnd(12 * batt / 100.0f); fill(100, 2, fw, 3, 1);
    if (S.charging) {
        float sweep = floorf(fmodf(T / 3, 13 - fw + 4)); fill(100 + fw, 2, fminf(12 - fw, sweep), 3, 1);
        bolt(93, 0, 1);
    } else if (S.low && blink(12)) fill(100, 2, 12, 3, 0);
    char b[8];
    if (S.batt_ok) snprintf(b, sizeof(b), "%d", batt > 99 ? 99 : batt); else strcpy(b, S.usb_power ? "US" : "--");
    T4(b, 116, 0);
    dottedH(0, 127, 7, 0, 1);
}

static void pHome(float pt)
{
    const float c = 13;
    const float tiers[4][3] = { { 18, 6, 4 }, { 22, 7, 7 }, { 27, 7, 10 }, { 32, 7, 12 } };
    for (int k = 0; k < 4; k++) for (int y = 0; y < tiers[k][1]; y++) {
        float w = rnd(tiers[k][2] * (y + 1) / tiers[k][1]); hl(c - w, c + w, tiers[k][0] + y, 1);
    }
    fill(c - 1, 39, 3, 2, 0); hl(1, 26, 40, 1);
    vl(c, 15, 18, 1);
    if (S.linkUp) {
        clip(0, 9, 30, 40);
        for (int k = 0; k < 2; k++) {
            float r = fmodf(T * 0.15f + k * 3, 6) + 2;
            for (float a = -2.7f; a <= -0.44f; a += 0.6f / r) {
                float x = c + cosf(a) * r, y = 15 + sinf(a) * r * 0.8f;
                if (y >= 9) pset(x, y, 1);
            }
            if (r < 4) { pset(c - r - 0.4f, 16, 1); pset(c + r + 0.4f, 16, 1); }
        }
        if (blink(10)) { pset(c - 1, 15, 1); pset(c + 1, 15, 1); }
        noclip();
    } else if (blink(14)) { pset(c - 1, 14, 1); pset(c + 1, 14, 1); }
    char tag[12]; snprintf(tag, sizeof(tag), "[%s]", S.me); T4(tag, 2, 43);
    if (S.linkUp) { int k = (int)floorf(T / 3) % 8; for (int i = 0; i < 3; i++) if ((k + i) % 8 < 3) pset(4 + i * 10 + (k % 3), 54 + i % 2, 1); }
    char me[8], l[6][32];
    lower(me, S.me);
    snprintf(l[0], 32, "%s:~$ status", me);
    if (!S.radio_ok) snprintf(l[1], 32, "LINK NO RADIO");
    else snprintf(l[1], 32, "LINK %s %s", S.linkUp ? "UP  " : "DOWN", fmtAge(S.seen));
    snprintf(l[2], 32, "PEER %s", S.peer);
    if (S.wifiDev < 0) snprintf(l[3], 32, "WIFI off");
    else snprintf(l[3], 32, "WIFI %d phones", S.wifiDev);
    snprintf(l[4], 32, "MSGS %d new", S.unread);
    if (S.charging) snprintf(l[5], 32, "BATT charging");
    else if (S.batt_ok) snprintf(l[5], 32, "BATT %.2fV", S.battV);
    else snprintf(l[5], 32, "BATT %s", S.usb_power ? "USB power" : "none");
    T4(typed(l[0], pt * 1.2f), 32, 10);
    for (int i = 1; i < 6; i++) T4(decode(l[i], (pt - 8 - i * 3) / 10, i * 11), 32, 10 + i * 7);
    if (pt > 30) T4(blink(20) ? "$ _" : "$ ", 32, 54);
}

static void pInbox(float pt)
{
    char r[16]; snprintf(r, sizeof(r), "%d msgs", S.nmsgs);
    invBar(" INBOX", 9, r);
    if (S.nmsgs == 0) { if (pt > 6) textC(decode("no messages yet", (pt - 6) / 12, 3), 64, 33, F46, 1, 1); return; }
    for (int i = 0; i < 3 && i < S.nmsgs; i++) {
        const ui_msg_t *m = &S.msgs[i];
        float y = 18 + i * 15, p = (pt - i * 5) / 10;
        if (p <= 0) continue;
        char head[32]; snprintf(head, sizeof(head), "%s%s %s", i == 0 ? "> " : "  ", m->from, fmtAge((int)(S.uptime - m->rx_up)));
        T4(decode(head, p, i), 0, y);
        float w = tw(m->text, F57, 1), sx = 6;
        clip(sx, y + 6, W, y + 14);
        if (w > W - sx) {
            float off = fmodf(pt * 0.75f + i * 37, w + 30);
            text(m->text, sx - off, y + 6, F57, 1, 1); text(m->text, sx - off + w + 30, y + 6, F57, 1, 1);
        } else text(m->text, sx, y + 6, F57, 1, 1);
        noclip(); clip(0, 8, W, H);
    }
}

static float merc = 0;
static int trend(void)   // -1 falling, 0 stable, 1 rising (hPa over the stored history)
{
    if (S.nhist < 2) return 0;
    float d = S.hist[S.nhist - 1] - S.hist[0];
    return d < -1.0f ? -1 : d > 1.0f ? 1 : 0;
}
static void pSensors(float pt)
{
    char sp[2] = { "|/-\\"[(int)floorf(T / 4) % 4], 0 };
    invBar(" SENSORS", 9, sp);
    // tube with a rounded top, bulb at the bottom
    hl(6, 8, 17, 1); pset(5, 18, 1); pset(9, 18, 1); vl(4, 19, 49, 1); vl(10, 19, 49, 1); disc(7, 54, 5, 1);
    for (int y = 22; y < 48; y += 5) hl(12, 13, y, 1);
    float target = clamp01(S.tAir / 50) * 30;
    merc = pt < 2 ? 0 : lerp(merc, target, 1 - powf(0.88f, DT));
    fill(6, rnd(49 - merc), 3, rnd(merc) + 1, 1);
    int tr = trend();
    char v[4][16];
    const char *lab[4] = { "PRESS", "ALT", "T.AIR", "T.CPU" };
    const char *unit[4] = { "hPa", "m", "\xb0" "C", "\xb0" "C" };
    const char *ar[4] = { tr < 0 ? "v" : tr > 0 ? "^" : "-", "-", "^", "-" };
    if (S.press_ok) { snprintf(v[0], 16, "%.1f", S.press); snprintf(v[1], 16, "%d", rnd(S.alt)); snprintf(v[2], 16, "%.1f", S.tAir); }
    else { strcpy(v[0], "---"); strcpy(v[1], "---"); strcpy(v[2], "---"); }
    snprintf(v[3], 16, "%.1f", S.tCpu);
    for (int i = 0; i < 4; i++) {
        float y = 18 + i * 11;
        T4(decode(lab[i], (pt - i * 3) / 8, i), 20, y + 1);
        char start[16]; const char *st = NULL;
        if (pt == 0) { strcpy(start, v[i]); for (char *q = start; *q; q++) if (isdigit((unsigned char)*q)) *q = '0'; st = start; }
        float ex = rollText(i, padStart(v[i], 6), 44, y, F57, st);
        T4(unit[i], ex + 2, y + 1);
        T4(ar[i], 122, y + rnd(sinf(T / 6 + i) * 1));
    }
}

static void sun(float cx, float cy, float r)
{
    disc(cx, cy, r - 2, 1);
    float a0 = T * 0.05f;
    for (int i = 0; i < 8; i++) {
        float a = a0 + i * (float)M_PI / 4, len = r + 4 + sinf(T / 5 + i) * 1.2f;
        line(cx + cosf(a) * (r + 1), cy + sinf(a) * (r + 1), cx + cosf(a) * len, cy + sinf(a) * len, 1);
    }
}
static void pWeather(float pt)
{
    invBar(" BARO TREND 3h", 9, NULL);
    const float g0 = 4, g1 = 84, y0 = 18, y1 = 46;
    vl(2, y0, y1 + 1, 1); for (float x = 2; x <= g1; x += 2) pset(x, y1 + 1, 1);
    float hist[UI_HIST]; int n = S.nhist;
    if (n < 2) { for (int i = 0; i < UI_HIST; i++) hist[i] = S.press_ok ? S.press : 1013.2f; n = UI_HIST; }
    else memcpy(hist, S.hist, sizeof(float) * n);
    float mn = 1e9f, mx = -1e9f;
    for (int i = 0; i < n; i++) { mn = fminf(mn, hist[i]); mx = fmaxf(mx, hist[i]); }
    mn -= 0.3f; mx += 0.3f;
    float P[UI_HIST][2];
    for (int i = 0; i < n; i++) { P[i][0] = g0 + (g1 - g0) * i / (n - 1); P[i][1] = y1 - (y1 - y0) * (hist[i] - mn) / (mx - mn); }
    float upto = easeInOut(pt / 40) * (n - 1);
    for (int i = 0; i < (int)floorf(upto); i++) line(P[i][0], P[i][1], P[i + 1][0], P[i + 1][1], 1);
    int k = (int)floorf(upto); float f = upto - k;
    const float *nx = P[k + 1 < n ? k + 1 : n - 1];
    float hx = lerp(P[k][0], nx[0], f), hy = lerp(P[k][1], nx[1], f);
    if (f > 0) line(P[k][0], P[k][1], hx, hy, 1);
    if (blink(10)) fill(rnd(hx) - 1, rnd(hy) - 1, 3, 3, 1);
    const float ix = 90, iy = 19;
    int tr = trend();
    if (tr != 1) {
        if (tr == 0) sun(ix + 26, iy + 4, 5);
        static const char *cl[3] = { "  .--.  ", " (    ).", "(___.__)" };
        for (int i = 0; i < 3; i++) { fill(ix + (i ? 0 : 6), iy + 4 + i * 6, i ? 32 : 20, 6, 0); T4(cl[i], ix, iy + 2 + i * 6); }
        if (tr == -1) for (int i = 0; i < 6; i++) {
            float dx = ix + 4 + i * 5, ph = fmodf(T * 0.9f + hashf(i, 3) * 20, 16);
            vl(dx - floorf(ph / 8), iy + 21 + ph, iy + 22 + ph, 1);
        }
    } else sun(ix + 16, iy + 12, 6);
    float d = hist[n - 1] - hist[0];
    char b[40];
    if (S.press_ok) snprintf(b, sizeof(b), "%.1fhPa %s%.1f/%s", S.press, d >= 0 ? "+" : "", d, S.nhist >= UI_HIST ? "3h" : fmtAge((S.nhist - 1) * 300));
    else snprintf(b, sizeof(b), "%s", S.baro_err[0] ? S.baro_err : "BMP280 not found");
    T4(b, 0, 51);
    const char *fc = tr < 0 ? "RAIN LIKELY" : tr > 0 ? "CLEARING UP" : "STEADY";
    char o[32]; snprintf(o, sizeof(o), "> %s%s", typed(fc, (pt - 30) * 0.6f), blink(16) ? "_" : "");
    T4(o, 0, 58);
}

static struct { bool on; float t0, launch; int ok, rtt; } ping;   // ok: 1, 0, -1 = waiting
#define PING_WIND 14
static float trail[10][2]; static int ntrail;
static float qbar = 0;
static void tower(float x, bool pulse)
{
    float c = x + 6;
    pset(c, 19, 1); circ(c, 19, 3, 1, 2); if (pulse) circ(c, 19, 5, 1, 2);
    const float tt[2][3] = { { 22, 5, 4 }, { 26, 7, 6 } };
    for (int k = 0; k < 2; k++) {
        for (int y = 0; y < tt[k][1]; y++) { float w = rnd(tt[k][2] * (y + 1) / tt[k][1]); pset(c - w, tt[k][0] + y, 1); pset(c + w, tt[k][0] + y, 1); }
        hl(c - tt[k][2], c + tt[k][2], tt[k][0] + tt[k][1] - 1, 1);
    }
    vl(c, 22, 33, 1); hl(c - 2, c + 2, 33, 1);
}
static void pLink(float pt)
{
    invBar(" LINK CH001 433.4MHz", 9, NULL);
    const float L = 16, R = 110, yl = 25;
    const float out = 16, hold = 6, back = 16;
    if (ping.on && ping.ok < 0) {   // real radio: hold the packet at DELTA until the answer comes
        float t = T - ping.t0;
        if (t > out + hold - 1) ping.t0 = T - (out + hold - 1);
        if (T - ping.launch > 75 + out) ping.ok = 0;   // ~2.5 s without an answer: timeout
    }
    if (!ping.on) {
        tower(2, false); tower(114, false);
        if (S.linkUp) {
            for (int i = 0; i < 3; i++) { float x = L + fmodf(T * 1.1f + i * 31, R - L); pset(x, yl - 2, 1); pset(x - 2, yl - 2, 1); dottedH(x - 6, x - 4, yl - 2, 0, 1); }
            for (int i = 0; i < 2; i++) { float x = R - fmodf(T * 0.8f + i * 47, R - L); pset(x, yl + 2, 1); pset(x + 2, yl + 2, 1); }
        } else {
            for (float x = L; x < R; x += 6) hl(x, x + 2, yl, 1);
            if (blink(14)) textC(S.radio_ok ? "NO LINK" : "NO RADIO", 64, yl - 9, F46, 1, 1);
        }
    } else {
        float t = T - ping.t0, bx = L, by = yl;
        bool ok = ping.ok == 1;
        if (t < 0) {
            tower(2, (int)floorf(T / 3) % 2 == 0); tower(114, false);
            for (float x = L; x < R; x += 6) dottedH(x, x + 2, yl, 0, 1);
            if (t > -PING_WIND) {
                float w = (t + PING_WIND) / PING_WIND;
                for (int k = 0; k < 2; k++) { float r = (1 - fmodf(w * 2 + k * 0.5f, 1)) * 12; if (r > 1) circ(L - 6, yl - 6, r, 1, 2); }
                disc(L, yl, w > 0.6f && (int)floorf(T / 2) % 2 ? 2 : 1, 1);
                fill(38, 15, 52, 9, 1); textC(blink(6) ? "PING >" : "PING  ", 64, 17, F46, 0, 1);
            } else disc(L, yl, 1, 1);
            ntrail = 0;
        } else {
            if (t < 4) { fill(38, 15, 52, 9, 1); textC("PING >", 64, 17, F46, 0, 1); }
            if (t < out) { float p = easeInOut(t / out); bx = lerp(L, R, p); by = yl - sinf(p * (float)M_PI) * 9; }
            else if (t < out + hold || ping.ok < 0) { bx = R; by = yl; }
            else if (t < out + hold + back && ok) { float p = easeInOut((t - out - hold) / back); bx = lerp(R, L, p); by = yl + sinf(p * (float)M_PI) * 6; }
            else { bx = ok ? L : R; by = yl; }
            tower(2, ok && t >= out + hold + back && t < out + hold + back + 10);
            tower(114, t >= out && t < out + hold + 4 && ping.ok != 0);
            if (ntrail == 10) { memmove(trail, trail[1], sizeof(trail[0]) * 9); ntrail = 9; }
            trail[ntrail][0] = bx; trail[ntrail][1] = by; ntrail++;
            for (int i = 0; i < ntrail; i += 2) pset(trail[i][0], trail[i][1], 1);
            bool done = t >= out + hold + back && ping.ok >= 0;
            if (!done || !ok) disc(rnd(bx), rnd(by), 1, 1);
            if (t >= out && t < out + 14) for (int k = 0; k < 2; k++) { float r = (t - out) * 1.2f - k * 5; if (r > 0) circ(R + 6, yl - 6, r, 1, 2); }
            if (done && ok) { float r = (t - out - hold - back) * 1.3f; if (r < 14) circ(L - 6, yl - 6, r, 1, 2); }
            if (ping.ok == 0 && t > out + 4) {
                float sh = t < out + 20 ? rnd(sinf(t * 2) * 2) : 0;
                fill(36 + sh, 15, 56, 9, 1); textC("TIMEOUT", 64 + sh, 17, F46, 0, 1);
            }
            if (done && ok && t < out + hold + back + 40) { fill(40, 15, 48, 9, 1); textC("PONG!", 64, 17, F46, 0, 1); }
            if (t > out + hold + back + 60 && ping.ok >= 0) ping.on = false;
        }
    }
    T4(S.me, 0, 35); textR(S.peer, 127, 35, F46, 1);
    char pv[16];
    if ((ping.on && T - ping.t0 < 38) || S.ping < 0) strcpy(pv, "---"); else snprintf(pv, sizeof(pv), "%d", S.ping);
    T4("PING", 0, 43);
    float ex = rollText(4, padStart(pv, 4), 18, 42, F57, NULL); T4("ms", ex + 1, 43);
    char b[40];
    snprintf(b, sizeof(b), "LOSS %d%%", S.loss); T4(b, 84, 43);
    snprintf(b, sizeof(b), "TX %d RX %d  P4 FU3", S.tx, S.rx); T4(b, 0, 50);
    float q = S.linkUp ? clampf(100 - S.loss * 4, 0, 100) : 0;
    T4("Q", 0, 57); rect(6, 57, 92, 6, 1);
    qbar = lerp(qbar, q, 1 - powf(0.9f, DT)); fill(8, 59, rnd(88 * qbar / 100), 2, 1);
    snprintf(b, sizeof(b), "%d%%", rnd(qbar)); textR(b, 127, 57, F46, 1);
}

static void pSys(float pt)
{
    const float cx = 18;
    for (int t = 0; t < 4; t++) {
        float g = clamp01((pt - t * 4) / 6); if (g <= 0) continue;
        float ty = 16 + t * 8, h = rnd(12 * easeOut(g)), hw = 5 + t * 3;
        for (int y = 0; y < h; y++) { float w = rnd(hw * (y + 1) / 12); hl(cx - w, cx + w, ty + y, 1); }
    }
    if (pt > 18) { fill(cx - 2, 51, 5, 4, 1); float gw = rnd(17 * easeOut((pt - 18) / 8)); hl(cx - gw, cx + gw, 55, 1); for (float x = cx - gw; x <= cx + gw; x += 3) pset(x, 54, 1); }
    if (pt > 22) { float r = rnd(4 * easeOut((pt - 22) / 8)); disc(cx + 14, 16, r, 1); disc(cx + 12, 14, fmaxf(0, r - 1), 0); }
    const int sx[4] = { 3, 8, 30, 35 }, sy[4] = { 13, 20, 26, 9 };
    for (int i = 0; i < 4; i++) if (sinf(T * 0.12f + i * 1.9f) > -0.2f && pt > 26) pset(sx[i], sy[i], 1);
    char me[8], info[7][32];
    lower(me, S.me);
    snprintf(info[0], 32, "op@%s", me);
    strcpy(info[1], "--------");
    snprintf(info[2], 32, "OS   %s", S.os_ver);
    strcpy(info[3], "CPU  C3 @160MHz");
    snprintf(info[4], 32, "RAM  %luK free", (unsigned long)(S.heap_free / 1024));
    snprintf(info[5], 32, "UP   %s", fmtUp(S.uptime));
    snprintf(info[6], 32, "TEMP %.1f\xb0" "C", S.tCpu);
    for (int i = 0; i < 7; i++) T4(decode(info[i], (pt - 6 - i * 3) / 10, i + 40), 40, 9 + i * 7);
    if (pt > 34) for (int i = 0; i < 6; i++) for (int y = 0; y < 5; y++) for (int x = 0; x < 6; x++) if (((x + y) % (i + 1)) == 0) pset(40 + i * 8 + x, 58 + y, 1);
}
static void (*const DRAW[NPAGES])(float) = { pHome, pInbox, pSensors, pWeather, pLink, pSys };

// ---------------------------------------------------------------- screen-wide states and overlays
enum { M_BOOT, M_ON, M_OFF, M_UPD };
enum { THEN_NONE, THEN_OFF, THEN_MSG, THEN_PING };
static int mode = M_BOOT;
static float modeT = 0, bootGo = NAN, lastInput = 0, bootReadyT = NAN, bootDoneT = NAN;
#define BOOT_HOLD (2 * TPS)   // results stay on screen this long before Home
static struct { bool on; int kind; float t0, len; int then; } crt;   // kind 0 = on, 1 = off
static struct { int kind; float t0, skipT, doneT; bool skip; int warn; ui_msg_t m; } overlay;   // kind 0 none, 1 msg, 2 warn
static struct { bool on; float t0; } dissolve;
static int64_t holdStart = -1;
static bool longFired;
static int clicks;
static uint32_t clickDeadline;
static bool clickPending;

static void drawPage(int id, uint8_t *buf)
{
    setTarget(buf); clearBuf(); clip(0, 8, W, H);
    DRAW[id](isnan(entered[id]) ? 0 : T - entered[id]);
    noclip();
}
static void goPage(int i, int dir)
{
    i = (i + NPAGES) % NPAGES;
    if (i == pageIdx && !trans.on) return;
    trans.on = true; trans.from = pageIdx; trans.to = i; trans.t0 = T; trans.dir = dir; trans.fromIdx = pageIdx; trans.toIdx = i;
    entered[i] = T; pageIdx = i;
}
static void startDissolve(void) { memcpy(prevFrame, fb, sizeof(fb)); dissolve.on = true; dissolve.t0 = T; }
static void dissolveFx(void)
{
    float t = (T - dissolve.t0) / 14;
    if (t >= 1) { dissolve.on = false; return; }
    int th = (int)(t * 255);
    for (int i = 0; i < W * H; i++) if (dissolveTh[i] > th) fb[i] = prevFrame[i];
}

static void bootScreen(float bt)
{
    clearBuf();
    if (isnan(bootGo)) {
        forest(bt, 1.3f);
        const char *s = decode(TITLE, (bt - 8) / 26, 3);
        if (bt > 8) textOutlined(s, rnd(64 - tw(s, F610, 2) / 2), 2, F610, 2);
        if (bt > 50 && sinf(bt * 0.16f) > -0.35f) {
            const char *pr = "[ press button ]";
            fill(rnd(64 - tw(pr, F46, 1) / 2) - 2, 56, tw(pr, F46, 1) + 4, 8, 0); T4(pr, rnd(64 - tw(pr, F46, 1) / 2), 57);
        }
        return;
    }
    const int n = 7, per = 11, gap = 5;   // a new line every 11 ticks; results 5 ticks apart
    float lt = T - bootGo;
    if (!S.boot_ready) {   // firmware still starting: keep the last line spinning
        float cap = (n - 1) * per + per - 1;
        if (lt > cap) { bootGo = T - cap; lt = cap; }
    } else if (isnan(bootReadyT)) bootReadyT = T;
    int done = (int)fminf(n, floorf(lt / per));
    int shown = done + 1 < n ? done + 1 : n, first = shown - 7 > 0 ? shown - 7 : 0;
    int revealed = 0;
    for (int i = first; i < shown; i++) {
        float y = 1 + (i - first) * 7, li = lt - i * per;
        // a line's result shows once the line has finished typing AND the results are in,
        // cascading down the list so each one can be read
        bool res = i < done && S.boot_ready && T - bootReadyT >= i * gap;
        char st[8];
        if (res) { strcpy(st, S.boot_ok[i] ? "[ OK ]" : "[FAIL]"); revealed++; }
        else snprintf(st, sizeof(st), "[ %c  ]", "|/-\\"[(int)floorf(T / 2) % 4]);
        T4(st, 0, y); T4(typed(S.boot_line[i], li * 3), 28, y);
    }
    float prog = S.boot_ready ? (float)revealed / n : clamp01(lt / (n * per)) * 0.9f;
    rect(0, 56, 128, 8, 1); fill(2, 58, rnd(124 * easeOut(prog)), 4, 1);
    if (revealed == n) {
        if (isnan(bootDoneT)) bootDoneT = T;
        if (blink(8)) { fill(44, 22, 40, 11, 0); rect(44, 22, 40, 11, 1); textC("READY", 64, 25, F46, 1, 1); }
    }
}

static void msgOverlay(void)
{
    float t = T - overlay.t0; clearBuf();
    if (t < 34 && !overlay.skip) {
        ripple(t, 13, 28, 0, W);
        fill(4, 25, 19, 23, 0); text(" Y ", 6, 26, F57, 1, 1); text("/|\\", 6, 33, F57, 1, 1); text("/ \\", 6, 40, F57, 1, 1);
        const char *s1 = decode("INCOMING", t / 14, 5); fill(42, 20, tw(s1, F610, 1) + 4, 12, 0); text(s1, 44, 21, F610, 1, 1);
        char f[16]; snprintf(f, sizeof(f), "<<< %s", overlay.m.from);
        const char *s2 = typed(f, (t - 12) * 0.8f);
        if (*s2) { fill(42, 34, tw(s2, F57, 1) + 4, 9, 0); text(s2, 44, 35, F57, 1, 1); }
        return;
    }
    float t2 = overlay.skip ? 999 : t - 34;
    if (t2 < 46) {
        static const char *env[5] = { " ________ ", "|\\      /|", "| \\    / |", "|  \\__/  |", "|________|" };
        static const char *opn[5] = { " __/\\__   ", "|  /  \\  |", "| /    \\ |", "|/      \\|", "|________|" };
        float yy = rnd(lerp(-32, 18, easeBounce(t2 / 22)));
        const char **art = t2 > 28 ? opn : env;
        if (t2 > 28) {
            float ly = rnd(lerp(yy + 6, yy - 16, easeOut((t2 - 28) / 14)));
            fill(48, ly, 32, 18, 0); rect(48, ly, 32, 18, 1);
            for (int k = 0; k < 3; k++) hl(52, 75 - k * 6, ly + 4 + k * 4, 1);
        }
        for (int i = 0; i < 5; i++) T4(art[i], 44, yy + i * 6);
        if (t2 > 22 && t2 < 34) for (int k = 0; k < 5; k++) {
            float a = k * 1.256f + t2 * 0.2f, r = (t2 - 22) * 2.2f;
            T4("+", 62 + cosf(a) * r * 1.6f, yy + 12 + sinf(a) * r);
        }
        return;
    }
    float t3 = overlay.skip ? (T - overlay.skipT) + 999 : t2 - 46;
    char hb[24]; snprintf(hb, sizeof(hb), " MSG %s", overlay.m.from);
    invBar(hb, 0, "just now");
    // word wrap at 25 characters
    char lines[8][32]; int nl = 0; char ln[128] = "";
    char words[96]; snprintf(words, sizeof(words), "%s", overlay.m.text);
    for (char *w = strtok(words, " "); w; w = strtok(NULL, " ")) {
        char trial[160]; snprintf(trial, sizeof(trial), "%s %s", ln, w);
        const char *tt = trial; while (*tt == ' ') tt++;
        if (strlen(tt) > 25 && *ln) { if (nl < 8) { const char *l0 = ln; while (*l0 == ' ') l0++; snprintf(lines[nl++], 32, "%s", l0); } snprintf(ln, sizeof(ln), "%s", w); }
        else snprintf(ln, sizeof(ln), "%s", tt);
    }
    if (nl < 8) snprintf(lines[nl++], 32, "%s", ln);
    float n = overlay.skip ? 1e9f : t3 * 1.4f;
    for (int i = 0; i < nl && i < 6; i++) {
        int len = (int)strlen(lines[i]);
        const char *s = typed(lines[i], n); n -= len;
        text(s, 0, 10 + i * 8, F57, 1, 1);
        if (n < 0 && n > -len && (int)strlen(s) < len && blink(8)) fill(tw(s, F57, 1) + 1, 10 + i * 8, 4, 7, 1);
    }
    if (n >= 0) {
        if (overlay.doneT == 0) overlay.doneT = T + 8 * TPS;
        if (blink(24)) textR("[btn] ok", 127, 58, F46, 1);
    }
}

static void warnOverlay(void)
{
    float t = T - overlay.t0;
    fill(6, 10, 116, 46, 0);
    for (int x = 6; x < 122; x++) if (fmodf(x + t, 4) < 2) { pset(x, 10, 1); pset(127 - x, 55, 1); }
    for (int y = 10; y < 56; y++) if (fmodf(y + t, 4) < 2) { pset(121, y, 1); pset(6, 65 - y, 1); }
    if (overlay.warn == 1) {
        if (blink(10)) text("!", 14, 17, F610, 1, 2);
        T4("USB-C + SWITCH ON", 30, 16); T4("turn the switch OFF", 30, 26); T4("before plugging", 30, 33); T4("the ESP into USB", 30, 40);
    } else {
        text("[", 14, 22, F610, 1, 1); rect(20, 25, 16, 9, 1); fill(36, 27, 2, 5, 1); if (blink(10)) fill(22, 27, 2, 5, 1);
        char b[16]; snprintf(b, sizeof(b), "%d%% left", S.batt);
        text("LOW BATTERY", 46, 17, F57, 1, 1); T4(b, 46, 27); T4("charge via", 46, 35); T4("USB-C", 46, 42);
    }
    if (blink(24)) textC("press = ok", 64, 48, F46, 1, 1);
}

static void chgToast(float t)
{
    float inY = easeOut(t / 10), outY = t > 60 ? easeInOut((t - 60) / 15) : 0, y = rnd(lerp(-14, 22, inY) - outY * 36);
    const char *label = S.charging ? "CHARGING" : "ON BATTERY";
    float w = tw(label, F57, 1) + 22, x = rnd(64 - w / 2);
    fill(x - 1, y - 1, w + 2, 15, 0); fill(x, y, w, 13, 1);
    if (S.charging) bolt(x + 5, y + 3, 0);
    else { rect(x + 4, y + 4, 9, 5, 0); fill(x + 13, y + 5, 1, 3, 0); fill(x + 5, y + 5, rnd(7 * S.batt / 100.0f), 3, 0); }
    text(label, x + 17, y + 3, F57, 0, 1);
}

// same toast as chgToast: an antenna icon and "PING <peer>"
static void pingToast(float t)
{
    float inY = easeOut(t / 10), outY = t > 60 ? easeInOut((t - 60) / 15) : 0, y = rnd(lerp(-14, 22, inY) - outY * 36);
    char label[16]; snprintf(label, sizeof(label), "PING %s", pingRxFrom);
    float w = tw(label, F57, 1) + 22, x = rnd(64 - w / 2);
    fill(x - 1, y - 1, w + 2, 15, 0); fill(x, y, w, 13, 1);
    vl(x + 8, y + 5, y + 10, 0); line(x + 5, y + 2, x + 8, y + 5, 0); line(x + 11, y + 2, x + 8, y + 5, 0);
    if (blink(6)) { pset(x + 4, y + 6, 0); pset(x + 12, y + 6, 0); }
    text(label, x + 17, y + 3, F57, 0, 1);
}

static void crtFx(void)
{
    float t = T - crt.t0, p = clamp01(t / crt.len);
    memcpy(tmpBuf, fb, sizeof(fb)); memset(fb, 0, sizeof(fb)); setTarget(fb);
    if (crt.kind == 0) {
        if (p < 0.35f) { float w = rnd(easeOut(p / 0.35f) * W); hl(64 - w / 2, 64 + w / 2, 31, 1); hl(64 - w / 2, 64 + w / 2, 32, 1); }
        else {
            float s = easeOut((p - 0.35f) / 0.65f), band = fmaxf(2, s * H);
            for (int y = 0; y < H; y++) {
                if (fabsf(y - 31.5f) > band / 2) continue;
                int sy = rnd(31.5f + (y - 31.5f) / (band / H));
                if (sy >= 0 && sy < H) memcpy(fb + y * W, tmpBuf + sy * W, W);
            }
            if (s < 0.9f) hl(0, 127, rnd(31.5f - band / 2), 1);
        }
    } else {
        if (p < 0.55f) {
            float band = fmaxf(2, (1 - easeInOut(p / 0.55f)) * H);
            for (int y = 0; y < H; y++) {
                if (fabsf(y - 31.5f) > band / 2) continue;
                int sy = rnd(31.5f + (y - 31.5f) / (band / H));
                if (sy >= 0 && sy < H) memcpy(fb + y * W, tmpBuf + sy * W, W);
            }
        } else if (p < 0.85f) { float w = rnd((1 - easeInOut((p - 0.55f) / 0.3f)) * W); hl(64 - w / 2, 64 + w / 2, 31, 1); hl(64 - w / 2, 64 + w / 2, 32, 1); }
        else if ((int)floorf(t / 2) % 2 == 0) fill(63, 31, 2, 2, 1);
    }
    if (p >= 1) {
        int then = crt.then; crt.on = false;
        if (then == THEN_OFF) mode = M_OFF;
        else if (then == THEN_MSG) { overlay.kind = 1; overlay.t0 = T; overlay.skip = false; overlay.doneT = 0; }
        else if (then == THEN_PING) {
            ui_do_ping();   // double press woke the screen: ping once it is back on
        }
    }
}

// ---------------------------------------------------------------- Wi-Fi update mode
static struct { char ssid[33], pass[33], ip[16], msg[28]; int pct; float t0, pctShown; } upd;
static void updScreen(void)
{
    float t = T - upd.t0;
    char sp[2] = { "|/-\\"[(int)floorf(T / 4) % 4], 0 };
    invBar(" UPDATE MODE", 0, upd.pct == 101 ? "" : sp);
    // access point mast with waves going out
    float c = 9;
    vl(c, 14, 33, 1); hl(c - 3, c + 3, 33, 1);
    for (int y = 0; y < 14; y++) { float w = rnd(5.0f * (y + 1) / 14); pset(c - w, 20 + y, 1); pset(c + w, 20 + y, 1); }
    disc(c, 13, 1, 1);
    for (int k = 0; k < 3; k++) {
        float r = fmodf(t * 0.35f + k * 4, 12);
        if (r > 2) for (float a = -2.6f; a <= -0.54f; a += 0.5f / r) pset(c + cosf(a) * r, 13 + sinf(a) * r, 1);
    }
    T4("WIFI", 24, 10); T4(decode(upd.ssid, (t - 4) / 12, 7), 46, 10);
    T4("PASS", 24, 18); T4(decode(upd.pass, (t - 8) / 12, 8), 46, 18);
    T4("OPEN", 24, 27); text(decode(upd.ip, (t - 12) / 12, 9), 46, 26, F57, 1, 1);
    // progress
    if (upd.pct >= 0) upd.pctShown = lerp(upd.pctShown, upd.pct > 100 ? 100 : upd.pct, 1 - powf(0.75f, DT));
    rect(0, 37, 128, 9, 1);
    if (upd.pct >= 0) fill(2, 39, rnd(124 * upd.pctShown / 100), 5, 1);
    else for (int x = 2; x < 126; x++) if (((int)(x + T) / 3) % 4 == 0) vl(x, 39, 43, 1);   // idle "marching" bar
    char b[40];
    if (upd.pct == -2) snprintf(b, sizeof(b), "! %s", upd.msg);
    else if (upd.pct >= 0 && upd.pct <= 100 && !strcmp(upd.msg, "receiving")) snprintf(b, sizeof(b), "> receiving %d%%", upd.pct);
    else snprintf(b, sizeof(b), "> %s%s", upd.msg, blink(16) ? "_" : "");
    if (upd.pct == -2 && !blink(12)) b[0] = ' ';
    T4(b, 0, 49);
    if (upd.pct < 0 || upd.pct == -2) textR("hold = exit", 127, 58, F46, 1);
    if (upd.pct == 101) textR("bye!", 127, 58, F46, 1);
}
void ui_update_mode(const char *ssid, const char *pass, const char *ip)
{
    snprintf(upd.ssid, sizeof(upd.ssid), "%s", ssid);
    snprintf(upd.pass, sizeof(upd.pass), "%s", pass);
    snprintf(upd.ip, sizeof(upd.ip), "%s", ip);
    snprintf(upd.msg, sizeof(upd.msg), "starting Wi-Fi");
    upd.pct = -1; upd.pctShown = 0; upd.t0 = T;
    mode = M_UPD; overlay.kind = 0; trans.on = false;
    crt.on = true; crt.kind = 0; crt.t0 = T; crt.len = 15; crt.then = THEN_NONE;
}
void ui_update_status(int pct, const char *msg)
{
    upd.pct = pct;
    if (pct == 0) upd.pctShown = 0;
    snprintf(upd.msg, sizeof(upd.msg), "%s", msg);
}

// ---------------------------------------------------------------- the single button
static void endOverlay(void)
{
    if (overlay.kind == 1 && S.unread > 0) S.unread--;
    overlay.kind = 0; startDissolve(); entered[pageIdx] = T;
}
static void wake(int then)
{
    mode = M_ON; entered[pageIdx] = T; trans.on = false;
    crt.on = true; crt.kind = 0; crt.t0 = T; crt.len = 15; crt.then = then; lastInput = T;
}
static bool bootTitlePress(void)
{
    if (mode != M_BOOT || !isnan(bootGo) || crt.on || T - modeT < 20) return mode == M_BOOT;
    bootGo = T; startDissolve(); return true;
}
static bool anyPressConsumed(void)
{
    lastInput = T;
    if (overlay.kind == 2) { overlay.kind = 0; startDissolve(); return true; }
    if (overlay.kind == 1) {
        float t = T - overlay.t0;
        if (!overlay.skip && t < 34 + 46 + 40) { overlay.skip = true; overlay.skipT = T; }
        else endOverlay();
        return true;
    }
    return false;
}
static void onShort(void)
{
    if (mode == M_UPD) return;
    if (bootTitlePress()) return;
    if (mode == M_BOOT || crt.on) return;
    if (mode == M_OFF) { wake(THEN_NONE); return; }
    if (anyPressConsumed()) return;
    goPage(pageIdx + 1, 1);
}
void ui_do_ping(void)
{
    bool jump = pageIdx != P_LINK;
    if (jump) goPage(P_LINK, 1);
    float transLen = jump ? 18 + 6 : 0;
    ping.on = true; ping.t0 = T + transLen + PING_WIND; ping.launch = ping.t0; ping.rtt = 0;
    ping.ok = (S.radio_ok && S.linkUp) ? -1 : 0;
    if (S.radio_ok) ui_port_ping();
}
static void onDouble(void)
{
    if (mode == M_UPD) return;
    if (bootTitlePress()) return;
    if (mode == M_BOOT || crt.on) return;
    lastInput = T;
    if (overlay.kind == 2) anyPressConsumed();
    overlay.kind = 0;
    if (mode == M_OFF) wake(THEN_PING); else ui_do_ping();
}
static void onLong(void)
{
    if (mode == M_UPD) { if (upd.pct < 0) ui_port_reboot(); return; }   // not while flashing
    if (mode != M_ON || crt.on) return;
    lastInput = T; overlay.kind = 0;
    crt.on = true; crt.kind = 1; crt.t0 = T; crt.len = 18; crt.then = THEN_OFF;
}

void ui_button(bool down, uint32_t now)
{
    if (down) { holdStart = now; longFired = false; return; }
    if (holdStart < 0) return;
    holdStart = -1;
    if (longFired) return;
    clicks++;
    if (clicks == 2) { clickPending = false; clicks = 0; onDouble(); return; }
    clickPending = true; clickDeadline = now + 280;
}

// ---------------------------------------------------------------- events from the firmware
void ui_message(const char *from, const char *txt)
{
    if (S.nmsgs == UI_MSGS) S.nmsgs--;
    memmove(&S.msgs[1], &S.msgs[0], sizeof(ui_msg_t) * S.nmsgs);
    snprintf(S.msgs[0].from, sizeof(S.msgs[0].from), "%s", from);
    snprintf(S.msgs[0].text, sizeof(S.msgs[0].text), "%s", txt);
    S.msgs[0].rx_up = S.uptime;
    S.nmsgs++; S.unread++;
    if (mode == M_BOOT) return;
    lastInput = T;
    overlay.m = S.msgs[0];
    if (mode == M_OFF) { mode = M_ON; crt.on = true; crt.kind = 0; crt.t0 = T; crt.len = 15; crt.then = THEN_MSG; }
    else { overlay.kind = 1; overlay.t0 = T; overlay.skip = false; overlay.doneT = 0; }
}
void ui_set_charging(bool on)
{
    S.charging = on; chgT = T;
    if (on) S.low = false;
    if (mode == M_OFF) wake(THEN_NONE);
}
void ui_warn(int kind)
{
    if (mode == M_BOOT) return;
    if (mode == M_OFF) wake(THEN_NONE);
    overlay.kind = 2; overlay.t0 = T; overlay.warn = kind;
}
void ui_ping_rx(const char *from)
{
    snprintf(pingRxFrom, sizeof(pingRxFrom), "%s", from);
    pingRxT = T;
    if (mode == M_OFF) wake(THEN_NONE);
}
void ui_ping_result(bool ok, int rtt)
{
    if (ok) S.ping = rtt;
    if (ping.on && ping.ok < 0) { ping.ok = ok ? 1 : 0; ping.rtt = rtt; }
}

// ---------------------------------------------------------------- frame
static uint32_t lastMs;
static bool started;
void ui_frame(uint32_t now)
{
    nowMs = now;
    if (!started) { started = true; lastMs = now; DT = 1; }
    else { DT = (now - lastMs) * TPS / 1000.0f; if (DT > 4) DT = 4; if (DT < 0) DT = 0; }
    lastMs = now;
    T += DT;
    // button timers (the mockup uses setTimeout for these)
    if (holdStart >= 0 && !longFired && now - (uint32_t)holdStart >= 990) { longFired = true; onLong(); }
    if (clickPending && (int32_t)(now - clickDeadline) >= 0) { clickPending = false; clicks = 0; onShort(); }
    if (mode == M_ON && !crt.on && overlay.kind == 0 && T - lastInput > AUTO_SLEEP_S * TPS) onLong();

    setTarget(fb); clearBuf(); noclip();
    if (mode == M_OFF) { if (crt.on) crtFx(); return; }
    if (mode == M_UPD) {
        updScreen();
        if (holdStart >= 0 && upd.pct < 0) { float p = clamp01((now - (uint32_t)holdStart) / 1000.0f); if (p > 0.15f) { fill(0, 62, W, 2, 0); fill(0, 62, rnd(W * p), 2, 1); } }
        if (crt.on) crtFx();
        return;
    }
    if (mode == M_BOOT) {
        bootScreen(T - modeT);
        if (!isnan(bootDoneT) && T - bootDoneT > BOOT_HOLD) {
            mode = M_ON; startDissolve(); entered[pageIdx] = T; trans.on = false; lastInput = T;
        }
    } else if (overlay.kind == 1) {
        msgOverlay();
        if (overlay.doneT && T - overlay.doneT > 0) endOverlay();
    } else {
        float dotPos = pageIdx;
        if (trans.on) {
            float p = clamp01((T - trans.t0) / 18);
            drawPage(trans.from, bufA); drawPage(trans.to, bufB); setTarget(fb);
            asciiWave(bufA, bufB, easeInOut(p));
            dotPos = lerp(trans.fromIdx, trans.toIdx, p);
            if (p >= 1) trans.on = false;
        } else { drawPage(pageIdx, bufA); setTarget(fb); memcpy(fb, bufA, sizeof(fb)); }
        statusBar(dotPos);
        if (!isnan(chgT) && T - chgT < 75) chgToast(T - chgT);
        if (!isnan(pingRxT) && T - pingRxT < 75) pingToast(T - pingRxT);
        if (overlay.kind == 2) warnOverlay();
    }
    if (holdStart >= 0 && mode != M_BOOT) {
        float p = clamp01((now - (uint32_t)holdStart) / 1000.0f);
        if (p > 0.15f) { fill(0, 62, W, 2, 0); fill(0, 62, rnd(W * p), 2, 1); }
    }
    if (dissolve.on) dissolveFx();
    if (crt.on) crtFx();
}

const uint8_t *ui_fb(void) { return fb; }
ui_data_t *ui_data(void) { return &S; }

void ui_init(const char *title)
{
    if (title) TITLE = title;
    for (int f = 0; f < 3; f++) {
        for (int c = 0; c < 256; c++) glyphIdx[f][c] = -1;
        for (int i = 0; i < FONTS[f].n; i++) glyphIdx[f][FONTS[f].codes[i]] = (int16_t)i;
    }
    for (int i = 0; i < W * H; i++) dissolveTh[i] = (uint8_t)(hashf(i, 99) * 255);
    for (int i = 0; i < NPAGES; i++) entered[i] = NAN;
    memset(&S, 0, sizeof(S));
    S.seen = -1; S.ping = -1; S.wifiDev = -1;
    strcpy(S.me, "ECHO"); strcpy(S.peer, "DELTA"); strcpy(S.os_ver, "ESP-IDF");
    (void)BOOT_INTRO_unused;
    // power-on: CRT turn-on into the title screen
    mode = M_BOOT; modeT = 0; crt.on = true; crt.kind = 0; crt.t0 = 0; crt.len = 15; crt.then = THEN_NONE;
}
