/*
 * navboard.h — shared AR.Drone 2.0 navboard (IMU) reader. HEADER-ONLY module.
 *
 * Single source of truth for: the /dev/ttyO1 frame protocol, the factory
 * gyro/accel calibration, fresh startup gyro-bias, and the frame-validation /
 * lock-on gate. Included by navread.c (CLI) and the control-loop tools
 * (ratebench.c, …) so the subtle framing/gate logic lives in ONE place.
 *
 * All functions are `static` (compiled into each including TU), so build.sh
 * stays single-file — each tool is still one self-contained .c that #includes
 * this header. We only ever read one navboard per process.
 *
 * Provenance + validation: docs/sensors.md (on-drone verified 2026-07-06):
 * still→gyro≈0/az≈−1g; hand-rotation lights the right axis; gate drops the
 * non-unique-0x3A00 false-sync (0 impossible frames over a gated capture).
 *
 * Calibration (verbatim from backups/config.ini):
 *   gyro:  rate = (raw−bias)·gyros_gains[axis](rad/s per count)·(180/π) → °/s
 *          gyros_gains = { +1.0589919e-3, −1.0588102e-3, −1.0600387e-3 }
 *          BIAS measured fresh from stillness (stored gyros_offset is volatile).
 *   accel: a_mg = accs_gains(3×3)·raw + accs_offset ; g = a_mg/1000 (factory cal)
 *          (az reads ≈ −1 g flat: gravity is −Z in this frame — expected.)
 *
 * SAFETY: read-only. No motor bus, no GPIO, no /dev/mem, no process kills.
 */
#ifndef NAVBOARD_H
#define NAVBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <time.h>

#define NAV_DEV_DEFAULT "/dev/ttyO1"
#define NAV_BAUD        B460800     /* ardrone2 navboard rate (program.elf leaves it set) */
#define FRAME_LEN       58          /* taille = 0x3A = 58 bytes per frame */
#define NAV_HZ          200.0       /* ~200 frames/s from the navboard */
#define DEG_PER_RAD     57.295779513082320876

/* ---- factory calibration (backups/config.ini) ---- */
static const double GYRO_GAIN_RADPS[3] = { 1.0589919e-03, -1.0588102e-03, -1.0600387e-03 };
static const double ACC_GAIN[3][3] = {
    {  1.9873557e+00, -5.4023052e-03, -4.5317975e-03 },
    { -1.6823394e-02, -1.9521351e+00, -1.3380921e-02 },
    { -1.8335162e-02,  4.4719707e-03, -1.9520746e+00 },
};
static const double ACC_OFF_MG[3] = { -4.1148950e+03, 4.0706472e+03, 4.0540405e+03 };

/* ---- frame-validation / lock-on gate tuning (see docs/sensors.md) ---- */
#define NB_LOCK_FRAMES  5       /* consecutive good frames to declare startup lock */
#define NB_MAX_SEQ_GAP  8       /* accept seq steps of 1..this (tolerate a few drops) */
#define NB_RELOCK_AFTER 12      /* consecutive rejects => re-anchor to the live stream */
#define NB_ACC_RAW_MAX  4096    /* 12-bit accel ADC ceiling; a real count never reaches it */
#define NB_MOTION_DPS   1.5     /* bias-calibration motion guard */

/* ---- Ctrl-C: no SA_RESTART so blocking reads return EINTR ---- */
static volatile sig_atomic_t nb_stop = 0;
static void nb_on_signal(int sig) { (void)sig; nb_stop = 1; }
static void navboard_install_signals(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = nb_on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

static double nb_now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ============================ state ============================ */
typedef struct {
    unsigned seq;
    int      raw_a[3];      /* accel counts (unsigned, stored int) */
    int      raw_g[3];      /* gyro counts (signed) */
} nb_frame_t;

typedef struct {
    double   gx, gy, gz;    /* gyro body rates, °/s (bias-corrected, signed gains applied) */
    double   ax, ay, az;    /* accel, g (factory cal) */
    int      raw_a[3], raw_g[3];
    unsigned seq;
} nav_sample_t;

typedef struct {
    int      fd;
    uint8_t  buf[1024];
    int      buflen;
    long     resyncs;       /* junk bytes skipped by the parser */
    /* gate */
    int      locked;
    unsigned last_seq;
    int      have_last;
    long     rejects;       /* frames dropped by the gate (bad seq / insane) */
    long     relocks;       /* times we lost lock and re-anchored */
    long     dropped;       /* genuine dropped frames (accepted small seq gaps) */
    /* calibration */
    double   bias[3];       /* gyro bias, counts */
    int      have_bias;
} navboard_t;

/* ============================ serial ============================ */
static int nb_open_port(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open navboard"); return -1; }
    struct termios t;
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        cfsetispeed(&t, NAV_BAUD);     /* set 460800 explicitly (don't depend on prior state) */
        cfsetospeed(&t, NAV_BAUD);
        t.c_cflag |= (CLOCAL | CREAD);
        t.c_cc[VMIN]  = 1;
        t.c_cc[VTIME] = 0;
        tcsetattr(fd, TCSANOW, &t);
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}

/* send 0x01, let it ramp, drop stale bytes so we read fresh frames */
static void nb_start_acquisition(int fd)
{
    uint8_t start = 0x01;
    if (write(fd, &start, 1) != 1) perror("write start byte");
    usleep(200000);
    tcflush(fd, TCIFLUSH);
}

/* ============================ frame parser ============================
 * Byte-stream resync on the 0x3A 0x00 header. 1 = frame in out[], 0 = stop/EOF,
 * -1 = error. Header alone is NOT trustworthy (0x3A 0x00 recurs in data) — the
 * gate below validates; this just delivers 58-byte candidates. */
static int nb_next_frame(navboard_t *nb, uint8_t out[FRAME_LEN])
{
    for (;;) {
        int h = -1;
        for (int i = 0; i + 1 < nb->buflen; i++)
            if (nb->buf[i] == 0x3A && nb->buf[i + 1] == 0x00) { h = i; break; }
        if (h > 0) {                                     /* junk before header -> drop (resync) */
            nb->resyncs += h;
            memmove(nb->buf, nb->buf + h, nb->buflen - h);
            nb->buflen -= h;
            h = 0;
        }
        if (h == 0 && nb->buflen >= FRAME_LEN) {         /* full candidate frame */
            memcpy(out, nb->buf, FRAME_LEN);
            memmove(nb->buf, nb->buf + FRAME_LEN, nb->buflen - FRAME_LEN);
            nb->buflen -= FRAME_LEN;
            return 1;
        }
        if (h < 0) {                                     /* no header; keep a possible trailing 0x3A */
            if (nb->buflen > 0 && nb->buf[nb->buflen - 1] == 0x3A) { nb->buf[0] = 0x3A; nb->buflen = 1; }
            else nb->buflen = 0;
        }
        int space = (int)sizeof(nb->buf) - nb->buflen;
        if (space <= 0) { nb->buflen = 0; space = (int)sizeof(nb->buf); }
        ssize_t n = read(nb->fd, nb->buf + nb->buflen, space);
        if (n < 0) {
            if (errno == EINTR) { if (nb_stop) return 0; continue; }
            perror("read navboard");
            return -1;
        }
        if (n == 0) return 0;                            /* EOF */
        nb->buflen += n;
    }
}

/* little-endian field extractors */
static int      nb_s16(const uint8_t *b, int o) { int v = b[o] | (b[o + 1] << 8); return (v >= 32768) ? v - 65536 : v; }
static unsigned nb_u16(const uint8_t *b, int o) { return (unsigned)(b[o] | (b[o + 1] << 8)); }

static void nb_decode(const uint8_t *b, nb_frame_t *f)
{
    f->seq      = nb_u16(b, 0x02);
    f->raw_a[0] = (int)nb_u16(b, 0x04);
    f->raw_a[1] = (int)nb_u16(b, 0x06);
    f->raw_a[2] = (int)nb_u16(b, 0x08);
    f->raw_g[0] = nb_s16(b, 0x0A);
    f->raw_g[1] = nb_s16(b, 0x0C);
    f->raw_g[2] = nb_s16(b, 0x0E);
}

/* raw gyro counts -> °/s (bias in counts, signed gain) */
static double nb_gyro_dps(int raw, double bias, int axis)
{
    return (raw - bias) * GYRO_GAIN_RADPS[axis] * DEG_PER_RAD;
}
/* raw accel counts (3) -> g (3) via the factory 3x3 + offset */
static void nb_accel_g(const int raw[3], double g[3])
{
    for (int i = 0; i < 3; i++) {
        double mg = ACC_OFF_MG[i];
        for (int j = 0; j < 3; j++) mg += ACC_GAIN[i][j] * raw[j];
        g[i] = mg / 1000.0;
    }
}

/* ==================== frame validation / lock-on gate ====================
 * The 0x3A 0x00 header (=58) also occurs inside frames, so a mid-stream start
 * can false-sync one frame at the wrong offset. We validate a frame is *real*:
 * startup lock-on (NB_LOCK_FRAMES consecutive seq+1), per-frame seq-continuity
 * (1..NB_MAX_SEQ_GAP), physical sanity (accel raw < 12-bit), and self-heal
 * (NB_RELOCK_AFTER rejects -> re-anchor). Full rationale: docs/sensors.md. */
static int nb_frame_sane(const nb_frame_t *f)
{
    for (int i = 0; i < 3; i++)
        if (f->raw_a[i] < 0 || f->raw_a[i] >= NB_ACC_RAW_MAX) return 0;
    return 1;
}

/* discard frames until locked onto the true 58-byte cadence. 0 ok, -1 stop/EOF/stall. */
static int nb_lock_on(navboard_t *nb)
{
    uint8_t   raw[FRAME_LEN];
    nb_frame_t f;
    unsigned  prev = 0;
    int       have = 0, consec = 0, tries = 0;
    while (!nb_stop) {
        if (nb_next_frame(nb, raw) <= 0) return -1;
        if (++tries > 3000) return -1;                   /* ~15 s @200Hz: never locked */
        nb_decode(raw, &f);
        if (!nb_frame_sane(&f)) { consec = 0; have = 0; continue; }
        if (!have) { prev = f.seq; have = 1; consec = 1; continue; }
        unsigned gap = (f.seq - prev) & 0xFFFF;
        prev = f.seq;
        if      (gap == 1) consec++;
        else if (gap == 0) { /* duplicate */ }
        else               consec = 1;
        if (consec >= NB_LOCK_FRAMES) {
            nb->locked = 1; nb->last_seq = f.seq; nb->have_last = 1;
            return 0;
        }
    }
    return -1;
}

/* next VALIDATED frame into *f (seq-continuity + sanity, past false-syncs).
   1 ok, 0 stop/EOF. Assumes nb_lock_on() already ran. */
static int nb_next_valid(navboard_t *nb, nb_frame_t *f)
{
    uint8_t raw[FRAME_LEN];
    int     consec_rej = 0;
    for (;;) {
        if (nb_next_frame(nb, raw) <= 0) return 0;
        nb_decode(raw, f);
        if (!nb_frame_sane(f)) { nb->rejects++; consec_rej++; goto maybe_relock; }
        {
            unsigned gap = (f->seq - nb->last_seq) & 0xFFFF;
            if (gap == 0) continue;                       /* duplicate */
            if (gap <= NB_MAX_SEQ_GAP) {                  /* in sequence -> ACCEPT */
                if (gap > 1) nb->dropped += (gap - 1);
                nb->last_seq = f->seq;
                return 1;
            }
            nb->rejects++; consec_rej++;                  /* bogus seq (false-sync) */
        }
    maybe_relock:
        if (consec_rej >= NB_RELOCK_AFTER) {              /* truly lost lock: re-anchor */
            nb->last_seq = f->seq; nb->relocks++; consec_rej = 0;
        }
    }
}

/* ==================== public API ==================== */

/* open + configure + start acquisition + lock onto the frame cadence.
   Returns 0 ok, -1 on failure. Prints [sync] status to stderr. */
static int navboard_open(navboard_t *nb, const char *dev)
{
    memset(nb, 0, sizeof(*nb));
    nb->fd = nb_open_port(dev ? dev : NAV_DEV_DEFAULT);
    if (nb->fd < 0) return -1;
    nb_start_acquisition(nb->fd);
    fprintf(stderr, "[sync] locking onto the navboard frame cadence...\n"); fflush(stderr);
    if (nb_lock_on(nb) < 0) {
        if (!nb_stop) fprintf(stderr, "[sync] FAILED to lock — is program.elf killed and the stream live?\n");
        return -1;
    }
    fprintf(stderr, "[sync] locked (seq=%u).\n", nb->last_seq);
    return 0;
}

/* measure fresh gyro bias from `secs` of stillness (motion-guarded, up to 3 tries).
   Fills nb->bias (counts). Prints [bias] status to stderr. 0 ok, -1 stop/stall. */
static int navboard_measure_bias(navboard_t *nb, double secs)
{
    if (secs <= 0) secs = 2.0;
    for (int attempt = 1; attempt <= 3 && !nb_stop; attempt++) {
        fprintf(stderr, "[bias] hold STILL — averaging gyro for %.1f s (attempt %d)...\n", secs, attempt);
        fflush(stderr);
        double sum[3] = {0,0,0}, sumsq[3] = {0,0,0};
        long   n = 0;
        double t0 = nb_now_s();
        nb_frame_t f;
        while (!nb_stop && (nb_now_s() - t0) < secs) {
            if (nb_next_valid(nb, &f) <= 0) return -1;
            for (int a = 0; a < 3; a++) { sum[a] += f.raw_g[a]; sumsq[a] += (double)f.raw_g[a]*f.raw_g[a]; }
            n++;
        }
        if (nb_stop) return -1;
        if (n < 10) { fprintf(stderr, "[bias] too few frames (%ld) — stream stalled?\n", n); return -1; }
        int moved = 0; double std_dps[3];
        for (int a = 0; a < 3; a++) {
            double mean = sum[a]/n, var = sumsq[a]/n - mean*mean;
            if (var < 0) var = 0;
            std_dps[a] = __builtin_sqrt(var) * GYRO_GAIN_RADPS[a] * DEG_PER_RAD;
            if (std_dps[a] < 0) std_dps[a] = -std_dps[a];
            nb->bias[a] = mean;
            if (std_dps[a] > NB_MOTION_DPS) moved = 1;
        }
        fprintf(stderr, "[bias] n=%ld  raw counts: gx=%.1f gy=%.1f gz=%.1f   noise °/s: %.2f %.2f %.2f\n",
               n, nb->bias[0], nb->bias[1], nb->bias[2], std_dps[0], std_dps[1], std_dps[2]);
        fprintf(stderr, "[bias]   = °/s bias applied: %+.3f %+.3f %+.3f  "
               "(stored gyros_offset was 35.6/25.0/29.6 — expected to differ, it's volatile)\n",
               nb->bias[0]*GYRO_GAIN_RADPS[0]*DEG_PER_RAD,
               nb->bias[1]*GYRO_GAIN_RADPS[1]*DEG_PER_RAD,
               nb->bias[2]*GYRO_GAIN_RADPS[2]*DEG_PER_RAD);
        if (!moved) { fprintf(stderr, "[bias] OK — still. bias locked.\n"); nb->have_bias = 1; return 0; }
        fprintf(stderr, "[bias] MOTION detected (noise > %.1f °/s) — keep it still, retrying...\n", NB_MOTION_DPS);
    }
    fprintf(stderr, "[bias] giving up after 3 noisy attempts — using last average (may be off).\n");
    nb->have_bias = 1;
    return nb_stop ? -1 : 0;
}

/* read the next VALIDATED sample, converted to physical units (uses nb->bias;
   if bias not measured it's 0 -> raw-scaled rates). 1 ok, 0 stop/EOF. */
static int navboard_read(navboard_t *nb, nav_sample_t *s)
{
    nb_frame_t f;
    if (nb_next_valid(nb, &f) <= 0) return 0;
    s->seq = f.seq;
    for (int i = 0; i < 3; i++) { s->raw_a[i] = f.raw_a[i]; s->raw_g[i] = f.raw_g[i]; }
    s->gx = nb_gyro_dps(f.raw_g[0], nb->bias[0], 0);
    s->gy = nb_gyro_dps(f.raw_g[1], nb->bias[1], 1);
    s->gz = nb_gyro_dps(f.raw_g[2], nb->bias[2], 2);
    double a[3]; nb_accel_g(f.raw_a, a);
    s->ax = a[0]; s->ay = a[1]; s->az = a[2];
    return 1;
}

static void navboard_close(navboard_t *nb)
{
    if (nb->fd >= 0) { close(nb->fd); nb->fd = -1; }
}

#endif /* NAVBOARD_H */
