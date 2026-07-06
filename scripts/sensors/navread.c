/*
 * navread.c — real-time gyro + accel reader for the AR.Drone 2.0 navboard.
 *
 *   Usage:  navread [--hz N] [--calib-secs S] [--bias-only] [--raw] [--csv]
 *                   [--frames N] [--dev /dev/ttyO1]
 *   e.g.:   navread                 # live °/s + g, updates in place, Ctrl-C to stop
 *           navread --bias-only     # just measure & print the fresh gyro bias, exit
 *           navread --csv > /data/video/nav.csv   # log full-rate CSV
 *
 * This is the SENSOR HALF of the control fast loop (docs/control.md, acro-first plan):
 * a real-time C port of scripts/sensors/log_idle.sh's decode, in physical units, with
 * the gyro bias measured FRESH at startup (the stored gyros_offset is volatile — see
 * docs/sensors.md). Read-only: it touches NO motors and kills NO processes, so it is
 * genuinely safe to run props-off. It's the acro inner-loop input (body rates in °/s);
 * accel in g is a sanity channel now, the outer-loop (auto-level) input later.
 *
 * ---- HOW THE NAVBOARD STREAM WORKS (docs/sensors.md) ----
 * Free the navboard first (kill respawner + program.elf — done in the run block, NOT
 * here), then this tool: opens /dev/ttyO1, sends 0x01 to start acquisition, and reads
 * fixed 58-byte frames. Each frame = little-endian uint16 fields:
 *   +0x00 taille(=58=0x3A) +0x02 seq  +0x04 ax +0x06 ay +0x08 az (accel, uint16 counts)
 *   +0x0A gx +0x0C gy +0x0E gz (gyro, int16 counts)   ... (mag/baro/sonar/cksum: not decoded)
 * Frames start 0x3A 0x00; we resync on that header, exactly like log_idle.sh.
 *
 * ---- CALIBRATION (verbatim from backups/config.ini, validated in docs/sensors.md) ----
 * gyro:  rate = (raw - bias) * gyros_gains[axis] [rad/s per count] * (180/pi) -> °/s
 *        gyros_gains = { +1.0589919e-3, -1.0588102e-3, -1.0600387e-3 }  (Y,Z inverted vs X)
 *        BIAS is measured fresh here from stillness; stored gyros_offset is last-boot-only.
 * accel: a_mg = accs_gains(3x3) · raw + accs_offset ; g = a_mg/1000   (factory cal, stable)
 *        Reproduces the documented at-rest ~(-0.015, +0.005, -0.99) g to the digit.
 *        (az reads ≈ -1 g flat: gravity is negative on Z in this frame — expected.)
 *
 * ---- SAFETY ----
 *   Read-only sensor tool. No motor bus, no GPIO, no /dev/mem, no process kills.
 *   Still: run it with props off out of habit while we bring up the control loop.
 */
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

/* Factory gyro gains (rad/s per count) — backups/config.ini "gyros_gains".
 * Signs {+,-,-}: the Y and Z sensor axes are inverted vs body X. Applied AFTER
 * bias subtraction, so at-rest raw≈bias => rate≈0 regardless of sign. */
static const double GYRO_GAIN_RADPS[3] = { 1.0589919e-03, -1.0588102e-03, -1.0600387e-03 };

/* Factory accel cal — backups/config.ini "accs_gains" (3x3 row-major) + "accs_offset" (mg).
 * a_mg[i] = sum_j GAIN[i][j]*raw[j] + OFF[i]. Can't be re-zeroed at rest (gravity present). */
static const double ACC_GAIN[3][3] = {
    {  1.9873557e+00, -5.4023052e-03, -4.5317975e-03 },
    { -1.6823394e-02, -1.9521351e+00, -1.3380921e-02 },
    { -1.8335162e-02,  4.4719707e-03, -1.9520746e+00 },
};
static const double ACC_OFF_MG[3] = { -4.1148950e+03, 4.0706472e+03, 4.0540405e+03 };

static volatile sig_atomic_t stop_requested = 0;
static void on_sigint(int sig) { (void)sig; stop_requested = 1; }

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ============================ serial ============================ */
static int open_navboard(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open navboard"); return -1; }

    struct termios t;
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);                 /* 8N1, no echo, no processing */
        cfsetispeed(&t, NAV_BAUD);     /* set 460800 explicitly (don't depend on prior state) */
        cfsetospeed(&t, NAV_BAUD);
        t.c_cflag |= (CLOCAL | CREAD);
        t.c_cc[VMIN]  = 1;             /* block until >=1 byte */
        t.c_cc[VTIME] = 0;
        tcsetattr(fd, TCSANOW, &t);
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}

/* start acquisition: send 0x01, let it ramp, drop stale bytes so we read fresh frames */
static void start_acquisition(int fd)
{
    uint8_t start = 0x01;
    if (write(fd, &start, 1) != 1) perror("write start byte");
    usleep(200000);
    tcflush(fd, TCIFLUSH);
}

/* ============================ frame parser ============================
 * Byte-stream resync on the 0x3A 0x00 header (same method as log_idle.sh's awk).
 * Returns 1 = frame in out[], 0 = stop/EOF, -1 = error. *resyncs counts dropped junk. */
static uint8_t  g_buf[1024];
static int      g_len = 0;

static int next_frame(int fd, uint8_t out[FRAME_LEN], long *resyncs)
{
    for (;;) {
        int h = -1;                                     /* find frame header */
        for (int i = 0; i + 1 < g_len; i++) {
            if (g_buf[i] == 0x3A && g_buf[i + 1] == 0x00) { h = i; break; }
        }
        if (h > 0) {                                    /* junk before header -> drop (resync) */
            if (resyncs) *resyncs += h;
            memmove(g_buf, g_buf + h, g_len - h);
            g_len -= h;
            h = 0;
        }
        if (h == 0 && g_len >= FRAME_LEN) {             /* full frame available */
            memcpy(out, g_buf, FRAME_LEN);
            memmove(g_buf, g_buf + FRAME_LEN, g_len - FRAME_LEN);
            g_len -= FRAME_LEN;
            return 1;
        }
        if (h < 0) {                                    /* no header; keep a possible trailing 0x3A */
            if (g_len > 0 && g_buf[g_len - 1] == 0x3A) { g_buf[0] = 0x3A; g_len = 1; }
            else g_len = 0;
        }
        /* need more bytes */
        int space = (int)sizeof(g_buf) - g_len;
        if (space <= 0) { g_len = 0; space = (int)sizeof(g_buf); }   /* safety reset */
        ssize_t n = read(fd, g_buf + g_len, space);
        if (n < 0) {
            if (errno == EINTR) { if (stop_requested) return 0; continue; }
            perror("read navboard");
            return -1;
        }
        if (n == 0) return 0;                           /* EOF */
        g_len += n;
    }
}

/* little-endian field extractors at a byte offset */
static int      s16(const uint8_t *b, int o) { int v = b[o] | (b[o + 1] << 8); return (v >= 32768) ? v - 65536 : v; }
static unsigned u16(const uint8_t *b, int o) { return (unsigned)(b[o] | (b[o + 1] << 8)); }

/* decoded one frame */
typedef struct {
    unsigned seq;
    int      raw_a[3];      /* accel counts (unsigned, stored int) */
    int      raw_g[3];      /* gyro counts (signed) */
} frame_t;

static void decode(const uint8_t *b, frame_t *f)
{
    f->seq      = u16(b, 0x02);
    f->raw_a[0] = (int)u16(b, 0x04);
    f->raw_a[1] = (int)u16(b, 0x06);
    f->raw_a[2] = (int)u16(b, 0x08);
    f->raw_g[0] = s16(b, 0x0A);
    f->raw_g[1] = s16(b, 0x0C);
    f->raw_g[2] = s16(b, 0x0E);
}

/* raw gyro counts -> °/s (bias in counts, gain signed) */
static double gyro_dps(int raw, double bias, int axis)
{
    return (raw - bias) * GYRO_GAIN_RADPS[axis] * DEG_PER_RAD;
}

/* raw accel counts (3) -> g (3) via the factory 3x3 + offset */
static void accel_g(const int raw[3], double g[3])
{
    for (int i = 0; i < 3; i++) {
        double mg = ACC_OFF_MG[i];
        for (int j = 0; j < 3; j++) mg += ACC_GAIN[i][j] * raw[j];
        g[i] = mg / 1000.0;
    }
}

/* ============================ bias calibration ============================
 * Average the gyro over `secs` of stillness -> fresh per-axis bias (counts).
 * Guards against motion: if any axis std exceeds MOTION_DPS, warn and retry. */
#define MOTION_DPS 1.5

static int measure_bias(int fd, double secs, double bias_out[3], long *resyncs)
{
    for (int attempt = 1; attempt <= 3 && !stop_requested; attempt++) {
        fprintf(stderr, "[bias] hold STILL — averaging gyro for %.1f s (attempt %d)...\n", secs, attempt);
        fflush(stderr);

        double sum[3] = {0, 0, 0}, sumsq[3] = {0, 0, 0};
        long   n = 0;
        double t0 = now_s();
        uint8_t frame[FRAME_LEN];
        frame_t f;

        while (!stop_requested && (now_s() - t0) < secs) {
            int r = next_frame(fd, frame, resyncs);
            if (r <= 0) return -1;
            decode(frame, &f);
            for (int a = 0; a < 3; a++) {
                sum[a]   += f.raw_g[a];
                sumsq[a] += (double)f.raw_g[a] * f.raw_g[a];
            }
            n++;
        }
        if (stop_requested) return -1;
        if (n < 10) { fprintf(stderr, "[bias] too few frames (%ld) — stream stalled?\n", n); return -1; }

        int moved = 0;
        double std_dps[3];
        for (int a = 0; a < 3; a++) {
            double mean = sum[a] / n;
            double var  = sumsq[a] / n - mean * mean;
            if (var < 0) var = 0;
            double std_counts = 0.0, x = var;      /* sqrt via VFP (hard-float target) */
            std_counts = __builtin_sqrt(x);
            std_dps[a] = std_counts * GYRO_GAIN_RADPS[a] * DEG_PER_RAD;
            if (std_dps[a] < 0) std_dps[a] = -std_dps[a];
            bias_out[a] = mean;
            if (std_dps[a] > MOTION_DPS) moved = 1;
        }

        fprintf(stderr, "[bias] n=%ld  raw counts: gx=%.1f gy=%.1f gz=%.1f   "
               "noise °/s: %.2f %.2f %.2f\n",
               n, bias_out[0], bias_out[1], bias_out[2],
               std_dps[0], std_dps[1], std_dps[2]);
        fprintf(stderr, "[bias]   = °/s bias applied: %+.3f %+.3f %+.3f  "
               "(stored gyros_offset was 35.6/25.0/29.6 — expected to differ, it's volatile)\n",
               bias_out[0] * GYRO_GAIN_RADPS[0] * DEG_PER_RAD,
               bias_out[1] * GYRO_GAIN_RADPS[1] * DEG_PER_RAD,
               bias_out[2] * GYRO_GAIN_RADPS[2] * DEG_PER_RAD);

        if (!moved) { fprintf(stderr, "[bias] OK — still. bias locked.\n"); return 0; }
        fprintf(stderr, "[bias] MOTION detected (noise > %.1f °/s) — keep it still, retrying...\n", MOTION_DPS);
    }
    fprintf(stderr, "[bias] giving up after 3 noisy attempts — using last average (may be off).\n");
    return stop_requested ? -1 : 0;
}

/* ============================ main ============================ */
static void usage(const char *p)
{
    fprintf(stderr,
        "usage: %s [--hz N] [--calib-secs S] [--bias-only] [--raw] [--csv]\n"
        "          [--frames N] [--dev PATH]\n"
        "  --hz N          print rate for the live view (default 15; 0 = every frame)\n"
        "  --calib-secs S  gyro-bias averaging window at startup (default 2.0)\n"
        "  --bias-only     measure & print the fresh gyro bias, then exit\n"
        "  --raw           also show raw ADC counts\n"
        "  --csv           CSV output (t_s,seq,gx,gy,gz,ax,ay,az[,raw...]); implies newline rows\n"
        "  --frames N      stop after N printed samples (0 = run until Ctrl-C)\n"
        "  --dev PATH      navboard device (default %s)\n",
        p, NAV_DEV_DEFAULT);
}

int main(int argc, char **argv)
{
    double      print_hz   = 15.0;
    double      calib_secs = 2.0;
    int         bias_only  = 0, show_raw = 0, csv = 0;
    long        max_frames = 0;
    const char *dev        = NAV_DEV_DEFAULT;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--hz")         && i + 1 < argc) print_hz   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--calib-secs") && i + 1 < argc) calib_secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--frames")     && i + 1 < argc) max_frames = atol(argv[++i]);
        else if (!strcmp(argv[i], "--dev")        && i + 1 < argc) dev        = argv[++i];
        else if (!strcmp(argv[i], "--bias-only"))                  bias_only  = 1;
        else if (!strcmp(argv[i], "--raw"))                        show_raw   = 1;
        else if (!strcmp(argv[i], "--csv"))                        csv        = 1;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(argv[0]); return 0; }
        else { fprintf(stderr, "unknown arg: %s\n", argv[i]); usage(argv[0]); return 2; }
    }
    if (calib_secs <= 0) calib_secs = 2.0;

    struct sigaction sa;                          /* no SA_RESTART: Ctrl-C breaks blocking read */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigint;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int fd = open_navboard(dev);
    if (fd < 0) return 1;
    start_acquisition(fd);

    long resyncs = 0;
    double bias[3];
    if (measure_bias(fd, calib_secs, bias, &resyncs) < 0) {
        if (!stop_requested) fprintf(stderr, "bias measurement failed — is program.elf killed and the stream live?\n");
        close(fd);
        return stop_requested ? 0 : 1;
    }
    if (bias_only) { close(fd); return 0; }

    int decim = (print_hz > 0.0) ? (int)(NAV_HZ / print_hz + 0.5) : 1;
    if (decim < 1) decim = 1;

    if (csv) {
        /* CSV -> stdout only (progress goes to stderr), so `--csv > file` is clean */
        printf("t_s,seq,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g%s\n",
               show_raw ? ",ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw" : "");
        fflush(stdout);
    } else {
        fprintf(stderr, "[live] cols: gyro(x y z) °/s   acc(x y z) g @ ~%.0f Hz (Ctrl-C to stop).\n"
                        "       Still => gyro≈0, az≈-1; twist by hand to watch a spike.\n",
               print_hz > 0 ? print_hz : NAV_HZ);
        fflush(stderr);
    }

    uint8_t  frame[FRAME_LEN];
    frame_t  f;
    long     seen = 0, printed = 0, dropped = 0;
    unsigned last_seq = 0;
    int      have_last = 0;
    double   t_start = now_s();

    while (!stop_requested) {
        int r = next_frame(fd, frame, &resyncs);
        if (r <= 0) break;
        decode(frame, &f);

        if (have_last) {
            unsigned gap = (f.seq - last_seq) & 0xFFFF;   /* seq is uint16, wraps */
            if (gap == 0) continue;                       /* duplicate — skip */
            if (gap > 1) dropped += (gap - 1);
        }
        last_seq = f.seq; have_last = 1;
        seen++;

        if (seen % decim != 0) continue;

        double gx = gyro_dps(f.raw_g[0], bias[0], 0);
        double gy = gyro_dps(f.raw_g[1], bias[1], 1);
        double gz = gyro_dps(f.raw_g[2], bias[2], 2);
        double a[3]; accel_g(f.raw_a, a);
        double t = now_s() - t_start;

        if (csv) {
            printf("%.3f,%u,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f",
                   t, f.seq, gx, gy, gz, a[0], a[1], a[2]);
            if (show_raw)
                printf(",%d,%d,%d,%d,%d,%d",
                       f.raw_a[0], f.raw_a[1], f.raw_a[2], f.raw_g[0], f.raw_g[1], f.raw_g[2]);
            printf("\n");
        } else {
            /* fixed-width, space-padded rows terminated with an EXPLICIT \r\n: the
               BusyBox telnet pty doesn't reliably add the carriage return itself, so
               plain \n staircases. A doubled CR (if the pty does add one) is harmless.
               The %+7.1f gyro field fits ±2000°/s (acro) at a fixed width, so rows
               never smear on the 64-col telnet. */
            printf("gyro %+7.1f %+7.1f %+7.1f   acc %+5.2f %+5.2f %+5.2f",
                   gx, gy, gz, a[0], a[1], a[2]);
            if (show_raw)
                printf("  raw a[%4d %4d %4d] g[%+4d %+4d %+4d]",
                       f.raw_a[0], f.raw_a[1], f.raw_a[2], f.raw_g[0], f.raw_g[1], f.raw_g[2]);
            printf("\r\n");
            fflush(stdout);
        }

        printed++;
        if (max_frames > 0 && printed >= max_frames) break;
    }

    fprintf(stderr, "[done] frames=%ld printed=%ld dropped=%ld resync_bytes=%ld  %.1f s\n",
           seen, printed, dropped, resyncs, now_s() - t_start);
    close(fd);
    return 0;
}
