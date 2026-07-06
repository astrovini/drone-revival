/*
 * navread.c — real-time gyro + accel reader for the AR.Drone 2.0 navboard (CLI).
 *
 *   Usage:  navread [--hz N] [--calib-secs S] [--bias-only] [--raw] [--csv]
 *                   [--frames N] [--dev /dev/ttyO1]
 *   e.g.:   navread                 # live °/s + g, updates each row, Ctrl-C to stop
 *           navread --bias-only     # measure & print the fresh gyro bias, exit
 *           navread --csv > /data/video/nav.csv   # log full-rate CSV
 *
 * Thin front-end over navboard.h (the shared IMU reader: frame protocol, factory
 * calibration, fresh startup gyro-bias, and the frame-validation/lock-on gate).
 * The sensor half of the control fast loop (docs/control.md). Read-only: touches
 * NO motors and kills NO processes — safe props-off. Body rates in °/s are the
 * acro inner-loop input; accel in g is a sanity channel now, outer-loop later.
 *
 * On-drone validated 2026-07-06 (see docs/sensors.md): still→gyro≈0/az≈−1g;
 * hand-rotation lights the right axis; the gate drops the non-unique-0x3A00
 * false-sync (0 impossible frames over a gated capture).
 *
 * SAFETY: read-only sensor tool. No motor bus, no GPIO, no /dev/mem, no kills.
 */
#include "navboard.h"

static void usage(const char *p)
{
    fprintf(stderr,
        "usage: %s [--hz N] [--calib-secs S] [--bias-only] [--raw] [--csv]\n"
        "          [--frames N] [--dev PATH]\n"
        "  --hz N          print rate for the live view (default 15; 0 = every frame)\n"
        "  --calib-secs S  gyro-bias averaging window at startup (default 2.0)\n"
        "  --bias-only     measure & print the fresh gyro bias, then exit\n"
        "  --raw           also show raw ADC counts\n"
        "  --csv           CSV output (t_s,seq,gx,gy,gz,ax,ay,az[,raw...]); newline rows\n"
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

    navboard_install_signals();

    navboard_t nb;
    if (navboard_open(&nb, dev) < 0) { navboard_close(&nb); return nb_stop ? 0 : 1; }

    if (navboard_measure_bias(&nb, calib_secs) < 0) {
        if (!nb_stop) fprintf(stderr, "bias measurement failed — stream stalled?\n");
        navboard_close(&nb);
        return nb_stop ? 0 : 1;
    }
    if (bias_only) {
        fprintf(stderr, "[gate] rejected %ld bad frame(s), %ld relock(s) during bias.\n", nb.rejects, nb.relocks);
        navboard_close(&nb);
        return 0;
    }

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

    nav_sample_t s;
    long   seen = 0, printed = 0;
    double t_start = nb_now_s();

    while (!nb_stop) {
        if (!navboard_read(&nb, &s)) break;             /* gated: seq + sanity */
        seen++;
        if (seen % decim != 0) continue;

        double t = nb_now_s() - t_start;
        if (csv) {
            printf("%.3f,%u,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f",
                   t, s.seq, s.gx, s.gy, s.gz, s.ax, s.ay, s.az);
            if (show_raw)
                printf(",%d,%d,%d,%d,%d,%d",
                       s.raw_a[0], s.raw_a[1], s.raw_a[2], s.raw_g[0], s.raw_g[1], s.raw_g[2]);
            printf("\n");
        } else {
            /* fixed-width, space-padded rows with an EXPLICIT \r\n: the BusyBox telnet
               pty doesn't reliably add the carriage return, so plain \n staircases.
               The %+7.1f gyro field fits ±2000°/s (acro) at fixed width -> no smear. */
            printf("gyro %+7.1f %+7.1f %+7.1f   acc %+5.2f %+5.2f %+5.2f",
                   s.gx, s.gy, s.gz, s.ax, s.ay, s.az);
            if (show_raw)
                printf("  raw a[%4d %4d %4d] g[%+4d %+4d %+4d]",
                       s.raw_a[0], s.raw_a[1], s.raw_a[2], s.raw_g[0], s.raw_g[1], s.raw_g[2]);
            printf("\r\n");
            fflush(stdout);
        }

        printed++;
        if (max_frames > 0 && printed >= max_frames) break;
    }

    fprintf(stderr, "[done] frames=%ld printed=%ld dropped=%ld  gate: rejected=%ld relocks=%ld  "
           "resync_bytes=%ld  %.1f s\n",
           seen, printed, nb.dropped, nb.rejects, nb.relocks, nb.resyncs, nb_now_s() - t_start);
    navboard_close(&nb);
    return 0;
}
