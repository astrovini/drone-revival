/*
 * ratebench.c — PRINT-ONLY acro rate-loop bench tool (props-off, no motors).
 *
 *   Usage:  ratebench [--kp K] [--ki K] [--throttle T] [--hz N]
 *                     [--calib-secs S] [--dev /dev/ttyO1]
 *
 * The full inner acro chain — navboard gyro → rate PID → Hugo X-quad mixer —
 * with the motor drive DISABLED: it only PRINTS the 4 motor commands. Rotate
 * the drone by hand and watch the numbers respond, to confirm the controller's
 * SIGNS and the mixer math are right BEFORE anything ever spins under closed
 * loop. This is the safe bridge from navread (sensor half, done) to the real
 * flight loop — a sign error caught here is a wrong number on screen, not a
 * crash. (Physical which-motor-is-which-corner comes from the slot→corner map;
 * see docs/control.md — until then this validates the chain + relative signs.)
 *
 * Setpoint = 0 on the bench, so the rate PID is a RATE DAMPER: at rest all four
 * motors sit at the nominal throttle; rotate an axis and the opposing motors
 * split apart. Body-axis map (validated, docs/sensors.md): roll=gx, pitch=gy,
 * yaw=gz. Hugo X-quad mixer (docs/control.md):
 *   m0 = T +roll −pitch +yaw    m1 = T −roll −pitch −yaw
 *   m2 = T −roll +pitch +yaw    m3 = T +roll +pitch −yaw
 *
 * SAFETY: reads /dev/ttyO1 (navboard) ONLY. It NEVER opens /dev/ttyO0, never
 * touches GPIO/​/dev/mem, never drives a motor. As safe as navread. Props off.
 */
#include "../sensors/navboard.h"

#define PWM_MAX     511.0       /* motor command range (matches motorspin) */
#define I_LIMIT     200.0       /* integrator anti-windup clamp (per axis) */

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

int main(int argc, char **argv)
{
    double      kp = 0.40, ki = 0.0;   /* display/scaling gains for the bench (real tune later) */
    double      throttle = 150.0;      /* nominal motor level to sit at (headroom both ways) */
    double      print_hz = 15.0;
    double      calib_secs = 2.0;
    int         csv = 0;
    const char *dev = NAV_DEV_DEFAULT;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--kp")         && i + 1 < argc) kp         = atof(argv[++i]);
        else if (!strcmp(argv[i], "--ki")         && i + 1 < argc) ki         = atof(argv[++i]);
        else if (!strcmp(argv[i], "--throttle")   && i + 1 < argc) throttle   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--hz")         && i + 1 < argc) print_hz   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--calib-secs") && i + 1 < argc) calib_secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dev")        && i + 1 < argc) dev        = argv[++i];
        else if (!strcmp(argv[i], "--csv"))                        csv        = 1;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            fprintf(stderr, "usage: %s [--kp K] [--ki K] [--throttle T] [--hz N] [--csv]\n"
                            "          [--calib-secs S] [--dev PATH]\n"
                            "  PRINT-ONLY: reads the navboard, prints motor commands, drives NOTHING.\n"
                            "  default: live in-place gauge.  --csv: clean rows to stdout for capture.\n", argv[0]);
            return 0;
        }
        else { fprintf(stderr, "unknown arg: %s\n", argv[i]); return 2; }
    }

    navboard_install_signals();

    navboard_t nb;
    if (navboard_open(&nb, dev) < 0) { navboard_close(&nb); return nb_stop ? 0 : 1; }
    if (navboard_measure_bias(&nb, calib_secs) < 0) {
        if (!nb_stop) fprintf(stderr, "bias measurement failed — stream stalled?\n");
        navboard_close(&nb); return nb_stop ? 0 : 1;
    }

    fprintf(stderr,
        "[bench] PRINT-ONLY rate loop — NO motors driven, props-off safe.\n"
        "        setpoint=0 (rate damper): still => all motors ~%.0f; rotate an axis and watch them split.\n"
        "        gains Kp=%.2f Ki=%.2f  throttle=%.0f   cols: roll=gx pitch=gy yaw=gz, m=[m0 m1 m2 m3]\n"
        "        Ctrl-C to stop.\n",
        throttle, kp, ki, throttle);
    fflush(stderr);

    if (csv) {   /* clean rows -> stdout (status is on stderr), so `--csv > file` is clean */
        printf("t_s,seq,gx_dps,gy_dps,gz_dps,cmd_roll,cmd_pitch,cmd_yaw,m0,m1,m2,m3\n");
        fflush(stdout);
    }

    int decim = (print_hz > 0.0) ? (int)(NAV_HZ / print_hz + 0.5) : 1;
    if (decim < 1) decim = 1;

    double integ[3] = {0, 0, 0};       /* roll, pitch, yaw integrators */
    nav_sample_t s;
    long   seen = 0, printed = 0;
    double t_start = nb_now_s(), t_prev = t_start;

    while (!nb_stop) {
        if (!navboard_read(&nb, &s)) break;

        double now = nb_now_s();
        double dt  = now - t_prev;
        if (dt <= 0 || dt > 0.1) dt = 1.0 / NAV_HZ;   /* guard first sample / stalls */
        t_prev = now;

        /* body rates -> axes (validated map: roll=gx, pitch=gy, yaw=gz) */
        double rate[3] = { s.gx, s.gy, s.gz };
        double cmd[3];
        for (int a = 0; a < 3; a++) {
            double err = 0.0 - rate[a];               /* setpoint 0 on the bench */
            integ[a]  = clampd(integ[a] + err * dt, -I_LIMIT, I_LIMIT);
            cmd[a]    = kp * err + ki * integ[a];     /* Paparazzi rate law (P + clamped I) */
        }
        double roll = cmd[0], pitch = cmd[1], yaw = cmd[2];

        /* Hugo X-quad mixer */
        double m[4];
        m[0] = throttle + roll - pitch + yaw;
        m[1] = throttle - roll - pitch - yaw;
        m[2] = throttle - roll + pitch + yaw;
        m[3] = throttle + roll + pitch - yaw;
        for (int i = 0; i < 4; i++) m[i] = clampd(m[i], 0.0, PWM_MAX);

        seen++;
        if (seen % decim != 0) continue;

        if (csv) {
            printf("%.3f,%u,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.0f,%.0f,%.0f,%.0f\n",
                   now - t_start, s.seq, rate[0], rate[1], rate[2],
                   roll, pitch, yaw, m[0], m[1], m[2], m[3]);
        } else {
            /* in-place gauge: leading \r overwrites ONE fixed-width line — no \n, so it
               can't staircase on the flaky telnet pty; trailing spaces clear any residue */
            printf("\rroll%+6.0f pitch%+6.0f yaw%+6.0f | m %3.0f %3.0f %3.0f %3.0f   ",
                   rate[0], rate[1], rate[2], m[0], m[1], m[2], m[3]);
            fflush(stdout);
        }
        printed++;
    }

    if (!csv) printf("\n");   /* end the in-place gauge line before the summary */
    fprintf(stderr, "[done] frames=%ld printed=%ld  gate: dropped=%ld rejected=%ld relocks=%ld  %.1f s\n",
           seen, printed, nb.dropped, nb.rejects, nb.relocks, nb_now_s() - t_start);
    navboard_close(&nb);
    return 0;
}
