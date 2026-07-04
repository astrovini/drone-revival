/*
 * motorspin.c — hold ONE AR.Drone 2.0 motor at a fixed PWM (props-off bench test).
 *
 *   Usage:  motorspin <motor 1-4> <pwm 0-511> <seconds>
 *   e.g.:   motorspin 1 96 3        # motor 1 at PWM 96 for 3 s
 *
 * Purpose: the definitive motor check the stock firmware can't do — a steady-state
 * constant-PWM spin with the flight controller out of the loop. Props off, the BLC
 * self-cuts after ~3 s (no-load detect), which is our natural safety limit.
 *
 * ---- SOURCES (this is a port, not new protocol work) ----
 * Protocol/handshake/PWM-packing/GPIO numbers copied verbatim from Paparazzi UAV's
 * AR.Drone *2.0* driver: sw/airborne/boards/ardrone/actuators.c (Dino Hensen 2012,
 * from Hugo Perquin's original). Verified 2.0-correct: /dev/ttyO0, GPIO 171-176.
 * (Hugo's ardrone/ardrone repo is AR.Drone 1.0 — wrong tty + GPIOs; do not use.)
 *
 * GPIO is done by poking the OMAP3630 GPIO registers directly via /dev/mem —
 * the stock firmware claims 171-176 on the kernel side, so /sys/class/gpio "export"
 * refuses them (confirmed on-drone 2026-07-03). Direct register access is what
 * program.elf itself uses. See docs/motors.md for the full 1.0-vs-2.0 diff.
 *
 * ---- SAFETY ----
 *   - PROPS OFF, drone secured, healthy battery / >=10 A supply.
 *   - Kill program.elf's respawner THEN program.elf before running (it owns the bus).
 *   - Single-motor: only the requested motor's PWM slot is non-zero; others held at 0.
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
#include <sys/select.h>
#include <sys/mman.h>

/* ---- hardware constants (AR.Drone 2.0, from Paparazzi actuators.c) ---- */
#define MOTOR_DEV        "/dev/ttyO0"
#define GPIO_MOTOR1      171          /* motor select lines: 171..174 */
#define GPIO_IRQ_FLIPFLOP 175         /* toggle low->high to reset the fault flip-flop */
#define GPIO_IRQ_INPUT   176          /* motor-fault input; reads 1 after a fault/cutout */

#define PWM_MAX          0x1ff        /* 9-bit */
#define STARTUP_PWM      0xFF         /* "need 0xff for reliable startup" (per the BLC source) */
#define STARTUP_MS       400
#define LOOP_HZ          200          /* frame refresh rate (every 5 ms) */
#define MAX_SECONDS      60.0         /* defensive cap against a typo */

static int mot_fd = -1;
static volatile sig_atomic_t stop_requested = 0;

/* ============ GPIO (direct OMAP3 register access via /dev/mem) ============
 * All of 171-176 live in GPIO bank 6 (gpios 160-191) @ 0x49058000. bank6 is
 * already clocked (gpio177 is in use on the drone), so register access is safe. */
static const uint32_t gpio_base[6] = {
    0x48310000, 0x49050000, 0x49052000, 0x49054000, 0x49056000, 0x49058000
};
static volatile uint32_t *gpio_bank[6] = {0};
static int devmem_fd = -1;

#define R_OE      (0x034 / 4)   /* output enable: 0=output, 1=input */
#define R_DATAIN  (0x038 / 4)   /* actual pad level (may read 0 on output-only pads) */
#define R_DATAOUT (0x03C / 4)   /* driven output latch (reflects our writes directly) */
#define R_CLEAR   (0x090 / 4)   /* write (1<<bit) to drive the pin low  */
#define R_SET     (0x094 / 4)   /* write (1<<bit) to drive the pin high */

static volatile uint32_t *gpio_map(int pin)
{
    int b = pin / 32;
    if (b < 0 || b > 5) return NULL;
    if (!gpio_bank[b]) {
        if (devmem_fd < 0) {
            devmem_fd = open("/dev/mem", O_RDWR | O_SYNC);
            if (devmem_fd < 0) { perror("open /dev/mem"); return NULL; }
        }
        void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED,
                       devmem_fd, gpio_base[b]);
        if (m == MAP_FAILED) { perror("mmap gpio bank"); return NULL; }
        gpio_bank[b] = (volatile uint32_t *)m;
    }
    return gpio_bank[b];
}

static int gpio_setup_output(int pin)
{
    volatile uint32_t *g = gpio_map(pin);
    if (!g) return -1;
    g[R_OE] &= ~(1u << (pin % 32));   /* read-modify-write: touch only our bit */
    return 0;
}
static int gpio_setup_input(int pin)
{
    volatile uint32_t *g = gpio_map(pin);
    if (!g) return -1;
    g[R_OE] |= (1u << (pin % 32));
    return 0;
}
static void gpio_set(int pin)   { volatile uint32_t *g = gpio_map(pin); if (g) g[R_SET]   = (1u << (pin % 32)); }
static void gpio_clear(int pin) { volatile uint32_t *g = gpio_map(pin); if (g) g[R_CLEAR] = (1u << (pin % 32)); }
static int  gpio_read(int pin)  { volatile uint32_t *g = gpio_map(pin); return g ? (int)((g[R_DATAIN] >> (pin % 32)) & 1) : -1; }

/* ============================ serial ============================ */
static int open_motor_port(void)
{
    int fd = open(MOTOR_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) { perror("open " MOTOR_DEV); return -1; }
    fcntl(fd, F_SETFL, 0);   /* blocking reads (we gate with select) */

    struct termios o;
    tcgetattr(fd, &o);
    cfsetispeed(&o, B115200);
    cfsetospeed(&o, B115200);
    o.c_cflag |= (CLOCAL | CREAD);
    o.c_iflag = 0;
    o.c_lflag = 0;
    o.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &o);
    tcflush(fd, TCIOFLUSH);   /* drop any stale bytes (e.g. a previous run's shutdown frames) */
    return fd;
}

static int full_write(int fd, const void *buf, int len)
{
    const uint8_t *p = buf;
    int left = len;
    while (left > 0) {
        ssize_t n = write(fd, p, left);
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        p += n; left -= n;
    }
    return len;
}

/* read exactly len bytes, but give up after ~timeout_ms (so we never hang) */
static int read_reply(int fd, uint8_t *buf, int len, int timeout_ms)
{
    memset(buf, 0xFF, len);           /* 0xFF => "no reply" sentinel */
    int got = 0;
    while (got < len) {
        fd_set r; FD_ZERO(&r); FD_SET(fd, &r);
        struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
        int s = select(fd + 1, &r, NULL, NULL, &tv);
        if (s <= 0) break;            /* timeout or error */
        ssize_t n = read(fd, buf + got, len - got);
        if (n <= 0) break;
        got += n;
    }
    return got;
}

/* write one command byte, read replylen bytes back */
static int motor_cmd(uint8_t cmd, uint8_t *reply, int replylen)
{
    if (full_write(mot_fd, &cmd, 1) < 0) return -1;
    return read_reply(mot_fd, reply, replylen, 200);
}

/* 5-byte frame, four packed 9-bit PWM values (Paparazzi actuators_ardrone_set_pwm) */
static void set_pwm(uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3)
{
    uint8_t c[5];
    c[0] = 0x20 | ((p0 & 0x1ff) >> 4);
    c[1] = ((p0 & 0x1ff) << 4) | ((p1 & 0x1ff) >> 5);
    c[2] = ((p1 & 0x1ff) << 3) | ((p2 & 0x1ff) >> 6);
    c[3] = ((p2 & 0x1ff) << 2) | ((p3 & 0x1ff) >> 7);
    c[4] = ((p3 & 0x1ff) << 1);
    full_write(mot_fd, c, 5);
}

/* all-green LEDs (ardrone2 format) — a visible "handshake worked" indicator */
static void set_leds_green(void)
{
    uint8_t g = 2;                    /* 2 = green */
    uint8_t c[2];
    c[0] = 0x60 | ((g & 1) << 4) | ((g & 1) << 3) | ((g & 1) << 2) | ((g & 1) << 1);
    c[1] = ((g & 2) << 3) | ((g & 2) << 2) | ((g & 2) << 1) | ((g & 2) << 0);
    full_write(mot_fd, c, 2);
}

/* ============================ init ============================ */
static int motor_init(int verbose)
{
    if (verbose) printf("[init] mapping GPIO 171-176 via /dev/mem (OMAP bank 6 @ 0x49058000)...\n");
    if (gpio_setup_input(GPIO_IRQ_INPUT) < 0) {
        fprintf(stderr, "[init] FAILED to map GPIO registers via /dev/mem.\n"
                        "       Need root and an accessible /dev/mem (ls -l /dev/mem).\n");
        return -1;
    }

    /* reset the IRQ flip-flop: 175 low, brief pulse, then high */
    gpio_setup_output(GPIO_IRQ_FLIPFLOP);
    gpio_clear(GPIO_IRQ_FLIPFLOP);
    usleep(20000);
    gpio_set(GPIO_IRQ_FLIPFLOP);

    /* select lines: outputs, all high (inactive) */
    for (int m = 0; m < 4; m++) { gpio_setup_output(GPIO_MOTOR1 + m); gpio_set(GPIO_MOTOR1 + m); }

    /* self-test: does our /dev/mem write reach the GPIO registers?
       DATAOUT = the output latch we drive (true test of register writes);
       DATAIN  = actual pad level (may stay 0 on an output pad — informational only). */
    volatile uint32_t *g6 = gpio_map(GPIO_MOTOR1);
    if (verbose && g6) {
        printf("[init] bank6 before: OE=0x%08x DATAOUT=0x%08x DATAIN=0x%08x\n",
               g6[R_OE], g6[R_DATAOUT], g6[R_DATAIN]);
        for (int m = 0; m < 4; m++) {
            int bit = (GPIO_MOTOR1 + m) % 32;
            gpio_set(GPIO_MOTOR1 + m);
            int so = (g6[R_DATAOUT] >> bit) & 1, si = (g6[R_DATAIN] >> bit) & 1;
            gpio_clear(GPIO_MOTOR1 + m);
            int co = (g6[R_DATAOUT] >> bit) & 1, ci = (g6[R_DATAIN] >> bit) & 1;
            gpio_set(GPIO_MOTOR1 + m);   /* leave inactive (high) */
            printf("  gpio%d(bit%2d): set[OUT=%d IN=%d] clear[OUT=%d IN=%d]  %s\n",
                   GPIO_MOTOR1 + m, bit, so, si, co, ci,
                   (so == 1 && co == 0) ? "regwrite OK" : "regwrite FAIL");
        }
        printf("[init] bank6 after:  OE=0x%08x DATAOUT=0x%08x DATAIN=0x%08x\n",
               g6[R_OE], g6[R_DATAOUT], g6[R_DATAIN]);
    }

    /* per-motor config: select (low), send 0xe0 (expect e0 00), send id, deselect (high) */
    tcflush(mot_fd, TCIFLUSH);   /* clean RX before reading handshake replies */
    int ok = 1;
    uint8_t reply[8];
    for (int m = 0; m < 4; m++) {
        gpio_clear(GPIO_MOTOR1 + m);
        motor_cmd(0xe0, reply, 2);
        if (verbose)
            printf("[init] motor%d handshake: reply=0x%02x 0x%02x %s\n",
                   m + 1, reply[0], reply[1],
                   (reply[0] == 0xe0 && reply[1] == 0x00) ? "OK" : "(unexpected/no-reply)");
        if (reply[0] != 0xe0 || reply[1] != 0x00) ok = 0;
        motor_cmd((uint8_t)(m + 1), reply, 1);
        gpio_set(GPIO_MOTOR1 + m);
    }

    /* De-select all motors by releasing the select lines to hi-Z (input) for the multicast run.
       VERIFIED 2026-07-03 (motortest.c): driving these LOW — as Paparazzi's actuators.c does —
       NEVER spins on this board; hi-Z (Hugo's 1.0 method) spins with a green status LED. */
    for (int m = 0; m < 4; m++) gpio_setup_input(GPIO_MOTOR1 + m);

    /* enable: 5x multicast 0xa0 */
    for (int i = 0; i < 5; i++) motor_cmd(0xa0, reply, 1);

    /* reset flip-flop again */
    gpio_clear(GPIO_IRQ_FLIPFLOP);
    gpio_set(GPIO_IRQ_FLIPFLOP);

    set_leds_green();
    if (verbose)
        printf("[init] done%s.\n", ok ? "" : "  (WARNING: a handshake was unexpected — see above)");
    return 0;
}

static void all_stop(void)
{
    for (int i = 0; i < 5; i++) { set_pwm(0, 0, 0, 0); usleep(5000); }
}

static void cleanup(void)
{
    if (mot_fd >= 0) { all_stop(); close(mot_fd); mot_fd = -1; }
    for (int b = 0; b < 6; b++)
        if (gpio_bank[b]) { munmap((void *)gpio_bank[b], 0x1000); gpio_bank[b] = NULL; }
    if (devmem_fd >= 0) { close(devmem_fd); devmem_fd = -1; }
}

static void on_sigint(int sig) { (void)sig; stop_requested = 1; }

/* Sweep mode: step PWM up level-by-level to find the minimum that spins the motor from rest.
   NO startup kick (so it measures the true start-from-rest threshold), and a fresh quiet
   re-enable per level so the ~3 s no-load cutout never carries between levels. The human
   watches/listens and notes the lowest level that produces motion. */
static int do_sweep(int motor, int from, int to, int step, double dwell)
{
    int slot = motor - 1;
    printf("=== motorspin SWEEP: motor %d, PWM %d..%d step %d (%.1f s/level, NO kick) ===\n",
           motor, from, to, step, dwell);
    printf(">>> PROPS OFF. Note the lowest PWM at which the motor starts turning. <<<\n");
    signal(SIGINT, on_sigint);

    for (int i = 3; i > 0 && !stop_requested; i--) { printf("  %d...\n", i); sleep(1); }
    if (stop_requested) { printf("aborted.\n"); return 0; }

    mot_fd = open_motor_port();
    if (mot_fd < 0) return 1;

    for (int pwm = from; pwm <= to && !stop_requested; pwm += step) {
        if (motor_init(0) < 0) { cleanup(); return 1; }   /* fresh, quiet re-enable */
        uint16_t v[4] = {0, 0, 0, 0};
        v[slot] = (uint16_t)pwm;
        printf("PWM %3d  >>> watch <<<\n", pwm);
        fflush(stdout);
        long total = (long)(dwell * LOOP_HZ);
        for (long i = 0; i < total && !stop_requested; i++) {
            set_pwm(v[0], v[1], v[2], v[3]);
            usleep(1000000 / LOOP_HZ);
        }
        all_stop();
        usleep(1000000);                                  /* ~1 s stop so each level starts from rest */
    }
    cleanup();
    printf("=== sweep done ===\n");
    return 0;
}

/* ============================ main ============================ */
int main(int argc, char **argv)
{
    /* sweep mode: motorspin sweep <motor 1-4> [from] [to] [step] [sec-per-level] */
    if (argc >= 3 && strcmp(argv[1], "sweep") == 0) {
        int    motor = atoi(argv[2]);
        int    from  = (argc > 3) ? atoi(argv[3]) : 40;
        int    to    = (argc > 4) ? atoi(argv[4]) : 220;
        int    step  = (argc > 5) ? atoi(argv[5]) : 15;
        double dwell = (argc > 6) ? atof(argv[6]) : 5.0;
        if (motor < 1 || motor > 4) { fprintf(stderr, "motor must be 1-4\n"); return 2; }
        if (from < 0) from = 0;
        if (to > PWM_MAX) to = PWM_MAX;
        if (step < 1) step = 1;
        if (dwell <= 0) dwell = 5.0;
        return do_sweep(motor, from, to, step, dwell);
    }

    if (argc != 4) {
        fprintf(stderr, "usage: %s <motor 1-4|all> <pwm 0-%d> <seconds>\n"
                        "       %s sweep <motor 1-4> [from] [to] [step] [sec-per-level]\n",
                argv[0], PWM_MAX, argv[0]);
        return 2;
    }
    int    all_motors = (strcmp(argv[1], "all") == 0);   /* drive all 4 slots at once */
    int    motor = all_motors ? 0 : atoi(argv[1]);
    int    pwm   = atoi(argv[2]);
    double secs  = atof(argv[3]);

    if (!all_motors && (motor < 1 || motor > 4)) { fprintf(stderr, "motor must be 1-4 (or 'all')\n"); return 2; }
    if (pwm < 0 || pwm > PWM_MAX) { fprintf(stderr, "pwm must be 0-%d\n", PWM_MAX); return 2; }
    if (secs <= 0) { fprintf(stderr, "seconds must be > 0\n"); return 2; }
    if (secs > MAX_SECONDS) { secs = MAX_SECONDS; printf("(capped to %.0f s)\n", secs); }

    int slot = motor - 1;             /* which slot to drive (unused when all_motors) */

    if (all_motors) printf("=== motorspin: ALL 4 motors @ PWM %d for %.2f s ===\n", pwm, secs);
    else            printf("=== motorspin: motor %d @ PWM %d for %.2f s ===\n", motor, pwm, secs);
    printf(">>> PROPS OFF. Motor(s) spin in 3 s — Ctrl-C to abort. <<<\n");

    signal(SIGINT, on_sigint);

    /* Safety countdown FIRST — and critically, BEFORE motor-enable. The BLCs disarm on a
       PWM-command timeout, so any silent gap between enabling them and the first frame leaves
       them dead. That was the no-spin bug: a 3 s countdown sat between init and the PWM loop.
       Now we enable and immediately start streaming, with no gap. */
    for (int i = 3; i > 0 && !stop_requested; i--) { printf("  %d...\n", i); sleep(1); }
    if (stop_requested) { printf("aborted.\n"); return 0; }

    mot_fd = open_motor_port();
    if (mot_fd < 0) return 1;
    if (motor_init(1) < 0) { cleanup(); return 1; }

    /* build a 4-slot PWM vector with only our motor non-zero; stream immediately after enable */
    uint16_t v[4] = {0, 0, 0, 0};

    /* startup kick */
    printf("[run] startup kick @ %d for %d ms\n", STARTUP_PWM, STARTUP_MS);
    if (all_motors) { v[0] = v[1] = v[2] = v[3] = STARTUP_PWM; } else v[slot] = STARTUP_PWM;
    for (int i = 0; i < STARTUP_MS / (1000 / LOOP_HZ) && !stop_requested; i++) {
        set_pwm(v[0], v[1], v[2], v[3]); usleep(1000000 / LOOP_HZ);
    }

    /* steady-state hold at the requested PWM */
    printf("[run] holding PWM %d ...\n", pwm);
    if (all_motors) { v[0] = v[1] = v[2] = v[3] = (uint16_t)pwm; } else v[slot] = (uint16_t)pwm;
    long total = (long)(secs * LOOP_HZ);
    int fault_reported = 0;
    for (long i = 0; i < total && !stop_requested; i++) {
        set_pwm(v[0], v[1], v[2], v[3]);
        usleep(1000000 / LOOP_HZ);
        if (i % (LOOP_HZ / 4) == 0) {              /* ~4x/sec status */
            int fault = gpio_read(GPIO_IRQ_INPUT);
            printf("  t=%5.2fs  pwm=%d  fault(gpio176)=%d\n",
                   (double)i / LOOP_HZ, pwm, fault);
            if (fault == 1 && !fault_reported) {
                printf("  >>> fault/cutout latched (expected props-off ~3 s no-load cut). stopping.\n");
                fault_reported = 1;
                break;
            }
        }
    }

    printf("[run] stopping.\n");
    cleanup();
    printf("done.\n");
    return 0;
}
