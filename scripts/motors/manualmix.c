/*
 * manualmix.c — arm-to-idle + live stick mixing for the AR.Drone 2.0 (PROPS-OFF BENCH TEST).
 *
 *   Runs ON THE DRONE. Kill program.elf first (see run line), then this listens on UDP for
 *   stick packets from the Mac (scripts/control/tango_mix.py) and drives /dev/ttyO0 directly:
 *     arm  -> all four motors idle at a low constant PWM
 *     then throttle/roll/pitch/yaw sticks modulate the four motors via the quad mixer.
 *
 *   ⚠ THIS HAS NO STABILIZATION (no gyro, no PID) — it is NOT flyable. A quad with no attitude
 *     loop flips instantly. This is a **props-off bench test** of arm+idle and the stick->motor
 *     mixing (and a handy way to map which motor is which corner). Real flight needs the rate
 *     controller — see docs/control.md.
 *
 *   Reuses motorspin.c's proven motor driver (GPIO via /dev/mem, hi-Z select during run, the
 *   0xe0/0xa0 handshake, 5-byte PWM frame). Mixer signs from Hugo's ardrone/ardrone fly/controlthread.c.
 *
 *   Because our loop holds the motors continuously, the ~3 s no-load cutout no longer protects us,
 *   so the safety lives here: rising-edge arm with throttle low, an EMERGENCY stop, and a comms
 *   watchdog that cuts the motors if stick packets stop arriving.
 *
 *   UDP packet (ASCII, ~50 Hz):  "M <arm> <emerg> <thr01> <roll> <pitch> <yaw>\n"
 *     arm,emerg in {0,1}; thr01 in [0,1]; roll/pitch/yaw in [-1,1].
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>

/* ---- hardware (from motorspin.c / Paparazzi 2.0) ---- */
#define MOTOR_DEV        "/dev/ttyO0"
#define GPIO_MOTOR1      171
#define GPIO_IRQ_FLIPFLOP 175
#define GPIO_IRQ_INPUT   176
#define LOOP_HZ          200

/* ---- mixing / limits (bench-safe: capped low, tune as needed) ---- */
#define IDLE_PWM         90      /* all four sit here when armed, sticks centered */
#define MAX_PWM          220     /* bench cap (full scale is 511) — keep it tame */
#define MIX_GAIN         50      /* max per-axis differential from a full stick */
#define UDP_PORT         5560
#define FAILSAFE_MS      500     /* no stick packet for this long -> cut motors */

static int mot_fd = -1;
static volatile sig_atomic_t stop_requested = 0;

/* ===================== GPIO via /dev/mem (from motorspin) ===================== */
static const uint32_t gpio_base[6] = {
    0x48310000, 0x49050000, 0x49052000, 0x49054000, 0x49056000, 0x49058000
};
static volatile uint32_t *gpio_bank[6] = {0};
static int devmem_fd = -1;
#define R_OE      (0x034 / 4)
#define R_CLEAR   (0x090 / 4)
#define R_SET     (0x094 / 4)

static volatile uint32_t *gpio_map(int pin)
{
    int b = pin / 32;
    if (b < 0 || b > 5) return NULL;
    if (!gpio_bank[b]) {
        if (devmem_fd < 0) {
            devmem_fd = open("/dev/mem", O_RDWR | O_SYNC);
            if (devmem_fd < 0) { perror("open /dev/mem"); return NULL; }
        }
        void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, devmem_fd, gpio_base[b]);
        if (m == MAP_FAILED) { perror("mmap gpio bank"); return NULL; }
        gpio_bank[b] = (volatile uint32_t *)m;
    }
    return gpio_bank[b];
}
static void gpio_out(int pin)   { volatile uint32_t *g = gpio_map(pin); if (g) g[R_OE] &= ~(1u << (pin % 32)); }
static void gpio_in(int pin)    { volatile uint32_t *g = gpio_map(pin); if (g) g[R_OE] |=  (1u << (pin % 32)); }
static void gpio_set(int pin)   { volatile uint32_t *g = gpio_map(pin); if (g) g[R_SET]   = (1u << (pin % 32)); }
static void gpio_clear(int pin) { volatile uint32_t *g = gpio_map(pin); if (g) g[R_CLEAR] = (1u << (pin % 32)); }

/* ===================== serial + motor protocol (from motorspin) ===================== */
static int open_motor_port(void)
{
    int fd = open(MOTOR_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) { perror("open " MOTOR_DEV); return -1; }
    fcntl(fd, F_SETFL, 0);
    struct termios o;
    tcgetattr(fd, &o);
    cfsetispeed(&o, B115200);
    cfsetospeed(&o, B115200);
    o.c_cflag |= (CLOCAL | CREAD);
    o.c_iflag = 0; o.c_lflag = 0; o.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &o);
    tcflush(fd, TCIOFLUSH);
    return fd;
}
static void full_write(int fd, const void *buf, int len)
{
    const uint8_t *p = buf; int left = len;
    while (left > 0) { ssize_t n = write(fd, p, left); if (n < 0) { if (errno == EINTR) continue; return; } p += n; left -= n; }
}
static void motor_cmd(uint8_t cmd, int read_len)
{
    uint8_t reply[8];
    full_write(mot_fd, &cmd, 1);
    if (read_len > 0) {
        memset(reply, 0xFF, sizeof reply);
        fd_set r; FD_ZERO(&r); FD_SET(mot_fd, &r);
        struct timeval tv = {0, 100000};
        if (select(mot_fd + 1, &r, NULL, NULL, &tv) > 0) read(mot_fd, reply, read_len);
    }
}
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
static void motor_init(void)
{
    gpio_in(GPIO_IRQ_INPUT);
    gpio_out(GPIO_IRQ_FLIPFLOP); gpio_clear(GPIO_IRQ_FLIPFLOP); usleep(20000); gpio_set(GPIO_IRQ_FLIPFLOP);
    for (int m = 0; m < 4; m++) { gpio_out(GPIO_MOTOR1 + m); gpio_set(GPIO_MOTOR1 + m); }
    tcflush(mot_fd, TCIFLUSH);
    for (int m = 0; m < 4; m++) {
        gpio_clear(GPIO_MOTOR1 + m);
        motor_cmd(0xe0, 2);
        motor_cmd((uint8_t)(m + 1), 1);
        gpio_set(GPIO_MOTOR1 + m);
    }
    /* hi-Z select for the multicast run — the verified-2026-07-03 fix */
    for (int m = 0; m < 4; m++) gpio_in(GPIO_MOTOR1 + m);
    for (int i = 0; i < 5; i++) motor_cmd(0xa0, 1);
    gpio_clear(GPIO_IRQ_FLIPFLOP); gpio_set(GPIO_IRQ_FLIPFLOP);
}
static void all_stop(void) { for (int i = 0; i < 5; i++) { set_pwm(0, 0, 0, 0); usleep(5000); } }
static void cleanup(void)
{
    if (mot_fd >= 0) { all_stop(); close(mot_fd); mot_fd = -1; }
    for (int b = 0; b < 6; b++) if (gpio_bank[b]) { munmap((void *)gpio_bank[b], 0x1000); gpio_bank[b] = NULL; }
    if (devmem_fd >= 0) { close(devmem_fd); devmem_fd = -1; }
}
static void on_sigint(int s) { (void)s; stop_requested = 1; }

/* ===================== helpers ===================== */
static long now_ms(void)
{
    struct timeval tv; gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
static uint16_t clamp_pwm(double v)
{
    if (v < 0) return 0;
    if (v > MAX_PWM) return MAX_PWM;
    return (uint16_t)(v + 0.5);
}

int main(void)
{
    printf("=== manualmix: arm-to-idle + stick mixing (PROPS OFF — NO stabilization, NOT flyable) ===\n");
    printf("Listening on UDP %d for stick packets from the Mac (tango_mix.py).\n", UDP_PORT);
    printf("Idle=%d  max=%d  mix_gain=%d  failsafe=%dms\n", IDLE_PWM, MAX_PWM, MIX_GAIN, FAILSAFE_MS);
    signal(SIGINT, on_sigint);

    /* UDP socket */
    int us = socket(AF_INET, SOCK_DGRAM, 0);
    if (us < 0) { perror("socket"); return 1; }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_ANY); addr.sin_port = htons(UDP_PORT);
    if (bind(us, (struct sockaddr *)&addr, sizeof addr) < 0) { perror("bind"); return 1; }
    fcntl(us, F_SETFL, O_NONBLOCK);

    mot_fd = open_motor_port();
    if (mot_fd < 0) { close(us); return 1; }
    motor_init();

    /* control state */
    int    armed = 0, prev_arm = 1;   /* prev_arm=1 -> arm must go low then high to engage */
    double thr = 0, roll = 0, pitch = 0, yaw = 0;
    int    arm_pkt = 0, emerg_pkt = 0;
    long   last_rx = 0;               /* 0 => nothing received yet */
    int    tick = 0;
    const char *state = "DISARMED";

    while (!stop_requested) {
        /* drain the socket to the freshest packet */
        char buf[128];
        struct sockaddr_in from; socklen_t fl = sizeof from;
        int got = 0;
        for (;;) {
            ssize_t n = recvfrom(us, buf, sizeof buf - 1, 0, (struct sockaddr *)&from, &fl);
            if (n <= 0) break;
            buf[n] = 0;
            int a, e; double t, r, p, y;
            if (sscanf(buf, "M %d %d %lf %lf %lf %lf", &a, &e, &t, &r, &p, &y) == 6) {
                arm_pkt = a; emerg_pkt = e; thr = t; roll = r; pitch = p; yaw = y;
                got = 1;
            }
        }
        if (got) last_rx = now_ms();

        int stale = (last_rx == 0) || (now_ms() - last_rx > FAILSAFE_MS);

        /* arm/disarm logic: rising-edge arm with throttle low; emergency/stale/loss cut it */
        if (emerg_pkt || stale) {
            armed = 0;
            state = emerg_pkt ? "EMERGENCY" : "NO-LINK";
        } else {
            if (!prev_arm && arm_pkt && thr < 0.10) { armed = 1; }
            if (!arm_pkt) armed = 0;
            state = armed ? "ARMED" : "DISARMED";
        }
        prev_arm = stale ? 1 : arm_pkt;   /* while no link, force a fresh low->high to re-arm */

        /* mix + drive */
        uint16_t m0, m1, m2, m3;
        if (!armed) {
            m0 = m1 = m2 = m3 = 0;
        } else {
            double base = IDLE_PWM + thr * (MAX_PWM - IDLE_PWM);
            double r = roll * MIX_GAIN, p = pitch * MIX_GAIN, yw = yaw * MIX_GAIN;
            /* Hugo ardrone mixer (slot->corner mapping still TBD — this tool helps reveal it) */
            m0 = clamp_pwm(base + r - p + yw);
            m1 = clamp_pwm(base - r - p - yw);
            m2 = clamp_pwm(base - r + p + yw);
            m3 = clamp_pwm(base + r + p - yw);
        }
        set_pwm(m0, m1, m2, m3);

        if (++tick % (LOOP_HZ / 4) == 0) {   /* ~4 Hz status */
            printf("\r[%-9s] thr=%.2f r%+.2f p%+.2f y%+.2f  M[%3d %3d %3d %3d]   ",
                   state, thr, roll, pitch, yaw, m0, m1, m2, m3);
            fflush(stdout);
        }
        usleep(1000000 / LOOP_HZ);
    }

    printf("\nstopping.\n");
    cleanup();
    close(us);
    return 0;
}
