/*
 * motortest.c — diagnostic battery for the AR.Drone 2.0 motor bus.
 *
 * We've confirmed: GPIO register writes land (DATAOUT tracks), the 0xe0 handshake
 * replies e0 00 on all four BLCs, and program.elf CAN spin the motors — yet our
 * faithful port of the Paparazzi/Hugo init produces no motion. So something in the
 * *sequencing* differs. This tool runs several approaches back-to-back, each clearly
 * announced, so a human can watch and note which test (if any) moves a motor.
 *
 *   Usage:  motortest [motor 1-4]     (default 1 — the slot driven in multicast tests)
 *
 * The main unknowns being probed:
 *   - select-line mode during PWM: drive-LOW (Paparazzi) vs hi-Z (Hugo 1.0) vs drive-HIGH
 *   - the fault flip-flop / motor-power enable on GPIO 175 (held low? long reset?)
 *   - re-sending the 0xa0 enable continuously; max throttle; per-motor addressing
 *
 * PROPS OFF. Each test streams ~200 Hz for a few seconds then stops; a healthy motor
 * self-cuts ~3 s props-off anyway.
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

#define MOTOR_DEV        "/dev/ttyO0"
#define GPIO_MOTOR1      171
#define GPIO_IRQ_FLIPFLOP 175
#define GPIO_IRQ_INPUT   176
#define LOOP_HZ          200

/* select-line modes during the PWM run */
enum { SEL_LOW, SEL_HIZ, SEL_HIGH, SEL_PERMOTOR };
/* flip-flop (gpio175) handling */
enum { FF_NORMAL, FF_LONG, FF_HOLD_LOW };

static int mot_fd = -1;
static volatile sig_atomic_t stop_requested = 0;

/* ===================== GPIO via /dev/mem (proven working) ===================== */
static const uint32_t gpio_base[6] = {
    0x48310000, 0x49050000, 0x49052000, 0x49054000, 0x49056000, 0x49058000
};
static volatile uint32_t *gpio_bank[6] = {0};
static int devmem_fd = -1;
#define R_OE      (0x034 / 4)
#define R_DATAIN  (0x038 / 4)
#define R_DATAOUT (0x03C / 4)
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
static int  gpio_read(int pin)  { volatile uint32_t *g = gpio_map(pin); return g ? (int)((g[R_DATAIN] >> (pin % 32)) & 1) : -1; }

/* ===================== serial ===================== */
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
static int read_reply(int fd, uint8_t *buf, int len, int ms)
{
    memset(buf, 0xFF, len); int got = 0;
    while (got < len) {
        fd_set r; FD_ZERO(&r); FD_SET(fd, &r);
        struct timeval tv = { ms / 1000, (ms % 1000) * 1000 };
        if (select(fd + 1, &r, NULL, NULL, &tv) <= 0) break;
        ssize_t n = read(fd, buf + got, len - got);
        if (n <= 0) break;
        got += n;
    }
    return got;
}
static void motor_cmd(uint8_t cmd, int read_len)
{
    uint8_t reply[8];
    full_write(mot_fd, &cmd, 1);
    if (read_len > 0) read_reply(mot_fd, reply, read_len, 100);
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

/* ===================== test harness ===================== */
typedef struct {
    const char *name;
    int select_mode;   /* SEL_* during the PWM run     */
    int ff_mode;       /* FF_* flip-flop handling      */
    int read_replies;  /* read handshake replies?      */
    int resend_enable; /* re-send 0xa0 every frame?    */
    int pwm;           /* steady hold PWM              */
    int kick;          /* startup kick PWM             */
} test_t;

static void flipflop_prep(int ff_mode)
{
    gpio_out(GPIO_IRQ_FLIPFLOP);
    gpio_clear(GPIO_IRQ_FLIPFLOP);
    if (ff_mode == FF_LONG) usleep(500000); else usleep(20000);
    if (ff_mode != FF_HOLD_LOW) gpio_set(GPIO_IRQ_FLIPFLOP);   /* else leave it low during run */
}

/* handshake + enable, then leave select lines in the requested run-mode */
static void motor_init(const test_t *t, int target)
{
    gpio_in(GPIO_IRQ_INPUT);
    flipflop_prep(t->ff_mode);

    /* per-motor config (always drive select low to address each, like both refs) */
    for (int m = 0; m < 4; m++) { gpio_out(GPIO_MOTOR1 + m); gpio_set(GPIO_MOTOR1 + m); }
    tcflush(mot_fd, TCIFLUSH);
    for (int m = 0; m < 4; m++) {
        gpio_clear(GPIO_MOTOR1 + m);
        motor_cmd(0xe0, t->read_replies ? 2 : 0);
        motor_cmd((uint8_t)(m + 1), t->read_replies ? 1 : 0);
        gpio_set(GPIO_MOTOR1 + m);
    }

    /* set select lines to the RUN mode being tested */
    for (int m = 0; m < 4; m++) {
        int pin = GPIO_MOTOR1 + m;
        switch (t->select_mode) {
            case SEL_HIZ:      gpio_in(pin); break;
            case SEL_HIGH:     gpio_out(pin); gpio_set(pin); break;
            case SEL_PERMOTOR: gpio_out(pin); if (m == target) gpio_clear(pin); else gpio_set(pin); break;
            default:           gpio_out(pin); gpio_clear(pin); break;   /* SEL_LOW */
        }
    }

    /* enable: 5x multicast 0xa0 */
    for (int i = 0; i < 5; i++) motor_cmd(0xa0, t->read_replies ? 1 : 0);

    if (t->ff_mode != FF_HOLD_LOW) { gpio_clear(GPIO_IRQ_FLIPFLOP); gpio_set(GPIO_IRQ_FLIPFLOP); }
}

static void run_test(const test_t *t, int idx, int target, double secs)
{
    uint16_t v[4] = {0, 0, 0, 0};
    printf("\n===== TEST %d: %s =====\n", idx, t->name);
    printf("  select=%s flipflop=%s reads=%d resend_enable=%d kick=%d hold=%d motor=%d\n",
           (t->select_mode == SEL_LOW ? "drive-LOW" : t->select_mode == SEL_HIZ ? "hi-Z" :
            t->select_mode == SEL_HIGH ? "drive-HIGH" : "per-motor"),
           (t->ff_mode == FF_NORMAL ? "normal" : t->ff_mode == FF_LONG ? "long-reset" : "hold-LOW"),
           t->read_replies, t->resend_enable, t->kick, t->pwm, target);
    printf("  >>> WATCH NOW <<< (starts in 1s)\n");
    fflush(stdout);
    sleep(1);
    if (stop_requested) return;

    motor_init(t, target);

    /* startup kick (~300 ms) then steady hold */
    v[target] = t->kick;
    for (int i = 0; i < LOOP_HZ * 3 / 10 && !stop_requested; i++) {
        set_pwm(v[0], v[1], v[2], v[3]);
        if (t->resend_enable) motor_cmd(0xa0, 0);
        usleep(1000000 / LOOP_HZ);
    }
    v[target] = t->pwm;
    long total = (long)(secs * LOOP_HZ);
    for (long i = 0; i < total && !stop_requested; i++) {
        set_pwm(v[0], v[1], v[2], v[3]);
        if (t->resend_enable) motor_cmd(0xa0, 0);
        usleep(1000000 / LOOP_HZ);
        if (i % (LOOP_HZ / 2) == 0)
            printf("    t=%4.1fs pwm=%d fault176=%d(unreliable)\n", (double)i / LOOP_HZ, t->pwm, gpio_read(GPIO_IRQ_INPUT));
    }

    /* stop + park selects inactive */
    for (int i = 0; i < 5; i++) { set_pwm(0, 0, 0, 0); usleep(5000); }
    for (int m = 0; m < 4; m++) { gpio_out(GPIO_MOTOR1 + m); gpio_set(GPIO_MOTOR1 + m); }
    printf("  --- end test %d ---\n", idx);
    fflush(stdout);
    sleep(2);   /* gap so you can tell tests apart */
}

static void on_sigint(int sig) { (void)sig; stop_requested = 1; }

int main(int argc, char **argv)
{
    int target = (argc > 1) ? atoi(argv[1]) - 1 : 0;
    if (target < 0 || target > 3) { fprintf(stderr, "motor must be 1-4\n"); return 2; }

    /* the battery: each entry is one approach, run for ~3.5 s */
    static const test_t tests[] = {
        { "baseline (Paparazzi: select drive-low)", SEL_LOW,      FF_NORMAL,   1, 0, 200, 255 },
        { "Hugo-style: select hi-Z, no reply reads", SEL_HIZ,     FF_NORMAL,   0, 0, 200, 255 },
        { "select drive-HIGH during run",            SEL_HIGH,    FF_NORMAL,   1, 0, 200, 255 },
        { "flip-flop HELD LOW during run",           SEL_LOW,     FF_HOLD_LOW, 1, 0, 200, 255 },
        { "long flip-flop reset (500ms low)",        SEL_LOW,     FF_LONG,     1, 0, 200, 255 },
        { "re-send 0xa0 enable every frame",         SEL_LOW,     FF_NORMAL,   1, 1, 200, 255 },
        { "per-motor addressing (only this select)", SEL_PERMOTOR,FF_NORMAL,   1, 1, 200, 255 },
        { "MAX throttle 511 + resend enable",        SEL_LOW,     FF_NORMAL,   1, 1, 511, 511 },
    };
    int n = sizeof(tests) / sizeof(tests[0]);

    printf("=== motortest: %d approaches, driving motor %d. PROPS OFF. Ctrl-C to abort. ===\n", n, target + 1);
    printf("Watch/listen the whole time and note which TEST number (if any) moves a motor.\n");
    signal(SIGINT, on_sigint);

    mot_fd = open_motor_port();
    if (mot_fd < 0) return 1;

    for (int i = 0; i < n && !stop_requested; i++)
        run_test(&tests[i], i + 1, target, 3.5);

    for (int i = 0; i < 5; i++) { set_pwm(0, 0, 0, 0); usleep(5000); }
    close(mot_fd);
    for (int b = 0; b < 6; b++) if (gpio_bank[b]) munmap((void *)gpio_bank[b], 0x1000);
    if (devmem_fd >= 0) close(devmem_fd);
    printf("\n=== all tests done ===\n");
    return 0;
}
