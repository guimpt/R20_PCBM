#include "cli.h"
#include "main.h"
#include "tusb.h"
#include "pid.h"
#include "mct8316.h"
#define CLI_BUF_SIZE 64

// Externs from main.c
extern PID posPID;
extern PID velPID;
extern ctrl_mode_t ctrl_mode;
extern volatile int32_t hall_count;
extern volatile float hall_speed;
extern volatile float vel_filtered;
extern volatile int32_t vel_prev_count;
extern int32_t action_k;
extern int32_t action_km1;
extern MCT8316 mct8316;
extern uint8_t enable_flag;

static char cli_buf[CLI_BUF_SIZE];
static uint8_t cli_buf_idx = 0;

static volatile uint8_t cli_monitor_active = 0;
static volatile uint16_t cli_monitor_period = 0;
static volatile uint16_t cli_monitor_counter = 0;
static volatile uint8_t cli_monitor_flag = 0;

#define CLI_TX_BUF_SIZE 256
static char cli_tx_buf[CLI_TX_BUF_SIZE];
static uint16_t cli_tx_len = 0;
static uint16_t cli_tx_idx = 0;

static void cli_flush(void);
static void cli_puts(const char *s);
static void cli_putc(char c);
static void cli_puti(int32_t val);
static void cli_putu(uint32_t val);
static void cli_putf(float val, uint8_t decimals);
static float cli_atof(const char *s);
static int32_t cli_atoi(const char *s);
static int cli_strcasecmp(const char *a, const char *b);
static void CLI_Execute(char *line);
static void CLI_PrintStatus(void);
static void cli_print_faults(uint8_t ic_status);

static void cmd_help(int argc, char **argv);
static void cmd_status(int argc, char **argv);
static void cmd_enable(int argc, char **argv);
static void cmd_disable(int argc, char **argv);
static void cmd_clear(int argc, char **argv);
static void cmd_pos(int argc, char **argv);
static void cmd_home(int argc, char **argv);
static void cmd_kp(int argc, char **argv);
static void cmd_ki(int argc, char **argv);
static void cmd_kd(int argc, char **argv);
static void cmd_pid(int argc, char **argv);
static void cmd_umax(int argc, char **argv);
static void cmd_umin(int argc, char **argv);
static void cmd_freq(int argc, char **argv);
static void cmd_mode(int argc, char **argv);
static void cmd_vel(int argc, char **argv);
static void cmd_brake(int argc, char **argv);
static void cmd_monitor(int argc, char **argv);
static PID *active_pid(void);

typedef struct {
    const char *name;
    void (*handler)(int argc, char **argv);
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    {"help",    cmd_help},    {"?",  cmd_help},
    {"status",  cmd_status},  {"s",  cmd_status},
    {"enable",  cmd_enable},  {"e",  cmd_enable},
    {"disable", cmd_disable}, {"d",  cmd_disable},
    {"clear",   cmd_clear},
    {"pos",     cmd_pos},     {"p",  cmd_pos},
    {"home",    cmd_home},    {"h",  cmd_home},
    {"kp",      cmd_kp},
    {"ki",      cmd_ki},
    {"kd",      cmd_kd},
    {"pid",     cmd_pid},
    {"umax",    cmd_umax},
    {"umin",    cmd_umin},
    {"freq",    cmd_freq},
    {"mode",    cmd_mode},
    {"vel",     cmd_vel},     {"v",  cmd_vel},
    {"brake",   cmd_brake},
    {"monitor", cmd_monitor}, {"m",  cmd_monitor},
};

static void cli_puts(const char *s)
{
    while (*s) {
        if (cli_tx_len >= CLI_TX_BUF_SIZE) {
            tud_task();
            cli_flush();
        }
        if (cli_tx_len < CLI_TX_BUF_SIZE) {
            cli_tx_buf[cli_tx_len++] = *s++;
        } else {
            s++;
        }
    }
}

static void cli_flush(void)
{
    if (cli_tx_idx >= cli_tx_len) return;

    uint32_t n = tud_cdc_write(cli_tx_buf + cli_tx_idx, cli_tx_len - cli_tx_idx);
    cli_tx_idx += (uint16_t)n;
    tud_cdc_write_flush();

    if (cli_tx_idx >= cli_tx_len) {
        cli_tx_len = 0;
        cli_tx_idx = 0;
    } else if (cli_tx_idx > 0) {
        uint16_t rem = cli_tx_len - cli_tx_idx;
        for (uint16_t i = 0; i < rem; i++)
            cli_tx_buf[i] = cli_tx_buf[cli_tx_idx + i];
        cli_tx_len = rem;
        cli_tx_idx = 0;
    }
}

static void cli_putc(char c)
{
    if (cli_tx_len < CLI_TX_BUF_SIZE) {
        cli_tx_buf[cli_tx_len++] = c;
    }
    cli_flush();
}

static void cli_puti(int32_t val)
{
    char buf[12];
    int i = 0;
    uint8_t neg = 0;
    if (val < 0) {
        neg = 1;
        val = -val;
    }
    do {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);
    if (neg) buf[i++] = '-';
    buf[i] = '\0';
    for (int j = 0; j < i / 2; j++) {
        char t = buf[j];
        buf[j] = buf[i - 1 - j];
        buf[i - 1 - j] = t;
    }
    cli_puts(buf);
}

static void cli_putu(uint32_t val)
{
    char buf[12];
    int i = 0;
    do {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);
    buf[i] = '\0';
    for (int j = 0; j < i / 2; j++) {
        char t = buf[j];
        buf[j] = buf[i - 1 - j];
        buf[i - 1 - j] = t;
    }
    cli_puts(buf);
}


static void cli_putf(float val, uint8_t decimals)
{
    if (val < 0) {
        cli_puts("-");
        val = -val;
    }
    int32_t int_part = (int32_t)val;
    cli_puti(int_part);
    if (decimals > 0) {
        cli_puts(".");
        float frac = val - int_part;
        int32_t pow10 = 1;
        for (uint8_t i = 0; i < decimals; i++) pow10 *= 10;
        int32_t frac_int = (int32_t)(frac * pow10 + 0.5f);
        if (frac_int >= pow10) frac_int = pow10 - 1;
        int32_t div = pow10 / 10;
        while (div > 0 && frac_int < div) {
            cli_puts("0");
            div /= 10;
        }
        if (frac_int > 0) cli_puti(frac_int);
    }
}

static float cli_atof(const char *s)
{
    float val = 0.0f;
    float sign = 1.0f;
    float div = 1.0f;

    while (*s == ' ') s++;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') s++;

    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            div *= 10.0f;
            val += (*s - '0') / div;
            s++;
        }
    }
    return sign * val;
}

static int32_t cli_atoi(const char *s)
{
    int32_t val = 0;
    int32_t sign = 1;
    while (*s == ' ') s++;
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') {
        val = val * 10 + (*s - '0');
        s++;
    }
    return sign * val;
}

static int cli_strcasecmp(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a;
        char cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca += ('a' - 'A');
        if (cb >= 'A' && cb <= 'Z') cb += ('a' - 'A');
        if (ca != cb) return ca - cb;
        a++; b++;
    }
    char ca = *a;
    char cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca += ('a' - 'A');
    if (cb >= 'A' && cb <= 'Z') cb += ('a' - 'A');
    return ca - cb;
}

void CLI_Init(void)
{
    cli_buf_idx = 0;
    cli_monitor_active = 0;
    cli_monitor_period = 0;
    cli_monitor_counter = 0;
    cli_monitor_flag = 0;
}

void CLI_Process(void)
{
    cli_flush();

    while (tud_cdc_available()) {
        char c;
        if (tud_cdc_read(&c, 1) == 0) break;

        if (c == '\b' || c == 0x7F) {
            if (cli_buf_idx > 0) {
                cli_buf_idx--;
                cli_puts("\b \b");
                cli_flush();
            }
            continue;
        }

        if (c == '\r' || c == '\n') {
            if (c == '\n' && cli_buf_idx == 0) continue;
            cli_buf[cli_buf_idx] = '\0';
            cli_puts("\r\n");
            if (cli_buf_idx > 0) {
                CLI_Execute(cli_buf);
            }
            if (!cli_monitor_active) {
                cli_puts("> ");
            }
            cli_flush();
            cli_buf_idx = 0;
            continue;
        }

        if (cli_buf_idx < CLI_BUF_SIZE - 1) {
            cli_buf[cli_buf_idx++] = c;
            cli_putc(c);
        } else {
            cli_puts("\r\noverflow\r\n");
            if (!cli_monitor_active) {
                cli_puts("> ");
            }
            cli_flush();
            cli_buf_idx = 0;
        }
    }

    if (cli_monitor_flag) {
        cli_monitor_flag = 0;
        CLI_PrintStatus();
        cli_flush();
    }
}

void CLI_Tick(void)
{
    if (cli_monitor_active && cli_monitor_counter) {
        cli_monitor_counter--;
        if (cli_monitor_counter == 0) {
            cli_monitor_flag = 1;
            cli_monitor_counter = cli_monitor_period;
        }
    }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)rts;
    if (dtr) {
        cli_puts("\r\nR20 PCBM v1.0 (? for help)\r\n> ");
        cli_flush();
    }
}

static void CLI_Execute(char *line)
{
    char *argv[4];
    int argc = 0;
    char *p = line;

    while (*p == ' ') p++;
    while (*p && argc < 4) {
        argv[argc++] = p;
        while (*p && *p != ' ') p++;
        if (*p == ' ') {
            *p = '\0';
            p++;
            while (*p == ' ') p++;
        }
    }
    if (argc == 0) return;

    for (size_t i = 0; i < sizeof(cmd_table) / sizeof(cmd_table[0]); i++) {
        if (cli_strcasecmp(argv[0], cmd_table[i].name) == 0) {
            cmd_table[i].handler(argc, argv);
            return;
        }
    }
    cli_puts("unknown: ");
    cli_puts(argv[0]);
    cli_puts(" (? for help)\r\n");
}

static void cli_print_faults(uint8_t ic_status)
{
    if (ic_status == 0) {
        cli_puts("none");
        return;
    }
    uint8_t first = 1;
    static const struct { uint8_t bit; const char *name; } faults[] = {
        {MCT8316_IC_STATUS_MTR_LOCK, "LOCK"},
        {MCT8316_IC_STATUS_BK_FLT,  "BUCK"},
        {MCT8316_IC_STATUS_SPI_FLT, "SPI"},
        {MCT8316_IC_STATUS_OCP,     "OCP"},
        {MCT8316_IC_STATUS_NPOR,    "POR"},
        {MCT8316_IC_STATUS_OVP,     "OVP"},
        {MCT8316_IC_STATUS_OT,      "OT"},
        {MCT8316_IC_STATUS_FAULT,   "FLT"},
    };
    for (uint8_t i = 0; i < sizeof(faults)/sizeof(faults[0]); i++) {
        if (ic_status & faults[i].bit) {
            if (!first) cli_puts(" ");
            cli_puts(faults[i].name);
            first = 0;
        }
    }
}

static void CLI_PrintStatus(void)
{
    if (ctrl_mode == MODE_POSITION) {
        cli_puts("pos:");
        cli_puti((int32_t)hall_count);
        cli_puts(" set:");
        cli_puti((int32_t)(posPID.x_k / (int32_t)posPID.multiplier));
    } else {
        cli_puts("vel:");
        cli_putf(vel_filtered / (float)HALL_COUNTS_PER_REV, 1);
        cli_puts(" set:");
        cli_putf((float)(velPID.x_k / (int32_t)velPID.multiplier) / (float)HALL_COUNTS_PER_REV, 1);
    }
    cli_puts(" spd:");
    cli_putf(hall_speed, 1);
    cli_puts(" u:");
    cli_puti((int32_t)action_k);
    cli_puts(" pwm:");
    cli_putu((uint32_t)TIM2->CCR4);
    cli_puts(" flt:");
    cli_print_faults(mct8316.ic_status);
    cli_puts("\r\n");
}

static void cmd_help(int argc, char **argv)
{
    (void)argc; (void)argv;
    cli_puts(
        "status  (s)  System state\r\n"
        "enable  (e)  Start motor\r\n"
        "disable (d)  Stop motor\r\n"
        "clear        Clear faults\r\n"
        "mode pos|vel Control mode\r\n"
        "pos [n] (p)  Position setpoint\r\n"
        "vel [n] (v)  Velocity setpoint (rev/s)\r\n"
        "home    (h)  Zero encoder\r\n"
        "kp/ki/kd [v] PID gains\r\n"
        "pid          All PID params\r\n"
        "umax/umin[v] Output limits\r\n"
        "freq [v]     Filter cutoff Hz\r\n"
        "brake on|off Brake control\r\n"
        "monitor (m)  <ms>|off\r\n"
    );
}

static void cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    cli_puts("pos:  ");
    cli_puti((int32_t)hall_count);
    if (ctrl_mode == MODE_POSITION) {
        cli_puts("  setpt: ");
        cli_puti((int32_t)(posPID.x_k / (int32_t)posPID.multiplier));
    } else {
        cli_puts("  vel_sp: ");
        cli_putf((float)(velPID.x_k / (int32_t)velPID.multiplier) / (float)HALL_COUNTS_PER_REV, 1);
        cli_puts("  vel_filt: ");
        cli_putf(vel_filtered / (float)HALL_COUNTS_PER_REV, 1);
    }
    cli_puts("\r\nspd:  ");
    cli_putf(hall_speed, 1);
    cli_puts("  pwm: ");
    cli_putu((uint32_t)TIM2->CCR4);
    cli_puts("  u: ");
    cli_puti((int32_t)action_k);
    cli_puts("\r\nmode: ");
    cli_puts(ctrl_mode == MODE_POSITION ? "pos" : "vel");
    cli_puts("  state: ");
    cli_puts(enable_flag ? "ENABLED" : "DISABLED");
    cli_puts("  fault: ");
    cli_print_faults(mct8316.ic_status);
    cli_puts("\r\n");
}

static void cmd_enable(int argc, char **argv)
{
    (void)argc; (void)argv;
    enable_flag = 1;
    cli_puts("enabled\r\n");
}

static void cmd_disable(int argc, char **argv)
{
    (void)argc; (void)argv;
    enable_flag = 0;
    TIM2->CCR4 = 0;
    cli_puts("disabled\r\n");
}

static void cmd_clear(int argc, char **argv)
{
    (void)argc; (void)argv;
    MCT8316_ClearFaults(&mct8316);
    cli_puts("faults cleared\r\n");
}

static void cmd_pos(int argc, char **argv)
{
    if (ctrl_mode != MODE_POSITION) {
        cli_puts("err: not in position mode (use: mode pos)\r\n");
        return;
    }
    if (argc >= 2) {
        int32_t target = cli_atoi(argv[1]);
        posPID.x_k = target * (int32_t)posPID.multiplier;
    }
    cli_puts("pos = ");
    cli_puti((int32_t)(posPID.x_k / (int32_t)posPID.multiplier));
    cli_puts("\r\n");
}

static void cmd_home(int argc, char **argv)
{
    (void)argc; (void)argv;
    __disable_irq();
    hall_count = 0;
    posPID.x_k = 0;
    __enable_irq();
    cli_puts("ok\r\n");
}

static PID *active_pid(void)
{
    return (ctrl_mode == MODE_VELOCITY) ? &velPID : &posPID;
}

static void cmd_mode(int argc, char **argv)
{
    if (argc >= 2) {
        if (cli_strcasecmp(argv[1], "pos") == 0) {
            if (ctrl_mode != MODE_POSITION) {
                TIM2->CCR4 = 0;
                action_k = 0;
                action_km1 = 0;
                ctrl_mode = MODE_POSITION;
                PID_Initialize(&posPID);
                posPID.x_k = hall_count * posPID.multiplier;
            }
            cli_puts("mode: position\r\n");
        } else if (cli_strcasecmp(argv[1], "vel") == 0) {
            if (ctrl_mode != MODE_VELOCITY) {
                TIM2->CCR4 = 0;
                action_k = 0;
                action_km1 = 0;
                ctrl_mode = MODE_VELOCITY;
                vel_prev_count = hall_count;
                vel_filtered = 0;
                PID_Initialize(&velPID);
            }
            cli_puts("mode: velocity\r\n");
        } else {
            cli_puts("usage: mode pos|vel\r\n");
        }
    } else {
        cli_puts("mode: ");
        cli_puts(ctrl_mode == MODE_POSITION ? "position" : "velocity");
        cli_puts("\r\n");
    }
}

static void cmd_vel(int argc, char **argv)
{
    if (ctrl_mode != MODE_VELOCITY) {
        cli_puts("err: not in velocity mode (use: mode vel)\r\n");
        return;
    }
    if (argc >= 2) {
        float target = cli_atof(argv[1]);
        velPID.x_k = (int32_t)(target * (float)HALL_COUNTS_PER_REV * (float)velPID.multiplier);
    }
    cli_puts("vel = ");
    cli_putf((float)(velPID.x_k / (int32_t)velPID.multiplier) / (float)HALL_COUNTS_PER_REV, 1);
    cli_puts(" rev/s\r\n");
}

static void cmd_kp(int argc, char **argv)
{
    PID *pid = active_pid();
    if (argc >= 2) {
        pid->kp = cli_atof(argv[1]);
        PID_UpdateCoefficients(pid);
    }
    cli_puts("kp = ");
    cli_putf(pid->kp, 4);
    cli_puts("\r\n");
}

static void cmd_ki(int argc, char **argv)
{
    PID *pid = active_pid();
    if (argc >= 2) {
        pid->ki = cli_atof(argv[1]);
        PID_UpdateCoefficients(pid);
    }
    cli_puts("ki = ");
    cli_putf(pid->ki, 4);
    cli_puts("\r\n");
}

static void cmd_kd(int argc, char **argv)
{
    PID *pid = active_pid();
    if (argc >= 2) {
        pid->kd = cli_atof(argv[1]);
        PID_UpdateCoefficients(pid);
    }
    cli_puts("kd = ");
    cli_putf(pid->kd, 4);
    cli_puts("\r\n");
}

static void cmd_pid(int argc, char **argv)
{
    PID *pid = active_pid();
    (void)argc; (void)argv;
    cli_puts(ctrl_mode == MODE_POSITION ? "pos" : "vel");
    cli_puts(" pid\r\n");
    cli_puts("kp   = "); cli_putf(pid->kp, 4); cli_puts("\r\n");
    cli_puts("ki   = "); cli_putf(pid->ki, 4); cli_puts("\r\n");
    cli_puts("kd   = "); cli_putf(pid->kd, 4); cli_puts("\r\n");
    cli_puts("umax = "); cli_putf(pid->Umax, 1); cli_puts("\r\n");
    cli_puts("umin = "); cli_putf(pid->Umin, 1); cli_puts("\r\n");
    cli_puts("freq = "); cli_putf(pid->w_cutoff, 1); cli_puts(" Hz\r\n");
}

static void cmd_umax(int argc, char **argv)
{
    PID *pid = active_pid();
    if (argc >= 2) {
        pid->Umax = cli_atof(argv[1]);
        PID_UpdateCoefficients(pid);
    }
    cli_puts("umax = ");
    cli_putf(pid->Umax, 1);
    cli_puts("\r\n");
}

static void cmd_umin(int argc, char **argv)
{
    PID *pid = active_pid();
    if (argc >= 2) {
        pid->Umin = cli_atof(argv[1]);
        PID_UpdateCoefficients(pid);
    }
    cli_puts("umin = ");
    cli_putf(pid->Umin, 1);
    cli_puts("\r\n");
}

static void cmd_freq(int argc, char **argv)
{
    PID *pid = active_pid();
    if (argc >= 2) {
        float val = cli_atof(argv[1]);
        if (val <= 0) {
            cli_puts("err: freq must be > 0\r\n");
            return;
        }
        pid->w_cutoff = val;
        PID_UpdateCoefficients(pid);
    }
    cli_puts("freq = ");
    cli_putf(pid->w_cutoff, 1);
    cli_puts(" Hz\r\n");
}

static void cmd_brake(int argc, char **argv)
{
    if (argc >= 2 && cli_strcasecmp(argv[1], "on") == 0) {
        MCT8316_Brake(&mct8316, 1);
        cli_puts("ok\r\n");
    } else if (argc >= 2 && cli_strcasecmp(argv[1], "off") == 0) {
        MCT8316_Brake(&mct8316, 0);
        cli_puts("ok\r\n");
    } else {
        cli_puts("usage: brake on/off\r\n");
    }
}

static void cmd_monitor(int argc, char **argv)
{
    if (argc >= 2 && cli_strcasecmp(argv[1], "off") == 0) {
        cli_monitor_active = 0;
        cli_puts("monitor off\r\n");
    } else if (argc >= 2) {
        int period = (int)cli_atoi(argv[1]);
        if (period > 0 && period <= 10000) {
            cli_monitor_period = (uint16_t)period;
            cli_monitor_counter = (uint16_t)period;
            cli_monitor_active = 1;
            cli_puts("monitor every ");
            cli_puti(period);
            cli_puts(" ms\r\n");
        } else {
            cli_puts("err: period 1-10000 ms\r\n");
        }
    } else {
        if (cli_monitor_active) {
            cli_puts("monitor on, ");
            cli_putu(cli_monitor_period);
            cli_puts(" ms\r\n");
        } else {
            cli_puts("monitor off\r\n");
        }
    }
}
