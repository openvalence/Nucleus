// main -- board monitor (U12, CH32V003F4U6) glue: pins, safe state at boot,
// the 1 kHz ADC scan, the CLAMP_TRIM PWM, FAULT_N, the I2C target, the
// independent watchdog
// Constraints:
// - The first thing after SystemInit is the safe state: FAULT_N pulled (motor
//   cannot enable), CLAMP_TRIM released to its reset level (43.5 V). Nothing
//   runs before it.
// - PD1 is SWIO and PD7 is NRST. Never reconfigure either: SWIO is the only
//   way the P4 can reprogram this chip (val-091.20).
// - The IWDG runs from boot. A hang must not leave the trim PWM driving a
//   stale duty: a reset floats the trim pin back to 43.5 V and FAULT_N back
//   to released, and boot pulls FAULT_N again.
// - The I2C ISR owns g_tx and the receive side of the mailbox; the main loop
//   owns McState and every published block. They meet only at the volatile
//   front indexes and the mailbox length, each a single byte store.
// - Memory: 2 KB SRAM. Everything here is static; no heap.
// See: docs/supervisor.md, monitor_core.h, flagship_p4/src/system/Supervisor.h

#include "ch32fun.h"

#include "monitor_core.h"

// ---- pins (Hardware flagship schematic, U12 sheet) ------------------------------

#define PIN_FAULT_N    PC7   // open-drain, 1k into the motor-switch EN wired-OR
#define PIN_CLAMP_TRIM PC3   // TIM1_CH3, 20 kHz PWM into the regen comparator threshold
#define PIN_PUMP_FLT   PC0   // U14 FLT, active low, 10k pull-up
#define PIN_MON_LED    PC6   // 10 x WS2812-class status bank
#define PIN_SDA        PC1   // I2C1, INA bus
#define PIN_SCL        PC2
#define PIN_SPARE_A    PD0   // NC
#define PIN_SPARE_B    PC5   // NC

// ADC channel per SV_CH_* index, then Vrefint.
static const uint8_t kAdcCh[SV_CH_COUNT] = {
    3,   // VIN_RAW    PD2
    4,   // +BUS       PD3
    7,   // +12V       PD4
    5,   // +5V        PD5
    1,   // +5V_SYS    PA1
    6,   // +3V3_ACC   PD6
    0,   // SHUNT_TEMP PA2
    2,   // CLAMP_MON  PC4
};
// TODO(val-091.19): confirm on silicon that channel 8 reads Vrefint with no enable bit.
#define ADC_CH_VREFINT 8

#define FLASH_BASE_ADDR 0x08000000u
#define TRIM_PWM_TOP    2399u   // 48 MHz / 2400 = 20 kHz

// ---- shared with the I2C ISR ------------------------------------------------------

static uint8_t g_ident[SV_IDENT_LEN];
static uint8_t g_status[2][SV_STATUS_LEN];
static uint8_t g_limits[2][SV_LIMITS_LEN];
static volatile uint8_t g_status_front, g_limits_front;

static uint8_t g_rx[SV_BLOCK_MAX];
static uint8_t g_rx_n, g_rx_over, g_ptr;
static uint8_t g_tx[SV_BLOCK_MAX];
static uint8_t g_tx_len, g_tx_i;

static uint8_t g_mbox[SV_BLOCK_MAX];
static volatile uint8_t g_mbox_len;
static volatile uint8_t g_isr_drops;

static McState g_mc;

// ---- safe state -------------------------------------------------------------------

static void faultN(int pull) {
    funDigitalWrite(PIN_FAULT_N, pull ? FUN_LOW : FUN_HIGH);
}

static void trimRelease(void) {
    TIM1->CH3CVR = 0;
    funPinMode(PIN_CLAMP_TRIM, GPIO_CFGLR_IN_FLOAT);
}

static void trimDrive(uint16_t permille) {
    TIM1->CH3CVR = (uint32_t)permille * (TRIM_PWM_TOP + 1u) / 1000u;
    funPinMode(PIN_CLAMP_TRIM, GPIO_CFGLR_OUT_10Mhz_AF_PP);
}

static void safeState(void) {
    funGpioInitAll();
    funPinMode(PIN_FAULT_N, GPIO_CFGLR_OUT_10Mhz_OD);
    faultN(1);
    funPinMode(PIN_CLAMP_TRIM, GPIO_CFGLR_IN_FLOAT);
    funPinMode(PIN_MON_LED, GPIO_CFGLR_OUT_10Mhz_PP);
    funDigitalWrite(PIN_MON_LED, FUN_LOW);
    funPinMode(PIN_PUMP_FLT, GPIO_CFGLR_IN_FLOAT);
    funPinMode(PIN_SPARE_A, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_SPARE_A, FUN_HIGH);
    funPinMode(PIN_SPARE_B, GPIO_CFGLR_IN_PUPD);
    funDigitalWrite(PIN_SPARE_B, FUN_HIGH);
}

// ---- peripherals ------------------------------------------------------------------

static void adcInit(void) {
    funPinMode(PA1, GPIO_CFGLR_IN_ANALOG);
    funPinMode(PA2, GPIO_CFGLR_IN_ANALOG);
    funPinMode(PC4, GPIO_CFGLR_IN_ANALOG);
    funPinMode(PD2, GPIO_CFGLR_IN_ANALOG);
    funPinMode(PD3, GPIO_CFGLR_IN_ANALOG);
    funPinMode(PD4, GPIO_CFGLR_IN_ANALOG);
    funPinMode(PD5, GPIO_CFGLR_IN_ANALOG);
    funPinMode(PD6, GPIO_CFGLR_IN_ANALOG);
    RCC->CFGR0 &= ~(0x1Fu << 11);   // ADCCLK = HCLK / 2 = 24 MHz
    RCC->APB2PCENR |= RCC_APB2Periph_ADC1;
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
    ADC1->RSQR1 = 0;
    ADC1->RSQR2 = 0;
    // 241 cycles on every channel: the 100 nF at each pin is the charge source.
    ADC1->SAMPTR2 = 0x3FFFFFFFu;
    ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;
    ADC1->CTLR2 |= ADC_RSTCAL;
    while (ADC1->CTLR2 & ADC_RSTCAL) {}
    ADC1->CTLR2 |= ADC_CAL;
    while (ADC1->CTLR2 & ADC_CAL) {}
}

static uint16_t adcRead(uint8_t ch) {
    ADC1->RSQR3 = ch;
    ADC1->CTLR2 |= ADC_SWSTART;
    while (!(ADC1->STATR & ADC_EOC)) {}
    return (uint16_t)ADC1->RDATAR;
}

static void trimPwmInit(void) {
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1;
    TIM1->PSC = 0;
    TIM1->ATRLR = TRIM_PWM_TOP;
    TIM1->CH3CVR = 0;
    TIM1->CHCTLR2 = TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC3PE;
    TIM1->CCER = TIM_CC3E;
    TIM1->BDTR = TIM_MOE;
    TIM1->SWEVGR = TIM_UG;
    TIM1->CTLR1 = TIM_ARPE | TIM_CEN;
}

static void i2cTargetInit(void) {
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;
    RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;
    funPinMode(PIN_SDA, GPIO_CFGLR_OUT_10Mhz_AF_OD);
    funPinMode(PIN_SCL, GPIO_CFGLR_OUT_10Mhz_AF_OD);
    I2C1->CTLR2 = (FUNCONF_SYSTEM_CORE_CLOCK / 1000000u) | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITBUFEN
                | I2C_CTLR2_ITERREN;
    I2C1->OADDR1 = SV_I2C_ADDR << 1;
    I2C1->CTLR1 = I2C_CTLR1_PE;
    I2C1->CTLR1 = I2C_CTLR1_PE | I2C_CTLR1_ACK;   // ACK holds only once PE is set
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

// TODO(val-091.20): confirm whether the IWDG keeps counting while the core is
// halted over SWIO; if it does, the programmer sets the debug freeze first.
static void iwdgInit(void) {
    IWDG->CTLR = 0x5555;
    IWDG->PSCR = 3;       // LSI 128 kHz / 32 = 4 kHz
    IWDG->RLDR = 400;     // 100 ms
    IWDG->CTLR = 0xAAAA;
    IWDG->CTLR = 0xCCCC;
}

static void iwdgFeed(void) { IWDG->CTLR = 0xAAAA; }

// ---- I2C target ---------------------------------------------------------------------

static void txLoad(void) {
    const uint8_t* src = 0;
    uint8_t n = 0;
    switch (g_ptr) {
        case SV_REG_IDENT:  src = g_ident; n = SV_IDENT_LEN; break;
        case SV_REG_STATUS: src = g_status[g_status_front]; n = SV_STATUS_LEN; break;
        case SV_REG_LIMITS: src = g_limits[g_limits_front]; n = SV_LIMITS_LEN; break;
        default: break;
    }
    for (uint8_t i = 0; i < n; i++) g_tx[i] = src[i];
    g_tx_len = n;
    g_tx_i = 0;
}

static void rxFinish(void) {
    if (g_rx_over) {
        g_isr_drops++;
    } else if (g_rx_n == 1) {
        g_ptr = g_rx[0];
    } else if (g_rx_n > 1) {
        if (g_mbox_len) {
            g_isr_drops++;
        } else {
            for (uint8_t i = 0; i < g_rx_n; i++) g_mbox[i] = g_rx[i];
            g_mbox_len = g_rx_n;
        }
    }
    g_rx_n = 0;
    g_rx_over = 0;
}

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    const uint16_t s1 = I2C1->STAR1;
    if (s1 & I2C_STAR1_ADDR) {
        const uint16_t s2 = I2C1->STAR2;   // reading STAR1 then STAR2 clears ADDR
        if (s2 & I2C_STAR2_TRA) {
            if (g_rx_n == 1) g_ptr = g_rx[0];   // write-then-read with a repeated start
            g_rx_n = 0;
            g_rx_over = 0;
            txLoad();
        } else {
            g_rx_n = 0;
            g_rx_over = 0;
        }
    }
    if (s1 & I2C_STAR1_RXNE) {
        const uint8_t b = (uint8_t)I2C1->DATAR;
        if (g_rx_n < sizeof(g_rx)) g_rx[g_rx_n++] = b;
        else g_rx_over = 1;
    }
    if (s1 & I2C_STAR1_TXE) {
        I2C1->DATAR = g_tx_i < g_tx_len ? g_tx[g_tx_i++] : 0xFF;
    }
    if (s1 & I2C_STAR1_STOPF) {
        I2C1->CTLR1 |= I2C_CTLR1_PE;   // the write after the STAR1 read clears STOPF
        rxFinish();
    }
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) {
    // AF is the controller's NACK ending every read; the rest are bus errors.
    const uint16_t s1 = I2C1->STAR1;
    if (s1 & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_OVR)) g_isr_drops++;
    I2C1->STAR1 = 0;
}

// ---- main loop ----------------------------------------------------------------------

static void publishLimits(void) {
    const uint8_t back = (uint8_t)(g_limits_front ^ 1u);
    sv_limits_encode(&g_mc.active, g_limits[back]);
    g_limits_front = back;
}

static void publishStatus(void) {
    SvStatus st;
    mc_status(&g_mc, &st);
    const uint8_t back = (uint8_t)(g_status_front ^ 1u);
    sv_status_encode(&st, g_status[back]);
    g_status_front = back;
}

static void drainMailbox(void) {
    static uint8_t seen_drops;
    const uint8_t drops = g_isr_drops;
    while (seen_drops != drops) {
        seen_drops++;
        mc_on_link_drop(&g_mc);
    }
    const uint8_t n = g_mbox_len;
    if (!n) return;
    uint8_t counter, op;
    uint16_t arg;
    SvLimits req;
    switch (g_mbox[0]) {
        case SV_REG_HEARTBEAT:
            if (sv_heartbeat_decode(g_mbox, n, &counter) == SV_OK) mc_on_heartbeat(&g_mc, counter);
            else mc_on_link_drop(&g_mc);
            break;
        case SV_REG_COMMAND:
            if (sv_command_decode(g_mbox, n, &op, &arg) == SV_OK) mc_on_command(&g_mc, op, arg);
            else mc_on_link_drop(&g_mc);
            break;
        case SV_REG_LIMITS:
            if (sv_limits_decode(g_mbox, n, &req) == SV_OK) {
                mc_on_limits(&g_mc, &req);
                publishLimits();
            } else {
                mc_on_link_drop(&g_mc);
            }
            break;
        default:
            mc_on_link_drop(&g_mc);
            break;
    }
    g_mbox_len = 0;
}

static uint32_t imageCrc32(void) {
    const uint8_t* flash = (const uint8_t*)FLASH_BASE_ADDR;
    return ~sv_crc32_update(0xFFFFFFFFu, flash, SV_FLASH_BYTES);
}

int main(void) {
    SystemInit();
    safeState();

    // RSTSCKR bits 31:24 are the reset flags; RMVF clears them for next time.
    const uint8_t reset_cause = (uint8_t)(RCC->RSTSCKR >> 24);
    RCC->RSTSCKR |= RCC_RMVF;

    iwdgInit();
    SvIdent id;
    id.link_version = SV_LINK_VERSION;
    id.fw_major = MC_FW_MAJOR;
    id.fw_minor = MC_FW_MINOR;
    id.fw_patch = MC_FW_PATCH;
    id.image_crc32 = imageCrc32();
    id.reset_cause = reset_cause;
    sv_ident_encode(&id, g_ident);
    iwdgFeed();

    mc_init(&g_mc);
    adcInit();
    trimPwmInit();
    publishLimits();
    publishStatus();
    i2cTargetInit();

    uint32_t next = SysTick->CNT;
    for (;;) {
        while ((int32_t)(SysTick->CNT - next) < 0) {}
        next += DELAY_MS_TIME;

        McInputs in;
        for (int i = 0; i < SV_CH_COUNT; i++) in.raw[i] = adcRead(kAdcCh[i]);
        in.raw_vrefint = adcRead(ADC_CH_VREFINT);
        in.pump_flt_low = !funDigitalRead(PIN_PUMP_FLT);

        drainMailbox();
        mc_tick(&g_mc, &in);

        faultN(mc_fault_n(&g_mc));
        if (g_mc.trim_mode == MC_TRIM_DRIVEN) trimDrive(g_mc.trim_permille);
        else trimRelease();

        publishStatus();
        iwdgFeed();
    }
}
