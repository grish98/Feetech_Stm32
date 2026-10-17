/* THROWAWAY spike for R10 (circular RX DMA ring). Lives on spike/r10-ring-rx only. */
#ifdef SPIKE_RING

#include "main.h"
#include "sts_protocol.h"
#include "sts_registers.h"
#include "SEGGER_RTT.h"
#include <string.h>

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;

#define SPIKE_SERVO_ID  3U     /* the ID your startup scan reports */
#define RING_N          32U    /* deliberately small: wraps within a few replies */
#define LOG_N           64U

typedef struct {
    uint32_t type;   /* HAL_UART_RXEVENT_TC (0), _HT (1), _IDLE (2) */
    uint16_t size;   /* ring position HAL reports */
    uint32_t cyc;    /* DWT stamp */
} spike_evt_t;

static uint8_t              s_ring[RING_N];
static volatile spike_evt_t s_log[LOG_N];
static volatile uint32_t    s_log_n;
static volatile uint32_t    s_idr_during_pulse;   /* GPIOA->IDR sampled at the end of the low pulse */

/* ISR context (USART2 IRQ for IDLE, DMA1 Ch6 IRQ for HT/TC): record only, never print. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart2) {
        return;
    }
    uint32_t i = s_log_n;
    if (i < LOG_N) {
        s_log[i].type = HAL_UARTEx_GetRxEventType(huart);
        s_log[i].size = Size;
        s_log[i].cyc  = DWT->CYCCNT;
        s_log_n = i + 1U;
    }
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Switch the RX channel to circular mode without touching CubeMX files, then arm the ring once. */
static HAL_StatusTypeDef ring_start(void)
{
    (void)HAL_UART_AbortReceive(&huart2);
    (void)HAL_DMA_DeInit(&hdma_usart2_rx);
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
        return HAL_ERROR;
    }
    memset(s_ring, 0xAA, sizeof s_ring);   /* marker: bytes never written stay 0xAA */
    s_log_n = 0U;
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_ring, RING_N);
}

/* Mode A: today's switching, receiver off during TX. Returns the DWT stamp at TX complete. */
static uint32_t send_mode_a(const uint8_t *pkt, uint16_t len)
{
    (void)HAL_HalfDuplex_EnableTransmitter(&huart2);      /* TE=1, RE=0 */
    (void)HAL_UART_Transmit(&huart2, pkt, len, 10U);      /* returns after TC */
    uint32_t t_tc = DWT->CYCCNT;
    (void)HAL_HalfDuplex_EnableReceiver(&huart2);         /* TE=0, RE=1 */
    return t_tc;
}

/* Mode B: receiver left on during TX, to see whether our own bytes echo into the ring. */
static uint32_t send_mode_b(const uint8_t *pkt, uint16_t len)
{
    ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_TE | USART_CR1_RE);
    (void)HAL_UART_Transmit(&huart2, pkt, len, 10U);
    uint32_t t_tc = DWT->CYCCNT;
    ATOMIC_CLEAR_BIT(huart2.Instance->CR1, USART_CR1_TE);
    return t_tc;
}

static const char *evt_name(uint32_t t)
{
    return (t == HAL_UART_RXEVENT_IDLE) ? "IDLE" : (t == HAL_UART_RXEVENT_HT) ? "HT  " : "TC  ";
}

static void dump(const char *label, uint32_t t_tc)
{
    const int32_t cyc_per_us = (int32_t)(SystemCoreClock / 1000000U);
    uint32_t n = s_log_n;

    SEGGER_RTT_printf(0, "--- %s | events=%u NDTR=%u err=0x%x RxState=0x%x dma=%u\n",
                      label, (unsigned)n, (unsigned)__HAL_DMA_GET_COUNTER(&hdma_usart2_rx),
                      (unsigned)huart2.ErrorCode, (unsigned)huart2.RxState,
                      (unsigned)hdma_usart2_rx.State);
    for (uint32_t i = 0U; (i < n) && (i < LOG_N); i++) {
        int32_t dt_us = (int32_t)(s_log[i].cyc - t_tc) / cyc_per_us;
        SEGGER_RTT_printf(0, "    %s size=%u  t=%d us after TC\n",
                          evt_name(s_log[i].type), (unsigned)s_log[i].size, (int)dt_us);
    }
    SEGGER_RTT_printf(0, "    ring:");
    for (uint32_t i = 0U; i < RING_N; i++) {
        SEGGER_RTT_printf(0, " %02x", s_ring[i]);
    }
    SEGGER_RTT_printf(0, "\n");
    s_log_n = 0U;
}


static void pulse_pin_init(void)
{
    GPIO_InitTypeDef g = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   /* released before it becomes an output */
    g.Pin   = GPIO_PIN_0;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
}

/* Pull the bus low for 'us' microseconds through the series resistor. Bus must be idle. */
static void bus_pulse_low(uint32_t us)
{
    const uint32_t cycles = us * (SystemCoreClock / 1000000U);
    __disable_irq();                               /* keeps the width exact */
    const uint32_t t0 = DWT->CYCCNT;
    GPIOA->BSRR = (uint32_t)GPIO_PIN_0 << 16U;     /* low */
    while ((DWT->CYCCNT - t0) < cycles) { }
    s_idr_during_pulse = GPIOA->IDR;               /* actual pin levels, line settled */
    GPIOA->BSRR = GPIO_PIN_0;                      /* release */
    __enable_irq();
}


void Spike_RingRun(void)
{
    uint8_t ping[8];
    uint8_t rd16[10];
    const uint8_t rd16_params[2] = { STS_REG_PRESENT_POSITION, 2U };
    const uint16_t ping_len = STS_MIN_PACKET_SIZE;        /* 6 */
    const uint16_t rd16_len = STS_MIN_PACKET_SIZE + 2U;   /* 8 */
    uint32_t t = 0U;

    dwt_init();
    pulse_pin_init();
    SEGGER_RTT_printf(0, "\n>> SPIKE R10 (built " __TIME__ "): ring=%u, ID %u, DWT %s\n", (unsigned)RING_N,
                                               (unsigned)SPIKE_SERVO_ID, (DWT->CYCCNT != 0U) ? "running" : "NOT RUNNING");

    if ((sts_create_packet(SPIKE_SERVO_ID, STS_INST_PING, NULL, 0U, ping, sizeof ping) != STS_OK) ||
        (sts_create_packet(SPIKE_SERVO_ID, STS_INST_READ, rd16_params, 2U, rd16, sizeof rd16) != STS_OK)) {
        SEGGER_RTT_printf(0, ">> packet build failed\n");
        return;
    }

    /* Run 1: Mode A, 10 pings (P1, P2, P3, P5). */
    if (ring_start() != HAL_OK) {
        SEGGER_RTT_printf(0, ">> ring_start failed\n");
        return;
    }
    HAL_Delay(5);
    dump("run1 baseline, no TX", DWT->CYCCNT);
    for (uint32_t k = 0U; k < 10U; k++) {
        t = send_mode_a(ping, ping_len);
        HAL_Delay(3);
        dump("run1 A ping", t);
    }

    /* Run 2: fresh ring, Mode A, 5 READ16 (P4): the 4th 8-byte reply ends exactly at 32. */
    (void)ring_start();
    for (uint32_t k = 0U; k < 5U; k++) {
        t = send_mode_a(rd16, rd16_len);
        HAL_Delay(3);
        dump("run2 A read16", t);
    }

    /* Run 3: fresh ring, Mode B, 5 pings (P6). */
    (void)ring_start();
    for (uint32_t k = 0U; k < 5U; k++) {
        t = send_mode_b(ping, ping_len);
        HAL_Delay(3);
        dump("run3 B ping", t);
    }

    /* Run 5: fresh ring, 10 pings with no reads between: HAL wraps over unread data silently. */
    (void)ring_start();
    for (uint32_t k = 0U; k < 10U; k++) {
        t = send_mode_a(ping, ping_len);
        HAL_Delay(3);
    }
    dump("run5 A ping x10, no reads", t);

    /* Run 6: stray byte, then framing error and restart (P7). */
    (void)ring_start();
    t = send_mode_a(ping, ping_len);
    HAL_Delay(2);
    dump("run6a ping", t);

    t = DWT->CYCCNT;
    bus_pulse_low(3U);
    HAL_Delay(2);
    dump("run6b 3 us pulse: stray byte", t);

    t = DWT->CYCCNT;
    bus_pulse_low(20U);
    HAL_Delay(2);
    dump("run6c 20 us pulse: framing error", t);
    SEGGER_RTT_printf(0, "    during pulse: PA0=%u PA2=%u\n",
                      (unsigned)(s_idr_during_pulse & 1U), (unsigned)((s_idr_during_pulse >> 2) & 1U));

    t = send_mode_a(ping, ping_len);
    HAL_Delay(2);
    dump("run6d ping, no restart", t);

    (void)ring_start();
    t = send_mode_a(ping, ping_len);
    HAL_Delay(2);
    dump("run6e ping after restart", t);

    SEGGER_RTT_printf(0, ">> SPIKE done\n");
}

#endif /* SPIKE_RING */
