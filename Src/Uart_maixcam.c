#include "Uart_maixcam.h"
#include "balance_control.h"
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart5;

#define VISION_FRAME_MAX_LEN 32U

static uint8_t rx_byte;
static volatile uint8_t rx_state;
static volatile uint8_t rx_len;
static uint8_t rx_buf[VISION_FRAME_MAX_LEN];
static volatile uint8_t raw_frame_ready;
static volatile uint8_t frame_pending;
static volatile uint32_t rx_byte_count;
static uint8_t target_valid;
static int16_t target_x;

static volatile uint8_t cm_frame_pending;
static uint8_t target_cm_valid;
static int32_t target_pos_cm_x100;
static int32_t target_vel_cms_x100;

/* 坐标统一预处理结果。解析完成后立即低通，D 项不再直接面对像素抖动。 */
static volatile uint8_t filtered_position_pending;
static uint8_t vision_filter_initialized;
static uint8_t filtered_target_valid;
static float filtered_target_pos_cm;
static float filtered_target_vel_cms;

void VisionUart_Init(void)
{
    rx_state = 0U;
    rx_len = 0U;
    raw_frame_ready = 0U;
    frame_pending = 0U;
    target_valid = 0U;
    target_x = 0;
    cm_frame_pending = 0U;
    target_cm_valid = 0U;
    target_pos_cm_x100 = 0;
    target_vel_cms_x100 = 0;
    filtered_position_pending = 0U;
    vision_filter_initialized = 0U;
    filtered_target_valid = 0U;
    filtered_target_pos_cm = 0.0f;
    filtered_target_vel_cms = 0.0f;
    HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;

    if (huart->Instance != UART5) return;

    rx_byte_count++;
    ch = rx_byte;

    /* Keep one complete frame stable until the main loop parses it. */
    if (raw_frame_ready != 0U) {
        HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
        return;
    }

    if (ch == '$') {
        rx_state = 1U;
        rx_len = 0U;
        rx_buf[rx_len++] = ch;
    } else if (rx_state != 0U) {
        if (ch == '\r') {
            /* Ignore CR so both LF and CRLF are accepted. */
        } else if (ch == '\n') {
            if (raw_frame_ready == 0U && rx_len < VISION_FRAME_MAX_LEN) {
                rx_buf[rx_len] = '\0';
                raw_frame_ready = 1U;
            }
            rx_state = 0U;
            rx_len = 0U;
        } else if (rx_len < (VISION_FRAME_MAX_LEN - 1U)) {
            rx_buf[rx_len++] = ch;
        } else {
            rx_state = 0U;
            rx_len = 0U;
        }
    }

    HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
}

/* HAL treats an overrun as a blocking error: it disables RXNEIE/EIE and
   parks RxState back at "ready", then calls this (weak by default, empty)
   callback -- nothing else re-arms the byte-at-a-time receive afterwards.
   An isolated manual test frame rarely hits this, but the camera's
   continuous back-to-back $C stream first overrun silently and permanently
   stops the vision link for the rest of the session even though the camera
   keeps sending fine. Clear the fault and restart reception so a transient
   overrun/framing/noise glitch does not end it. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    rx_state = 0U;
    rx_len = 0U;
    HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
}

static uint8_t VisionUart_IntToStr(int32_t value, char *buf)
{
    char tmp[12];
    uint8_t tmp_len = 0U;
    uint8_t len = 0U;
    uint32_t uval;

    if (value < 0) {
        buf[len++] = '-';
        uval = (uint32_t)(-value);
    } else {
        uval = (uint32_t)value;
    }

    if (uval == 0U) {
        tmp[tmp_len++] = '0';
    } else {
        while (uval > 0U) {
            tmp[tmp_len++] = (char)('0' + (uval % 10U));
            uval /= 10U;
        }
    }

    while (tmp_len > 0U) {
        buf[len++] = tmp[--tmp_len];
    }

    return len;
}

/* Debug ack so a PC serial tool can confirm a frame was received and parsed. */
static void VisionUart_SendAck(uint8_t valid, int16_t x)
{
    char resp[24];
    uint8_t len = 0U;

    resp[len++] = '$';
    resp[len++] = 'A';
    resp[len++] = ',';
    len += VisionUart_IntToStr((int32_t)valid, &resp[len]);
    resp[len++] = ',';
    len += VisionUart_IntToStr((int32_t)x, &resp[len]);
    resp[len++] = '\n';

    HAL_UART_Transmit(&huart5, (uint8_t *)resp, len, 20U);
}

/* Same idea as VisionUart_SendAck but for the $C (cm+velocity) frame type,
   uses a different prefix ('$D') so the two ack styles aren't ambiguous on
   a raw serial monitor. */
static void VisionUart_SendCmAck(uint8_t valid, int32_t pos_cm_x100, int32_t vel_cms_x100)
{
    char resp[32];
    uint8_t len = 0U;

    resp[len++] = '$';
    resp[len++] = 'D';
    resp[len++] = ',';
    len += VisionUart_IntToStr((int32_t)valid, &resp[len]);
    resp[len++] = ',';
    len += VisionUart_IntToStr(pos_cm_x100, &resp[len]);
    resp[len++] = ',';
    len += VisionUart_IntToStr(vel_cms_x100, &resp[len]);
    resp[len++] = '\n';

    HAL_UART_Transmit(&huart5, (uint8_t *)resp, len, 20U);
}

static float VisionUart_PixelToCm(long pixel_x)
{
    float pos_cm = ((float)pixel_x - BAL_VISION_CENTER_PIXEL) *
                   BAL_ROD_LENGTH_CM / BAL_VISION_ROD_PIXELS;

    if (pos_cm < -0.5f * BAL_ROD_LENGTH_CM) {
        pos_cm = -0.5f * BAL_ROD_LENGTH_CM;
    }
    if (pos_cm > 0.5f * BAL_ROD_LENGTH_CM) {
        pos_cm = 0.5f * BAL_ROD_LENGTH_CM;
    }
    return pos_cm;
}

static void VisionUart_UpdateLowPass(uint8_t valid,
                                     float raw_pos_cm,
                                     float raw_vel_cms)
{
    filtered_target_valid = valid;
    if (valid == 0U) {
        /* 丢球后清除历史，重新识别时不让旧坐标拖慢滤波收敛。 */
        vision_filter_initialized = 0U;
        filtered_target_vel_cms = 0.0f;
        filtered_position_pending = 1U;
        return;
    }

    if (vision_filter_initialized == 0U) {
        filtered_target_pos_cm = raw_pos_cm;
        filtered_target_vel_cms = raw_vel_cms;
        vision_filter_initialized = 1U;
    } else {
        /* y[k] = y[k-1] + alpha * (x[k] - y[k-1]) */
        filtered_target_pos_cm += BAL_VISION_LPF_ALPHA *
                                  (raw_pos_cm - filtered_target_pos_cm);
        filtered_target_vel_cms += BAL_VISION_LPF_ALPHA *
                                   (raw_vel_cms - filtered_target_vel_cms);
    }
    filtered_position_pending = 1U;
}

void VisionUart_Process(void)
{
    char *endptr;
    long valid;
    long a, b;

    if (raw_frame_ready == 0U) return;
    raw_frame_ready = 0U;

    if (rx_buf[0] != '$' || rx_buf[2] != ',') return;

    if (rx_buf[1] == 'B') {
        valid = strtol((char *)&rx_buf[3], &endptr, 10);
        if (*endptr != ',' || (valid != 0L && valid != 1L)) return;

        a = strtol(endptr + 1, &endptr, 10);
        if (*endptr != '\0' || a < -32768L || a > 32767L) return;

        target_valid = (uint8_t)valid;
        target_x = (int16_t)a;
        frame_pending = 1U;

        VisionUart_UpdateLowPass(target_valid,
                                 VisionUart_PixelToCm(a),
                                 0.0f);

        VisionUart_SendAck(target_valid, target_x);
    } else if (rx_buf[1] == 'C') {
        valid = strtol((char *)&rx_buf[3], &endptr, 10);
        if (*endptr != ',' || (valid != 0L && valid != 1L)) return;

        a = strtol(endptr + 1, &endptr, 10);
        if (*endptr != ',') return;

        b = strtol(endptr + 1, &endptr, 10);
        if (*endptr != '\0') return;

        target_cm_valid = (uint8_t)valid;
        target_pos_cm_x100 = (int32_t)a;
        target_vel_cms_x100 = (int32_t)b;
        cm_frame_pending = 1U;

        VisionUart_UpdateLowPass(target_cm_valid,
                                 (float)target_pos_cm_x100 * 0.01f,
                                 (float)target_vel_cms_x100 * 0.01f);

        VisionUart_SendCmAck(target_cm_valid, target_pos_cm_x100, target_vel_cms_x100);
    }
}

uint8_t VisionUart_ConsumeFrame(uint8_t *valid, int16_t *x)
{
    if (frame_pending == 0U) return 0U;

    *valid = target_valid;
    *x = target_x;
    frame_pending = 0U;
    return 1U;
}

uint8_t VisionUart_ConsumeCmFrame(uint8_t *valid, int32_t *pos_cm_x100, int32_t *vel_cms_x100)
{
    if (cm_frame_pending == 0U) return 0U;

    *valid = target_cm_valid;
    *pos_cm_x100 = target_pos_cm_x100;
    *vel_cms_x100 = target_vel_cms_x100;
    cm_frame_pending = 0U;
    return 1U;
}

uint8_t VisionUart_ConsumeFilteredPosition(uint8_t *valid,
                                           float *pos_cm,
                                           float *vel_cms)
{
    if (valid == 0 || pos_cm == 0 || vel_cms == 0 ||
        filtered_position_pending == 0U) {
        return 0U;
    }

    *valid = filtered_target_valid;
    *pos_cm = filtered_target_pos_cm;
    *vel_cms = filtered_target_vel_cms;
    filtered_position_pending = 0U;
    return 1U;
}

uint32_t VisionUart_GetRxByteCount(void)
{
    return rx_byte_count;
}

uint8_t VisionUart_GetFrameReadyFlag(void)
{
    return filtered_position_pending;
}

uint8_t VisionUart_GetParserState(void)
{
    return rx_state;
}
