#ifndef DRIVER_SLIP_H
#define DRIVER_SLIP_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

// SLIP special characters
#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define SLIP_MSG_WRITE 0x80

#define SLIP_MAX_READ_LENGTH 8
#define SLIP_MAX_WRITE_LENGTH 8

typedef enum
{
    IDLE = 0,
    ACTIVE = 1,
    ESCAPED = 2
} slip_state_t;

typedef enum
{
	FRAME_CONTINUE = 0,
    FRAME_COMPLETE,
    FRAME_EMPTY,
    FRAME_ERROR,
    FRAME_OVERFLOW
} slip_status_t;

typedef struct
{
    slip_state_t state;
    uint8_t *rxBuffer;
    uint32_t rxIndex;
    uint32_t rxLimit;
} slip_context_t;

slip_status_t slipSetRxBuffer(slip_context_t *context, uint8_t *rxBuffer, uint32_t rxLimit);
slip_status_t slipUnslipData(slip_context_t *context, uint8_t *inBuffer, uint32_t rxLen);
slip_status_t slipSlipData(uint8_t *inBuffer, uint32_t inLen, uint8_t *outBuffer, uint32_t *outLen);

#ifdef __cplusplus
}
#endif

#endif
