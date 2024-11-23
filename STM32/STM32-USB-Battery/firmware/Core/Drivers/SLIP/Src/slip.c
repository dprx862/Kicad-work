#include "slip.h"

slip_status_t slipSetRxBuffer(slip_context_t *context, uint8_t *rxBuffer, uint32_t rxLimit)
{
    if (context)
    {
        context->state = IDLE;
        context->rxBuffer = rxBuffer;
        context->rxIndex = 0;
        context->rxLimit = rxLimit;
        return FRAME_COMPLETE;
    }

    return FRAME_ERROR;
}

static slip_status_t slipUnslipDataByte(slip_context_t *context, uint8_t data)
{

    slip_status_t status = FRAME_CONTINUE;

    switch (context->state)
    {
    case IDLE:
        if (data == SLIP_END)
        {
            context->state = ACTIVE;
            context->rxIndex = 0;
        }
        break;

    case ACTIVE:
        if (data == SLIP_END)
        {
            if (context->rxIndex)
            {
                context->state = IDLE;
                status = FRAME_COMPLETE;
            }
        }
        else if (context->rxIndex < context->rxLimit)
        {
            if (data == SLIP_ESC)
            {
                context->state = ESCAPED;
            }
            else
            {
                context->rxBuffer[context->rxIndex] = data;
                context->rxIndex++;
            }
        }
        else
        {
            context->state = IDLE;
        }
        break;

    case ESCAPED:
        context->state = ACTIVE;
        if (data == SLIP_ESC_END)
        {
            context->rxBuffer[context->rxIndex] = SLIP_END;
        }
        else if (data == SLIP_ESC_ESC)
        {
            context->rxBuffer[context->rxIndex] = SLIP_ESC;
        }
        else
        {
            status = FRAME_ERROR;
            context->state = IDLE;
        }
        context->rxIndex++;
        break;
    }

    return status;
}

slip_status_t slipUnslipData(slip_context_t *context, uint8_t *inBuffer, uint32_t rxLen)
{
    slip_status_t status = FRAME_ERROR;
    uint8_t *rxData = inBuffer;
    uint32_t rxResidue = rxLen;

    if (!(context && context->rxBuffer))
    {
        return status;
    }

    while (rxResidue)
    {
        status = slipUnslipDataByte(context, *rxData);

        switch (status)
        {
        case FRAME_CONTINUE:
            rxData++;
            rxResidue--;
            break;
        default:
            rxResidue = 0;
            break;
        }
    }

    return status;
}

slip_status_t slipSlipData(uint8_t *inBuffer, uint32_t inLen, uint8_t *outBuffer, uint32_t *outLen)
{
    slip_status_t status = FRAME_ERROR;
    uint8_t *txData = outBuffer;
    uint32_t i = 0;
    uint32_t txResidue = *outLen;
    uint8_t curByte;
    uint8_t outByte;
    slip_state_t state = IDLE;

    if (!(inBuffer && outBuffer && outLen))
    {
        return status;
    }

    while (txResidue > 1)
    {
        if (i < inLen)
        {
            curByte = inBuffer[i];;

            switch(state)
            {
            case IDLE:
                state = ACTIVE;
                outByte = SLIP_END;
                break;

            case ACTIVE:
                if (curByte == SLIP_END || curByte == SLIP_ESC)
                {
                    outByte = SLIP_ESC;
                    state = ESCAPED;
                }
                else
                {
                    outByte = curByte;
                    i++;
                }
                break;

            case ESCAPED:
                if (curByte == SLIP_END) {
                    outByte = SLIP_ESC_END;
                }
                else
                {
                    outByte = SLIP_ESC_ESC;
                }
                state = ACTIVE;
                i++;
                break;
            }

            *txData = outByte;
            txData++;
            txResidue--;

        }
        else
        {
            *txData = SLIP_END;
            txResidue--;
            *outLen = (*outLen - txResidue);
            status = FRAME_COMPLETE;
            break;
        }
    }

    return status;

}
