#include "mcp2515.h"

// #include "main.h"
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_def.h>

typedef unsigned char u8;

#define SPI_TIMEOUT 0xFFFF
extern SPI_HandleTypeDef hspi1;

static const struct TXBn_REGS TXB[N_TXBUFFERS] =
    {
        {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
        {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
        {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}};

static const struct RXBn_REGS RXB[N_RXBUFFERS] =
    {
        {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
        {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}};

void setRegister(u8 reg, const uint8_t value)
{
    u8 buffer[3] = {INSTRUCTION_WRITE, reg, value};
    SPI_SELECT();
    HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 3, SPI_TIMEOUT);
    SPI_DESELECT();
}

void setRegisters(u8 reg, u8 *value, const u8 n)
{
    u8 *buffer = (u8 *)malloc(n + 2);
    *(buffer + 0) = INSTRUCTION_WRITE;
    *(buffer + 1) = reg;
    memcpy(buffer + 2, value, n);
    SPI_SELECT();
    HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, n + 2, SPI_TIMEOUT);
    SPI_DESELECT();
    if (buffer != NULL)
    {
        free(buffer);
        buffer = NULL;
    }
}

u8 readRegister(const u8 reg)
{
    u8 buffer[3] = {INSTRUCTION_READ, reg, 0};
    SPI_SELECT();
    HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 3, SPI_TIMEOUT);
    SPI_DESELECT();
    return buffer[2];
}

void readRegisters(const u8 reg, u8 *value, const u8 n)
{
    u8 *buffer = (u8 *)malloc(n + 2);
    *(buffer + 0) = INSTRUCTION_READ;
    *(buffer + 1) = reg;
    memcpy(buffer + 2, value, n);
    SPI_SELECT();
    HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, n + 2, SPI_TIMEOUT);
    SPI_DESELECT();
    memcpy(value, buffer + 2, n);
    if (buffer != NULL)
    {
        free(buffer);
        buffer = NULL;
    }
}

void modifyRegister(const u8 reg, const u8 mask, const u8 data)
{
    u8 buffer[4] = {INSTRUCTION_BITMOD, reg, mask, data};
    SPI_SELECT();
    HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 4, SPI_TIMEOUT);
    SPI_DESELECT();
}

u8 getStatus()
{
    u8 buffer[2] = {INSTRUCTION_READ_STATUS, 0x00};
    SPI_SELECT();
    HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 2, SPI_TIMEOUT);
    SPI_DESELECT();
    return buffer[1];
}

eERROR setMode(const eCANCTRL_REQOP_MODE mode)
{
    modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    unsigned long endTime = HAL_GetTick() + 10;
    bool modeMatch = false;
    while (HAL_GetTick() < endTime)
    {
        uint8_t newMode = readRegister(MCP_CANSTAT);
        newMode &= CANSTAT_OPMOD;

        modeMatch = newMode == mode;
        if (modeMatch)
            break;
    }
    return modeMatch ? ERROR_OK : ERROR_FAIL;
}

eERROR setNormalMode()
{
    return setMode(CANCTRL_REQOP_NORMAL);
}

eERROR setConfigMode()
{
    return setMode(CANCTRL_REQOP_CONFIG);
}

eERROR setListenOnlyMode()
{
    return setMode(CANCTRL_REQOP_LISTENONLY);
}

eERROR setSleepMode()
{
    return setMode(CANCTRL_REQOP_SLEEP);
}

eERROR setLoopbackMode()
{
    return setMode(CANCTRL_REQOP_LOOPBACK);
}
eERROR setBitrate(const eCAN_SPEED canSpeed, eCAN_CLOCK canClock)
{
    eERROR error = setConfigMode();
    if (error != ERROR_OK)
    {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
    case (MCP_8MHZ):
        switch (canSpeed)
        {
        case (CAN_5KBPS): //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

        case (CAN_10KBPS): //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

        case (CAN_20KBPS): //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

        case (CAN_31K25BPS): //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

        case (CAN_33KBPS): //  33.333KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            break;
        }
        break;

    case (MCP_16MHZ):
        switch (canSpeed)
        {
        case (CAN_5KBPS): //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

        case (CAN_10KBPS): //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

        case (CAN_20KBPS): //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

        case (CAN_33KBPS): //  33.333Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

        case (CAN_83K3BPS): //  83.333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            break;
        }
        break;

    case (MCP_20MHZ):
        switch (canSpeed)
        {
        case (CAN_33KBPS): //  33.333Kbps
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;
            break;

        case (CAN_40KBPS): //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

        case (CAN_50KBPS): //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

        case (CAN_80KBPS): //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

        case (CAN_83K3BPS): //  83.333Kbps
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;
            break;

        case (CAN_100KBPS): // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

        case (CAN_125KBPS): // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

        case (CAN_200KBPS): // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

        case (CAN_250KBPS): // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

        case (CAN_500KBPS): // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

        case (CAN_1000KBPS): //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

        default:
            set = 0;
            break;
        }
        break;

    default:
        set = 0;
        break;
    }

    if (set)
    {
        setRegister(MCP_CNF1, cfg1);
        setRegister(MCP_CNF2, cfg2);
        setRegister(MCP_CNF3, cfg3);
        return ERROR_OK;
    }
    else
    {
        return ERROR_FAIL;
    }
}

eERROR setClkOut(const eCAN_CLKOUT divisor)
{
    if (divisor == CLKOUT_DISABLE)
    {
        /* Turn off CLKEN */
        modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

        /* Turn on CLKOUT for SOF */
        modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);

        return ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    /* Turn on CLKEN */
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    /* Turn off CLKOUT for SOF */
    modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
    return ERROR_OK;
}

eERROR reset()
{
    u8 zeros[14];
    memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14);
    setRegisters(MCP_TXB1CTRL, zeros, 14);
    setRegisters(MCP_TXB2CTRL, zeros, 14);

    setRegister(MCP_RXB0CTRL, 0);
    setRegister(MCP_RXB1CTRL, 0);

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    modifyRegister(MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT);
    modifyRegister(MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_STDEXT);

    return ERROR_OK;
}

void prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext)
    {
        buffer[MCP_EID0] = (uint8_t)(canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t)(canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t)(canid >> 5);
    }
    else
    {
        buffer[MCP_SIDH] = (uint8_t)(canid >> 3);
        buffer[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

eERROR setFilterMask(const eMASK mask, const bool ext, const uint32_t ulData)
{
    eERROR res = setConfigMode();
    if (res != ERROR_OK)
    {
        return res;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    u8 reg;
    switch (mask)
    {
    case MASK0:
        reg = MCP_RXM0SIDH;
        break;
    case MASK1:
        reg = MCP_RXM1SIDH;
        break;
    default:
        return ERROR_FAIL;
    }

    setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

eERROR setFilter(const eRXF num, const bool ext, const uint32_t ulData)
{
    eERROR res = setConfigMode();
    if (res != ERROR_OK)
    {
        return res;
    }

    u8 reg;

    switch (num)
    {
    case RXF0:
        reg = MCP_RXF0SIDH;
        break;
    case RXF1:
        reg = MCP_RXF1SIDH;
        break;
    case RXF2:
        reg = MCP_RXF2SIDH;
        break;
    case RXF3:
        reg = MCP_RXF3SIDH;
        break;
    case RXF4:
        reg = MCP_RXF4SIDH;
        break;
    case RXF5:
        reg = MCP_RXF5SIDH;
        break;
    default:
        return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

eERROR sendMessage(const eTXBn txbn, const struct can_frame *frame)
{
    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

    modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    return ERROR_OK;
}

eERROR sendMessages(const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAILTX;
    }

    eTXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

    for (int i = 0; i < N_TXBUFFERS; i++)
    {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = readRegister(txbuf->CTRL);
        if ((ctrlval & TXB_TXREQ) == 0)
        {
            return sendMessage(txBuffers[i], frame);
        }
    }

    return ERROR_FAILTX;
}

eERROR readMessage(const eRXBn rxbn, struct can_frame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    readRegisters(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

    if ((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK)
    {
        id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id << 8) + tbufdata[MCP_EID8];
        id = (id << 8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN)
    {
        return ERROR_FAIL;
    }

    uint8_t ctrl = readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR)
    {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    readRegisters(rxb->DATA, frame->data, dlc);

    modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return ERROR_OK;
}

eERROR readMessages(struct can_frame *frame)
{
    eERROR rc;
    uint8_t stat = getStatus();

    if (stat & STAT_RX0IF)
    {
        rc = readMessage(RXB0, frame);
    }
    else if (stat & STAT_RX1IF)
    {
        rc = readMessage(RXB1, frame);
    }
    else
    {
        rc = ERROR_NOMSG;
    }

    return rc;
}

bool checkReceive(void)
{
    uint8_t res = getStatus();
    if (res & STAT_RXIF_MASK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool checkError(void)
{
    uint8_t eflg = getErrorFlags();

    if (eflg & EFLG_ERRORMASK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t getErrorFlags(void)
{
    return readRegister(MCP_EFLG);
}

void clearRXnOVRFlags(void)
{
    modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t getInterrupts(void)
{
    return readRegister(MCP_CANINTF);
}

void clearInterrupts(void)
{
    setRegister(MCP_CANINTF, 0);
}

uint8_t getInterruptMask(void)
{
    return readRegister(MCP_CANINTE);
}

void clearTXInterrupts(void)
{
    modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void clearRXnOVR(void)
{
    uint8_t eflg = getErrorFlags();
    if (eflg != 0)
    {
        clearRXnOVRFlags();
        clearInterrupts();
        // modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
    }
}

void clearMERR()
{
    // modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    // clearInterrupts();
    modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void clearERRIF()
{
    // modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    // clearInterrupts();
    modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

void mcp2515_init()
{
    eERROR err;

    err = reset();
    if (err != ERROR_OK)
    {
        printf("Error %d  \r\n", __LINE__);
    }
    err = setBitrate(CAN_500KBPS, MCP_8MHZ);
    if (err != ERROR_OK)
    {
        printf("Error %d  \r\n", __LINE__);
    }
    err = setNormalMode();
    if (err != ERROR_OK)
    {
        printf("Error %d  \r\n", __LINE__);
    }
}
