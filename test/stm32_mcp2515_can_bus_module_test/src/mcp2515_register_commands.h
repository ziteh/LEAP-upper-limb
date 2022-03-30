/**
 * @file   mcp2515_register_commands.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  The register address and command defines.
 * @remark Reference: https://ithelp.ithome.com.tw/articles/10284376
 */

#pragma one

/* Configuration Registers. */
#define CANSTAT 0x0E
#define CANCTRL 0x0F
#define BFPCTRL 0x0C
#define TEC 0x1C
#define REC 0x1D
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2A
#define CANINTE 0x2B
#define CANINTF 0x2C
#define EFLG 0x2D
#define TXRTSCTRL 0x0D

/* Receive Filters. */
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF1EID8 0x06
#define RXF1EID0 0x07
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0A
#define RXF2EID0 0x0B
#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF3EID8 0x12
#define RXF3EID0 0x13

/* Receive Masks. */
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27

/* Tx Buffer 0. */
#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC 0x35
#define TXB0D0 0x36
#define TXB0D1 0x37
#define TXB0D2 0x38
#define TXB0D3 0x39
#define TXB0D4 0x3A
#define TXB0D5 0x3B
#define TXB0D6 0x3C
#define TXB0D7 0x3D

/* Rx Buffer 0. */
#define RXB0CTRL 0x60
#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66
#define RXB0D1 0x67
#define RXB0D2 0x68
#define RXB0D3 0x69
#define RXB0D4 0x6A
#define RXB0D5 0x6B
#define RXB0D6 0x6C
#define RXB0D7 0x6D

/* Bit Timing. */
// CNF1.
#define SJW_1TQ 0x40
#define SJW_2TQ 0x80
#define SJW_3TQ 0x90
#define SJW_4TQ 0xC0
// CNF2.
#define BTLMODE_CNF3 0x80
#define BTLMODE_PH1_IPT 0x00
#define SMPL_3X 0x40
#define SMPL_1X 0x00
#define PHSEG1_8TQ 0x38
#define PHSEG1_7TQ 0x30
#define PHSEG1_6TQ 0x28
#define PHSEG1_5TQ 0x20
#define PHSEG1_4TQ 0x18
#define PHSEG1_3TQ 0x10
#define PHSEG1_2TQ 0x08
#define PHSEG1_1TQ 0x00
#define PRSEG_8TQ 0x07
#define PRSEG_7TQ 0x06
#define PRSEG_6TQ 0x05
#define PRSEG_5TQ 0x04
#define PRSEG_4TQ 0x03
#define PRSEG_3TQ 0x02
#define PRSEG_2TQ 0x01
#define PRSEG_1TQ 0x00
// CNF3.
#define PHSEG2_8TQ 0x07
#define PHSEG2_7TQ 0x06
#define PHSEG2_6TQ 0x05
#define PHSEG2_5TQ 0x04
#define PHSEG2_4TQ 0x03
#define PHSEG2_3TQ 0x02
#define PHSEG2_2TQ 0x01
#define PHSEG2_1TQ 0x00
#define SOF_ENABLED 0x80
#define WAKFIL_ENABLED 0x40
#define WAKFIL_DISABLED 0x00

/* DLC. */
#define DLC_0 0x00
#define DLC_1 0x01
#define DLC_2 0x02
#define DLC_3 0x03
#define DLC_4 0x04
#define DLC_5 0x05
#define DLC_6 0x06
#define DLC_7 0x07
#define DLC_8 0x08

/* Commands. */
#define CAN_RESET 0xC0
#define CAN_READ 0x03
#define CAN_WRITE 0x02
#define CAN_RTS 0x80
#define CAN_RTS_TXB0 0x81
#define CAN_RTS_TXB1 0x82
#define CAN_RTS_TXB2 0x84
#define CAN_RD_STATUS 0xA0
#define CAN_BIT_MODIFY 0x05
#define CAN_RX_STATUS 0xB0
#define CAN_RD_RX_BUFF 0x90
#define CAN_LOAD_TX 0X40