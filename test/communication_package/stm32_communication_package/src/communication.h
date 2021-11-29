/**
 * @file   communication.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Define communication.
 */

#ifndef COMMNUICATION_H_
#define COMMNUICATION_H_

#define BUFFER_LENGTH (8)

#define INFO_BYTE_MASK (0x80)

/* MSB = 1: is info byte, else  data byte. */
#define IS_INFO_BYTE(b) ((b & INFO_BYTE_MASK) == INFO_BYTE_MASK)

#define MOTOR_BASIC_CONTROL_INFO_BYTE (0x80)
#define MOTOR_BASIC_CONTROL_DATA_BYTE_NUMBER (2)

#define MOTOR_POSITION_CONTROL_INFO_BYTE (0x81)
#define MOTOR_POSITION_CONTROL_DATA_BYTE_NUMBER (3)

#define MOTOR_STATE_INFO_BYTE (0x8e)
#define MOTOR_STATE_DATA_BYTE_NUMBER (4)

#define REQUEST_MOTOR_STATE_INFO_BYTE (0x87)
#define REQUEST_MOTOR_STATE_DATA_BYTE_NUMBER (1)

#define DO_NOTHING_INFO_BYTE (0x8f)

#endif /* COMMNUICATION_H_ */