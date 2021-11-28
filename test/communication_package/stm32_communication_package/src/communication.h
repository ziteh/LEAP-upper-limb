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

#define MOTOR_CONTROL_INFO_BYTE (0x80)
#define MOTOR_CONTROL_DATA_BYTE_NUMBER (4)

#endif /* COMMNUICATION_H_ */