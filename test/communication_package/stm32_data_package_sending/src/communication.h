/**
 * @file   communication.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Define communication.
 */

#ifndef COMMNUICATION_H_
#define COMMNUICATION_H_

#define EOT (0xff)

#define BUFFER_LENGTH (8)

#define HEADER_MASK (0x80)

/* MSB = 1: is header, else  payload. */
#define IS_HEADER(b) ((b & HEADER_MASK) == HEADER_MASK)

#define MOTOR_BASIC_CONTROL_HEADER (0x80)
#define MOTOR_BASIC_CONTROL_PAYLOAD_NUMBER (2)

#define MOTOR_POSITION_CONTROL_HEADER (0x81)
#define MOTOR_POSITION_CONTROL_PAYLOAD_NUMBER (3)

#define REQUEST_MOTOR_STATE_HEADER (0x87)
#define REQUEST_MOTOR_STATE_PAYLOAD_NUMBER (1)

#define MOTOR_STATE_HEADER (0x8e)
#define MOTOR_STATE_PAYLOAD_NUMBER (4)

#define REQUEST_FORCE_SENSOR_VALUE_HEADER (0x86)
#define REQUEST_FORCE_SENSOR_VALUE_PAYLOAD_NUMBER (1)

#define FORCE_SENSOR_VALUE_HEADER (0x8d)
#define FORCE_SENSOR_VALUE_PAYLOAD_NUMBER (7)

#define DO_NOTHING_HEADER (0x8f)

#endif /* COMMNUICATION_H_ */