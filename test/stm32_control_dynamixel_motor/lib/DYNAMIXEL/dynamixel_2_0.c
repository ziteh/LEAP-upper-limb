/**
 * @file   dynamixel_2_0.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  Robotis DYNAMIXEL Protocol 2.0.
 * @remark Ref: https://emanual.robotis.com/docs/en/dxl/protocol2/
 */

#include "dynamixel_2_0.h"

#define GET_LOW_ORDER_BYTE(bytes) ((uint8_t)(((uint16_t)(bytes)) & 0xFF))
#define GET_HIGH_ORDER_BYTE(bytes) ((uint8_t)((((uint16_t)(bytes)) >> 8) & 0xFF))

uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void dynamixel2_send_packet(uint8_t id, dynamixel2_instruction_t inst, uint8_t *params, uint16_t params_length);
bool dynamixel2_parse_status_packet(uint8_t *packet, uint32_t packet_length, uint8_t *id, uint8_t *params, uint16_t *params_length, uint8_t *error, bool *crc_check);
bool dynamixel2_get_status_packet(uint8_t *packet, uint16_t *packet_length);

extern uint8_t BUFFER[];
extern uint16_t BUFFER_INDEX;

void max485_send(uint8_t *data, uint32_t length)
{
  /* MAX485 enable Tx, disable Rx. */
  gpio_set(MAX485_DE_RE_GPIO_PORT, MAX485_DE_RE_GPIO_PIN);

  /* Send packet. */
  for (uint32_t i = 0; i < length; i++)
  {
    usart_send_blocking(MAX485_UART, data[i]);
  }

  /* Wait for transmission complete. */
  while (!(USART_SR(MAX485_UART) & USART_SR_TC))
  {
    /* Do nothing. */
  }

  /* MAX485 enable Rx, disable Tx. */
  gpio_clear(MAX485_DE_RE_GPIO_PORT, MAX485_DE_RE_GPIO_PIN);
}

void dynamixel2_write(uint8_t id, uint16_t address, uint8_t *data, uint16_t data_length)
{
  uint32_t params_length = data_length + 2;
  uint8_t params[params_length];

  /* Parameter 1~2: Starting address. */
  params[0] = GET_LOW_ORDER_BYTE(address);
  params[1] = GET_HIGH_ORDER_BYTE(address);

  /* Parameter 3~X: Data. */
  for (uint16_t i = 0; i < data_length; i++)
  {
    params[2 + i] = data[i];
  }

  dynamixel2_send_packet(id, write, params, params_length);
}

bool dynamixel2_read(uint8_t id, uint16_t address, uint16_t data_length, uint8_t *return_data, uint16_t *return_data_length)
{
  uint8_t params[4];

  /* Parameter 1~2: Starting address. */
  params[0] = GET_LOW_ORDER_BYTE(address);
  params[1] = GET_HIGH_ORDER_BYTE(address);

  /* Parameter 2~3: Data length. */
  params[2] = GET_LOW_ORDER_BYTE(data_length);
  params[3] = GET_HIGH_ORDER_BYTE(data_length);

  dynamixel2_clear_receive_buffer();
  dynamixel2_send_packet(id, read, params, 4);

  uint8_t status_packet[64];
  uint16_t status_packet_length;
  uint32_t timeout = 0xAFFFFFFF;
  while (!dynamixel2_get_status_packet(status_packet, &status_packet_length))
  {
    timeout--;
    if (timeout == 0)
    {
      return false; /* Timeout. */
    }
  }

  uint8_t id_r;
  uint8_t error;
  bool crc_check;
  dynamixel2_parse_status_packet(status_packet,
                                 status_packet_length,
                                 &id_r,
                                 return_data,
                                 return_data_length,
                                 &error,
                                 &crc_check);

  return (id_r == id) && (error == 0x00) && (crc_check);
}

void dynamixel2_reset(uint8_t id)
{
  /*
     0xFF: Reset all.
     0x01: Reset all except ID.
     0x02: reset all except ID and Baudrate.
  */
  uint8_t parameter = 0x02;
  dynamixel2_send_packet(id, factory_reset, &parameter, 1);
}

void dynamixel2_set_torque_enable(uint8_t id, bool enable)
{
  uint16_t address = 562;
  uint8_t data = enable ? 1 : 0;
  dynamixel2_write(id, address, &data, 1);
}

void dynamixel2_set_goal_position(uint8_t id, int32_t position)
{
  uint16_t address = 596;
  uint8_t data[4];
  data[0] = (uint8_t)(position & 0xFF);
  data[1] = (uint8_t)((position >> 8) & 0xFF);
  data[2] = (uint8_t)((position >> 16) & 0xFF);
  data[3] = (uint8_t)((position >> 24) & 0xFF);

  dynamixel2_write(id, address, data, 4);
}

int32_t dynamixel2_read_present_position(uint8_t id)
{
  uint16_t address = 611;
  uint8_t return_data[4];
  uint16_t return_data_length;

  dynamixel2_read(id, address, 4, return_data, &return_data_length);

  int32_t position = return_data[0] +
                     ((return_data[1] << 8) & 0xFF00) +
                     ((return_data[2] << 16) & 0xFF0000) +
                     ((return_data[3] << 24) & 0xFF000000);
  return position;
}

void dynamixel2_send_packet(uint8_t id, dynamixel2_instruction_t inst, uint8_t *params, uint16_t params_length)
{
  uint32_t packet_length = 10 + params_length;
  uint8_t packet[packet_length];
  packet[0] = 0xFF; /* Header 1. */
  packet[1] = 0xFF; /* Hedaer 2. */
  packet[2] = 0xFD; /* Hedaer 3. */
  packet[3] = 0x00; /* Reserved. */

  packet[4] = id; /* Packet ID. */

  /* Length = Parameter length + 3. */
  packet[5] = GET_LOW_ORDER_BYTE(params_length + 3);  /* Length 1 (Low-order byte). */
  packet[6] = GET_HIGH_ORDER_BYTE(params_length + 3); /* Lenget 2 (High-order byte). */

  packet[7] = (uint8_t)inst; /* Instrucion. */

  /* Parameter 1~X. */
  for (uint16_t i = 0; i < params_length; i++)
  {
    packet[8 + i] = params[i];
  }

  /* CRC. */
  uint16_t crc = update_crc(0, packet, packet_length - 2); /* Calculating CRC. */
  packet[packet_length - 2] = GET_LOW_ORDER_BYTE(crc);     /* CRC 1 (Low-order byte). */
  packet[packet_length - 1] = GET_HIGH_ORDER_BYTE(crc);    /* CRC 2 (High-order byte). */

  max485_send(packet, packet_length);
}

void dynamixel2_receive_callback(uint8_t received_data)
{
  buffer[buffer_index] = received_data;

  if (buffer_index < (BUFFER_LENGTH - 1))
  {
    buffer_index++;
  }
  else
  {
    buffer_index = 0;
  }
}

void dynamixel2_clear_receive_buffer(void)
{
  for (int i = 0; i < BUFFER_LENGTH; i++)
  {
    buffer[i] = 0;
  }
  buffer_index = 0;
}

bool dynamixel2_parse_status_packet(uint8_t *packet, uint32_t packet_length, uint8_t *id, uint8_t *params, uint16_t *params_length, uint8_t *error, bool *crc_check)
{
  /* Check instruction. */
  if (packet[7] == 0x55)
  {
    *id = packet[4];
    *error = packet[8];

    uint16_t length = packet[5] + ((uint16_t)(packet[6] << 8) & 0xFF00);
    *params_length = length - 3 - 1;
    for (uint16_t i = 0; i < *params_length; i++)
    {
      params[i] = packet[i + 9];
    }

    /* CRC. */
    uint16_t crc = update_crc(0, packet, packet_length - 2);            /* Calculating CRC. */
    bool crc_l = packet[packet_length - 2] == GET_LOW_ORDER_BYTE(crc);  /* CRC 1 (Low-order byte). */
    bool crc_h = packet[packet_length - 1] == GET_HIGH_ORDER_BYTE(crc); /* CRC 2 (High-order byte). */
    *crc_check = (crc_l && crc_h);

    return true;
  }
  return false;
}

bool dynamixel2_get_status_packet(uint8_t *packet, uint16_t *packet_length)
{
  if (buffer_index < 10)
  {
    /* Receive didn't complete. */
    return false;
  }

  uint16_t packet_starting_index = 0;
  bool header_ok = false;

  /* Find the head of packet. */
  while (!header_ok)
  {
    bool header1 = buffer[packet_starting_index] == 0xFF;
    bool header2 = buffer[packet_starting_index + 1] == 0xFF;
    bool header3 = buffer[packet_starting_index + 2] == 0xFD;
    if (!(header1 && header2 && header3))
    {
      if (packet_starting_index < (buffer_index - 10))
      {
        packet_starting_index++;
        continue;
      }
      else
      {
        /* Receive didn't complete. */
        return false;
      }
    }
    else
    {
      header_ok = true;
    }
  }

  uint16_t length = buffer[packet_starting_index + 5] + (buffer[packet_starting_index + 6] << 8);
  uint16_t packet_ending_index = packet_starting_index + (length - 3) + 9;

  if (packet_ending_index > buffer_index)
  {
    /* Receive didn't complete. */
    return false;
  }

  /* Catch. */
  *packet_length = packet_ending_index - packet_starting_index + 1;
  for (uint16_t i = 0; i < *packet_length; i++)
  {
    packet[i] = buffer[packet_starting_index + i];
  }

  /* Clear. */
  for (uint16_t i = 0; i < (packet_ending_index + 1); i++)
  {
    buffer[i] = 0;
  }

  /* Move. */
  for (uint16_t i = 0; i < (buffer_index - packet_ending_index); i++)
  {
    buffer[i] = buffer[packet_ending_index + i + 1];
    buffer[packet_ending_index + i + 1] = 0;
  }
  buffer_index = buffer_index - packet_ending_index;

  return true;
}

/* Source: https://emanual.robotis.com/docs/en/dxl/crc/ */
uint16_t update_crc(uint16_t crc_accum,
                    uint8_t *data_blk_ptr,
                    uint16_t data_blk_size)
{
  uint16_t i, j;
  uint16_t crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}
