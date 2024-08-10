#ifdef TWI_TRANSIMISSION_API

#include <string.h>

enum twi_result_t twi_transmission_init(
    struct twi_t *twi, const twi_u8 addr)
{
  if(twi->buffer.addr != 0) {
    return TWI_BUSY;
  }
  twi->buffer.addr = addr;
  twi->buffer.length = 0;
  return TWI_OK;
}

enum twi_result_t twi_transmission_write(struct twi_t *twi, const twi_u8 *data, twi_u32 length)
{
  if(twi->buffer.addr == 0) {
    return TWI_NOT_INITIALIZED;
  }
  if((twi->buffer.length + length) > TWI_MAX_BUFFER_SIZE) {
    return TWI_MEMORY_ALLOCATION_ERROR;
  }
  memcpy(twi->buffer.buffer + twi->buffer.length, data, length);
  twi->buffer.length += length;
  return TWI_OK;
}

enum twi_result_t twi_transmission_write_byte(struct twi_t *twi, const twi_u8 byte)
{
  return twi_transmission_write(twi, &byte, 1);
}

enum twi_result_t twi_transmission_end(struct twi_t *twi, twi_u32* write_byte_count)
{
  if(twi->buffer.addr == 0 || twi->buffer.length == 0) {
    return TWI_NOT_INITIALIZED;
  }
  *write_byte_count = twi->buffer.length;
  TWI_CHECK_ERROR(twi_write_blocking(twi, twi->buffer.addr, twi->buffer.buffer, write_byte_count, TWI_COMMAND_FLAG_NONE));
  twi->buffer.addr = 0;
  twi->buffer.length = 0;
  return TWI_OK;
}

#endif
