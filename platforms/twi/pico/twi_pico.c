#include "twi_pico.h"

#include <hardware/i2c.h>

enum twi_result_t pico_i2c_write_blocking(const twi_u8 addr, const twi_u8 *data,
    twi_u32 *len, enum twi_command_flag_t flags, void *user_data)
{
  struct i2c_inst *i2c = (struct i2c_inst *)(user_data);
  const int written_bytes = i2c_write_blocking(
      i2c, addr, data, *len, (flags & TWI_COMMAND_FLAG_NO_STOP) ? 1 : 0);
  if (written_bytes == PICO_ERROR_GENERIC)
  {
    return TWI_IMPLEMENTATION_ERROR;
  }
  *len = written_bytes;
  return TWI_OK;
}

enum twi_result_t pico_i2c_write_byte_raw(
    const twi_u8 addr, twi_u8 byte, void *user_data)
{
  struct i2c_inst *i2c = (struct i2c_inst *)(user_data);
  i2c_write_byte_raw(i2c, byte);
  return TWI_OK;
}

enum twi_result_t pico_i2c_read_byte_raw(
    const twi_u8 addr, twi_u8 *byte, void *user_data)
{
  struct i2c_inst *i2c = (struct i2c_inst *)(user_data);
  *byte = i2c_read_byte_raw(i2c);
  return TWI_OK;
}

enum twi_result_t pico_i2c_read_blocking(const twi_u8 addr, twi_u8 *dest,
    twi_u32 *len, enum twi_command_flag_t flags, void *user_data)
{
  struct i2c_inst *i2c = (struct i2c_inst *)(user_data);
  const int read_byte_count = i2c_read_blocking(
      i2c, addr, dest, *len, (flags & TWI_COMMAND_FLAG_NO_STOP) ? 1 : 0);
  if (read_byte_count == PICO_ERROR_GENERIC)
  {
    return TWI_IMPLEMENTATION_ERROR;
  }
  if (read_byte_count != *len)
  {
    return TWI_MEMORY_ALLOCATION_ERROR;
  }
  // Set the actual read byte count
  *len = read_byte_count;
  return TWI_OK;
}

enum twi_result_t pico_i2c_get_write_available(
    size_t *available, void *user_data)
{
  struct i2c_inst *i2c = (struct i2c_inst *)(user_data);
  *available = i2c_get_write_available(i2c);
  return TWI_OK;
}

enum twi_result_t pico_i2c_get_read_available(
    size_t *available, void *user_data)
{
  struct i2c_inst *i2c = (struct i2c_inst *)(user_data);
  *available = i2c_get_read_available(i2c);
  return TWI_OK;
}