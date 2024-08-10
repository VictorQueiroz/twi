#ifndef TWI_PICO_H
#define TWI_PICO_H

#include <twi/twi.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  enum twi_result_t pico_i2c_write_blocking(const twi_u8 addr,
      const twi_u8 *data, twi_u32 *len, enum twi_command_flag_t flags,
      void *user_data);

  enum twi_result_t pico_i2c_read_blocking(const twi_u8 addr, twi_u8 *dest,
      twi_u32 *len, enum twi_command_flag_t flags, void *user_data);

  enum twi_result_t pico_i2c_get_write_available(
      size_t *available, void *user_data);

  enum twi_result_t pico_i2c_get_read_available(
      size_t *available, void *user_data);

  enum twi_result_t pico_i2c_write_byte_raw(
      twi_u8 addr, twi_u8 byte, void *user_data);

  enum twi_result_t pico_i2c_read_byte_raw(
      twi_u8 addr, twi_u8 *byte, void *user_data);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // TWI_PICO_H
