#include <twi/twi.h>

enum twi_result_t twi_init(
    struct twi_t *twi, struct twi_callbacks_t *callbacks, void *user_data)
{
  twi->callbacks.read_byte = callbacks->read_byte;
  twi->callbacks.read_blocking = callbacks->read_blocking;
  twi->callbacks.write_byte = callbacks->write_byte;
  twi->callbacks.write_blocking = callbacks->write_blocking;
  twi->callbacks.get_write_available = callbacks->get_write_available;
  twi->callbacks.get_read_available = callbacks->get_read_available;
  twi->user_data = user_data;
  return TWI_OK;
}

enum twi_result_t twi_write_blocking(const struct twi_t *twi, const twi_u8 addr,
    const twi_u8 *data, twi_u32 *length, const enum twi_command_flag_t flags)
{
  /**
   * Clear the incoming byte length in order to avoid having a wrong
   * byte count if something is successful but not all bytes are written.
   */
  twi_u32 byte_count = *length;
  *length = 0;
  TWI_CHECK_ERROR(
      twi->callbacks.write_blocking(addr, data, &byte_count, flags, twi->user_data));
  /**
   * Assign the returned byte count to the output parameter.
   */
  *length = byte_count;
  return TWI_OK;
}

// ! Make sure we find a convenient way to return the read byte count
enum twi_result_t twi_read_blocking(const struct twi_t *twi, const twi_u8 addr,
    twi_u8 *data, twi_u32 *length, const enum twi_command_flag_t flags)
{
  TWI_CHECK_ERROR(
      twi->callbacks.read_blocking(addr, data, length, flags, twi->user_data));
  return TWI_OK;
}

enum twi_result_t twi_get_write_available(
    const struct twi_t *twi, twi_size_t *available)
{
  TWI_CHECK_ERROR(
      twi->callbacks.get_write_available(available, twi->user_data));
  return TWI_OK;
}

enum twi_result_t twi_get_read_available(
    const struct twi_t *twi, twi_size_t *available)
{
  TWI_CHECK_ERROR(twi->callbacks.get_read_available(available, twi->user_data));
  return TWI_OK;
}

enum twi_result_t twi_write_byte(
    const struct twi_t *twi, const twi_u8 addr, const twi_u8 byte)
{
  TWI_CHECK_ERROR(twi->callbacks.write_byte(addr, byte, twi->user_data));
  return TWI_OK;
}

enum twi_result_t twi_read_byte(
    const struct twi_t *twi, const twi_u8 addr, twi_u8 *byte)
{
  TWI_CHECK_ERROR(twi->callbacks.read_byte(addr, byte, twi->user_data));
  return TWI_OK;
}
