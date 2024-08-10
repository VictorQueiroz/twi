#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifndef TWI_H
#define TWI_H

#define TWI_MAX_BUFFER_SIZE 32

#include <twi/config.h>
  
  enum twi_result_t
  {
    TWI_OK = 0,
    /**
     * Internal failure
     */
    TWI_IMPLEMENTATION_ERROR = 1,
    TWI_MEMORY_ALLOCATION_ERROR = 2,
    TWI_NOT_INITIALIZED = 3,
    TWI_BUSY = 3,
    TWI_NOT_FOUND = 4,
    TWI_MISMATCH = 5,
    TWI_BAD_ARGUMENT = 6
  };

#include <stdio.h>

#define TWI_CHECK_ERROR(expr)                                                         \
  do                                                                                  \
  {                                                                                   \
    enum twi_result_t result = expr;                                                  \
    if (result != TWI_OK) {                                                           \
      printf("%s:%d: \"%s\" failed with: %d\n", __FILE__, __LINE__, #expr, result);   \
      return result;                                                                  \
    }                                                                                 \
  } while (0)

  enum twi_command_flag_t
  {
    TWI_COMMAND_FLAG_NONE = 0,
    /**
     * If set the bus will be released after the command is completed.
     *
     * This flag should be checked by the implementation.
     */
    TWI_COMMAND_FLAG_NO_STOP = 2,
  };

  typedef enum twi_result_t(twi_read_callback_t)(twi_u8 addr, twi_u8 *data,
      twi_u32 *length, enum twi_command_flag_t flags, void *user_data);
  typedef enum twi_result_t(twi_read_byte_callback_t)(
      twi_u8 addr, twi_u8 *byte, void *user_data);
  typedef enum twi_result_t(twi_write_callback_t)(twi_u8 addr,
      const twi_u8 *data, twi_u32 *length, enum twi_command_flag_t flags,
      void *user_data);
  typedef enum twi_result_t(twi_write_byte_callback_t)(
      twi_u8 addr, twi_u8 byte, void *user_data);
  typedef enum twi_result_t(twi_get_available_byte_count_callback_t)(
      twi_size_t *available, void *user_data);

  struct twi_callbacks_t
  {
    twi_write_callback_t *write_blocking;
    twi_write_byte_callback_t *write_byte;
    twi_read_callback_t *read_blocking;
    twi_read_byte_callback_t *read_byte;
    twi_get_available_byte_count_callback_t *get_write_available;
    twi_get_available_byte_count_callback_t *get_read_available;
  };

  struct twi_buffer_t
  {
    twi_u8 buffer[TWI_MAX_BUFFER_SIZE];
    twi_u8 addr;
    twi_u32 length;
  };

  struct twi_t
  {
    struct twi_buffer_t buffer;
    struct twi_callbacks_t callbacks;
    void *user_data;
    // 1 if the bus should not be released after a write operation
    twi_u8 release_after : 1;
  };

  enum twi_result_t twi_init(
      struct twi_t *twi, struct twi_callbacks_t *callbacks, void *user_data);

  enum twi_result_t twi_write_blocking(const struct twi_t *twi, twi_u8 addr,
      const twi_u8 *data, twi_u32 *length, enum twi_command_flag_t flags);

  /**
   *
   * @param twi
   * @param addr
   * @param data
   * @param length
   * @param flags
   * @return
   */
  enum twi_result_t twi_read_blocking(const struct twi_t *twi, twi_u8 addr,
      twi_u8 *data, twi_u32 *length, enum twi_command_flag_t flags);

  enum twi_result_t twi_get_write_available(
      const struct twi_t *twi, twi_size_t *available);

  enum twi_result_t twi_get_read_available(
      const struct twi_t *twi, twi_size_t *available);

  enum twi_result_t twi_write_byte(
      const struct twi_t *twi, twi_u8 addr, twi_u8 byte);

  enum twi_result_t twi_read_byte(
      const struct twi_t *twi, twi_u8 addr, twi_u8 *byte);

#endif // TWI_H

#ifdef __cplusplus
}
#endif // __cplusplus
