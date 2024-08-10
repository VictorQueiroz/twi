#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#ifdef TWI_TRANSIMISSION_API
  /**
   * Transmission API
   */
  enum twi_result_t twi_transmission_init(
      struct twi_t *twi, twi_u8 addr);

enum twi_result_t twi_transmission_write(struct twi_t *twi, const twi_u8 *data, twi_u32 length);
enum twi_result_t twi_transmission_write_byte(struct twi_t *twi, twi_u8 byte);

/**
 * Write data to the transmission buffer
 * @param twi TWI instance
 * @param write_byte_count Written byte count
 * @return TWI_OK if the transmission was successful
 */
enum twi_result_t twi_transmission_end(struct twi_t *twi, twi_u32* write_byte_count);

#endif 

#endif //TRANSMISSION_H

#ifdef __cplusplus
}
#endif // __cplusplus
