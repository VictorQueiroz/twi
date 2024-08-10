#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifndef ONE_WIRE_H_
#define ONE_WIRE_H_

#include <twi/twi.h>

  enum owi_search_rom_read_result_t
  {
    OWI_SEARCH_ROM_READ_RESULT_BIT_DISCREPANCY = 0b00,
    OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ZEROES = 0b01,
    OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ONES = 0b10,
    OWI_SEARCH_ROM_READ_RESULT_BIT_NOT_FOUND = 0b11
  };

#define OWI_TIME_RSTL 480

#define OWI_TIME_RSTH_MIN 480

  // Presence-detect
#define OWI_TIME_PDLOW_MIN 60
#define OWI_TIME_PDLOW_MAX 240
#define OWI_TIME_PDHIGH_MIN 15
#define OWI_TIME_PDHIGH_MAX 60

  // Write LOW time
#define OWI_TIME_TLOW0_MIN 60
#define OWI_TIME_TLOW0_MAX 120

  // Write HIGH time
#define OWI_TIME_TLOW1_MIN 1
#define OWI_TIME_TLOW1_MAX 15

  // Time slot
#define OWI_TIME_SLOT_MIN 60
#define OWI_TIME_SLOT_MAX 120

  // Recovery Time
#define OWI_TIME_RECOVERY 1

#define OWI_TIME_READ_DATA_VALID 15

#define OWI_GET_BIT(value, bit) ((value >> bit) & 1)

#define OWI_ROM_CODE_BYTE_LENGTH ((twi_u8)8)

  enum owi_command_t
  {
    OWI_COMMAND_SEARCH_ROM = 0xF0,
    OWI_COMMAND_SKIP_ROM = 0xCC,
  };

  struct owi_t
  {
    twi_u8 pin;
  };

  enum twi_result_t owi_reset(const struct owi_t *owi);

  enum twi_result_t owi_init(struct owi_t *owi, twi_u8 pin);

  enum twi_result_t owi_write_bit(
    const struct owi_t* owi,
    twi_u8 bit
  );

  struct owi_rom_code_t
  {
    twi_u64 rom_code;
    /**
     * Bit count that we're missing to read to have a complete ROM code, zero if the
     * ROM is ready.
     */
    twi_u8 bit_count;
  };

  /**
   * Search all the ROM codes on the bus.
   * @param owi OWI interface
   * @param rom_codes Array to put the ROM codes into
   * @param rom_code_count Count of ROM codes found. It must be one for the first search
   * @param max_rom_code_count Maximum amount of ROM code structs that the `rom_codes` array can fit
   * @return TWI_OK if the search was successful, TWI_NOT_FOUND if no ROM codes were found
   */
  enum twi_result_t owi_search_rom(const struct owi_t* owi, struct owi_rom_code_t* rom_codes, twi_u8* rom_code_count, twi_u8 max_rom_code_count);

  enum twi_result_t owi_write_byte(
    const struct owi_t *owi,
    twi_u8 byte
  );

  twi_u8 owi_rom_code_crc(twi_u64);

  enum twi_result_t owi_write(
    struct owi_t *owi,
    const twi_u8 *data, twi_u32 len
  );

  twi_u8 owi_read_bit(
    const struct owi_t* owi
  );

  twi_u8 owi_read_byte(const struct owi_t* owi);

  enum twi_result_t owi_read(
    const struct owi_t* owi,
    twi_u8 *dest, twi_u32 len
  );

#endif // ONE_WIRE_H_

#ifdef __cplusplus
}
#endif // __cplusplus
