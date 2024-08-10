#include <owi/owi.h>

enum twi_result_t owi_init(struct owi_t *owi, twi_u8 pin)
{
  owi->pin = pin;
  return TWI_OK;
}

enum twi_result_t owi_search_rom_read_result(const struct owi_t* owi, enum owi_search_rom_read_result_t* read_result)
{
  const twi_u8 true_bit = owi_read_bit(owi);
  const twi_u8 complement_bit = owi_read_bit(owi);

  *read_result = (true_bit << 1) | complement_bit;

  switch(*read_result)
  {
    case OWI_SEARCH_ROM_READ_RESULT_BIT_DISCREPANCY:
    case OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ONES:
    case OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ZEROES:
      break;
    case OWI_SEARCH_ROM_READ_RESULT_BIT_NOT_FOUND:
      return TWI_NOT_FOUND;
    default:
      return TWI_IMPLEMENTATION_ERROR;
  }

  return TWI_OK;
}


/**
 * Go to a specific position of the tree, taking decisions based on the given rom_code
 * argument.
 *
 * This function allow us to find a specific ROM code on the bus if a full 64-bit ROM code
 * is given.
 *
 * If a 20-bit ROM code is given, the function will go to the 20th bit and then return. The caller
 * might want to continue from there.
 *
 * This function will restart the search ROM algorithm from the beginning.
 *
 * The function will require the search to match exactly the given ROM code. For instance,
 * if the given ROM code has a zero on bit 0, and the search bit result is `OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ONES`,
 * this function will return `TWI_MISMATCH`.
 * @param owi OWI interface
 * @param rom_code Partial ROM code that we want to go to
 * @param bit_count Number of bits that are present in the rom_code argument.
 * @return TWI_OK if the ROM code was found, TWI_NOT_FOUND if the ROM code was not found
 */
static enum twi_result_t owi_search_rom_until(
  const struct owi_t* owi,
  twi_u64 rom_code,
  twi_u8 bit_count
)
{
  // 1. The master begins the initialization sequence by issuing a Reset Pulse.
  TWI_CHECK_ERROR(owi_reset(owi));

  // 2. The master will then issue the Search ROM command on the 1–Wire Bus.
  TWI_CHECK_ERROR(owi_write_byte(owi, OWI_COMMAND_SEARCH_ROM));

  enum owi_search_rom_read_result_t read_result;
  twi_u8 write_bit;

  for(twi_u8 bit_position = 0; bit_position < bit_count; bit_position++)
  {
    TWI_CHECK_ERROR(owi_search_rom_read_result(owi, &read_result));

    switch(read_result)
    {
    case OWI_SEARCH_ROM_READ_RESULT_BIT_DISCREPANCY:
      // Decide which path to take in case of discrepancy
      write_bit = OWI_GET_BIT(rom_code, bit_position);
      break;
    case OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ONES:
      // If the bit is 1, the master must select the 1 branch.
      write_bit = 1;
      break;
    case OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ZEROES:
      // If the bit is 0, the master must select the 0 branch.
      write_bit = 0;
      break;
    case OWI_SEARCH_ROM_READ_RESULT_BIT_NOT_FOUND:
      return TWI_NOT_FOUND;
    default:
      return TWI_IMPLEMENTATION_ERROR;
    }

    /**
     * If the given ROM code has X on bit position 0, and the search bit result does not corroborate with this value,
     * `TWI_MISMATCH` will be returned.
     */
    if(OWI_GET_BIT(rom_code, bit_position) != write_bit)
    {
      return TWI_MISMATCH;
    }

    TWI_CHECK_ERROR(owi_write_bit(owi, write_bit));
  }

  return TWI_OK;
}

enum twi_result_t owi_search_rom(const struct owi_t* owi, struct owi_rom_code_t* rom_codes, twi_u8* rom_code_count, const twi_u8 max_rom_code_count) {
  // The rom_codes should not be NULL, and `max_rom_code_count` cannot be lower than one
  if(rom_codes == NULL || max_rom_code_count < 1) {
    return TWI_BAD_ARGUMENT;
  }

  // The ROM code count should be greater than 0
  if(max_rom_code_count == 0)
  {
    return TWI_BAD_ARGUMENT;
  }

  twi_u32 current_rom_code = 0;
  twi_u8 write_bit = 0;

  while(current_rom_code < *rom_code_count)
  {
    struct owi_rom_code_t* rom_code = &rom_codes[current_rom_code];

    TWI_CHECK_ERROR(owi_search_rom_until(owi, rom_code->rom_code, rom_code->bit_count));

    while(rom_code->bit_count < OWI_ROM_CODE_BYTE_LENGTH * 8) {
      enum owi_search_rom_read_result_t read_result;

      TWI_CHECK_ERROR(owi_search_rom_read_result(owi, &read_result));

      switch(read_result)
      {
        case OWI_SEARCH_ROM_READ_RESULT_BIT_DISCREPANCY: {
          write_bit = 2;

          // Add another ROM code with the value set to one
          if((max_rom_code_count - *rom_code_count) < 1)
          {
            return TWI_MEMORY_ALLOCATION_ERROR;
          }
          struct owi_rom_code_t* new_rom_code = &rom_codes[*rom_code_count];
          // No need to increase the bit count anymore, since we already did it in the previous step
          new_rom_code->bit_count = rom_code->bit_count + 1;
          new_rom_code->rom_code =
              rom_code->rom_code | (1 << rom_code->bit_count);
          *rom_code_count += 1;

          // Increase the bit count for the current ROM code. This is the zero version of the discrepancy resolution
          rom_code->bit_count++;
          break;
        }
        case OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ONES:
          // If the bit is 1, the master must select the 1 branch.
          write_bit = 1;
          break;
        case OWI_SEARCH_ROM_READ_RESULT_BIT_ONLY_ZEROES:
          // If the bit is 0, the master must select the 0 branch.
          write_bit = 0;
          break;
        case OWI_SEARCH_ROM_READ_RESULT_BIT_NOT_FOUND:
          return TWI_NOT_FOUND;
        default:
          return TWI_IMPLEMENTATION_ERROR;
      }

      // If nothing was assigned to `write_bit`, this is an internal failure
      if(write_bit == 3)
      {
        return TWI_IMPLEMENTATION_ERROR;
      }

      /**
       * If `write_bit` is two, then it means we've found a discrepancy. We've updated the current
       * ROM code with two versions of it. One with a leading one, and another with a leading zero.
       *
       * So, we break the loop immediately, so we can reiterate the current ROM code and the next new one at the end
       * of the loop.
       */
      if(write_bit == 2)
      {
        break;
      }

      TWI_CHECK_ERROR(owi_write_bit(owi, write_bit));

      // Set the bit in the found ROM code
      rom_code->rom_code |= (twi_u64)write_bit << rom_code->bit_count;

      // Increment the bit index
      rom_code->bit_count++;
    }

    /**
     * We do not update the current rom code index so the current ROM code can be re-iterated.
     */
    if(write_bit == 2)
    {
      continue;
    }

    // Increment the current ROM code index
    current_rom_code++;
  }

  // Check CRC for all ROM codes
  for(twi_u32 i = 0; i < *rom_code_count; i++)
  {
    if(owi_rom_code_crc(rom_codes[i].rom_code) != 0)
    {
      return TWI_MISMATCH;
    }
  }

  return TWI_OK;
}

twi_u8 owi_rom_code_crc(const twi_u64 rom_code) {
  twi_u8 crc = 0;
  twi_u8 data[8];

  // Split the 64-bit rom_code into an 8-byte array
  for (int i = 0; i < OWI_ROM_CODE_BYTE_LENGTH; i++) {
    data[i] = (rom_code >> (i * 8)) & 0xFF;
  }

  for (twi_u8 j = 0; j < OWI_ROM_CODE_BYTE_LENGTH; j++) {
    twi_u8 inbyte = data[j];
    for (twi_u8 i = 8; i; i--) {
      const twi_u8 mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

enum twi_result_t owi_write_byte(
  const struct owi_t *owi,
  const twi_u8 byte
)
{
  for (twi_u8 bit_index = 0; bit_index < 8; bit_index++) {
    TWI_CHECK_ERROR(owi_write_bit(owi, (byte >> bit_index) & 1));
  }

  return TWI_OK;
}

enum twi_result_t owi_write(
  struct owi_t *owi,
  const twi_u8 *data, const twi_u32 len
)
{
  for (twi_u32 i = 0; i < len; i++) {
    TWI_CHECK_ERROR(owi_write_byte(owi, data[i]));
  }

  return TWI_OK;
}

twi_u8 owi_read_byte(const struct owi_t* owi)
{
  twi_u8 result = 0;

  for(twi_u8 bit = 0; bit < 8; bit++) {
    // Read the bit
    if(owi_read_bit(owi))
    {
      result |= 1 << bit;
    }
  }

  return result;
}

enum twi_result_t owi_read(
  const struct owi_t* owi,
  twi_u8 *dest, const twi_u32 len
)
{
  // P. 16: Figure 16. Read/Write Time Slot Timing Diagram
  for(twi_u32 i = 0; i < len; i++)
  {
    dest[i] = owi_read_byte(owi);
  }

  return TWI_OK;
}

enum twi_result_t pico_owi_get_write_available(
    size_t *available, void *user_data)
{
  return 0;
}

enum twi_result_t pico_owi_get_read_available(
    size_t *available, void *user_data)
{
  return 0;
}