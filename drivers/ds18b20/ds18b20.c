#include "ds18b20.h"

#include <string.h>

enum ds18b20_command_t {
  DS18B20_READ_ROM = 0x33,
  DS18B20_CONVERT_T = 0x44,
  DS18B20_READ_SCRATCHPAD = 0xBE,
  DS18B20_MATCH_ROM = 0x55,
};

enum twi_result_t ds18b20_match_rom(const struct ds18b20_t* sensor, const twi_u64 rom_code)
{
  struct owi_t* owi = sensor->owi;

  TWI_CHECK_ERROR(owi_write_byte(
    owi,
    DS18B20_MATCH_ROM
  ));

  TWI_CHECK_ERROR(owi_write(owi, (twi_u8*) &rom_code, OWI_ROM_CODE_BYTE_LENGTH));

  return TWI_OK;
}

enum twi_result_t ds18b20_init(struct ds18b20_t* sensor, struct owi_t* owi)
{
  sensor->owi = owi;
  sensor->power_mode = DS18B20_POWER_EXTERNAL;

  return TWI_OK;
}

enum twi_result_t ds18b20_send_byte_command(const struct ds18b20_t* sensor, const enum ds18b20_command_t command)
{
  struct owi_t* owi = sensor->owi;

  TWI_CHECK_ERROR(owi_write(
    owi,
    &command,
    1
  ));

  return TWI_OK;
}

enum twi_result_t ds18b20_read_scratchpad(struct ds18b20_t* sensor, twi_u8* scratchpad, twi_u32 len)
{
  struct owi_t* owi = sensor->owi;

  TWI_CHECK_ERROR(owi_write_byte(owi, DS18B20_READ_SCRATCHPAD));

  TWI_CHECK_ERROR(owi_read(owi, scratchpad, len));

  return TWI_OK;
}

enum twi_result_t ds18b20_read_scratchpad_float(struct ds18b20_t* sensor, float* temperature)
{
  twi_u8 scratchpad[DS18B20_SCRATCHPAD_MAX_BYTE_LENGTH];
  TWI_CHECK_ERROR(ds18b20_read_scratchpad(sensor, scratchpad, DS18B20_SCRATCHPAD_MAX_BYTE_LENGTH));

  twi_s16 rawTemperature = ((twi_s16)scratchpad[1] << 8) | (twi_s16)scratchpad[0];

  *temperature = (float)rawTemperature / 16.0f;

  return TWI_OK;
}

enum twi_result_t ds18b20_read_rom(const struct ds18b20_t* sensor, twi_u64* rom_code)
{
  const struct owi_t* owi = sensor->owi;

  TWI_CHECK_ERROR(owi_write_byte(owi, DS18B20_READ_ROM));

  // Transform `rom_code_buffer` into `rom_code`
  *rom_code = 0;

  twi_u8 bit_value;
  for(twi_u8 i = 0; i < OWI_ROM_CODE_BYTE_LENGTH; i++)
  {
    for(twi_u8 bit_index = 0; bit_index < 8; bit_index++)
    {
      bit_value = owi_read_bit(owi);
      *rom_code |= (twi_u64)bit_value << (i * 8 + bit_index);
    }
  }

  printf("%llu\n", *rom_code);

  if(owi_rom_code_crc(*rom_code) != 0) {
    *rom_code = 0;
    return TWI_MISMATCH;
  }

  return TWI_OK;
}

enum twi_result_t ds18b20_update(struct ds18b20_t* sensor) {
  struct owi_rom_code_t rom_codes[20];
  twi_u8 rom_code_count = 0;

  memset(rom_codes, 0, 20 * sizeof(struct owi_rom_code_t));

  rom_code_count = 1;

  TWI_CHECK_ERROR(owi_search_rom(sensor->owi, rom_codes, &rom_code_count, 20));

  float temp;
  twi_u8 temperature_state;

  for(twi_u8 i = 0; i < rom_code_count; i++) {
    const struct owi_rom_code_t* rom_code = &rom_codes[i];

    printf("ROM code: %llu\n", rom_code->rom_code);

    TWI_CHECK_ERROR(owi_reset(sensor->owi));

    TWI_CHECK_ERROR(ds18b20_match_rom(sensor, rom_code->rom_code));

    TWI_CHECK_ERROR(owi_write_byte(sensor->owi, DS18B20_CONVERT_T));

    switch(sensor->power_mode)
    {
      case DS18B20_POWER_PARASITE:
        // TODO: sleep
        return TWI_IMPLEMENTATION_ERROR;
      case DS18B20_POWER_EXTERNAL:
      {
        do {
          temperature_state = owi_read_bit(sensor->owi);
        } while(temperature_state == 0);
        break;
      }
    }

    TWI_CHECK_ERROR(owi_reset(sensor->owi));

    TWI_CHECK_ERROR(ds18b20_match_rom(sensor, rom_code->rom_code));

    TWI_CHECK_ERROR(ds18b20_read_scratchpad_float(sensor, &temp));

    printf("Temperature: %.2f\n", temp);
  }

  return TWI_OK;
}