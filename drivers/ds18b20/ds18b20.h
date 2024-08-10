#ifndef DS18B20_H
#define DS18B20_H

#include "twi.h"
#include "owi.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define DS18B20_SCRATCHPAD_MAX_BYTE_LENGTH 9

enum ds18b20_power_mode_t
{
  DS18B20_POWER_PARASITE,
  DS18B20_POWER_EXTERNAL
};

struct ds18b20_t
{
  struct owi_t* owi;
  enum ds18b20_power_mode_t power_mode;
};

  enum twi_result_t ds18b20_init(struct ds18b20_t*, struct owi_t*);

  enum twi_result_t ds18b20_update(struct ds18b20_t* sensor);

  /**
   * Read the scratchpad of the DS18B20 sensor
   * @param sensor Sensor instnace
   * @param temperature Output temperature in degrees Celsius
   * @return TWI_OK if the temperature was read successfully
   */
  enum twi_result_t ds18b20_read_scratchpad_float(struct ds18b20_t* sensor, float* temperature);

  struct ds18b20_rom_t
  {
    struct ds18b20_t* sensor;
  };


#ifdef __cplusplus
}
#endif // __cplusplus

#endif //DS18B20_H
