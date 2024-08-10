#include "max30100/max30100.h"

#include <stdio.h>

enum twi_result_t max30100_read(const struct max30100_t *sensor,
    const twi_u8 target, twi_u8 *out, twi_u32 out_len)
{
  twi_u32 write_byte_count = 1;
  TWI_CHECK_ERROR(twi_write_blocking(sensor->twi, sensor->addr, &target,
      &write_byte_count, TWI_COMMAND_FLAG_NO_STOP));
  TWI_CHECK_ERROR(twi_read_blocking(
      sensor->twi, sensor->addr, out, &out_len, TWI_COMMAND_FLAG_NONE));
  // TODO: Check if `out_len` is the expected length
  return TWI_OK;
}

enum twi_result_t max30100_write(
    const struct max30100_t *sensor, const twi_u8 *cmd, twi_u32 cmd_len)
{
  TWI_CHECK_ERROR(twi_write_blocking(
      sensor->twi, sensor->addr, cmd, &cmd_len, TWI_COMMAND_FLAG_NONE));
  // TODO: Check if `cmd_len` is the expected length
  return TWI_OK;
}

enum twi_result_t max30100_read_byte(
    const struct max30100_t *sensor, const twi_u8 target, twi_u8 *out)
{
  TWI_CHECK_ERROR(max30100_read(sensor, target, out, 1));
  return TWI_OK;
}

enum twi_result_t max30100_set_spo2_config(const struct max30100_t *sensor,
    const struct max30100_spo2_config_t *config)
{
  twi_u8 reg = 0;
  // TWI_CHECK_ERROR(
  //     max30100_read_byte(sensor, MAX30100_SPO2_CONFIGURATION, &reg));

  // SPO2 high resolution
  if (config->high_resolution)
  {
    reg |= MAX30100_BIT(MAX30100_SPO2_CONFIGURATION_HI_RES_EN);

    // Clear sample rate and pulse width bits
    reg &= ~(MAX30100_SPO2_PULSE_WIDTH_MAX
               << MAX30100_SPO2_CONFIGURATION_LED_PW_1) |
           ~(MAX30100_SPO2_SAMPLE_RATE_MAX
               << MAX30100_SPO2_CONFIGURATION_SPO2_SR_1);
  }
  else
  {
    reg &= ~MAX30100_BIT(MAX30100_SPO2_CONFIGURATION_HI_RES_EN);

    /**
     *                                       B4 | B3 | B2
     * Unknown | SPO2_HI_RES_EN | Reserved | SPO2_SR[2:0] | LED_PW[1:0]
     */
    reg |= config->sample_rate << MAX30100_SPO2_CONFIGURATION_SPO2_SR_1;

    // LED pulse width
    reg |= config->pulse_width << MAX30100_SPO2_CONFIGURATION_LED_PW_1;
  }

  // Set reserved to low (just in case)
  reg &= ~MAX30100_BIT(MAX30100_SPO2_CONFIGURATION_RESERVED_0);

  printf("SPO2 configuration: %d\n", reg);

  const twi_u8 cmd[] = {MAX30100_SPO2_CONFIGURATION, reg};
  TWI_CHECK_ERROR(max30100_write(sensor, cmd, 2));

  return TWI_OK;
}

enum twi_result_t max30100_get_temperature(
    const struct max30100_t *sensor, float *temperature)
{
  twi_u8 integral, frac;
  TWI_CHECK_ERROR(
      max30100_read_byte(sensor, MAX30100_TEMPERATURE_INTG, &integral));
  TWI_CHECK_ERROR(max30100_read_byte(sensor, MAX30100_TEMPERATURE_FRAC, &frac));
  // The fractional part is 4 bits, so we need to divide by 16
  static const float FRAC_RESOLUTION = 0.0625;
  *temperature = (float)(integral) + ((float)frac * FRAC_RESOLUTION);
  return TWI_OK;
}

enum twi_result_t max30100_set_led_current(const struct max30100_t *sensor,
    const enum max30100_led_current_t red_current,
    const enum max30100_led_current_t ir_current)
{
  const twi_u8 status = (red_current << 4 | ir_current);

  const twi_u8 cmd[] = {MAX30100_LED_CONFIGURATION, status};

  TWI_CHECK_ERROR(max30100_write(sensor, cmd, 2));

  return TWI_OK;
}

enum twi_result_t max30100_set_mode(
    const struct max30100_t *sensor, const enum max30100_mode_t mode)
{
  twi_u8 reg;
  TWI_CHECK_ERROR(
      max30100_read_byte(sensor, MAX30100_MODE_CONFIGURATION, &reg));

  // Set mode control
  reg |= mode << MAX30100_CONFIGURATION_MODE_1;

  switch (mode)
  {
  /**
   * The SpO2 write/read pointers should be cleared (back to 0x0) upon entering
   * SpO2 mode or heart-rate mode, so that there is no old data represented in
   * the FIFO. The pointers are not automatically cleared when changing modes,
   * but they are cleared if VDD is power cycled so that the VDD voltage drops
   * below its UVLO voltage.
   */
  case MAX30100_MODE_SPO2:
  case MAX30100_MODE_HR_ONLY:
    TWI_CHECK_ERROR(max30100_write(
        sensor, (twi_u8[]){MAX30100_FIFO_READ_POINTER, 0x00}, 2));
    break;
  default:
    break;
  }

  // SHDN
  reg &= ~MAX30100_BIT(MAX30100_CONFIGURATION_MODE_SHDN);

  // Initiate a single temperature measurement
  reg |= MAX30100_BIT(MAX30100_CONFIGURATION_MODE_TEMP_EN);

  const twi_u8 cmd[] = {MAX30100_MODE_CONFIGURATION, reg};
  TWI_CHECK_ERROR(max30100_write(sensor, cmd, 2));

  // printf("setting mode control: %d\n", reg);

  return TWI_OK;
}

enum twi_result_t max30100_read_fifo(
    const struct max30100_t *sensor, twi_u8 *fifo, const twi_u32 fifo_len)
{
  TWI_CHECK_ERROR(max30100_read(sensor, MAX30100_FIFO_DATA, fifo, fifo_len));
  return TWI_OK;
}

enum twi_result_t max30100_set_register_flag(const struct max30100_t *sensor,
    const enum max30100_register_address_t reg, const twi_u8 flag,
    const bool value)
{
  twi_u8 reg_value;
  TWI_CHECK_ERROR(max30100_read_byte(sensor, reg, &reg_value));
  twi_u8 mask = MAX30100_BIT(flag);
  if (value)
  {
    reg_value |= mask;
  }
  else
  {
    reg_value &= ~mask;
  }
  const twi_u8 cmd[] = {reg, reg_value};
  TWI_CHECK_ERROR(max30100_write(sensor, cmd, 2));
  return TWI_OK;
}

enum twi_result_t max30100_set_configuration_flag(
    const struct max30100_t *sensor,
    const enum max30100_configuration_mode_t flag, const bool value)
{
  return max30100_set_register_flag(
      sensor, MAX30100_MODE_CONFIGURATION, flag, value);
}

enum twi_result_t max30100_reset(const struct max30100_t *sensor)
{
  twi_u8 mode;
  TWI_CHECK_ERROR(
      max30100_read_byte(sensor, MAX30100_MODE_CONFIGURATION, &mode));
  if (!(mode & MAX30100_BIT(MAX30100_CONFIGURATION_MODE_RESET)))
  {
    TWI_CHECK_ERROR(max30100_set_configuration_flag(
        sensor, MAX30100_CONFIGURATION_MODE_RESET, true));
  }

  // Wait until software reset is finished by getting configuration mode again
  do
  {
    TWI_CHECK_ERROR(
        max30100_read_byte(sensor, MAX30100_MODE_CONFIGURATION, &mode));
  } while (mode & MAX30100_BIT(MAX30100_CONFIGURATION_MODE_RESET));

  return TWI_OK;
}

enum twi_result_t max30100_init(
    struct max30100_t *sensor, struct twi_t *twi, twi_u8 addr)
{
  sensor->twi = twi;
  sensor->addr = addr;

  // Reset the sensor
  TWI_CHECK_ERROR(max30100_reset(sensor));

  return TWI_OK;
}
