#include <owi/owi.h>

#include <hardware/gpio.h>

#include <hardware/gpio.h>
#include <pico/time.h>

enum twi_result_t owi_reset(const struct owi_t *owi)
{
  gpio_set_dir(owi->pin, GPIO_OUT);

  // P. 4: RESET PULSE FROM HOST
  gpio_put(owi->pin, 0);

  sleep_us(OWI_TIME_RSTL);

  // Release the bus
  gpio_set_dir(owi->pin, GPIO_IN);

  // Wait for the presence-detect low pulse
  sleep_us(OWI_TIME_PDHIGH_MAX);

  // P. 4: PRESENCE DETECT
  if (gpio_get(owi->pin) != 0)
  {
    return TWI_NOT_FOUND;
  }

  sleep_us(OWI_TIME_RSTH_MIN - OWI_TIME_PDHIGH_MAX);

  return TWI_OK;
}

enum twi_result_t owi_write_bit(
  const struct owi_t* owi,
  const twi_u8 bit
)
{
  gpio_set_dir(owi->pin, GPIO_OUT);

  // Write a bit
  // P. 16: Figure 16. Read/Write Time Slot Timing Diagram
  gpio_put(owi->pin, 0);

  const twi_u8 bit_low_time = bit ? OWI_TIME_TLOW1_MAX : OWI_TIME_TLOW0_MAX;

  // Since we require a pull-up resistor, this will define for how long the bus will be low.
  // As soon as the bus is released, the pull-up resistor will pull the bus high.
  sleep_us(bit_low_time);

  // Release the bus
  gpio_set_dir(owi->pin, GPIO_IN);

  sleep_us(OWI_TIME_SLOT_MIN - bit_low_time);

  // Wait for the time slot
  sleep_us(OWI_TIME_RECOVERY);

  return TWI_OK;
}

twi_u8 owi_read_bit(const struct owi_t* owi) {
  // Set the bit to 0
  twi_u8 bit;

  gpio_set_dir(owi->pin, GPIO_OUT);

  // Pull the bus low
  gpio_put(owi->pin, 0);

  // Hold the bus low for at least 1 us
  sleep_us(1);

  // Release the bus
  gpio_set_dir(owi->pin, GPIO_IN);

  /**
   * Output data from the DS18B20 is valid for 15µs after the falling edge that
   * initiated the read time slot. Therefore, the master must release the bus and
   * then sample the bus state within 15µs from the start of the slot.
   */
  sleep_us(OWI_TIME_READ_DATA_VALID - 5); // Wait 10 us

  // Read the bit
  bit = gpio_get(owi->pin) ? 1 : 0;

  // Wait additional 45 us
  sleep_us(45);

  // Recovery time
  sleep_us(OWI_TIME_RECOVERY);

  return bit;
}
