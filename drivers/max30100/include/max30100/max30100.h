#ifndef MAX30100_H
#define MAX30100_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <stdbool.h>
#include <twi.h>

#define MAX30100_FIFO_DEPTH ((twi_u8)16)

#define MAX30100_BIT(n) (1 << n)

/**
 * "The MAX30100 slave ID consists of seven fixed bits,
 * B7–B1 (set to 0b1010111). The most significant slave ID
 * bit (B7) is transmitted first, followed by the remaining bits."
 *
 * Refer to the "Slave Address" section of "Table 17. Slave ID Description" on
 * page 24 of the datasheet.
 */
#define MAX30100_I2C_MASTER_ADDRESS 0b1010111

/**
 * Refer to the "Slave Address" section of "Table 17. Slave ID Description" on
 * page 24 of the datasheet.
 */
#define MAX30100_I2C_SLAVE_WRITE_ADDRESS                                       \
  ((MAX30100_I2C_MASTER_ADDRESS << 1) | 1)

/**
 * Refer to the "Slave Address" section of "Table 17. Slave ID Description" on
 * page 24 of the datasheet.
 */
#define MAX30100_I2C_SLAVE_READ_ADDRESS (MAX30100_I2C_MASTER_ADDRESS << 1)

  typedef unsigned char max30100_u8;
  typedef signed char max30100_s8;

  struct max30100_t
  {
    // I²C interface
    struct twi_t *twi;
    twi_u8 addr;
  };

  enum max30100_configuration_mode_t : twi_u8
  {
    /**
     * Bits 2:0: Mode Control
     * These bits set the operating state of the MAX30100. Changing modes does
     * not change any other setting, nor does it erase any previously stored
     * data inside the data registers.
     */
    MAX30100_CONFIGURATION_MODE_1 = 0,
    MAX30100_CONFIGURATION_MODE_2 = 1,
    MAX30100_CONFIGURATION_MODE_3 = 2,
    /**
     * Bit 3: Temperature Enable (TEMP_EN)
     *
     * This is a self-clearing bit which, when set, initiates a single
     * temperature reading from the temperature sensor. This bit is cleared
     * automatically back to zero at the conclusion of the temperature reading
     * when the bit is set to one in heart rate or SpO2 mode.
     */
    MAX30100_CONFIGURATION_MODE_TEMP_EN = 3,
    MAX30100_CONFIGURATION_MODE_RESERVED_1 = 4,
    MAX30100_CONFIGURATION_MODE_RESERVED_2 = 5,
    /**
     * Bit 6: Reset Control (RESET)
     * When the RESET bit is set to one, all configuration, threshold, and data
     * registers are reset to their power-on-state. The only exception is
     * writing both RESET and TEMP_EN bits to one at the same time since
     * temperature data registers 0x16 and 0x17 are not cleared. The RESET bit
     * is cleared automatically back to zero after the reset sequence is
     * completed.
     */
    MAX30100_CONFIGURATION_MODE_RESET = 6,
    /**
     * Bit 7: Shutdown Control (SHDN)
     * The part can be put into a power-save mode by setting this bit to one.
     * While in power-save mode, all registers retain their values, and
     * write/read operations function as normal. All interrupts are cleared to
     * zero in this mode.
     */
    MAX30100_CONFIGURATION_MODE_SHDN = 7,
  };

  enum max30100_spo2_configuration_t
  {
    /**
     * Bits 1:0: LED Pulse Width Control
     * These bits set the LED pulse width (the IR and RED have the same pulse
     * width), and therefore, indirectly set the integration time of the ADC in
     * each sample. The ADC resolution is directly related to the integration
     * time.
     */
    MAX30100_SPO2_CONFIGURATION_LED_PW_1 = 0,
    MAX30100_SPO2_CONFIGURATION_LED_PW_2 = 1,
    /**
     * Bit 4:2: SpO2 Sample Rate Control
     * These bits define the effective sampling rate, with one sample consisting
     * of one IR pulse/conversion and one RED pulse/ conversion.
     *
     * The sample rate and pulse width are related, in that the sample rate sets
     * an upper bound on the pulse width time. If the user selects a sample rate
     * that is too high for the selected LED_PW setting, the highest possible
     * sample rate will instead be programmed into the register.
     */
    MAX30100_SPO2_CONFIGURATION_SPO2_SR_1 = 2,
    MAX30100_SPO2_CONFIGURATION_SPO2_SR_2 = 3,
    MAX30100_SPO2_CONFIGURATION_SPO2_SR_3 = 4,
    /**
     * Bit 5: Reserved. Set low (default).
     */
    MAX30100_SPO2_CONFIGURATION_RESERVED_0 = 5,
    /**
     * Bit 6: SpO2 High Resolution Enable (SPO2_HI_RES_EN)
     * Set this bit high. The SPO2 ADC resolution is 16-bit with 1.6ms LED pulse
     * width.
     */
    MAX30100_SPO2_CONFIGURATION_HI_RES_EN = 6,
  };

  enum max30100_mode_t
  {
    MAX30100_MODE_UNUSED = 0b000,
    /**
     * Reserved (Do not use)
     */
    MAX30100_MODE_RESERVED = 0b001,
    MAX30100_MODE_HR_ONLY = 0b010,
    MAX30100_MODE_SPO2 = 0b011,
    MAX30100_MODE_UNUSED_1 = 0b100,
    MAX30100_MODE_UNUSED_2 = 0b111
  };

  enum max30100_led_configuration_t
  {
    MAX30100_LED_CONFIGURATION_IR_PA_1 = 0,
    MAX30100_LED_CONFIGURATION_IR_PA_2 = 1,
    MAX30100_LED_CONFIGURATION_IR_PA_3 = 2,
    MAX30100_LED_CONFIGURATION_IR_PA_4 = 3,
    MAX30100_LED_CONFIGURATION_RED_PA_1 = 4,
    MAX30100_LED_CONFIGURATION_RED_PA_2 = 5,
    MAX30100_LED_CONFIGURATION_RED_PA_3 = 6,
    MAX30100_LED_CONFIGURATION_RED_PA_4 = 7,
  };

  enum max30100_interrupt_status_t : twi_u8
  {
    MAX30100_INTERRUPT_STATUS_PWR_RDY = 0,
    // This bit should be ignored and always be zero in normal operation.
    MAX30100_INTERRUPT_STATUS_RESERVED_1 = 1,
    // This bit should be ignored and always be zero in normal operation.
    MAX30100_INTERRUPT_STATUS_RESERVED_2 = 2,
    // This bit should be ignored and always be zero in normal operation.
    MAX30100_INTERRUPT_STATUS_RESERVED_3 = 3,
    /**
     * In SpO2 mode, this interrupt triggers after every data sample is
     * collected. An SpO2 data sample consists of one IR and one red data
     * points. This bit is automatically cleared when the FIFO data register is
     * read.
     */
    MAX30100_INTERRUPT_STATUS_SPO2_RDY = 4,
    /**
     * In heart rate or SPO2 mode, this interrupt triggers after every data
     * sample is collected. A heart rate data sample consists of one IR data
     * point only. This bit is automatically cleared when the FIFO data register
     * is read.
     */
    MAX30100_INTERRUPT_STATUS_HR_RDY = 5,
    /**
     * When an internal die temperature conversion is finished, this interrupt
     * is triggered so the processor can read the temperature data registers.
     */
    MAX30100_INTERRUPT_STATUS_TEMP_RDY = 6,
    /**
     * In SpO2 and heart-rate modes, this interrupt triggers when the FIFO write
     * pointer is the same as the FIFO read pointer minus one, which means that
     * the FIFO has only one unwritten space left. If the FIFO is not read
     * within the next conversion time, the FIFO becomes full and future data is
     * lost.
     */
    MAX30100_INTERRUPT_STATUS_A_FULL = 7,
  };

  enum max30100_register_address_t : twi_u8
  {
    MAX30100_INTERRUPT_STATUS = 0x00,
    MAX30100_INTERRUPT_ENABLE = 0x01,
    MAX30100_MODE_CONFIGURATION = 0x06,
    MAX30100_LED_CONFIGURATION = 0x09, // Pulse width and LED current
    MAX30100_SPO2_CONFIGURATION = 0x07,
    /**
     * Temperature integer part
     */
    MAX30100_TEMPERATURE_INTG = 0x16,
    /**
     * Temperature fractional part
     */
    MAX30100_TEMPERATURE_FRAC = 0x17,
    /**
     * FIFO write pointer
     */
    MAX30100_FIFO_WRITE_POINTER = 0x02,
    /**
     * FIFO overflow counter
     */
    MAX30100_FIFO_OVERFLOW_COUNTER = 0x03,
    /**
     * FIFO read pointer
     */
    MAX30100_FIFO_READ_POINTER = 0x04,
    /**
     * FIFO data register.
     *
     * The circular FIFO depth is 16 and can hold up to 16 samples of SpO2
     * channel data (Red and IR). The FIFO_DATA register in the I2C register map
     * points to the next sample to be read from the FIFO. FIFO_RD_PTR points to
     * this sample. Reading FIFO_DATA register does not automatically increment
     * the register address; burst reading this register reads the same address
     * over and over. Each sample is 4 bytes of data, so this register has to be
     * read 4 times to get one sample. The above registers can all be written
     * and read, but in practice, only the FIFO_RD_PTR register should be
     * written to in operation. The others are automatically incremented or
     * filled with data by the MAX30100.
     *
     * When starting a new SpO2 or heart-rate conversion, it is recommended to
     * first clear the FIFO_WR_PTR, OVF_COUNTER, and FIFO_RD_PTR registers to
     * all zeros (0x00) to ensure the FIFO is empty and in a known state. When
     * reading the MAX30100 registers in one burst-read I2C transaction, the
     * register address pointer typically increments so that the next byte of
     * data sent is from the next register, etc. The exception to this is the
     * FIFO data register, register 0x05. When reading this register, the
     * address pointer does not increment, but the FIFO_RD_PTR does. So the next
     * byte of data sent will represent the next byte of data available in the
     * FIFO.
     *
     * ## Reading from the FIFO
     *
     * Normally, reading registers from the I2C interface autoincrements the
     * register address pointer, so that all the registers can be read in a
     * burst read without an I2C restart event. In the MAX30100, this holds true
     * for all registers except for the FIFO_DATA register (0x05).
     *
     * Reading the FIFO_DATA register does not automatically increment the
     * register address; burst reading this register reads the same address over
     * and over. Each sample is 4 bytes of data, so this register has to be read
     * 4 times to get one sample. The other exception is 0xFF, reading more
     * bytes after the 0xFF register does not advance the address pointer back
     * to 0x00, and the data read is not meaningful.
     */
    MAX30100_FIFO_DATA = 0x05,
    // R
    MAX30100_REVISION_ID = 0xFE,
    // R/W
    MAX30100_PART_ID = 0xFF,
  };

  enum max30100_interrupt_enable_t
  {
    ENB_A_FULL = 7,
    ENB_TEP_RDY = 6,
    ENB_HR_RDY = 5,
    ENB_SO2_RDY = 4
  };

  enum max30100_spo2_sample_rate_t : max30100_s8
  {
    MAX30100_SPO2_SAMPLE_RATE_NONE = -1,
    MAX30100_SPO2_SAMPLE_RATE_50 = 0b000,
    MAX30100_SPO2_SAMPLE_RATE_100 = 0b001,
    MAX30100_SPO2_SAMPLE_RATE_167 = 0b010,
    MAX30100_SPO2_SAMPLE_RATE_200 = 0b011,
    MAX30100_SPO2_SAMPLE_RATE_400 = 0b100,
    MAX30100_SPO2_SAMPLE_RATE_600 = 0b101,
    MAX30100_SPO2_SAMPLE_RATE_800 = 0b110,
    MAX30100_SPO2_SAMPLE_RATE_1000 = 0b111,
    MAX30100_SPO2_SAMPLE_RATE_MAX = MAX30100_SPO2_SAMPLE_RATE_1000,
  };

  enum max30100_spo2_pulse_width_t : max30100_s8
  {
    MAX30100_SPO2_PULSE_WIDTH_NONE = -1,
    /**
     * ADC resolution = 13 bits
     */
    MAX30100_SPO2_PULSE_WIDTH_200 = 0b00,
    /**
     * ADC resolution = 14 bits
     */
    MAX30100_SPO2_PULSE_WIDTH_400 = 0b01,
    /**
     * ADC resolution = 15 bits
     */
    MAX30100_SPO2_PULSE_WIDTH_800 = 0b10,
    /**
     * ADC resolution = 16 bits
     */
    MAX30100_SPO2_PULSE_WIDTH_1600 = 0b11,
    MAX30100_SPO2_PULSE_WIDTH_MAX = MAX30100_SPO2_PULSE_WIDTH_1600,
  };

  struct max30100_spo2_config_t
  {
    enum max30100_spo2_sample_rate_t sample_rate;
    enum max30100_spo2_pulse_width_t pulse_width;
    /**
     * Set this bit high. The SpO2 ADC resolution is 16-bit with 1.6ms LED pulse
     * width.
     *
     * If this is set to true, both pulse width nor sample rate will be
     * set to zero.
     */
    bool high_resolution;
  };

  enum max30100_led_current_t
  {
    MAX30100_LED_CURRENT_0 = 0b0000,
    MAX30100_LED_CURRENT_4_4 = 0b0001,
    MAX30100_LED_CURRENT_7_6 = 0b0010,
    MAX30100_LED_CURRENT_11_0 = 0b0011,
    MAX30100_LED_CURRENT_14_2 = 0b0100,
    MAX30100_LED_CURRENT_17_4 = 0b0101,
    MAX30100_LED_CURRENT_20_8 = 0b0110,
    MAX30100_LED_CURRENT_24_0 = 0b0111,
    MAX30100_LED_CURRENT_27_1 = 0b1000,
    MAX30100_LED_CURRENT_30_6 = 0b1001,
    MAX30100_LED_CURRENT_33_8 = 0b1010,
    MAX30100_LED_CURRENT_37_0 = 0b1011,
    MAX30100_LED_CURRENT_40_2 = 0b1100,
    MAX30100_LED_CURRENT_43_6 = 0b1101,
    MAX30100_LED_CURRENT_46_8 = 0b1110,
    MAX30100_LED_CURRENT_50_0 = 0b1111,
    MAX30100_LED_CURRENT_MIN = MAX30100_LED_CURRENT_4_4,
    MAX30100_LED_CURRENT_MAX = MAX30100_LED_CURRENT_50_0,
  };

  /**
   * Set the LED current. A call to this can trigger a PWR_RDY interrupt status,
   * if the sensor is not receiving enough current.
   *
   * If this happens, try to use a lower current, such as
   * `MAX30100_LED_CURRENT_11_0` or increase your current supply.
   *
   * I am unsure, but I guess it's because the sensor starts to draw more
   * current than it can handle.
   * @param sensor Sensor instance
   * @param red_current Current for the red LED
   * @param ir_current Current for the IR LED
   * @return TWI_OK if the operation was successful
   */
  enum twi_result_t max30100_set_led_current(const struct max30100_t *sensor,
      enum max30100_led_current_t red_current,
      enum max30100_led_current_t ir_current);

  enum twi_result_t max30100_write(
      const struct max30100_t *sensor, const twi_u8 *cmd, twi_u32 cmd_len);

  /**
   * Set the sensor mode (e.g., HR or SPO²)
   * @param sensor Sensor instance
   * @param mode Mode we want to set
   * @return TWI_OK if the operation was successful
   */
  enum twi_result_t max30100_set_mode(
      const struct max30100_t *sensor, enum max30100_mode_t mode);

  /**
   * @param sensor Sensor to read from
   * @param target Register to read from
   * @param out Pointer to store the read value
   * @return TWI_OK if successful, otherwise an error code
   */
  enum twi_result_t max30100_read_byte(
      const struct max30100_t *sensor, twi_u8 target, twi_u8 *out);
  /**
   * Initialize the sensor
   * @param sensor Sensor instance
   * @param twi TWI interface
   * @param addr I²C address of the sensor
   * @return TWI_OK if the operation was successful
   */
  enum twi_result_t max30100_init(
      struct max30100_t *sensor, struct twi_t *twi, twi_u8 addr);

  /**
   * Set or clear a flag for a register
   * @param sensor Sensor instance
   * @param reg Register we want to change
   * @param flag Bit that we want to target on the register
   * @param value Whether we want to set or clear the bit
   * @return TWI_OK if the operation was successful
   */
  enum twi_result_t max30100_set_register_flag(const struct max30100_t *sensor,
      enum max30100_register_address_t reg, const twi_u8 flag,
      const bool value);

  /**
   * Set SPO² configuration
   * @param sensor Sensor instance
   * @param config Configuration to set for the SPO² sensor
   * @return TWI_OK if the operation was successful
   */
  enum twi_result_t max30100_set_spo2_config(const struct max30100_t *sensor,
      const struct max30100_spo2_config_t *config);

  /**
   * Read the FIFO data
   * @param sensor Sensor instance
   * @param fifo Buffer to write the FIFO data to
   * @param fifo_len Length of the FIFO buffer
   * @return TWI_OK if the operation was successful
   */
  enum twi_result_t max30100_read_fifo(
      const struct max30100_t *sensor, twi_u8 *fifo, twi_u32 fifo_len);

  /**
   * Read data from the sensor I2C bus
   * @param sensor Sensor instance
   * @param target Register to read from
   * @param out Buffer to write the read data to
   * @param out_len Length of the buffer
   * @return TWI_OK if the operation was successful
   */
  enum twi_result_t max30100_read(const struct max30100_t *sensor,
      twi_u8 target, twi_u8 *out, twi_u32 out_len);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MAX30100_H
