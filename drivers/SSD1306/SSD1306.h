#ifndef TWI_SSD1306_H
#define TWI_SSD1306_H

#include <twi.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  /**
   * The slave address is following the start condition for recognition use. For
   * the SSD1306, the slave address is either “b0111100” or “b0111101” by
   * changing the SA0 to LOW or HIGH (D/C pin acts as SA0).
   */
  // #define SSD1306_ADDRESS 0b011110

#define SSD1306_DEFAULT_WRITE_ADDRESS 0b0111100

#define SSD1306_DEFAULT_READ_ADDRESS 0b0111101

  enum ssd1306_charge_pump_state
  {
    SSD1306_CHARGE_PUMP_STATE_ENABLE = 0b00010100,
    SSD1306_CHARGE_PUMP_STATE_DISABLE = 0b00010000
  };

  enum ssd1306_continuation_bit_t
  {
    SSD1306_CONTINUATION_BIT_LAST = 0b00000000,
    SSD1306_CONTINUATION_BIT_NOT_LAST = 0b10000000
  };

  enum ssd1306_data_command_selection_bit
  {
    SSD1306_DATA_COMMAND_SELECTION_COMMAND = 0b00000000,
    SSD1306_DATA_COMMAND_SELECTION_DATA = 0b01000000
  };

  enum ssd1306_memory_addressing_mode_t
  {
    /**
     * Horizontal addressing mode
     */
    SSD1306_MEMORY_ADDRESSING_MODE_HORIZONTAL = 0b00,
    /**
     * Vertical addressing mode
     */
    SSD1306_MEMORY_ADDRESSING_MODE_VERTICAL = 0b01,
    /**
     * Page addressing mode
     */
    SSD1306_MEMORY_ADDRESSING_MODE_PAGE = 0b10,
  };

  enum ssd1306_setting_t
  {
    // 2.1 Command Table for Charge Bump Setting
    SSD1306_SET_CHARGE_PUMP = 0x8D,

    /**
     * Single-byte command
     */
    SSD1306_SET_DISPLAY_ON = 0b10101111,               // 0xAF
    SSD1306_SET_DISPLAY_OFF = 0b10101110,              // 0xAE
    SSD1306_ENTIRE_DISPLAY_ON_RESUME = 0b10100100,     // 0xA4
    /**
     * A5h, X0=1b: Entire display ON
     * Output ignores RAM content
     */
    SSD1306_ENTIRE_DISPLAY_ON_IGNORE_RAM = 0b10100101,
    /**
     * A6h, X[0]=0b: Normal display (RESET)
     * 0 in RAM: OFF in display panel
     * 1 in RAM: ON in display panel
     */
    SSD1306_SET_NORMAL_DISPLAY = 0b10100111, // 0xA6
    /**
     * A7h, X[0]=1b: Inverse display
     * 0 in RAM: ON in display panel
     * 1 in RAM: OFF in display panel
     */
    SSD1306_SET_INVERSE_DISPLAY = 0b10100111, // 0xA7
    /**
     * Command for no operation (NOP)
     */
    SSD1306_NOP = 0b11100000,                    // 0xE3
    SSD1306_SET_DISPLAY_OFFSET = 0b11010000,     // 0xD3
    SSD1306_SET_DISPLAY_START_LINE = 0b01000000, // 0x40
    SSD1306_SET_COLUMN_ADDRESS = 0x21,
    SSD1306_SET_PAGE_ADDRESS = 0x22,
    SSD1306_SET_MULTIPLEX_RATIO = 0b10101000, // 0xA8
    SSD1306_SET_GDDRAM_PAGE_START_ADDRESS = 0b10110000, // 1 0 1 1 0 X2 X1 X0
    SSD1306_SET_MEMORY_ADRESSING_MODE = 0b00100000, // 0x20
    /**
     * C0h, X[3]=0b: normal mode (RESET) Scan from
     * COM0 to COM[N –1]
     */
    SSD1306_SET_COM_OUTPUT_SCAN_DIRECTION_NORMAL = 0b11000000,
    /**
     * C8h, X[3]=1b: remapped mode. Scan from
     * COM[N-1] to COM0
     * Where N is the Multiplex ratio.
     */
    SSD1306_SET_COM_OUTPUT_SCAN_DIRECTION_REMAPPED = 0b11001000,
    /**
     * A0h, X[0]=0b: column address 0 is mapped to
     * SEG0 (RESET)
     */
    SSD1306_SET_SEGMENT_REMAP_RESET = 0b10100000,
    /**
     * A1h, X[0]=1b: column address 127 is mapped to
     * SEG0
     */
    SSD1306_SET_SEGMENT_REMAP_INVERTED = 0b10100001,
    SSD1306_SET_COM_PINS_HARDWARE_CONFIGURATION = 0b11011010
  };

  enum ssd1306_com_pins_hardware_configuration_t
  {
    /**
     * A[4]=0b, Sequential COM pin configuration
     */
    SSD1306_SEQUENTIAL_COM_PIN_CONFIGURATION = 0b00010000,
    /**
     * A[4]=1b(RESET), Alternative COM pin
     * configuration
     */
    SSD1306_ALTERNATIVE_COM_PIN_CONFIGURATION = 0b00000000,
    /**
     * A[5]=0b(RESET), Disable COM Left/Right
     * remap
     */
    SSD1306_DISABLE_COM_LEFT_RIGHT_REMAP = 0b00000000,
    /**
     * A[5]=1b, Enable COM Left/Right remap
     */
    SSD1306_ENABLE_COM_LEFT_RIGHT_REMAP = 0b00100000,
  };

  struct ssd1306_vec2_t
  {
    twi_u8 x;
    twi_u8 y;
  };


  struct ssd1306_configuration_t
  {
   twi_u8 write_address;
   twi_u8 read_address;
   struct ssd1306_vec2_t dimensions;
   struct twi_t *twi;
  };

  struct ssd1306_t
  {
    struct ssd1306_vec2_t dimensions;
    twi_u8 write_address;
    twi_u8 read_address;
    struct twi_t *twi;
  };

  enum twi_result_t ssd1306_init(struct ssd1306_t *, struct ssd1306_configuration_t*);

  enum twi_result_t ssd1306_update(struct ssd1306_t *display);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // TWI_SSD1306_H
