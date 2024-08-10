#include "SSD1306.h"

#include <stdbool.h>
#include <string.h>

enum twi_result_t ssd1306_transmission_init(
    const struct ssd1306_t *display)
{
  TWI_CHECK_ERROR(twi_transmission_init(display->twi, display->write_address));
  return TWI_OK;
}

/**
 * Send a command or data to the SSD1306 display. The implementation will
 * submit the transmission to the TWI bus. Making sure that if we create an invalid
 * message, we will get an error, since TWI will return an error if the transmission wasn't
 * initialized.
 *
 * @param display SSD1306 display instance
 * @param continuation_bit Whether this is the last command or not
 * @param data_command_selection_bit Whether it is data or a command
 * @param control_byte Control byte
 * @return TWI_OK if successful
 */
enum twi_result_t ssd1306_transmission_send(
  const struct ssd1306_t *display,
  const enum ssd1306_continuation_bit_t continuation_bit,
  const enum ssd1306_data_command_selection_bit data_command_selection_bit,
  const twi_u8 control_byte
) {
  twi_u32 written_byte_count;
  TWI_CHECK_ERROR(twi_transmission_write_byte(display->twi, data_command_selection_bit | continuation_bit));
  TWI_CHECK_ERROR(twi_transmission_write_byte(display->twi, control_byte));
  switch(continuation_bit) {
    case SSD1306_CONTINUATION_BIT_LAST:
      TWI_CHECK_ERROR(twi_transmission_end(display->twi, &written_byte_count));
      if(written_byte_count == 0) {
        return TWI_IMPLEMENTATION_ERROR;
      }
      return TWI_OK;
    case SSD1306_CONTINUATION_BIT_NOT_LAST:
      return TWI_OK;
  }
  return TWI_OK;
}

inline enum twi_result_t ssd1306_transmission_send_command_byte(
  const struct ssd1306_t *display,
  enum ssd1306_continuation_bit_t continuation_bit,
  const twi_u8 command
)
{
  return ssd1306_transmission_send(display, continuation_bit, SSD1306_DATA_COMMAND_SELECTION_COMMAND, command);
}

inline enum twi_result_t ssd1306_transmission_send_buffer(
  const struct ssd1306_t *display,
  const enum ssd1306_continuation_bit_t continuation_bit,
  const enum ssd1306_data_command_selection_bit data_command_selection_bit,
  const twi_u8* buffer,
  const twi_u32 buffer_length
)
{
  const twi_u32 last_index = buffer_length - 1;
  const bool is_last_command = continuation_bit == SSD1306_CONTINUATION_BIT_LAST;
  for(twi_u32 i = 0; i < buffer_length; i++)
  {
    const enum ssd1306_continuation_bit_t byte_continuation_bit = (
      is_last_command && (i == last_index) ?
        SSD1306_CONTINUATION_BIT_LAST :
        SSD1306_CONTINUATION_BIT_NOT_LAST
    );
    TWI_CHECK_ERROR(ssd1306_transmission_send(
      display,
      byte_continuation_bit,
      data_command_selection_bit,
      buffer[i]));
  }
  return TWI_OK;
}

inline enum twi_result_t ssd1306_transmission_send_data(
  const struct ssd1306_t *display,
  const enum ssd1306_continuation_bit_t continuation_bit,
  const twi_u8* data,
  const twi_u32 data_length
)
{
  return ssd1306_transmission_send_buffer(display, continuation_bit, SSD1306_DATA_COMMAND_SELECTION_DATA, data, data_length);
}

/**
 * Send a command to the display
 * @param display Display instance
 * @param command Command buffer
 * @param command_length Command buffer length
 * @param continuation_bit If SSD1306_CONTINUATION_BIT_LAST, it will send Co=0 at the last item of `command`
 * @return TWI_OK if successful
 */
inline enum twi_result_t ssd1306_transmission_send_command(
  const struct ssd1306_t *display,
  const enum ssd1306_continuation_bit_t continuation_bit,
  const twi_u8* command,
  const twi_u32 command_length
)
{
  return ssd1306_transmission_send_buffer(display, continuation_bit, SSD1306_DATA_COMMAND_SELECTION_COMMAND, command, command_length);
}

enum twi_result_t ssd1306_write_blocking(const struct ssd1306_t *display,
    const twi_u8 *command, twi_u32 command_length)
{
  TWI_CHECK_ERROR(twi_write_blocking(display->twi, display->write_address,
      command, &command_length, TWI_COMMAND_FLAG_NONE));
  return TWI_OK;
}

enum twi_result_t ssd1306_set_charge_pump(
    const struct ssd1306_t *display, const bool enable)
{
  TWI_CHECK_ERROR(ssd1306_write_blocking(display,
      (twi_u8[]){0x00, SSD1306_SET_CHARGE_PUMP,
          enable ? SSD1306_CHARGE_PUMP_STATE_ENABLE
                 : SSD1306_CHARGE_PUMP_STATE_DISABLE},
      3));
  return TWI_OK;
}

enum twi_result_t ssd1306_move_to(
  const struct ssd1306_t *display,
  const twi_u8 page_start,
  const twi_u8 page_end,
  const twi_u8 column_start,
  const twi_u8 column_end
)
{
  TWI_CHECK_ERROR(ssd1306_transmission_init(display));
  TWI_CHECK_ERROR(ssd1306_transmission_send_command(
    display,
    SSD1306_CONTINUATION_BIT_LAST,
    (twi_u8[]) {
      // Set page address
      SSD1306_SET_PAGE_ADDRESS,
      page_start,                    // Start page address
      (page_end / 8) - 1,             // End page address
      // Set column address
      SSD1306_SET_COLUMN_ADDRESS,
      column_start,                     // Start page address
      column_end - 1,                     // End page address
    },
    6
  ));
  return TWI_OK;
}

enum twi_result_t ssd1306_update(
  /* Make it const */
  struct ssd1306_t *display
) {
  twi_u8 col_size = display->dimensions.x;
  twi_u8 clear_buffer[col_size];
  memset(clear_buffer, 0xFF, col_size);
  for(twi_u8 page = 0; page < display->dimensions.y; page++)
  {
    TWI_CHECK_ERROR(ssd1306_move_to(
      display,
      page, page + 1,
      0, col_size
    ));
    TWI_CHECK_ERROR(ssd1306_transmission_init(display));
    TWI_CHECK_ERROR(ssd1306_transmission_send_data(
      display,
      SSD1306_CONTINUATION_BIT_LAST,
      clear_buffer,
      col_size
    ));
  }

  twi_u8 page_start = 0, page_end = 10, column_start = 0, column_end = 20;
  TWI_CHECK_ERROR(
    ssd1306_move_to(
      display,
      page_start, page_end,
      column_start, column_end
    )
  );

  twi_u32 page_count = page_end - page_start;
  twi_u32 column_size = column_end - column_start;
  twi_u32 length = page_count * column_size ;
  twi_u8 buffer[length];
  memset(buffer, 0x00, length);

  TWI_CHECK_ERROR(ssd1306_transmission_init(display));
  // Send data
  TWI_CHECK_ERROR(ssd1306_transmission_send_data(
    display,
    SSD1306_CONTINUATION_BIT_NOT_LAST,
    buffer,
    length
  ));
  TWI_CHECK_ERROR(
    ssd1306_transmission_send_command(
      display,
      SSD1306_CONTINUATION_BIT_LAST,
      (twi_u8[]) {SSD1306_NOP},
      1
    )
  );

  return TWI_OK;
}

enum twi_result_t ssd1306_init(struct ssd1306_t *display, struct ssd1306_configuration_t* config)
{
  display->twi = config->twi;
  display->dimensions = config->dimensions;
  display->write_address = config->write_address;
  display->read_address = config->read_address;

  TWI_CHECK_ERROR(ssd1306_transmission_init(display));
  TWI_CHECK_ERROR(ssd1306_transmission_send_command(
    display,
    SSD1306_CONTINUATION_BIT_LAST,
    (twi_u8[]) {
      // Set display off
      SSD1306_SET_DISPLAY_OFF,
      // Set charge pump
      SSD1306_SET_CHARGE_PUMP,
      SSD1306_CHARGE_PUMP_STATE_ENABLE,
      // Set display on
      SSD1306_SET_DISPLAY_ON,
      SSD1306_SET_COM_PINS_HARDWARE_CONFIGURATION,
      SSD1306_SEQUENTIAL_COM_PIN_CONFIGURATION | SSD1306_ENABLE_COM_LEFT_RIGHT_REMAP,
    },
    6
  ));

  twi_u8 dimensions_cmd[] = {
    // Set display offset
    SSD1306_SET_DISPLAY_OFFSET,
    0x00, // * * A5 A4 A3 A2 A1 A0
    // Set display clock divide ratio/oscillator frequency
    0xD5,
    0x00, // A7 A6 A5 A4 A3 A2 A1 A0
    // Set multiplex ratio
    SSD1306_SET_MULTIPLEX_RATIO,
    0x3F, // 1/64 duty
    SSD1306_SET_DISPLAY_START_LINE,
    SSD1306_ENTIRE_DISPLAY_ON_RESUME,
    SSD1306_SET_NORMAL_DISPLAY,
    SSD1306_SET_MEMORY_ADRESSING_MODE,
    SSD1306_MEMORY_ADDRESSING_MODE_HORIZONTAL,
  };
  TWI_CHECK_ERROR(ssd1306_transmission_init(display));
  TWI_CHECK_ERROR(ssd1306_transmission_send_command(
    display,
    SSD1306_CONTINUATION_BIT_LAST,
    dimensions_cmd,
    sizeof(dimensions_cmd) / sizeof(dimensions_cmd[0])
  ));

  return TWI_OK;
}
