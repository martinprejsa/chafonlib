#ifndef READER_READER_H
#define READER_READER_H

#include <stdint.h>

typedef enum reader_error {
  READER_NO_ERROR = 0,
  READER_DEVICE_CONFIGURATION_ERROR,
  READER_DEVICE_COMMUNICATION_ERROR,
  READER_INVALID_PARAMETER,
  READER_ERROR_SIZE
} reader_error;

typedef struct reader_handle {
  int device;
  int length;
  uint8_t* response;
} reader_handle;

/**
 * @brief Used to retrieve message from an error.
 * 
 * @param error the error
 * @return char const* message
 */
char const * reader_error_to_string(reader_error error);

/**
 * @brief Initializes reader handle.
 * 
 * @param reader the reader handle
 * @param device path to the reader device, usually: /dev/ttyUSB0
 * @return reader_error, use reader_error_to_string
 */
reader_error reader_init(reader_handle *reader, char const * const device);

/**
 * @brief Destroys the reader
 * 
 * @param reader the reader handle
 */
void reader_destroy(reader_handle *reader);

/**
 * @brief Executes a reader command
 *        and stores it's result in the reader handle.
 * 
 * @param reader reader handle
 * @param address reader's address, 0xFF broadcast, 0x00 default
 * @param command command to execute
 * @param data command data
 * @param size command data size, must not exceed 251
 * @return reader_error use reader_error_to_string
 */
reader_error reader_execute(reader_handle * const reader, uint8_t address,
                              uint8_t command, uint8_t* data, uint8_t size);
#endif