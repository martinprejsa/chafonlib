#ifndef READER_READER_H
#define READER_READER_H

#include <stdint.h>
#include <commands.h>

typedef enum {
  READER_NO_ERROR = 0,
  READER_DEVICE_CONFIGURATION_ERROR,
  READER_DEVICE_COMMUNICATION_ERROR,
  READER_INVALID_PARAMETER,
  READER_ERROR_SIZE
} reader_error;

typedef enum {
  READER_UNKNOWN_MODE,
  READER_ANSWER_MODE,
  READER_RTI_MODE, // real time inventory mode
  READER_RTIT_MODE, // real time inventory mode with trigger
} reader_mode;

typedef struct {
  uint8_t ant;
  uint8_t size;
  uint8_t *data;
  uint8_t rssi;
} reader_rti_data;

typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t size; // data size
  uint8_t *data;
} reader_command;

typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t status;
  uint8_t size; // data size
  uint8_t *data;
} reader_response;

typedef struct reader_handle {
  int device;
  reader_mode mode;
  reader_response response;
  reader_rti_data rti_data;
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
 * @param command command to execute
 * @return reader_error use reader_error_to_string
 */
reader_error reader_execute(reader_handle * const reader, 
                              reader_command const command);
#endif