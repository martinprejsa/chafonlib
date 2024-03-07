#include <stdio.h>
#include "reader.h"
#include "commands.h"

int main() {
  reader_handle r;

  int err = reader_init(&r, "/dev/ttyUSB0");
  if (err) {
    printf("Reader initialization failed: %s\n", reader_error_to_string(err));
    perror(NULL);
    return 1;
  }

  reader_command setreadermode = {
    .address = READER_ADR_BROADCAST,
    .command = READER_CMD_CHANGE_MODE,
    .size = 1,
    .data = (uint8_t[]) {READER_ARG_ANSWER_MODE},
  };

  err = reader_execute(&r, setreadermode);
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    perror(NULL);
    return 1;
  }

  printf("Reader set to answer mode\n");

  reader_command obtaintemp = {
    .address = READER_ADR_BROADCAST,
    .command = READER_CMD_OBTAIN_TEMP,
    .size = 0,
    .data = 0,
  };

  err = reader_execute(&r, obtaintemp);
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    perror(NULL);
    return 1;
  }

  printf("Temperature: %d\n", r.response.data[2]);

  reader_destroy(&r);
  return 0;
}