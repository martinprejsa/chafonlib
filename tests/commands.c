#include <stdio.h>

#include <chafonlib/reader.h>
#include <chafonlib/commands.h>

int main() {
  reader_handle r;

  reader_error err = reader_init(&r, "/dev/ttyUSB0");
  if (err.kind != READER_NO_ERROR) {
    printf("Reader initialization failed %s: %s\n", reader_error_to_string(err), err.message);
    return 1;
  }

  reader_command setreadermode = {
    .address = READER_ADR_BROADCAST,
    .command = READER_CMD_CHANGE_MODE,
    .size = 1,
    .data = (uint8_t[]) {READER_ARG_ANSWER_MODE},
  };

  err = reader_execute(&r, setreadermode);
  if (err.kind != READER_NO_ERROR) {
    printf("Reader command execution failed %s: %s\n", reader_error_to_string(err), err.message);
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
  if (err.kind != READER_NO_ERROR) {
    printf("Reader command execution failed %s: %s\n", reader_error_to_string(err), err.message);
    return 1;
  }

  printf("Temperature: %d\n", r.response.data[2]);

  reader_destroy(&r);
  return 0;
}