#include <stdint.h>
#include <stdio.h>
#include "reader.h"

int main(void) {
  reader_handle r;

  int err = reader_init(&r, "/dev/ttyUSB0");
  if (err) {
    printf("Reader initialization failed: %s\n", reader_error_to_string(err));
    perror(NULL);
  }

  uint8_t data[1] = {0x0};
  err = reader_execute(&r, 0xff, 0x76, data, 1);
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    perror(NULL);
  }
  if (r.response[2] != 0x76) {
    printf("Invalid response\n");
  } else {
    printf("Reader set to answer mode");
  }

  err = reader_execute(&r, 0xff, 0x92, NULL, 0);
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    perror(NULL);
  }

  if (r.response[2] != 0x92) {
    printf("Invalid response\n");
  }

  printf("Temperature: %d\n", r.response[5]);

  reader_destroy(&r);
}