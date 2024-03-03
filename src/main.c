
#include <stdio.h>
#include "reader.h"

int main(void) {
  reader_handle r;

  int err = reader_init(&r, "/dev/ttyUSB0");
  if (err) {
    printf("Reader initialization failed: %s\n", reader_error_to_string(err));
    perror(NULL);
  }

  char* data = {0x0};
  err = reader_execute(&r, 0xff, 0x76, data, 1);
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    perror(NULL);
  }

  err = reader_execute(&r, 0xff, 0x21, NULL, 0);
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    perror(NULL);
  }

  reader_destroy(&r);
}