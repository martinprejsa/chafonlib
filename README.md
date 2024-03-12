# chafon-reader
![](https://img.shields.io/badge/C-11-blue)

C library to interact with CHAFON UHF RFID Reader. <br>
<b>This is a work in progress. </b>

Tested on:
  - `CF-RU6403`

# Example
## Change reader mode

```C
#include <chafonlib/reader.h>
#include <chafonlib/commands.h>
...
  reader_handle r;

  // Initialize the reader handle, usually the reader device is available on
  // /dev/ttyUSB0, if no other serial devices are connected
  int err = reader_init(&r, "/dev/ttyUSB0");
  if (err) {
    printf("Reader initialization failed: %s\n", reader_error_to_string(err)); 
    return 1;
  }

  // Here we can set all the command parameters
  reader_command setreadermode = {
    .address = READER_ADR_BROADCAST,
    .command = READER_CMD_CHANGE_MODE,
    .size = 1,
    .data = (uint8_t[]) {READER_ARG_ANSWER_MODE},
  };

  // Here we execute the command. The reader's response will be stored
  // in the reader handle.
  err = reader_execute(&r, setreadermode);
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    reader_destroy(&r);
    return 1;
  }

  // Don't forget to check for reader errors
  if (r.response.status != 0x0) {
    printf("Reader mode unchanged.");
    reader_destroy(&r);
    return 1;
  }

  // And don't forget to clean up
  reader_destroy(&r);

  printf("Reader mode changed!"); 
...
```

