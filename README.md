# chafon-reader
![](https://img.shields.io/badge/C-11-blue)

C library to interact with CHAFON UHF RFID Reader. <br>
<b>This is a work in progress. </b>

Tested on:
  - `CF-RU6403`

# Usage

During answer mode:
```C
#include <chafonlib/reader.h>
#include <chafonlib/commands.h>
...
  reader_handle r; // create the handle

  int err = reader_init(&r, "/dev/ttyUSB0"); //initialize the handle
  if (err) {
    // use reader_error_to_string to obtain error strings
    printf("Reader initialization failed: %s\n", reader_error_to_string(err)); 
    return 1;
  }

  reader_command setreadermode = { // create the command
    .address = READER_ADR_BROADCAST,
    .command = READER_CMD_CHANGE_MODE,
    .size = 1,
    .data = (uint8_t[]) {READER_ARG_ANSWER_MODE},
  };

  err = reader_execute(&r, setreadermode); // execute the command
  if (err) {
    printf("Reader command execution failed: %s\n", reader_error_to_string(err));
    return 1;
  }

  reader_destroy(&r); // and clean up
  printf("Reader mode changed!"); 
...
```

