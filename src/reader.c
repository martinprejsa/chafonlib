#include "chafon-reader/reader.h"

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static char const *const reader_error_strings[] = {
    "READER_NO_ERROR",
    "READER_DEVICE_CONFIGURATION_ERROR",
    "READER_DEVICE_COMMUNICATION_ERROR",
    "READER_INVALID_PARAMETER"
};

char const *reader_error_to_string(reader_error r) {
  char const *error = NULL;
  if (r.kind < READER_ERROR_SIZE) {
    error = reader_error_strings[r.kind];
  }
  return error;
}

reader_error reader_init(reader_handle *reader, char const *const d_path) {
  int device = open(d_path, O_RDWR);
  if (device) {
    return (reader_error) {
      .kind = READER_DEVICE_CONFIGURATION_ERROR,
      .message = strerror(errno)
    };
  }

  struct termios tty;
  if (tcgetattr(device, &tty) != 0) {
    return (reader_error) {
      .kind = READER_DEVICE_CONFIGURATION_ERROR,
      .message = strerror(errno)
    };
  }

  struct termios {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[NCCS];
  };

  tty.c_cflag &= ~PARENB; // no parity-bit
  tty.c_cflag &= ~CSTOPB; // single bit stop

  tty.c_cflag &= ~CSIZE; // clear bit-size
  tty.c_cflag |= CS8;    // 8 bit per byte

  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
  tty.c_cflag |=
      CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON; // disable canonical mode

  tty.c_lflag &= ~ECHO;   // Disable echo
  tty.c_lflag &= ~ECHOE;  // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g.
                         // newline chars)
  tty.c_oflag &=
      ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 1;

  cfsetispeed(&tty, B57600);
  cfsetospeed(&tty, B57600);

  if (tcsetattr(device, TCSANOW, &tty) != 0) {
    return (reader_error) {
      .kind = READER_DEVICE_CONFIGURATION_ERROR,
      .message = strerror(errno)
    };
  }

  reader->device = device;
  reader->mode = READER_UNKNOWN_MODE;
  return (reader_error) {
      .kind = READER_NO_ERROR
  };
}

void reader_destroy(reader_handle *r) {
  close(r->device);
  free(r->response.data);
}

uint16_t crc16_mcrf4xx(uint16_t crc, uint8_t *data, size_t len) {
  if (!data || len < 0)
    return crc;

  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0x8408;
      else
        crc = (crc >> 1);
    }
  }
  return crc;
}

reader_error write_frame(reader_handle *const reader, reader_command const c) {
  if (c.size > 251) {
    return (reader_error) {
      .kind = READER_INVALID_PARAMETER,
      .message = "command data length too large"
    };
  }

  size_t len = c.size + 5;
  uint8_t *buff = (uint8_t*)calloc(sizeof(char), len);

  buff[0] = len - 1;
  // adr + cmd + crc = 4bytes
  buff[1] = c.address;
  buff[2] = c.command;

  memcpy(buff + sizeof(char) * 3, c.data, c.size);

  uint16_t crc = crc16_mcrf4xx(0xFFFF, buff, len - 2);

  buff[len - 2] = (uint8_t)crc;        // lsb
  buff[len - 1] = (uint8_t)(crc >> 8); // msb

  int w = write(reader->device, buff, len);
  free(buff);

  if (w == -1) {
    return (reader_error) {
      .kind = READER_DEVICE_COMMUNICATION_ERROR,
      .message = "unexpected EOF"
    };
  }

  return (reader_error) {
      .kind = READER_NO_ERROR
  };
}

reader_error read_frame(reader_handle *const reader) {
  uint8_t response[255] = {0};
  int length = 0;
  int r = read(reader->device, &response, sizeof(uint8_t));

  length = response[0];
  if (r <= 0 || length < 4) {
    return (reader_error) {
      .kind = READER_DEVICE_COMMUNICATION_ERROR,
      .message = "unexpected packet size recieved"
    };
  }

  r = read(reader->device, response + sizeof(uint8_t), length);
  if (r != length) {
    return (reader_error) {
      .kind = READER_DEVICE_COMMUNICATION_ERROR,
      .message = "unexpected packet size recieved"
    };
  }

  uint16_t crc = (response[length-2] | (response[length-1] << 7));
  //TODO: crc check

  reader_response resp = {
    .command = response[1],
    .address = response[2],
    .status = response[3],
    .size = length - 5,
    .data = calloc(sizeof(uint8_t), resp.size),
  };

  memcpy(resp.data, response + 4 * sizeof(uint8_t), resp.size);

  if (reader->response.data != NULL) {
    free(reader->response.data);
  }

  reader->response = resp;

  if (resp.command == READER_RCMD_RTI) {
    if (reader->rti_data.data != NULL) {
      free(reader->rti_data.data);
    }

    uint8_t rti_data_len = resp.size - 3;
    uint8_t *rti_data = (uint8_t*) calloc(sizeof(uint8_t), rti_data_len);
    memcpy(rti_data, resp.data+2, rti_data_len);
    reader->rti_data = (reader_rti_data) {
      .ant = resp.data[0],
      .data = resp.data,
      .rssi = resp.data[resp.size-1],
    };
  }

  return (reader_error) {
      .kind = READER_NO_ERROR
  };
}

reader_error reader_execute(reader_handle *const reader, reader_command const command) {
  if (
    (reader->mode == READER_RTI_MODE || reader->mode == READER_RTIT_MODE) &&
    (command.command != READER_CMD_OBTAIN_INFO || command.command != READER_CMD_CHANGE_MODE)
  ) {
    return (reader_error) {
      .kind = READER_INVALID_PARAMETER,
      .message = "invalid command for current mode"
    };
  }

  reader_error err;
  err = write_frame(reader, command);
  if (err.kind) {
    return err;
  }

  err = read_frame(reader);
  if (err.kind) {
    return err;
  }

  if (reader->response.command != command.command) {
    return (reader_error) {
      .kind = READER_DEVICE_COMMUNICATION_ERROR,
      .message = "response command does not match executed command"
    };
  }

  return (reader_error) {
      .kind = READER_NO_ERROR
  };
}