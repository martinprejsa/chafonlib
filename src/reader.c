#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

#include "reader.h"

char const * const reader_error_strings[] = {
  "READER_NO_ERROR",
  "READER_DEVICE_CONFIGURATION_ERROR",
  "READER_DEVICE_COMMUNICATION_ERROR"
};

char const * reader_error_to_string(reader_error r) {
  char const * error = NULL;
  if (r < READER_ERROR_SIZE) {
    error = reader_error_strings[r];
  } 
  return error;
}

reader_error reader_init(reader_handle *reader, char const * const d_path) {
  int device = open(d_path, O_RDWR);

  struct termios tty;
  if(tcgetattr(device, &tty) != 0) {
    return READER_DEVICE_CONFIGURATION_ERROR;
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
  tty.c_cflag |= CS8; // 8 bit per byte

  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON; // disable canonical mode
  	
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10; 
  tty.c_cc[VMIN] = 1; 

  cfsetispeed(&tty, B57600);
  cfsetospeed(&tty, B57600); 

  if (tcsetattr(device, TCSANOW, &tty) != 0) {
        return READER_DEVICE_CONFIGURATION_ERROR;
  }

  reader->device = device;

  return READER_NO_ERROR;
}

void reader_destroy(reader_handle *r) {
  close(r->device);
}

uint16_t crc16_mcrf4xx(uint16_t crc, char *data, size_t len)
{
    if (!data || len < 0)
        return crc;

    while (len--) {
        crc ^= *data++;
        for (int i=0; i<8; i++) {
            if (crc & 1)  crc = (crc >> 1) ^ 0x8408;
            else          crc = (crc >> 1);
        }
    }
    return crc;
}

reader_error reader_execute(reader_handle * const reader, char address, char command, char* data, char size) {
  size_t len = size + 5;
  char* buff = (char*) calloc(sizeof(char), len);

  buff[0] = len-1;
  // adr + cmd + crc = 4bytes
  buff[1] = address;
  buff[2] = command;
  
  memcpy(buff + sizeof(char) * 3, data, size);
  
  uint16_t crc = crc16_mcrf4xx(0xFFFF, buff, len-2);

  buff[len-2] = (char) crc; //lsb
  buff[len-1] = (char) (crc >> 8); //msb

  write(reader->device, buff, len);

  char response[256];
  int length = 0;
  int r = read(reader->device, &response, 1);
  
  if (r == 0 || r == -1 || length > 256) {
    return READER_DEVICE_COMMUNICATION_ERROR;
  }

  length = response[0];

  char buffer[256] = {0};
  r = read(reader->device, buffer, length);
  memcpy(response + sizeof(char), buffer, r);

  for (int i = 0; i < length; i++) {
    if(response[i] != 0x0) {
      printf("%0x", response[i]);
    }
    printf("\n");
  }

  reader->response_size = response[0] + 1;
  reader->response = (char*) calloc(sizeof(char), reader->response_size);

  memcpy(reader->response, response, reader->response_size);

  free(buff);
  free(reader->response);

  return READER_NO_ERROR;
}