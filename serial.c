/*  Open, write and read bytes from a serial port               */
/* Refrence:  https://www.cmrr.umn.edu/~strupp/serial.html      */
/* Refrence:  https://stackoverflow.com/a/38318768/550643       */

// set speed to 115,200 baud, 8n1 (no parity)
#define DISPLAY_STRING true
#define TERMINAL "/dev/ttyUSB0"
#define RATE B115200

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int set_interface_attribs(int fd, int speed) {
  struct termios tty;

  if (tcgetattr(fd, & tty) < 0) {
    printf("Error from tcgetattr: %s\n", strerror(errno));
    return -1;
  }

  cfsetospeed( &tty, (speed_t) speed);
  cfsetispeed( &tty, (speed_t) speed);

/* Refrence:  https://www.gnu.org/software/libc/manual/html_node/Control-Modes.html */

  /* ignore modem controls */
  tty.c_cflag |= (CLOCAL | CREAD); 
  /* mask for number of bits per character */
  tty.c_cflag &= ~CSIZE;
  /* 8-bit characters */
  tty.c_cflag |= CS8; 
  /* no parity bit */
  tty.c_cflag &= ~PARENB;
  /* only need 1 stop bit */
  tty.c_cflag &= ~CSTOPB;
  /* no hardware flowcontrol */
  tty.c_cflag &= ~CRTSCTS; 

  /* setup for non-canonical mode */
  /* Refrence:  https://www.gnu.org/software/libc/manual/html_node/Input-Modes.html */
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

  /* Refrence:  https://www.gnu.org/software/libc/manual/html_node/Local-Modes.html */
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  /* https://www.gnu.org/software/libc/manual/html_node/Output-Modes.html */
  tty.c_oflag &= ~OPOST;

  /* fetch bytes as they become available */
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  /* set the parameters for the terminal now - TCSANOW */ 
  if (tcsetattr(fd, TCSANOW, & tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

/* function for timed reads */
void set_mincount(int fd, int mcount) {
  struct termios tty;
  
  /* get the parameters for the teriminal in termios tty struct */  
  if (tcgetattr(fd, & tty) < 0) {
    printf("Error tcgetattr: %s\n", strerror(errno));
    return;
  }

  /* https://www.gnu.org/software/libc/manual/html_node/Noncanonical-Input.html */
  /* minimum number of bytes that must become availible in the queue */
  tty.c_cc[VMIN] = mcount ? 1 : 0;

  /* how long to wait for serial input before returning */
  /* half second timer */
  tty.c_cc[VTIME] = 5; 

  if (tcsetattr(fd, TCSANOW, & tty) < 0)
    printf("Error tcsetattr: %s\n", strerror(errno));
}

int main() {

  /* Source ( sending ) terminal */
  char * portname = TERMINAL;
  int fd;

  /* Destination ( receiving ) terminal */
  char * read_portname = "/dev/serial0";
  int fd2;

  int wlen;
  char * xstr = "Hello, Serial!\n";
  int xlen = strlen(xstr);

  /* open for r/w, don't make serial the controlling terminal, flush all data and metadata to hardware */
  fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("Error opening %s: %s\n", portname, strerror(errno));
    return -1;
  }
  /* set baudrate 115200, 8 bits, no parity, 1 stop bit */
  set_interface_attribs(fd, B115200);

  /* set to timed read */
  //set_mincount(fd, 0);                

  /* simple output */
  wlen = write(fd, xstr, xlen);
  if (wlen != xlen) {
    printf("Error from write: %d, %d\n", wlen, errno);
  }

  /* delay for output */
  tcdrain(fd); 
  
  /* open second serial fd to recieve messages */ 
  fd2 = open(read_portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("Error opening %s: %s\n", portname, strerror(errno));
    return -1;
  }
  /* set baudrate 115200, 8 bits, no parity, 1 stop bit */
  set_interface_attribs(fd2, B115200);
  

  /* simple noncanonical input */
  do {

    unsigned char buf[80];
    int rdlen;

    rdlen = read(fd2, buf, sizeof(buf) - 1);

    if (rdlen > 0) {

      #ifdef DISPLAY_STRING
        buf[rdlen] = 0;
        printf("Read %d: \"%s\"\n", rdlen, buf);
      #else 
      /* display hex */
        unsigned char * p;
        printf("Read %d:", rdlen);
      
      for (p = buf; rdlen--> 0; p++)
        printf(" 0x%x", * p);
      
      printf("\n");
      #endif

    } else if (rdlen < 0) {
      
      printf("Error from read: %d: %s\n", rdlen, strerror(errno));

    } else {
      
      /* rdlen == 0 */
      printf("Timeout from read\n");

    }

    /* repeat read to get full message */
  } while (1);
}

