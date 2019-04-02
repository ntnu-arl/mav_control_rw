
/* Includes ---------------------------------------------------------------------------------------------*/
#include "uart.h"

/* Defines ----------------------------------------------------------------------------------------------*/
#define MAX_DEVICE_LEN      16

/* Variables --------------------------------------------------------------------------------------------*/
static struct termios options;
static char portName[MAX_DEVICE_LEN];
int serialPort;

/* Functions --------------------------------------------------------------------------------------------*/
/* Open serial port*/
int openSerialPort(char *device, int port){
  if(device == NULL)
    device = DEV_ACM;

  snprintf(portName, MAX_DEVICE_LEN, "%s%d", device, port);
  serialPort = open(portName, O_RDWR | O_NOCTTY);

  if (serialPort != -1){
    if(!strcmp(device, DEV_ACM)) {  /* USB CDC (virtual COM port ) */
      tcgetattr(serialPort, &options);				/* get current port options */
      cfsetispeed(&options, B115200);				/* set baud rate */
      cfsetospeed(&options, B115200);

      options.c_cflag |= (CLOCAL | CREAD | CS8);			/* control options - local mode, enable receiver, 8N1 configuration, no flow control */
      options.c_cflag &= ~(CSIZE | PARENB | CSTOPB );
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | IEXTEN | ISIG);	/* local options - canonical input */
      options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

      options.c_oflag &= ~OPOST;					/* output options - raw output */
      
      // HUAN
      //options.c_cc[VMIN]  = 1; /* Read 1 characters */  
      //options.c_cc[VTIME] = 1;  /* Wait 0.1s   */
      
      tcsetattr(serialPort, TCSANOW, &options);
    } else {   /* /dev/serial or /dev/ttyUSB */
      tcgetattr(serialPort, &options);
      options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		/* set baud rate */
      options.c_iflag = IGNPAR;
      options.c_oflag = 0;
      options.c_lflag = 0;
      
      // HUAN
      //options.c_cc[VMIN]  = 1; /* Read 1 characters */  
      //options.c_cc[VTIME] = 1;  /* Wait 0.1s   */
      
      tcflush(serialPort, TCIFLUSH);
      tcsetattr(serialPort, TCSANOW, &options);
    }
    
    signal (SIGINT, (void*)signalHandler);
  }
  else {
    printf("Unable to open %s.  (Is device powered on?)\n", portName);
  }
  return (serialPort);
}

void closeSerialPort(int port){
  tcsetattr(serialPort,TCSANOW, &options);
  close(port);
}

int readSerial(uint8_t *readBuffer, uint32_t bytesToRead){
  uint32_t bytesRead = 0;

  while(bytesToRead > bytesRead){
    read(serialPort, readBuffer, 1);
    readBuffer++;
    bytesRead++;
  }

  return bytesRead;
}

void serialWrite(uint8_t *writeBuffer, uint32_t len){
  int bytesWritten;

  bytesWritten = write(serialPort, writeBuffer, len);
  if (bytesWritten < len){
    printf("Write failed \n");
  }
}

/* close serial port on ctrl+c */
void signalHandler(int sig){
  closeSerialPort(serialPort);
  exit (sig);
}

uint16_t crc_generate(uint8_t *buffer, size_t length, uint16_t startValue ) {
  uint16_t crc;
  uint8_t *p;
  int ii;

  crc = startValue;

  for(p = buffer, ii = 0; ii < length; ii++) {
    crc = (crc << 8) ^ crc_table[(crc >> 8) ^ *p];
    p++;
  }

  return crc;

}
