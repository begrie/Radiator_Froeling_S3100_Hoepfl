#include "serial.h"

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>

namespace radiator
{
  // #define SERIAL_TO_RADIATOR_RX 16 // ESP32-standard: 16
  // #define SERIAL_TO_RADIATOR_TX 17 // ESP32-standard: 17

  /*
   * Constructor.
   *
   * Opens and initializes the serial port.
   *
   * @param devicename The filename of the serial port device.
   * @return void
   */
  SerialPort::SerialPort(std::string devicename)
  {
    if (devicename == "Serial")
      Serial_to_Radiator = &Serial;
    else if (devicename == "Serial1")
      Serial_to_Radiator = &Serial1;
    else if (devicename == "Serial2")
      Serial_to_Radiator = &Serial2;
    else
    {
      ::perror("open_port: Unable to open serial port");
      throw("Unable to open serial port");
    }

#if defined(SERIAL_TO_RADIATOR_RX) && defined(SERIAL_TO_RADIATOR_TX)
    Serial_to_Radiator->begin(9600, SERIAL_8N1, SERIAL_TO_RADIATOR_RX, SERIAL_TO_RADIATOR_TX);
#else
    Serial_to_Radiator->begin(9600);
#endif

    std::cerr << millis() << " ms: Opened serial port " << devicename << " with 9600 baud and SERIAL_8N1 for connection to radiator Froeling P2/S3100 -> wait for answer ..." << std::endl;
  }

  /**
   * Destructor.
   */
  SerialPort::~SerialPort()
  {
    if (this->Serial_to_Radiator)
    {
      Serial_to_Radiator->end();
    }
  }

  /**
   * Writes the given data to the serial device.
   *
   * @param data The data to send.
   * @param len The number of bytes to send.
   * @return len on success or -1 on error.
   */
  int SerialPort::write(const uint8_t *data, size_t len)
  {
    if (Serial_to_Radiator)
    {
      const uint8_t *pos = data;
      size_t left = len;

      while (left > 0)
      {
        int written = Serial_to_Radiator->write(pos, left);
        // if (written < 0)
        if (written <= 0)
        {
          return -1;
        }

        left -= written;
        pos += written;
      }
    }

    return len;
  }

  /**
   * Reads data from the serial port if available.
   *
   * @param buffer The buffer to write data to.
   * @param maxlen The maximum number of bytes to read.
   * @return The number of bytes read or -1 on error.
   */
  int SerialPort::read(uint8_t *data, size_t maxlen)
  {
    int bytes;

    bytes = Serial_to_Radiator->available();
    if (bytes > 0)
    {
      // bytes = Serial_to_Radiator->read(data, maxlen); // read is non-blocking
      bytes = Serial_to_Radiator->readBytes(data, maxlen); // readBytes is blocking and wait for maxlen or timeout from Serial
    }

    return bytes;
  }

  /**
   * Wait for input being available.
   *
   * @param timeout_msec The maximum number of milliseconds to wait.
   * @return -1 on error, 0 if timeout occured. 1 if data available.
   */
  int SerialPort::waitForInput(int timeout_msec)
  {
    if (!Serial_to_Radiator)
      return -1;

    auto _start = millis();
    auto _end = _start + timeout_msec;

    while (millis() < _end)
    {
      if (Serial_to_Radiator->available())
      {
        return 1;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    return 0;
  }

}
