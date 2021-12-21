#ifndef __LINUXI2CCOMMUNICATOR_H__
#define __LINUXI2CCOMMUNICATOR_H__

#include "I2cCommunicator.h"

class LinuxI2cCommunicator final : public I2cCommunicator {
 public:
  LinuxI2cCommunicator(int bus_number = 1);
  ~LinuxI2cCommunicator();
  int read(unsigned char address) final;
  int write(unsigned char address, unsigned char value) final;
  char getFile() final;

 private:
  void reportError(int error);
  int file_;
};

#endif  // __LINUXI2CCOMMUNICATOR_H__