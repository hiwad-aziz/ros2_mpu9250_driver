#ifndef __I2CCOMMUNICATOR_H__
#define __I2CCOMMUNICATOR_H__

class I2cCommunicator {
 public:
  virtual ~I2cCommunicator() = default;
  virtual int read(unsigned char address) = 0;
  virtual int write(unsigned char address, unsigned char value) = 0;
  virtual char getFile() = 0;
};

#endif  // __I2CCOMMUNICATOR_H__