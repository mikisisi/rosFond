#ifndef CONTROLLEURPWMPCA9685_H
#define CONTROLLEURPWMPCA9685_H

#include "i2c8bit.h"


#define CHANNEL0    0

class controlleurPWMPCA9685: public i2c8Bit
{
public:
    long calibrator(long x, long in_min, long in_max, long out_min, long out_max);
    controlleurPWMPCA9685();
    controlleurPWMPCA9685(unsigned char dev_addr, std::string i2cfilename);
    virtual ~controlleurPWMPCA9685();
    void setPWM(int channel,int valeurDebut, int valeurFin);
    void setFreq(unsigned char freq);
    void reset();
protected:
private:



};

#endif // CONTROLLEURPWMPCA9685_H
