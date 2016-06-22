#include "rosfond/controlleurPWMPCA9685.h"
#include <math.h>

#include <iostream>


controlleurPWMPCA9685::controlleurPWMPCA9685()
{
    i2c8Bit();

}

/*******************************************************************
 * This is the overloaded constructor. It allows the programmer to
 * specify a custom I2C device & device address
 * The device descriptor is determined by the openI2C() private member
 * function call.
 * *****************************************************************/

controlleurPWMPCA9685::controlleurPWMPCA9685(unsigned char dev_addr, std::string i2c_file_name)
{
    this->i2cFileName = i2c_file_name;
    this->deviceAddress = dev_addr;
    this->i2cDescriptor = -1;
    std::cout << " Opening I2C Device  i2c Descriptor =  " << this->openI2C() << std::endl;

}





controlleurPWMPCA9685::~controlleurPWMPCA9685(void)
{
    std::cout << " Closing PWM Generator PCA9685 Device" << std::endl;
    this->closeI2C();
}



void controlleurPWMPCA9685::reset()
{
    this->writeReg(0x00,33);
    this->writeReg(0x01,4);
}


void controlleurPWMPCA9685::setPWM(int channel,int valeurDebut, int valeurFin)
{
    this->writeReg(7+channel*4,valeurDebut/256);
    this->writeReg(6+channel*4,valeurDebut%256);
    ///Registre 8 et 9 correpondent au moment de fin de montée la 1ere sortie PWM
    this->writeReg(9+channel*4,valeurFin/256);
    this->writeReg(8+channel*4,valeurFin%256);
}


void controlleurPWMPCA9685::setFreq(unsigned char freq)
{
    unsigned char  oldmode ,  newmode;
    unsigned char  prescale;
    float prescaleval ;

    freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
    prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;

    prescale = floor(prescaleval + 0.5);
    this->readReg(0x0,oldmode);
    newmode = (oldmode&0x7F) | 0x10; // sleep
    this->writeReg(0x0,newmode);
    this->writeReg(0xFE, prescale); // set the prescaler
    this->writeReg(0x0,oldmode);
    sleep(1);
    this->writeReg(0x0,oldmode | 0xa1);

}




///Fonction qui "calibre" le signal du JS et le mets à l'échelle
long controlleurPWMPCA9685::calibrator(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



