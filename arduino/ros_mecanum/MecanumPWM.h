#ifndef MECANUMPWM_H
#define MECANUMPWM_H

#include <Arduino.h>

// class used to vector mecanum motors driven by pwm motor controllers
class MecanumPWM {
  public:
    MecanumPWM(int _pwmFR, int _pwmFL, int _pwmRR, int _pwmRL, int _dirFRA, int _dirFLA,
  int _dirRRA, int _dirRLA,int _dirFRB, int _dirFLB, int _dirRRB, int _dirRLB, float _maxSpd);
    ~MecanumPWM();
    void commandMotors(float driveChar, float turnChar, float strafeChar);
    void allStop();
    void debugMotorPrint();

  private:
    // pwm and dir pins
    int pwmFR, pwmFL, pwmRR, pwmRL;
    int dirFRA, dirFLA, dirRRA, dirRLA;
    int dirFRB, dirFLB, dirRRB, dirRLB;

    // Motor params
    float maxSpd = 200;
    float driveVal, turnVal, strafeVal;

    typedef struct {
      int pulse;
      bool direction;
      bool brake;
    } MotorValues;

    // Defines structs for each motor
    MotorValues motorFR, motorFL, motorRR, motorRL;

    void setDirection();
    void normalizeVectors();
    void normalizePulses();
    float getAbsolute(float val);
    float convertByteToFloat(unsigned char val);
};
#endif
