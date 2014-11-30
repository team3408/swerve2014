#ifndef vector_h
#define vector_h

#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>
#include <ROEncoder.h>
#include <ROAnalog.h> 

class vector {
     public:
        vector(double,double);
        void changeto(double,double);
        void rotate(double radians);
        void rotatePiOver4();
        vector* combine(vector& second);
        double m();
        double r();
        double x();
        double y();
     private:
        double xx, yy, rotation, magnitude;
};
#endif
