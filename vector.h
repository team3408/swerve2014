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
        vector* changeto(double,double);//these 3 functions return a reference to the class.
        vector* rotate(double radians);//I think it will be useful
        vector* rotatePiOver2();
        vector* combine(vector& second);//this will point to a new vector.
        double m();
        double r();
        double x();
        double y();
     private:
        double xx, yy, rotation, magnitude;
};
#endif
