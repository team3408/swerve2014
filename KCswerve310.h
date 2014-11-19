#ifndef KCswerve310_h
#define KCswerve310_h

#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>
#include <ROEncoder.h>
#include <ROAnalog.h> 


//Originally a mix of ints and doubles, now all doubles
class vector {
     public:
        vector (double,double);
        void rotate (double radians);
        vector* operator + (vector& second);
        double m();
        double r();
        double x();
        double y();
     private:
        double xx, yy, rotation, magnitude;
};

// all PID opperations need these values and so layering them makes sense
struct PIDlong{
  long current;
  long previous;
  inline void shift(){previous = current;};// intended to make it faster
  inline long delta(){return current - previous;}
};
struct PIDdouble {
  double current;
  double previous;
  inline void shift(){previous = current;};
  inline double delta(){return current - previous;}
};

//In C++ all structs are classes,
//but this is being declared with the keyword struct as that is how I intend to use it.
//also struct has the default that all values are public.
struct MotorValues {
  	int pin, analogOffset; 
// for the absolute encoder connection and the movement/speed digital encoder
  	ROEncoder* SpeedE;// the encoder on the wheel
  	ROAnalog* RotationE;// the encoder on the top of the motor hub

  PIDlong distance;// This is used to calculate velocity
  PIDlong time;// REMEBER TO CONVERT to long as this is an unsigned long.
  PIDdouble velocity; // this is used to calculate wierdness
  PIDdouble assigned; // these values need to be after the inversion
 
  	bool working;//This is a boolean saying whether or not the 
  	double slope;// this is the weirdness/ efficiency
  	double intercept;
// motor values that would be sent are kept in the same array the whole time.
  	void derive();
//does the composition of the two functions needed to account for weirdness  
  	double lincomp(double m,double b, double generated);
  	MotorValues(int num);
  
};

class swerveClass {
    private:
	double output[8];
	double rotationLogic(double,double,int);
	vector* FRv;
	vector* BRv;	
	vector* BLv;
	vector* FLv;

	vector* rotation;
	vector* robot;
    public:	
	//this does a lot of logic through
        void mx_b(double&,double&,MotorValues&,MotorValues&,MotorValues&,MotorValues&);
        void maxGen(double*);
	void inversion(double*,double&,double&,double&,double&);
	byte invertOn;// boolean array, first four are inverting the motors, second four are whether or not it can move
        double* update( double x, double y, double r, 
                                      double rotationFR,
                                      double rotationBR, 
                                      double rotationBL,// This is in 360 degrees but is a double
                                      double rotationFL );//All translation to doubles should be done by main code.
        //order from FR clockwise to FL
        // start with wheels aligned at start
        // counterclockwise should be positive.
        // assume gyro is moved counterclockwise for positive values
        swerveClass(){};
        
};

extern swerveClass swerve;



#endif
