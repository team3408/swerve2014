#include "Arduino.h"
#include "KCswerve310.h"

#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>
#include <ROEncoder.h>
#include <algorithm>
using namespace std;

#include <math.h>
#define PI 3.141592654
#define s2 1.414213562

//Preprocessor directives = better constants
#define PwmZero 128
#define PwmSize 127

//class constructor
swerveClass swerve;

//all in radians
double swerveClass::rotationLogic(double assign, double encoder, int pin){ //function declaration, 
		/*
		This code is making the assumption that rotation is determined counterclockwise, 
		and that the values grabbed from the electrical components have zero as forward
		because they are crazy.
		*/
	encoder += PI / 2.0; // These two lines codes make forward PI/2.0 
        while (encoder >= 2.0 * PI){encoder -= 2.0*PI;} 
		/*
		This rotates the motors(vectors) into a valid position. which is between 0 and PI	
		Then all the motors that had to be rotated, have that fact recorded.
		ALL motors have counter clockwise rotation as positive
		*/
	if(assign > PI){assign -= PI;invertOn = (invertOn | byte(1 << (pin + 4)));} //invert on has greatest 4 place values
	else{invertOn = (invertOn & (~byte(1 << (pin + 4))) );}  // for inversion, and lower ones for on(whether or not the motor moves)
		/*
		This turns on the speed motors when the rotation is close enough to the encoder
		This converts to radians then turns it on if the vector is within 5 values
		C++ compiler will probably convert it to something more efficient.
		1024 is the encoder's precision
		*/
	double temp = 0.0;
	temp = (assign - encoder)/ PI * 180.0 /360.0 * 1024.0;
	if ((temp < 10.0) && (temp > -10.0)){ invertOn = (invertOn | byte(1 << pin)); }
	else { invertOn = (invertOn & (~byte(1 << pin))); }
		/*
		This last bit of code for the function returns a value between 1 and -1
		based on where the motor is to where it needs to be, for the stuff to work.
		Temp is in (1024 values per rotation),
		and to makes sure the value returned is outside the deadzone (4%)
		we are adding .0472, which will put outside the deadzone usually (unless the value is within 2)
		if we used the value of .048 it would always be outside
		as when converted to byte that is the value above the .04 deadzone
		we need it to eventually stop to prevent battery drain
		*/
	temp = temp / 1024;
	if (temp > 0.0){temp += .0472 ;} else {temp -= .0472;} 
	return temp;
}
void swerveClass::mx_b (double& minS, double& ddi, MotorValues& fr, MotorValues& br, MotorValues& bl, MotorValues& fl ){
  int CASE = 8*(fr.working) + 4*(br.working) + 2*(bl.working) + (fl.working);
  switch(CASE){
// everything is working
  case 15:
    minS = min( min(fr.slope, br.slope), min(bl.slope, fl.slope));
    ddi = min( min(fr.intercept, br.intercept), min(bl.intercept, fl.intercept));  
    return;
// one motor is missing
  case 14:// motor fl is missing
    minS = min( min(fr.slope, br.slope),  bl.slope);
    ddi = min( min(fr.intercept, br.intercept), bl.intercept);  
    return;
  case 13:// motor bl is missing
    minS = min( min(fr.slope, br.slope), fl.slope);
    ddi = min( min(fr.intercept, br.intercept), fl.intercept);  
    return;    
  case 11:// motor br is missing
    minS = min(fr.slope, min(bl.slope, fl.slope));
    ddi = min(fr.intercept, min(bl.intercept, fl.intercept));  
    return;
  case 7:// motor fr is missing
    minS = min( br.slope, min(bl.slope, fl.slope));
    ddi = min( br.intercept, min(bl.intercept, fl.intercept));  
    return;
//two motors are missing
  //fr is one of the motors present
  case 12:// also br
    minS = min(fr.slope, br.slope);
    ddi =  min(fr.intercept, br.intercept);  
    return;
  case 10:// also bl
    minS = min(fr.slope, bl.slope);
    ddi = min(fr.intercept, bl.intercept);  
    return;
  case 9: // also fl
    minS = min(fr.slope, fl.slope);
    ddi = min(fr.intercept, fl.intercept);  
    return;
  //fr is not present
  //br is present
  case 6: //br and bl
    minS = min(br.slope, bl.slope);
    ddi = min( br.intercept, bl.intercept);  
    return;    
  case 5:// br and fl
    minS = min(br.slope, fl.slope);
    ddi = min( br.intercept, fl.intercept);  
    return;
  //The LAST CASE
  case 3:// bl and fl
    minS = min(bl.slope, fl.slope);
    ddi =  min(bl.intercept, fl.intercept);  
    return;
    
  default:  
    minS = 1.0;
    ddi = 0.0;
    return;
}}

void swerveClass::inversion(double* output, double& fr, double& br, double& bl, double& fl){
  // inverts the motor values which happen to be in two places.
	 if (byte(invertOn) & (1 << 4)){output[0]*= -1.0; fr *= -1.0;}   
	 if (byte(invertOn) & (1 << 5)){output[1]*= -1.0; br *= -1.0;}
	 if (byte(invertOn) & (1 << 6)){output[2]*= -1.0; bl *= -1.0;}
	 if (byte(invertOn) & (1 << 7)){output[3]*= -1.0; fl *= -1.0;}
}

double* swerveClass::update(double x, double y, double r, 
                                      double rotationalFR,
                                      double rotationalBR, 
                                      double rotationalBL,
                                      double rotationalFL ){
	robot = new vector(x,y);
	rotation = new vector(s2 * r, s2 * r);// this the right direction for FR
	//vector creation 
	FRv = *robot + *rotation; // I overrided the addition operater 
	rotation -> rotate(PI / 2.0); // This is the rotation vector assuming the robot is a square. and this is a quarter rotation
	BRv = *robot + *rotation;
	rotation -> rotate(PI / 2.0);
	BLv = *robot + *rotation;
	rotation -> rotate(PI / 2.0);
	FLv = *robot + *rotation;

	// indexes start at 0 so the 5th index is 4 not 5.
	output[4]= rotationLogic(FRv -> r(), rotationalFR , 0);// these indexes refer to the whole hub and so start at 0
	output[5]= rotationLogic(BRv -> r(), rotationalBR , 1);
	output[6]= rotationLogic(BLv -> r(), rotationalBL , 2);
	output[7]= rotationLogic(FLv -> r(), rotationalFL , 3);
	
	/*
	Rather than a function for all the other motors, all that needs to be done is
	check that all of the values are close enough that the robot is allowed to move.
	The '&' in the if statement is not a boolean 'and' but a bitwise 'and'
	The if statement supposedly checks that all the ON bits in invertOn are on(or 1)
	*/
	if((invertOn & byte(0x0F)) == byte(0x0F)){
	output[0] = FRv -> m(); // all the inversion logic is done by the main code not this function
	output[1] = BRv -> m(); // this simply grabs the magnitude.
	output[2] = BLv -> m();	
	output[3] = FLv -> m();	
	}else{
	output[0]= output[1] = output[2] = output[3] = 0.0;// this assigned them all to zero if one of them can't move.
	}
		
        // clean memory since always new vectors/ Garbage collection
	// c++ doesn't have garbage collection
        delete robot;
        delete rotation;
        delete FRv;
        delete BRv;
        delete BLv;
        delete FLv;
        //this is all of the motor values ordered from left to right (wheels(FR,BR,BL,FL),turning(FR,BR,BL,FL))
	return output;
}
MotorValues::MotorValues(int num){pin = num;
    SpeedE = new ROEncoder(pin);
    RotationE = new ROAnalog(pin);

    distance.current = 0;// Do these need to be set????
    time.current = 0;
    
    assigned.current = 0.0;// this makes the literal a float instead of an int.
  //not needed  valueRotation.current = 0.0;
}
void MotorValues::derive(){//cant use velocity as already named
	// This moves all the values that were current
	// to previous so that the next operations work.
  velocity.shift();
  distance.shift();
  time.shift();
	//there is not an assigned shift as there is no assigned to be generated.
	//all the currents are being updated
  distance.current = SpeedE -> read();
  time.current = long( millis());// millis() returns an unsigned long
  velocity.current = double(distance.delta()) / double(time.delta());
  	/*
  	This is all of the weirdness calculation
  	The motors are disproportionate to each other, but proportionate to themselves
  	Therefore you think of all of them as the least efficient motor, get a speed.
  	Then figure out what value you would need to get that speed on the related motor.
  	*/
  	/*
	The assigned, can not have a failed/0 value in the last two cycles,
	so it checks the assigned value in those previous cycles,
	as then it won't throw off other motors as being extremely inefficient.
	The value needs to be .06 as when converted to byte that is the value above the .04 deadzone
	and we won't have to worry about it(hopefully)
  	*/
  working = ((assigned.current > 0.06) || (assigned.current < -0.06))
		&& ((assigned.previous > 0.06) || (assigned.previous < -0.06));
  if(working){
  slope = velocity.delta() / assigned.delta() ;
  intercept = velocity.current - slope * assigned.current;
     if(assigned.current < 0.0){intercept *= -1;}
}
}
void swerveClass::maxGen (double* beforeSend){
  /*
  This is set up so that the code uses a local variable that is easily seen rather than
  a global variable whose purpose isn't clear
  */
  double maxGenerated = max(max(beforeSend[0],beforeSend[1]),max(beforeSend[2],beforeSend[3]));// from <algorithm> sends back highest,
   // the values haven't been inverted yet.
 if(maxGenerated > 1.0) {         
   beforeSend[0] /= maxGenerated; //since the highest value we want is 1.0 
   beforeSend[1] /= maxGenerated;//we can just divide by highest without mutiplying, since highest will go to 1.0
   beforeSend[2] /= maxGenerated;
   beforeSend[3] /= maxGenerated;
 }
}
double MotorValues:: lincomp(double m,double b, double generated){
	
	assigned.shift(); //about to get new value so do this now

	//If the weirdness values could be calculated do the operation otherwise ignore it.
	if (! working){ assigned.current = generated;}
	else{ assigned.current = (((m * generated + b)-intercept) / slope);}
	return assigned.current;
}
 vector::vector (double x, double y){
        xx = x;
        yy = y;
        magnitude = sqrt( xx * xx + yy * yy ); // uses standard library
        rotation = acos( xx / magnitude );
        rotation = yy > 0.0 ? rotation : 2.0*PI - rotation;//acos is imperfect
}

void vector::rotate(double radians) {
        rotation += radians;
        while (rotation >= 2.0 * PI){rotation -= 2.0*PI;}// only + values
        while (rotation < 0) {rotation += 2.0 * PI;}
        xx = magnitude * cos(rotation);
        yy = magnitude * sin(rotation);
} 

vector* vector::operator + (vector& second){
        return new vector (xx + second.x(),yy + second.y() );
}

double vector::x(){ return xx; }
double vector::y(){ return yy; }
double vector::m(){ return magnitude; }
double vector::r(){ return rotation; }
