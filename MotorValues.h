#ifndef MotorValues_h
#define MotorValues_h

//In C++ all structs are classes,
//but this is being declared with the keyword struct as that is how I intend to use it.
//also struct has the default that all values are public.
struct MotorValues {
  	int pin, analogOffset; 
// for the absolute encoder connection and the movement/speed digital encoder
  	ROEncoder* SpeedE;// the encoder on the wheel
  	KCEncoder* RotationE;// the encoder on the top of the motor hub

  PIDlong distance;// This is used to calculate velocity
  PIDlong time;// REMEBER TO CONVERT to long as this is an unsigned long.
  PIDdouble velocity; // this is used to calculate wierdness
  PIDdouble assigned; // these values need to be after the inversion
 
  	bool working;//This is a boolean saying whether or not the motor is moving
     
  	//replaces lincomp and a few other things.
  mx_b wierdness;
// motor values that would be sent are kept in the same array the whole time.
  	void derive();
  	MotorValues(int num);
  
};
#endif
