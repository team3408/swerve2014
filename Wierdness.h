#ifndef wierdness_h
#define wierdness_h
class wierdness{
  public:
    mx_b worst;
    void compress(MotorValues*);// takes all the values, and lowers the ones that are out of bounds 
                                 //while keeping things proportional
    void selectw(MotorValues*);//chooses the worst of the motor values
    
};
#endif
