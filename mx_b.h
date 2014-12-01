#ifndef mx_b_h
#define mx_b_h

class mx_b {
  public:
    double apply(double);// do f(x)
    double inverse(double);// do f^-1(x)
    void update(double,double);// change the linear values
  private:
    double m;
    double b;

};

#endif
