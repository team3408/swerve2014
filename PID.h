#ifndef PID_h
#define PID_h

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

#endif
