#ifndef PID_h
#define PID_h

// all PID opperations need these values and so layering them makes sense
struct PIDlong{
  long* all;
  PIDlong();
  inline long current();
  inline long previous();
  void shift();
  inline long delta();
  long average();
  long average(int,int);
  long averageDelta();
};
struct PIDdouble {
  double* all;
  PIDlong();
  inline double current();
  inline double previous();
  void shift();
  inline double delta();
  double average();
  double average(int,int);
  double averageDelta();
  };
#endif
