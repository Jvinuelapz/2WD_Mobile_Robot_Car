#ifndef MyLibrary_h
#define MyLibrary_h

#include <Arduino.h>

class MyLibrary {
  public:
    MyLibrary();
    int addNumbers(int num1, int num2);
  private:
    int _number;
};

#endif

