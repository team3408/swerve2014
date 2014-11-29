swerve2014
==========

The swerve drive code for the sasquatch ONLY

The current organizational plan by different classes.
each class should have its own .h and .cpp file.

main
----
This file will have only four functions within it.
void loop
void setup
void in(int)
void out

The loop and setup are the two basic functions required for every arduino program
The in and out are going to be the two different handler functions called by the wire library(for the I2C connection).

The setup should initialize all the values and declare all objects that need to be in the scope of the main file
This include both the wire library initialization, and all the other objects. Outside of the setup the names will be declared.
Inside of it they will be initialized with a value.

The loop function will be arranged so that it calls the updating functions only once, every time the I2C connection is called, it will also have all the custom send back functions, and a way to track the speed of the actions

The in function should only read the 5 values from the array and store them in the memory somewhere.

The out function should only shift out the 16 values it needs to shift out. 8 for the actual drive, and 8 for data analysis.



