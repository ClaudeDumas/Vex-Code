void driveLoop();
void calibrationLoop();
void motor1Button(vex::controller::button onButton, vex::motor myMotor);
void motor2Button(vex::controller::button onButton, vex::controller::button offButton, vex::motor myMotor);
void toggle(vex::controller::button onButton, vex::controller::button offButton, vex::digital_out dOut);