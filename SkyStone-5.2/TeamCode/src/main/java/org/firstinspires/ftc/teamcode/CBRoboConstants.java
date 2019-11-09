package org.firstinspires.ftc.teamcode;

public interface CBRoboConstants {

    final double DRIVE_MOTOR_MOVE_POWER   = 0.71;
    final double DRIVE_MOTOR_ROTATE_POWER = 1.00;

    //drive function constants
    final double DRIVE_MINIMUM_DRIVE_PWR = 0.16;
    final double DRIVE_DECELERATION_THRESHOLD = 6.25;

    final double ARM_MOTOR_MOVE_POWER   = 0.71;
    final double ARM_MOTOR_ROTATE_POWER = 1.00;

    //TBD - Compute and fix this value
    final double SEERVO_CLAW_OPEN  = 0.3;
    final double SEERVO_CLAW_CLOSE = 0.8;

    final double SERVO_INITIAL = 0.0; //move to   0 degree
    final double SERVO_MIDDLE  = 0.5; //move to  90 degrees
    final double SERVO_FINAL   = 1.0; //move to 180 degrees

    // counts per inch (CPI) calculation
    final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    final double WHEEL_DIAMETER_INCHES = 4.0;     // To get the circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //0 1 0 ==> -1 1 -1 1 [Direction Vector]
    double[] moveForward = {-1, 1, -1, 1};

    //0 -1 0 ==> 1 -1 1 -1 [Direction Vector]
    double[] moveBackward = {1, -1, 1, -1};

    //1 0 0 ==> -1 -1 1 1 [Direction Vector]
    double[] moveRight = {-1, -1, 1, 1};

    //-1 0 0 ==> 1 1 -1 -1 [Direction Vector]
    double[] moveLeft = {1, 1, -1, -1};

    //0 0 -1 ==> 1 1 1 1 [Direction Vector] rotate the robot in anti-clockwise direction
    double[] rotateLeft = {1, 1, 1, 1};

    //0 0 1 ==> -1 -1 -1 -1 [Direction Vector] rotate the robot in clockwise direction
    double[] rotateRight = {-1, -1, -1, -1};

    //0.71 0.71 0 ==> -1 0 0 1 [Direction Vector] move diagonally in Quad Q1
    double[] moveDiagonalQ1 = {-1, 0, 0, 1};

    //-0.71 0.71 0 ==> 0 1 -1 0 [Direction Vector] move diagonally in Quad Q2
    double[] moveDiagonalQ2 = {0, 1, -1, 0};

    //-0.71 -0.71 0 ==> 1 0 0 -1 [Direction Vector] move diagonally in Quad Q3
    double[] moveDiagonalQ3 = {1, 0, 0, -1};

    //0.71 -0.71 0 ==> 0 -1 1 0 [Direction Vector] move diagonally in Quad Q4
    double[] moveDiagonalQ4 = {0, -1, 1, 0};


}
