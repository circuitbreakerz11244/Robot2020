package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardwareCB {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor linearSlide;
    DcMotor arm;

    BNO055IMU imu;

    Servo claw;
    Servo foundation;
    Servo holder;

    int i = 0;

    final double CLAW_INIT = 0.5; // starting position in auton init
    final double CLAW_OPEN = 0.3;
    final double CLAW_CLOSE = 0.8;
    final double CLAW_SHUT = 0.9;
    final double FOUNDATION_DOWN = 1.0;
    final double FOUNDATION_UP = 0.5;

    String lrName = "leftRear";
    String lfName = "leftFront";
    String rrName = "rightRear";
    String rfName = "rightFront";
    String lsName = "linearSlide";
    String armName = "arm";
    String clawName = "claw";
    String fName = "foundation";

}




