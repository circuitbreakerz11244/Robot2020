package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardwareCB {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor wheelyThing;
    DcMotor arm;

    BNO055IMU imu;

    Servo claw;

    int i = 0;

    final double CLAW_INIT = 0.5; // starting position in auton init
    final double CLAW_OPEN = 0.3;
    final double CLAW_CLOSE = 0.8;
    final double INTAKE_POWER = .7;

    String lrName = "leftRear";
    String lfName = "leftFront";
    String rrName = "rightrear";
    String rfName = "rightFront";
    String wtName = "wheelyThing";
    String armName = "arm";
    String clawName = "claw";
}




