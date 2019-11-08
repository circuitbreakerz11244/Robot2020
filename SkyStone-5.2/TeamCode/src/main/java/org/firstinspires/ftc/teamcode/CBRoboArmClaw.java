package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CBRoboArmClaw  {

    //Arm Motor
    public DcMotor arm;

    //Servo Range 0 to 1. [ 0 to 180 Degree ] (0 degree --> 0, 90 Degree --> 0.5, 180 Degree --> 1.0)
    //Servo for Claw
    public Servo claw;

    //Servo for pull foundation
    public Servo pullServo;

    //TBD - Compute and fix this value
    final double CLAW_OPEN  = 0.3;
    final double CLAW_CLOSE = 0.8;

    final double SERVO_INITIAL = 0.0; //move to   0 degree
    final double SERVO_MIDDLE  = 0.5; //move to  90 degrees
    final double SERVO_FINAL   = 1.0; //move to 180 degrees


    boolean bArmInitialized = false;

    CBRoboArmClaw() {
    }

    CBRoboArmClaw(HardwareMap hardwareMap) {
        this.bArmInitialized = initializeArm(hardwareMap);
    }

    CBRoboArmClaw(HardwareMap hardwareMap, boolean bEncodersOn) {
        this.bArmInitialized = initializeArm(hardwareMap, bEncodersOn);
    }

    public boolean initializeArm(HardwareMap hardwareMap) {
        return initializeArm(hardwareMap, true);
    }

    public boolean initializeArm(HardwareMap hardwareMap, boolean bEncodersOn) {

        //Arm Motor
        arm  = hardwareMap.get(DcMotor.class,  "arm"  );

        //Servos
        claw = hardwareMap.servo.get("claw");
        pullServo = hardwareMap.servo.get("pullServo");

        //Make sure all Hardware are initialized
        if(arm != null && claw != null && pullServo != null) {
            bArmInitialized = true;
        }

        //Set the encoders based on parameter passed
        encodeArmMotors(bEncodersOn);
        //Start with Zero Power Behaviour
        applyArmMotorsBrake();

        return bArmInitialized;
    }

    public void encodeArmMotors(boolean bEncodersOn) {
        if (bEncodersOn) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void applyArmMotorsBrake() {
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopArmMotors() {
        arm.setPower(0);
    }

    public void resetArmMotorsEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void clawOpen() {
        claw.setPosition(CLAW_OPEN);
    }

    public void clawOpen(double pDouble) {
        claw.setPosition(pDouble);
    }

    public void clawClose() {
        claw.setPosition(CLAW_CLOSE);
    }

    public void clawClose(double pDouble) {
        claw.setPosition(pDouble);
    }

    public boolean getArmInitializationStatus() {
        return bArmInitialized;
    }

}
