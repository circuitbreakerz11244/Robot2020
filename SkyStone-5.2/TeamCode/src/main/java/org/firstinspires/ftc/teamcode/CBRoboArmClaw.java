package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CBRoboArmClaw  {

    //Arm Motor
    public DcMotor armMotor;
    public DcMotor clawMotor;

    //Servo Range 0 to 1. [ 0 to 180 Degree ] (0 degree --> 0, 90 Degree --> 0.5, 180 Degree --> 1.0)
    //Servo for Claw
    public Servo claw;

    //Servo for pull foundation
    public Servo pullServo;

    //TBD - Compute and fix this value
    final double CLAW_OPEN  = 0.0;
    final double CLAW_CLOSE = 1.0;

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

        //Arm Motors
        armMotor  = hardwareMap.get(DcMotor.class,  "armMotor"  );
        clawMotor = hardwareMap.get(DcMotor.class,  "clawMotor"  );

        //Servos
        claw = hardwareMap.servo.get("claw");
        pullServo = hardwareMap.servo.get("pullServo");

        //Make sure all Hardware are initialized
        if(armMotor != null && clawMotor != null && claw != null && pullServo != null) {
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
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void applyArmMotorsBrake() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void applyClawMotorBrake() {
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopArmMotor() {
        armMotor.setPower(0);
    }

    public void stopClawMotor() {
        armMotor.setPower(0);
    }

    public void resetArmMotorEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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