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

    //Servo to grab skystone during autonomous
    public Servo skystoneServo;

    //Servo to hold capstone
    public Servo capstoneServo;

    boolean bArmInitialized = false;

    CBRoboArmClaw() {
    }

    CBRoboArmClaw(HardwareMap hardwareMap) {
        this.bArmInitialized = initializeArm(hardwareMap);
    }

    CBRoboArmClaw(HardwareMap hardwareMap, boolean bArmEncoderOn) {
        this.bArmInitialized = initializeArm(hardwareMap, bArmEncoderOn);
    }

    CBRoboArmClaw(HardwareMap hardwareMap, boolean bArmEncoderOn, boolean bClawEncoderOn) {
        this.bArmInitialized = initializeArm(hardwareMap, bArmEncoderOn, bClawEncoderOn);
    }

    public boolean initializeArm(HardwareMap hardwareMap) {
        return initializeArm(hardwareMap, false);
    }

    public boolean initializeArm(HardwareMap hardwareMap, boolean bArmEncoderOn) {
        return initializeArm(hardwareMap, bArmEncoderOn, false);
    }

    public boolean initializeArm(HardwareMap hardwareMap, boolean bArmEncoderOn, boolean bClawEncoderOn) {

        //Arm Motors
        armMotor  = hardwareMap.get(DcMotor.class,  "armMotor"  );
        clawMotor = hardwareMap.get(DcMotor.class,  "clawMotor"  );

        //Servos
        claw = hardwareMap.servo.get("claw");
        pullServo = hardwareMap.servo.get("pullServo");
        skystoneServo = hardwareMap.servo.get("skystoneServo");
        capstoneServo = hardwareMap.servo.get("capstoneServo");

        //Make sure all Hardware are initialized
        if(armMotor != null && clawMotor != null && claw != null && pullServo != null && skystoneServo != null && capstoneServo != null) {
            bArmInitialized = true;
        }

        //Set the encoders based on parameter passed
        encodeArmMotor(bArmEncoderOn);
        encodeClawMotor(bClawEncoderOn);

        //Start with Zero Power Behaviour
        applyArmMotorBrake();
        applyClawMotorBrake();

        return bArmInitialized;
    }

    public boolean initializeServosForAuto(HardwareMap hardwareMap) {
        pullServo = hardwareMap.servo.get("pullServo");
        skystoneServo = hardwareMap.servo.get("skystoneServo");
        //Make sure servos are initialized
        if(pullServo != null && skystoneServo != null) {
            return true;
        } else {
            return false;
        }
    }

    public void encodeArmMotor(boolean bEncodersOn) {
        if (bEncodersOn) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void encodeClawMotor(boolean bEncodersOn) {
        if (bEncodersOn) {
            clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void applyArmMotorBrake() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void applyClawMotorBrake() {
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopArmMotor() {
        armMotor.setPower(0);
    }

    public void stopClawMotor() {
        clawMotor.setPower(0);
    }

    public void resetArmMotorEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetClawMotorEncoder() {
        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void clawOpen() {
        claw.setPosition(CBRoboConstants.CLAW_SERVO_OPEN);
    }

    public void clawOpen(double pDouble) {
        claw.setPosition(pDouble);
    }

    public void clawClose() {
        claw.setPosition(CBRoboConstants.CLAW_SERVO_CLOSE);
    }

    public void clawClose(double pDouble) {
        claw.setPosition(pDouble);
    }

    public void pullServoOpen() {
        pullServo.setPosition(CBRoboConstants.PULL_SERVO_OPEN);
    }

    public void pullServoOpen(double pDouble) {
        pullServo.setPosition(pDouble);
    }

    public void pullServoClose() {
        pullServo.setPosition(CBRoboConstants.PULL_SERVO_CLOSE);
    }

    public void pullServoClose(double pDouble) {
        pullServo.setPosition(pDouble);
    }

    public void skystoneServoOpen() {
        skystoneServo.setPosition(CBRoboConstants.SKYSTONE_SERVO_OPEN);
    }

    public void skystoneServoOpen(double pDouble) {
        skystoneServo.setPosition(pDouble);
    }

    public void skystoneServoClose() {
        skystoneServo.setPosition(CBRoboConstants.SKYSTONE_SERVO_CLOSE);
    }

    public void skystoneServoClose(double pDouble) {
        skystoneServo.setPosition(pDouble);
    }

    //
    public void capstoneServoOpen() {
        capstoneServo.setPosition(CBRoboConstants.CAPSTONE_SERVO_OPEN);
    }

    public void capstoneServoOpen(double pDouble) {
        capstoneServo.setPosition(pDouble);
    }

    public void capstoneServoClose() {
        capstoneServo.setPosition(CBRoboConstants.CAPSTONE_SERVO_CLOSE);
    }

    public void capstoneServoClose(double pDouble) {
        capstoneServo.setPosition(pDouble);
    }

    public void servoOpen(Servo servo, double position) {
        servo.setPosition(position);
    }

    public boolean getArmInitializationStatus() {
        return bArmInitialized;
    }

}
