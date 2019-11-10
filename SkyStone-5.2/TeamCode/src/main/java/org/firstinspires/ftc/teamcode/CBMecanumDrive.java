package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;


public class CBMecanumDrive {

    //Mecanum wheel Motors. Power Range for all motors is -1 to +1
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;

    int leftFrontPreviousPosition  = 0;
    int rightFrontPreviousPosition = 0;
    int leftRearPreviousPosition   = 0;
    int rightRearPreviousPosition  = 0;

    private List<DcMotor> driveMotorList = new ArrayList<DcMotor>();
    boolean bDriveInitialized = false;

    CBMecanumDrive() {
    }

    CBMecanumDrive(HardwareMap hardwareMap) {
        this.bDriveInitialized = initializeDrive(hardwareMap);
    }

    CBMecanumDrive(HardwareMap hardwareMap, boolean bEncodersOn) {
        this.bDriveInitialized = initializeDrive(hardwareMap, bEncodersOn);
    }

    public boolean initializeDrive(HardwareMap hardwareMap) {
        return this.bDriveInitialized = initializeDrive(hardwareMap, false);
    }

    public boolean initializeDrive(HardwareMap hardwareMap, boolean bEncodersOn) {

        //Initialize all Hardware configured for Mecanum Wheel
        leftFront   = hardwareMap.get(DcMotor.class,  "leftFront"  );
        rightFront  = hardwareMap.get(DcMotor.class,  "rightFront" );
        leftRear    = hardwareMap.get(DcMotor.class,  "leftRear"   );
        rightRear   = hardwareMap.get(DcMotor.class,  "rightRear"  );

        driveMotorList.add(leftFront);
        driveMotorList.add(rightFront);
        driveMotorList.add(leftRear);
        driveMotorList.add(rightRear);

        //Make sure all Hardware are initialized
        if(leftFront != null && rightFront != null && leftRear != null && rightRear != null) {
            bDriveInitialized = true;
        }

        //Set the encoders based on parameter passed
        encodeDriveMotors(bEncodersOn);
        //Start with Zero Power Behaviour
        applyDriveMotorsBrake();

        return bDriveInitialized;

    }

    public void encodeDriveMotors(boolean bEncodersOn) {
        if (bEncodersOn) {
            for (DcMotor motor : driveMotorList) {
                motor.setMode(RunMode.RUN_USING_ENCODER);
            }
        } else {
            for (DcMotor motor : driveMotorList) {
                motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public void applyDriveMotorsBrake() {
        for(DcMotor motor: driveMotorList) {
            motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }
    }

    public void stopDriveMotors() {
        for (DcMotor motor : driveMotorList) {
            motor.setPower(0);
        }
    }

    public void resetDriveMotorsEncoder() {
        for(DcMotor motor : driveMotorList) {
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        }
        for(DcMotor motor : driveMotorList) {
            motor.setMode(RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean getDriveInitializationStatus() {
        return bDriveInitialized;
    }

    public int[] getDriveCurrentEncoderValues() {
        int[] iCurrentEncArray = {
                  leftFront.getCurrentPosition()
                , rightFront.getCurrentPosition()
                , leftRear.getCurrentPosition()
                , rightRear.getCurrentPosition()
        };
        return iCurrentEncArray;
    }

    public int[] getDrivePreviousEncoderValues() {
        int[] iPreviousDeltaArray = {
                  leftFrontPreviousPosition
                , rightFrontPreviousPosition
                , leftRearPreviousPosition
                , rightRearPreviousPosition
        };
        return iPreviousDeltaArray;
    }

    public int[] getDriveEncoderDeltaValues() {
        int[] iEncDeltaArray = {
                  leftFront.getCurrentPosition() - leftFrontPreviousPosition
                , rightFront.getCurrentPosition() - rightFrontPreviousPosition
                , leftRear.getCurrentPosition() - leftRearPreviousPosition
                , rightRear.getCurrentPosition() - rightRearPreviousPosition
        };
        return iEncDeltaArray;
    }

    public void resetMotorEncoderValues() {
        setMotorEncoderValues();
    }

    public void setMotorEncoderValues() {
        leftFrontPreviousPosition  = leftFront.getCurrentPosition();
        rightFrontPreviousPosition = rightFront.getCurrentPosition();
        leftRearPreviousPosition   = leftRear.getCurrentPosition();
        rightRearPreviousPosition  = rightRear.getCurrentPosition();
    }

    public void setMotorEncoderPosition(int iPosition) {

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(iPosition);
        rightFront.setTargetPosition(iPosition);
        leftRear.setTargetPosition(iPosition);
        rightRear.setTargetPosition(iPosition);

        leftFront.setPower(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
        rightFront.setPower(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
        leftRear.setPower(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
        rightRear.setPower(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);

    }

}
