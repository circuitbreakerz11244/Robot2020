package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CBDriveTest extends LinearOpMode {

    RoboUtil util = null;

    public void initialization() {

        String strVersion = "Nov 07 v1.1";
        util  = new RoboUtil("Manual", telemetry);
        util.robot.initializeDrive(hardwareMap,true);
        boolean bHWInitialized = util.robot.getRoboInitializationStatus();
        if(util.robot.drive.getDriveInitializationStatus()) {
            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
            util.updateStatus(">>", " Press Start...");
        } else {
            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
        }
        util.robot.drive.resetDriveMotorsEncoder();
    }

    public void sleepInSecs(int iSeconds) {
        sleep(iSeconds * 1000);
    }

    @Override
    public void runOpMode() {

        initialization();
        waitForStart();


        util.moveForward();
        sleepInSecs(1);
        util.robot.drive.applyDriveMotorsBrake();
        util.robot.drive.stopDriveMotors();
        util.displayDriveEncoderValues();
        util.robot.drive.resetMotorEncoderValues();

        sleepInSecs(10);


        util.moveLeft();
        sleepInSecs(1);
        util.robot.drive.applyDriveMotorsBrake();
        util.robot.drive.stopDriveMotors();
        util.displayDriveEncoderValues();
        util.robot.drive.resetMotorEncoderValues();


        sleepInSecs(10);

        util.moveRight();
        sleepInSecs(1);
        sleep(1000);
        util.robot.drive.applyDriveMotorsBrake();
        util.robot.drive.stopDriveMotors();
        util.displayDriveEncoderValues();
        util.robot.drive.resetMotorEncoderValues();

        sleepInSecs(10);

        util.moveBackward();
        sleepInSecs(1);
        util.robot.drive.applyDriveMotorsBrake();
        util.robot.drive.stopDriveMotors();
        util.displayDriveEncoderValues();
        util.robot.drive.resetMotorEncoderValues();

    }
}
