package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class CBAutonomousBase extends LinearOpMode {

    RoboUtil util = null;
    VuforiaCB vcb = new VuforiaCB();
    ColorSensor colorSensor;

    Orientation angles;

    final double GYRO_CORRECTION_FACTOR = 0.02;

    // test to fine tune value
    final double COLOR_THRESH = 70.0;

    public void initialization() {

        vcb.initVuforia();

        String strVersion = "Nov 08 v1.6";
        util  = new RoboUtil("Manual", telemetry);
        util.robot.initializeDrive(hardwareMap,true);
        util.robot.initializeIMU(hardwareMap);
        boolean bHWInitialized = util.robot.getRoboInitializationStatus();
        if(util.robot.drive.getDriveInitializationStatus() && util.robot.imu.isCalibrated()) {
            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
            util.updateStatus(">>", " Press Start...");
        } else {
            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
        }

        colorSensor = hardwareMap.colorSensor.get("color");

        util.robot.drive.resetDriveMotorsEncoder();
    }

    @Override
    public void runOpMode() {
    }

    public void encoderDrive(String strDirection, double distance, double maxPwr, double targetHeading) {
        encoderDrive(strDirection, distance, maxPwr, targetHeading, false, false);
    }

    public void encoderDrive(String strDirection, double distance, double maxPwr, double targetHeading, boolean checkStone) {
        encoderDrive(strDirection, distance, maxPwr, targetHeading, checkStone, false);
    }

    public void encoderDrive(String strDirection, double distance, double maxPwr, double targetHeading, boolean checkStone, boolean checkColor) {

        angles = util.robot.imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        util.robot.drive.resetDriveMotorsEncoder();
        if(strDirection.equalsIgnoreCase("MF") || strDirection.equalsIgnoreCase("MB")) {
            distance = distance/2;
        }

        double initPosLF = util.robot.drive.leftFront.getCurrentPosition();
        double initPosRF = util.robot.drive.rightFront.getCurrentPosition();
        double initPosLR = util.robot.drive.leftRear.getCurrentPosition();
        double initPosRR = util.robot.drive.rightRear.getCurrentPosition();

        double targetPos = distance * CBRoboConstants.COUNTS_PER_INCH;

        double currentRobotPos = 0.;

        double power = maxPwr;

        // slopes for proportional speed increase/decrease
        double decSlope = (maxPwr - CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR) / (CBRoboConstants.DRIVE_DECELERATION_THRESHOLD);

        while (Math.abs(currentRobotPos) < Math.abs(targetPos)) {

            angles = util.robot.imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double currentHeading = angles.firstAngle;

            double gyroCorrectionPwr = GYRO_CORRECTION_FACTOR * computeGyroDriveCorrectionError(targetHeading, currentHeading);

            gyroCorrectionPwr = 0; // gyro correction needs fixing

            double curPosLF = util.robot.drive.leftFront.getCurrentPosition() - initPosLF;
            double curPosRF = util.robot.drive.rightFront.getCurrentPosition() - initPosRF;
            double curPosLR = util.robot.drive.rightFront.getCurrentPosition() - initPosLR;
            double curPosRR = util.robot.drive.rightFront.getCurrentPosition() - initPosRR;

            currentRobotPos = (curPosLF + curPosRF + curPosLR + curPosRR) / 4;

            // calculating points on trapezoidal profile graph
            power = maxPwr - decSlope * (Math.abs(currentRobotPos) / CBRoboConstants.COUNTS_PER_INCH);

            if (power < CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR)
                power = CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR;

            double rightPwr = power + gyroCorrectionPwr;
            double leftPwr = power - gyroCorrectionPwr;

            if(strDirection.equalsIgnoreCase("MF")) {
                util.setPower(-leftPwr, rightPwr, -leftPwr, rightPwr);
            } else if(strDirection.equalsIgnoreCase("MB")) {
                util.setPower(leftPwr, -rightPwr, leftPwr, -rightPwr);
            } else if(strDirection.equalsIgnoreCase("ML")) {
                util.setPower(leftPwr, rightPwr, -leftPwr, -rightPwr);
            } else if(strDirection.equalsIgnoreCase("MR")) {
                util.setPower(-leftPwr, -rightPwr, leftPwr, rightPwr);
            }

            if (vcb.targetVisible() && checkStone) {
                break;
            }

//             test to see whether to check red, blue or green
            if(checkColor && colorSensor.red() > COLOR_THRESH){
             break;
            }


            util.addStatus(">", " target position = " + targetPos);
            util.addStatus(">", " current robot pos = " + currentRobotPos);
            util.addStatus(">", " Direction = " + strDirection);
            util.updateStatus(">", " power = " + power);

        }
        util.robot.drive.stopDriveMotors();
    }

//    public void encoderDrive(String strDirection, double distance, double maxPwr, boolean checkStone) {
//
//        //robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        //robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        util.robot.drive.resetDriveMotorsEncoder();
//        if(strDirection.equalsIgnoreCase("MF") || strDirection.equalsIgnoreCase("MB")) {
//            distance = distance/2;
//        }
//
//        double initPosLF = util.robot.drive.leftFront.getCurrentPosition();
//        double initPosRF = util.robot.drive.rightFront.getCurrentPosition();
//        double initPosLR = util.robot.drive.leftRear.getCurrentPosition();
//        double initPosRR = util.robot.drive.rightRear.getCurrentPosition();
//
//        double targetPos    = distance * CBRoboConstants.COUNTS_PER_INCH;
//
//        double currentRobotPos = 0.;
//
//        double power = maxPwr;
//
//        // slopes for proportional speed increase/decrease
//        double decSlope = (maxPwr - CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR) / (CBRoboConstants.DRIVE_DECELERATION_THRESHOLD);
//
//        while (Math.abs(currentRobotPos) < Math.abs(targetPos)){
//
//            double curPosLF = util.robot.drive.leftFront.getCurrentPosition() - initPosLF;
//            double curPosRF = util.robot.drive.rightFront.getCurrentPosition() - initPosRF;
//            double curPosLR = util.robot.drive.rightFront.getCurrentPosition() - initPosLR;
//            double curPosRR = util.robot.drive.rightFront.getCurrentPosition() - initPosRR;
//
//            currentRobotPos = (curPosLF + curPosRF + curPosLR + curPosRR) / 4;
//
//            // calculating points on trapezoidal profile graph
//            power = maxPwr - decSlope * (Math.abs(currentRobotPos) / CBRoboConstants.COUNTS_PER_INCH);
//
//            if (power < CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR)
//                power = CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR;
//
//            if(strDirection.equalsIgnoreCase("MF")) {
//                util.moveForward(power);
//            } else if(strDirection.equalsIgnoreCase("MB")) {
//                util.moveBackward(power);
//            } else if(strDirection.equalsIgnoreCase("ML")) {
//                util.moveLeft(power);
//            } else if(strDirection.equalsIgnoreCase("MR")) {
//                util.moveRight(power);
//            }
//
//            if (vcb.targetVisible() && checkStone) {
//                break;
//            }
//
//
//            util.addStatus(">", " target position = " + targetPos);
//            util.addStatus(">", " current robot pos = " + currentRobotPos);
//            util.addStatus(">", " Direction = " + strDirection);
//            util.updateStatus(">", " power = " + power);
//
//        }
//
//        util.robot.drive.stopDriveMotors();
//
//    }



    public void vuforiaNavigate(double y, String alliance) {
        vcb.getPose();

        String direction = "";

        if (alliance.equalsIgnoreCase("red"))
            direction = "ML";
        else
            direction = "MR";

        encoderDrive(direction, Math.abs(vcb.getY()), 0.3, 0);

        double x = vcb.getX();
        encoderDrive("MB", x, 0.4, 0);

        sleep(500);

        encoderDrive("MF", x, 0.4, 0);

    }

    double computeGyroDriveCorrectionError(double inputHeading, double currentHeading) {

        double error;
        if (inputHeading * currentHeading >= 0)
            error = currentHeading - inputHeading;
        else {
            if (Math.abs(inputHeading) > 90) {
                if (inputHeading < 0)
                    error = -((180 - currentHeading) + (180 + inputHeading));
                else
                    error = (180 + currentHeading) + (180 - inputHeading);
            } else
                error = currentHeading - inputHeading;
        }


        return error;
    }
}
