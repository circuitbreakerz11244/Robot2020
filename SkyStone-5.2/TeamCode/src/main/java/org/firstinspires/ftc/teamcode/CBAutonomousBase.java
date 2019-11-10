package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class CBAutonomousBase extends LinearOpMode {

    RoboUtil util = null;
    VuforiaCB vcb = new VuforiaCB();
    ColorSensor colorSensor;

    // test to fine tune value
    final double COLOR_THRESH = 20.0;

    public void initialization() {

        vcb.initVuforia();

        String strVersion = "Nov 08 v1.6";
        util  = new RoboUtil("Manual", telemetry);
        util.robot.initializeDrive(hardwareMap,true);
        boolean bHWInitialized = util.robot.getRoboInitializationStatus();
        if(util.robot.drive.getDriveInitializationStatus()) {
            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
            util.updateStatus(">>", " Press Start...");
        } else {
            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
        }

//        colorSensor = hardwareMap.colorSensor.get("color");

        util.robot.drive.resetDriveMotorsEncoder();
    }

    @Override
    public void runOpMode() {
    }

    public void encoderDrive(String strDirection, double distance, double maxPwr) {
        encoderDrive(strDirection, distance, maxPwr, false, false);
    }

    public void encoderDrive(String strDirection, double distance, double maxPwr, boolean checkStone) {
        encoderDrive(strDirection, distance, maxPwr, checkStone, false);
    }

    public void encoderDrive(String strDirection, double distance, double maxPwr, boolean checkStone, boolean checkColor) {



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

            //if(util.bForceStop) {
            //  break;
            //}

            double curPosLF = util.robot.drive.leftFront.getCurrentPosition() - initPosLF;
            double curPosRF = util.robot.drive.rightFront.getCurrentPosition() - initPosRF;
            double curPosLR = util.robot.drive.rightFront.getCurrentPosition() - initPosLR;
            double curPosRR = util.robot.drive.rightFront.getCurrentPosition() - initPosRR;

            currentRobotPos = (curPosLF + curPosRF + curPosLR + curPosRR) / 4;

            // calculating points on trapezoidal profile graph
            power = maxPwr - decSlope * (Math.abs(currentRobotPos) / CBRoboConstants.COUNTS_PER_INCH);

            if (power < CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR)
                power = CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR;

            if(strDirection.equalsIgnoreCase("MF")) {
                util.moveForward(power);
            } else if(strDirection.equalsIgnoreCase("MB")) {
                util.moveBackward(power);
            } else if(strDirection.equalsIgnoreCase("ML")) {
                util.moveLeft(power);
            } else if(strDirection.equalsIgnoreCase("MR")) {
                util.moveRight(power);
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
            direction = "MR";
        else
            direction = "ML";

        encoderDrive(direction, Math.abs(vcb.getY()), 0.3);

        double x = vcb.getX();
        encoderDrive("MB", -x, 0.4, false);

    }
}
