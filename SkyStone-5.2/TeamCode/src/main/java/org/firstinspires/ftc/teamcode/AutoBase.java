package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.RobotHardwareCB;
import org.firstinspires.ftc.teamcode.VuforiaCB;

public class AutoBase extends LinearOpMode {

    VuforiaCB vcb = new VuforiaCB();
    RobotHardwareCB robot = new RobotHardwareCB();

    Orientation angles;

    final double GYRO_CORRECTION_FACTOR = 0.02;

    public void initialize(){
        robot.leftRear = hardwareMap.get(DcMotor.class, robot.lrName);
        robot.rightRear = hardwareMap.get(DcMotor.class, robot.rrName);
        robot.leftFront = hardwareMap.get(DcMotor.class, robot.lfName);
        robot.rightFront = hardwareMap.get(DcMotor.class, robot.rfName);
//        robot.wheelyThing = hardwareMap.get(DcMotor.class, robot.wtName);
//        robot.arm = hardwareMap.get(DcMotor.class, robot.armName);
//        robot.claw = hardwareMap.servo.get(robot.clawName);

        robot.leftRear.setDirection(DcMotor.Direction.REVERSE);
        robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
        robot.rightRear.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);

        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData(">", "Press Start");
        telemetry.update();

//        robot.claw.setPosition(robot.CLAW_INIT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);

        while (!robot.imu.isGyroCalibrated()) {
            telemetry.addData("gyro not calibrated", "do not touch!");
            telemetry.update();
        }

        vcb.initVuforia();

        telemetry.addData(">", "robot initialized");
        telemetry.update();

    }

    public void vuforiaNavigate(double y) {
        vcb.getPose();

//        while(!(vcb.getY() >  y - 0.4 && vcb.getY() < y + 0.4)) {
//            vcb.getPose();
//            double error = vcb.getY() - y;
//            strafe(-Math.signum(error), 0.18, 0);
//        }

        double x = vcb.getX();
//        encoderDrive(-x, 0.4, 0);

    }

    // -1 for left and +1 for right
    public void strafe(double direction, double power, double targetHeading) {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        double gyroCorrectionPwr = GYRO_CORRECTION_FACTOR * computeGyroDriveCorrectionError(targetHeading, currentHeading);

        // adjusting powers of motors going in the same direction
//        robot.leftFront.setPower((power + gyroCorrectionPwr) * direction);
//        robot.leftRear.setPower(-(power - gyroCorrectionPwr) * direction);
//        robot.rightFront.setPower(-(power - gyroCorrectionPwr) * direction);
//        robot.rightRear.setPower((power + gyroCorrectionPwr) * direction);

        gyroCorrectionPwr = 0;
        //adjusting power of motors on the same side
        robot.leftFront.setPower((power + gyroCorrectionPwr) * direction);
        robot.leftRear.setPower(-(power + gyroCorrectionPwr) * direction);
        robot.rightFront.setPower(-(power - gyroCorrectionPwr) * direction);
        robot.rightRear.setPower((power - gyroCorrectionPwr) * direction);

    }

    public void drive(double power, double targetHeading) {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        double gyroCorrectionPwr = GYRO_CORRECTION_FACTOR * computeGyroDriveCorrectionError(targetHeading, currentHeading);

        gyroCorrectionPwr = 0;
        robot.leftFront.setPower(power + gyroCorrectionPwr);
        robot.leftRear.setPower(power + gyroCorrectionPwr);
        robot.rightFront.setPower(power - gyroCorrectionPwr);
        robot.rightRear.setPower(power - gyroCorrectionPwr);
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
//
//    public void encoderDrive(double distance, double maxPwr, double targetHeading) {
//
//        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        distance = distance/2;
//
//        double initPosLF = robot.leftFront.getCurrentPosition();
//        double initPosRF = robot.rightFront.getCurrentPosition();
//        double initPosLR = robot.leftRear.getCurrentPosition();
//        double initPosRR = robot.rightRear.getCurrentPosition();
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
//            double curPosLF = robot.leftFront.getCurrentPosition() - initPosLF;
//            double curPosRF = robot.rightFront.getCurrentPosition() - initPosRF;
//            double curPosLR = robot.rightFront.getCurrentPosition() - initPosLR;
//            double curPosRR = robot.rightFront.getCurrentPosition() - initPosRR;
//
//            currentRobotPos = (curPosLF + curPosRF + curPosLR + curPosRR) / 4;
//
//            // calculating points on trapezoidal profile graph
//            power = maxPwr - decSlope * (Math.abs(currentRobotPos) / CBRoboConstants.COUNTS_PER_INCH);
//
//            if (power < CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR)
//                power = CBRoboConstants.DRIVE_MINIMUM_DRIVE_PWR;
//
//            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            double currentHeading = angles.firstAngle;
//
//            double gyroCorrectionPwr = GYRO_CORRECTION_FACTOR * computeGyroDriveCorrectionError(targetHeading, currentHeading);
//
//            gyroCorrectionPwr = 0;
//            robot.leftFront.setPower((power + gyroCorrectionPwr) * Math.signum(distance));
//            robot.leftRear.setPower((power + gyroCorrectionPwr) * Math.signum(distance));
//            robot.rightFront.setPower((power - gyroCorrectionPwr) * Math.signum(distance));
//            robot.rightRear.setPower((power - gyroCorrectionPwr) * Math.signum(distance));
//
//            telemetry.addData(">", " target position = " + targetPos);
//            telemetry.addData(">", " current robot pos = " + currentRobotPos);
//            telemetry.addData(">", " power = " + power);
//
//        }
//
//        setMotorPowers(0.0);
//    }

    public void grabStone() {

    }

    public void setMotorPowers(double power) {
        robot.rightFront.setPower(power);
        robot.rightRear.setPower(power);
        robot.leftFront.setPower(power);
        robot.leftRear.setPower(power);
    }



    @Override
    public void runOpMode() throws InterruptedException {

    }
}
