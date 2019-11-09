package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        robot.wheelyThing = hardwareMap.get(DcMotor.class, robot.wtName);
        robot.arm = hardwareMap.get(DcMotor.class, robot.armName);
        robot.claw = hardwareMap.servo.get(robot.clawName);

        robot.rightRear.setDirection(DcMotor.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Press Start");
        telemetry.update();

        robot.claw.setPosition(robot.CLAW_INIT);

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

        while(!(vcb.getY() >  y - 0.4 && vcb.getY() < y + 0.4)) {
            vcb.getPose();
            double error = vcb.getY() - y;
            strafe(-Math.signum(error), 0.2, 0);
        }

        double x = vcb.getX();
        encoderDrive(x, 0.5, 0);

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

    public void encoderDrive(double distance, double power, double heading) {

    }

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
