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

    public void vuforiaNavigate(int x, int y) {
        vcb.targetsSkyStone.activate();
        vcb.getPose();

        while(vcb.y != y) {
            vcb.getPose();
            double error = vcb.y - y;
            strafe(-Math.signum(error), 0.2);
        }

        while(vcb.x != x) {
            vcb.getPose();
            double error = vcb.x - x;
            drive(0.4 * Math.signum(error));
        }

    }

    // -1 for left and +1 for right
    public void strafe(double direction, double power) {
        robot.leftFront.setPower(power * direction);
        robot.leftRear.setPower(-power * direction);
        robot.rightFront.setPower(-power * direction);
        robot.rightRear.setPower(power * direction);


    }

    public void drive(double power) {

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        double gyroCorrectionPwr = GYRO_CORRECTION_FACTOR * computeGyroDriveCorrectionError(targetHeading, currentHeading);

        robot.leftFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightRear.setPower(power);
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


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
