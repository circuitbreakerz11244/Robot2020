package org.firstinspires.ftc.teamcode.kavitha;

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

    CBRoboUtil util = null;
    private double vuforiaX=0;
    private double vuforiaY=0;
    Orientation angles;
    final double GYRO_CORRECTION_FACTOR = 0.02;
    ColorSensor color_sensor;


    public void initialize(){
        /**** jimi methods***start****/
//        robot.leftRear = hardwareMap.get(DcMotor.class, robot.lrName);
//        robot.rightRear = hardwareMap.get(DcMotor.class, robot.rrName);
//        robot.leftFront = hardwareMap.get(DcMotor.class, robot.lfName);
//        robot.rightFront = hardwareMap.get(DcMotor.class, robot.rfName);
//        robot.wheelyThing = hardwareMap.get(DcMotor.class, robot.wtName);
//        robot.arm = hardwareMap.get(DcMotor.class, robot.armName);
//        robot.claw = hardwareMap.servo.get(robot.clawName);
//
//        robot.rightRear.setDirection(DcMotor.Direction.REVERSE);
//        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);
        //        robot.claw.setPosition(robot.CLAW_INIT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);

        while (!robot.imu.isGyroCalibrated()) {
            telemetry.addData("gyro not calibrated", "do not touch!");
            telemetry.update();
        }
        /**** jimi methods***end****/
        String strVersion = "Nov 07 v1.1";
        color_sensor = hardwareMap.colorSensor.get("color");

        //Initialize all Hardware in the Robot
        util  = new CBRoboUtil("Auto", hardwareMap, telemetry);
        if(util.robot.getHWInitializationStatus()) {
            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
            util.updateStatus(">>", " Press Start...");
        } else {
            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
        }

        vcb.initVuforia();

        telemetry.addData(">", "robot initialized");
        telemetry.update();

    }

    /**** kavi methods***start****/
    public double getVuforiaX() {
        return vuforiaX;
    }

    public void setVuforiaX(double vuforiaX) {
        this.vuforiaX = vuforiaX;
    }

    public double getVuforiaY() {
        return vuforiaY;
    }

    public void setVuforiaY(double vuforiaY) {
        this.vuforiaY = vuforiaY;
    }
    public Boolean isSkystoneVisible(){
        vcb.getPose();
        setVuforiaX(vcb.x);
        setVuforiaX(vcb.y);
        return vcb.targetVisible;
    }

    // method to move robot forward/backward

    public void moveStraight(double direction, double power, double distance) {
        if(direction==0) {
            util.moveForward(power, distance);
        }else if(direction==1){
            util.moveBackward(power, distance);
        }
    }
    // method to move robot right/ left
    public void moveSideways(double direction, double power, double distance) {

        if(direction==0) {
            util.moveRight(power, distance);
        }else if(direction==1){
            util.moveLeft(power, distance);
        }

    }
    /**** kavi methods***end****/

    /**** jimi methods***start****/
    public void vuforiaNavigate(int x, int y) {
        //       vcb.targetsSkyStone.activate();
        vcb.getPose();

        while(vcb.y != y) {
            vcb.getPose();
            double error = vcb.y - y;
            strafe(-Math.signum(error), 0.2, 0);
        }

        while(vcb.x != x) {
            vcb.getPose();
            double error = vcb.x - x;
            drive(0.4 * Math.signum(error), 0);
        }

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


 //   @Override
 //   public void runOpMode() throws InterruptedException {

//    }
    /**** jimi methods***start****/
}
