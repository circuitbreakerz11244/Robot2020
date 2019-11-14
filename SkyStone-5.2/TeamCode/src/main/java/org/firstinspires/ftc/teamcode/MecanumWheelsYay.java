package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.List;


@TeleOp(name = "Mecanum Wheels YAY lol")
public class MecanumWheelsYay extends OpMode {

    RobotHardwareCB robot = new RobotHardwareCB();

    @Override
    public void init() {
        robot.leftRear = hardwareMap.get(DcMotor.class, robot.lrName);
        robot.rightRear = hardwareMap.get(DcMotor.class, robot.rrName);
        robot.leftFront = hardwareMap.get(DcMotor.class, robot.lfName);
        robot.rightFront = hardwareMap.get(DcMotor.class, robot.rfName);


        robot.linearSlide = hardwareMap.get(DcMotor.class,robot.lsName);

        robot.arm = hardwareMap.get(DcMotor.class, robot.armName);
        robot.claw = hardwareMap.servo.get(robot.clawName);
        robot.foundation = hardwareMap.servo.get(robot.fName);
        robot.rightRear.setDirection(DcMotor.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);

        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Press Start");
        telemetry.update();
    }


    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rotate = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rotate;
        double v2 = r * Math.sin(robotAngle) - rotate;
        double v3 = r * Math.sin(robotAngle) + rotate;
        double v4 = r * Math.cos(robotAngle) - rotate;

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftRear.setPower(v3);
        robot.rightRear.setPower(v4);


  /*     double leftX = gamepad1.left_stick_x;
       double leftY = -gamepad1.left_stick_y; //Gamepad Moving up is giving -ve values
       double rightX = gamepad1.right_stick_x;
       double rightY = -gamepad1.right_stick_y; //Gamepad Moving up is giving -ve values

       double[] tempArray = AppUtil.getVectorArrays(leftX, leftY, rightX);
       double v1 = tempArray[0];
       double v2 = tempArray[1];
       double v3 = tempArray[2];
       double v4 = tempArray[3];

       //It is necessary to set one side of motor to REVERSE
       //Motor is facing other directions so reverse it.-- Kadhir
       v1 = -v1;
       v3 = -v3;

       telemetry.addData("Status", i + " x " + leftX + " y " + leftY + " z " + rightX + " v1>> " + v1);
       telemetry.addData("Status", i + ".v2>> " + v2);
       telemetry.addData("Status", i + ".v3>> " + v3);
       telemetry.addData("Status", i + ".v4>> " + v4);
       telemetry.update();


       leftFront.setPower(v1);
       rightFront.setPower(v2);
       leftRear.setPower(v3);
       rightRear.setPower(v4);
       i++;
*/

        robot.arm.setPower(-gamepad2.right_stick_y);
        robot.linearSlide.setPower(-gamepad2.left_stick_y);




//open and close claw
        if (gamepad2.a) {
            robot.claw.setPosition(robot.CLAW_OPEN);
        } else if (gamepad2.b) {
            robot.claw.setPosition(robot.CLAW_CLOSE);
        } else if (gamepad2.x) {
            robot.claw.setPosition(robot.CLAW_SHUT);
        }




//Foundation open and close
        if (gamepad2.left_bumper) {
            robot.foundation.setPosition(robot.FOUNDATION_DOWN);
        } else if (gamepad2.right_bumper) {
            robot.foundation.setPosition(robot.FOUNDATION_UP);
        }




        telemetry.addData(">", "left front power: " + v1);
        telemetry.addData(">", "right front power. " + v2);
        telemetry.addData(">", "left rear power. " + v3);
        telemetry.addData(">", "right rear power. " + v4);
        telemetry.update();

        telemetry.addData(">", "Press Stop to end program.");
        telemetry.update();
    }


}
















