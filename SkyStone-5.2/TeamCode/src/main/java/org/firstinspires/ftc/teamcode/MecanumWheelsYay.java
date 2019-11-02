package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
        robot.wheelyThing = hardwareMap.get(DcMotor.class, robot.wtName);
        robot.arm = hardwareMap.get(DcMotor.class, robot.armName);
        robot.claw = hardwareMap.servo.get(robot.clawName);

        robot.rightRear.setDirection(DcMotor.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Press Start");
        telemetry.update();
    }


    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rotate = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rotate;
        final double v2 = r * Math.sin(robotAngle) - rotate;
        final double v3 = r * Math.sin(robotAngle) + rotate;
        final double v4 = r * Math.cos(robotAngle) - rotate;

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

        robot.arm.setPower(-gamepad2.right_stick_x);


        if (gamepad2.a) {
            robot.claw.setPosition(robot.CLAW_OPEN);
        } else if (gamepad2.b) {
            robot.claw.setPosition(robot.CLAW_CLOSE);
        }

        if (gamepad2.right_trigger > 0.1) {
            robot.wheelyThing.setPower(robot.INTAKE_POWER);
        } else if (gamepad2.left_trigger > 0.1) {
            robot.wheelyThing.setPower(-robot.INTAKE_POWER);

        } else {
            robot.wheelyThing.setPower(0.0);
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






