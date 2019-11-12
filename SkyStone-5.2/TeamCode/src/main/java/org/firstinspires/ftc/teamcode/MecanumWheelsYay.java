
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
        robot.arm = hardwareMap.get(DcMotor.class, robot.armName);
        robot.claw = hardwareMap.servo.get(robot.clawName);
       

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

        robot.arm.setPower(-gamepad2.right_stick_y);

        if (gamepad2.a) {
            robot.claw.setPosition(robot.CLAW_OPEN);
        } else if (gamepad2.b) {
            robot.claw.setPosition(robot.CLAW_CLOSE);
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
















