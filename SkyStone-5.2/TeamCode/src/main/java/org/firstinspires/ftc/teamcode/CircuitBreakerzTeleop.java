package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "CircuitBreakerzTeleop", group = "teleop")
//@Disabled

public class CircuitBreakerzTeleop extends OpMode {

    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor leftFront;
    DcMotor rightFront;

    @Override
    public void init() {
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Press Start");
        telemetry.update();
    }

    @Override
    public void loop() {

//        // Variables for joystick values
//        double leftFront = -gamepad1.left_stick_y;
//        double rightFront = -gamepad1.right_stick_y;
//        double rightRear = -gamepad1.right_stick_x;
//
//       /* leftFront.setPower(leftDrives);
//        rightFront.setPower(rightDrives);
//        leftRear.setPower(leftDrives);
//        rightRear.setPower(rightDrives);
//
//        Range.clip(leftDrives, -0.65, 0.65);
//        Range.clip(rightDrives, -0.65, 0.65);*/
//
//        telemetry.addData(">", "Press Stop to end program.");
//        telemetry.addData(">", "right motor position: " + rightRear + rightFront.getCurrentPosition());
//        telemetry.addData(">", "left motor position: " + leftRear + leftFront.getCurrentPosition());
//
}}
