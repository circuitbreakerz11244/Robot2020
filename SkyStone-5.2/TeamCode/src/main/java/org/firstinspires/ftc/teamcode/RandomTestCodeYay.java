package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "Random TestCode Yay", group = "teleop")
//@Disabled
        public class RandomTestCodeYay extends OpMode {


        DcMotor leftRear;
                DcMotor rightRear;

                @Override
                public void init() {
                        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
                        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

                        telemetry.addData(">", "Press Start");
                        telemetry.update();
                }

                @Override
                public void loop() {

                        // Variables for joystick values
                        double leftDrives = -gamepad1.left_stick_y;
                        double rightDrives = -gamepad1.right_stick_y;

                        Range.clip(leftDrives, -0.65, 0.65);
                        Range.clip(rightDrives, -0.65, 0.65);

                        leftRear.setPower(leftDrives * leftDrives * leftDrives);
                        rightRear.setPower(rightDrives * rightDrives * rightDrives);

                        telemetry.addData(">", "Press Stop to end program.");
                        telemetry.addData(">", "right motor position: " + rightRear.getCurrentPosition());
                        telemetry.addData(">", "left motor position: " + leftRear.getCurrentPosition());

        }

}



