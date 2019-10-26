package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "Mecanum Wheels YAY aka test")
public class MecanumWheelsYay extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor wheelyRight;
    DcMotor wheelyLeft;
    DcMotor arm;

    Servo claw;

    int i =0;

    final double CLAW_OPEN = 0.3;
    final double CLAW_CLOSE = 0.55;



    @Override
        public void init() {
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        wheelyLeft = hardwareMap.get(DcMotor.class,"wheelyLeft");
        wheelyRight = hardwareMap.get(DcMotor.class,"wheelyRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw  = hardwareMap.servo.get("leftClaw");

        telemetry.addData(">", "Press Start");
        telemetry.update();
    }


    @Override
    public void loop() {
        /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rotate = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rotate;
        final double v2 = r * Math.sin(robotAngle) - rotate;
        final double v3 = r * Math.sin(robotAngle) + rotate;
        final double v4 = r * Math.cos(robotAngle) - rotate;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);*/

        double leftX  =  gamepad1.left_stick_x;
        double leftY  = -gamepad1.left_stick_y; //Gamepad Moving up is giving -ve values
        double rightX =  gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; //Gamepad Moving up is giving -ve values

        double[] tempArray = AppUtil.getVectorArrays(leftX, leftY, rightX);
        double v1 = tempArray[0];
        double v2 = tempArray[1];
        double v3 = tempArray[2];
        double v4 = tempArray[3];

        //It is necessary to set one side of motor to REVERSE
        //Motor is facing other directions so reverse it.-- Kadhir
        v1=-v1;
        v3=-v3;

        telemetry.addData("Status", i + " x "+ leftX + " y " + leftY + " z " + rightX + " v1>> " + v1);
        telemetry.addData("Status", i + ".v2>> " + v2);
        telemetry.addData("Status", i + ".v3>> " + v3);
        telemetry.addData("Status", i + ".v4>> " + v4);
        telemetry.update();



        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
        i++;

        double wheelyThing= -gamepad2.right_trigger;
//        double arm =

        wheelyRight.setPower(wheelyThing);
        wheelyLeft.setPower(wheelyThing);


        if (gamepad2.a){
            claw.setPosition(CLAW_OPEN);
        } else if (gamepad2.b){
            claw.setPosition(CLAW_CLOSE);
        }

//        if(gamepad2.right_trigger){




        telemetry.addData(">", "left front power: " + v1);
        telemetry.addData(">","right front power. " + v2);
        telemetry.addData(">", "left rear power. " + v3);
        telemetry.addData(">", "right rear power. " + v4);
        telemetry.update();

        telemetry.addData(">", "Press Stop to end program." );
        telemetry.update();
    }



}




