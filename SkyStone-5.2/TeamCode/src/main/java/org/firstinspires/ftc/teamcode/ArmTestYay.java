package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AppUtil;

import java.util.List;
//This is for when you only have one hub and need to test the arm code
//QUestion for later: what code is needed to identify setpower?

@TeleOp(name = "Arm Code ONLY YAY v2")
public class ArmTestYay extends OpMode {


    DcMotor wheelyThing;
    DcMotor arm;

    Servo claw;


    final double CLAW_OPEN = 0.3;
    final double CLAW_CLOSE = 0.55;
    final double intake = .7;
    final double stop = 0;





    @Override
    public void init() {

        wheelyThing = hardwareMap.get(DcMotor.class,"wheelyThing");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw  = hardwareMap.servo.get("claw");



        wheelyThing.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Press Start");
        telemetry.update();
    }


    @Override
    public void loop() {




        arm.setPower(gamepad2.left_stick_y);






        if (gamepad2.a){
            claw.setPosition(CLAW_OPEN);
        } else if (gamepad2.b){
            claw.setPosition(CLAW_CLOSE);
        }




        if (gamepad2.right_trigger >0.1){
            wheelyThing.setPower(intake);


        } else if (gamepad2.left_trigger>0.1){
            wheelyThing.setPower(-intake);


        }else if (gamepad2.left_bumper){
           wheelyThing.setPower(stop);

        }else if (gamepad2.right_bumper){
            wheelyThing.setPower(stop);
        }






        telemetry.addData(">", "Press Stop to end program." );
        telemetry.update();
    }



}




