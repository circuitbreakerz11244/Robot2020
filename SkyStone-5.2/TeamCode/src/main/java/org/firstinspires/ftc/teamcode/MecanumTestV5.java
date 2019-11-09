package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MecanumTestV5 extends OpMode {

    RoboUtil util = null;

    @Override
    public void init() {

//        String strVersion = "Nov 03 v1.1";
//        //Initialize all Hardware in the Robot
//        util  = new RoboUtil("Manual", hardwareMap, telemetry);
//        if(util.robot.getHWInitializationStatus()) {
//            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
//            util.updateStatus(">>", " Press Start...");
//        } else {
//            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
//        }
    }

    @Override
    public void loop() {

        boolean bJoyStickMode = true;

        if(bJoyStickMode) {
            double leftX  = gamepad1.left_stick_x;
            double leftY  = -gamepad1.left_stick_y;    //Gamepad Moving up is giving -ve value.So fix it by reversing it
            double rightX = gamepad1.right_stick_x;
            //double rightY = -gamepad1.right_stick_y; //Gamepad Moving up is giving -ve value.So fix it by reversing it

            //Get the desired power settings based on given x, y and z (z is rotation --> rightX used for rotation)
            //Call the reusable method to get PowerVector
            double[] powerVectorArray = AppUtil.getVectorArrays(leftX, leftY, rightX);
            double v1 = powerVectorArray[0];
            double v2 = powerVectorArray[1];
            double v3 = powerVectorArray[2];
            double v4 = powerVectorArray[3];

            //It is necessary to set one side of motor to REVERSE
            //Motor is facing other directions so reverse it.-- Kadhir
            v1 = -v1;
            v3 = -v3;

            //Display the values in the Driver Station for Tank Drive
            util.addStatus(" x " + leftX + " y " + leftY + " z " + rightX);

            //set the power for the Mecanum wheel drive
            util.setPower(v1, v2, v3, v4);

            if(gamepad1.back) {
                bJoyStickMode = false;
                util.addStatus("Toggling Mode1 >> " + bJoyStickMode);
            }

        } else {

            //Instead of using joysticks we can use dpad also for forward, backward, right and left drives
            //DPAD press settings for movement of the Robot
            if (gamepad1.dpad_up) {
                util.moveForward();
            }
            if (gamepad1.dpad_down) {
                util.moveBackward();
            }
            if (gamepad1.dpad_right) {
                util.moveRight();
            }
            if (gamepad1.dpad_left) {
                util.moveLeft();
            }

            if (gamepad1.back) {
                bJoyStickMode = true;
                util.addStatus("Toggling Mode2 >> " + bJoyStickMode);
            }
        }

        if(gamepad1.x) {
            util.clawOpen();
        } else if(gamepad1.y) {
            util.clawClose();
        }

        //TBD Oct 17
        //Handle Arm movements..Expand, Squeeze
        //Handle Pickup and drop objects by claw
        //Autonomous Driving
        //Sensor read and act
        //Gyro ?


    }
}
