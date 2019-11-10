package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "CBArmTestV1", group = "teleop")
public class CBTeleOp extends OpMode {

    RoboUtil util = null;
    double throttle = 0;
    double throttleIncr = 0.001;

    public void initialization() {

        String strVersion = "Nov 09 v1.8";
        util  = new RoboUtil("Manual", telemetry);
        util.robot.initializeRoboHW(hardwareMap);
        boolean isInit = util.robot.getRoboInitializationStatus();
        if(isInit) {
            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
            util.updateStatus(">>", " Press Start...");
        } else {
            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
        }
        util.robot.roboArmClaw.stopArmMotor();
        util.robot.roboArmClaw.resetArmMotorEncoder();

//        util.robot.roboArmClaw.stopClawMotor();
    }

    @Override
    public void init() {
        initialization();
    }

    @Override
    public void loop() {

        double leftY2 = -gamepad2.left_stick_y;
        util.updateStatus("Stick>>"+leftY2);

        leftY2 = Range.clip(leftY2, -0.8, 0.8);

        util.robot.roboArmClaw.armMotor.setPower(leftY2);

        if(gamepad2.dpad_up) {
            util.robot.roboArmClaw.clawOpen();
        } else if(gamepad2.dpad_down) {
            util.robot.roboArmClaw.clawClose();
        }

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


    }

}