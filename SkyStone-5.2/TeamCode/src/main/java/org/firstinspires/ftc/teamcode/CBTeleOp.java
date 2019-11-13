package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "CB TeleOp", group = "teleop")
public class CBTeleOp extends OpMode {

    RoboUtil util = null;

    public void initialization() {

        String strVersion = "Nov 12 v1.1";
        util  = new RoboUtil("Manual", telemetry);

        boolean IsDriveReady = util.robot.initializeDrive(hardwareMap, false);
        boolean IsArmReady = util.robot.initializeArmClaw(hardwareMap,true,true);

        if(IsDriveReady && IsArmReady) {
            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
            util.updateStatus(">>", " Press Start...");
        } else {
            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
        }

        util.robot.roboArmClaw.stopArmMotor();
        util.robot.roboArmClaw.resetArmMotorEncoder();

        util.robot.roboArmClaw.stopClawMotor();
        util.robot.roboArmClaw.resetClawMotorEncoder();

    }

    @Override
    public void init() {
        initialization();
    }

    @Override
    public void loop() {

        //Drive Code START
        double leftX  = gamepad1.left_stick_x;
        double leftY  = -gamepad1.left_stick_y;    //Gamepad Moving up is giving -ve value.So fix it by reversing it
        double rightX = gamepad1.right_stick_x;

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
        util.addStatus(" G1.Motor.x " + leftX + " y " + leftY + " z " + rightX);
        //set the power for the Mecanum wheel drive
        util.setPower(v1, v2, v3, v4);

        //Drive Code END

        //ARM Code START
        //ARM Move Up or Down Functionality
        double leftY2 = -gamepad2.left_stick_y;
        leftY2 = Range.clip(leftY2, -0.8, 0.8);
        util.robot.roboArmClaw.armMotor.setPower(leftY2);
        double armMotorPosition = util.robot.roboArmClaw.armMotor.getCurrentPosition();
        util.updateStatus(" G2.ArmMotor.Y>>" + leftY2 + " Pos " + armMotorPosition);

        //Use DPAD Up/Down for Open/Close Claw
        if(gamepad2.dpad_up) {
            util.robot.roboArmClaw.clawOpen();
        } else if(gamepad2.dpad_down) {
            util.robot.roboArmClaw.clawClose();
        }

        //Claw Up/Down Motor Moving up and Down
        double rightY2 = -gamepad2.right_stick_y;
        rightY2 = Range.clip(rightY2, -0.01, 0.01);
        util.robot.roboArmClaw.clawMotor.setPower(rightY2);
        double clawMotorPosition = util.robot.roboArmClaw.armMotor.getCurrentPosition();
        util.updateStatus(" G2.ClawMotor.X>>" + rightY2+ " Pos " + clawMotorPosition);

    }
}
