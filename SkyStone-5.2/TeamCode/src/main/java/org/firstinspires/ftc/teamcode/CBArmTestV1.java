package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "CBArmTestV1", group = "teleop")
public class CBArmTestV1 extends OpMode {

    RoboUtil util = null;
    double throttle = 0;
    double throttleIncr = 0.001;

    public void initialization() {

        String strVersion = "Nov 09 v1.8";
        util  = new RoboUtil("Manual", telemetry);
        util.robot.initializeArmClaw(hardwareMap,true);
        boolean bHWInitialized = util.robot.getRoboInitializationStatus();
        if(util.robot.roboArmClaw.getArmInitializationStatus()) {
            util.addStatus("Initialized Circuit Breakerz. Ver " + strVersion);
            util.updateStatus(">>", " Press Start...");
        } else {
            util.updateStatus(">>", "Not All Hardware are Initialized. Ver " + strVersion);
        }
        util.robot.roboArmClaw.stopArmMotor();
        util.robot.roboArmClaw.resetArmMotorEncoder();

        util.robot.roboArmClaw.stopClawMotor();
    }

    @Override
    public void init() {
        initialization();
    }

    @Override
    public void loop() {

        double leftY = -gamepad2.left_stick_y;
        util.updateStatus("Stick>>"+leftY);
        util.robot.roboArmClaw.armMotor.setPower(leftY);

        double righBumper = gamepad2.right_trigger;
        // if the left trigger is pressed, go in reverse
        if (gamepad2.left_trigger != 0.0) {
            throttle = throttle + throttleIncr;
        }
        // if the right trigger is pressed, go in reverse
        if (gamepad2.right_trigger != 0.0) {
            throttle = throttle - throttleIncr;
        }
        util.updateStatus("throttle>>"+throttle);
        throttle = Range.clip(throttle,-0.1,0.1);
        //TEST AND UNCOMMENT
        //util.robot.roboArmClaw.clawMotor.setPower(throttle);

        if(gamepad2.x) {
            util.robot.roboArmClaw.clawMotor.setPower(0);
            util.robot.roboArmClaw.clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        if(gamepad2.dpad_up) {
            util.robot.roboArmClaw.clawOpen();
        }

        if(gamepad2.dpad_down) {
            util.robot.roboArmClaw.clawClose();
        }

/*
        if (util.robot.roboArmClaw.arm.getCurrentPosition() <= CBRoboConstants.ARM_MAX_EXTENSION &&
                util.robot.roboArmClaw.arm.getCurrentPosition() >= CBRoboConstants.ARM_MIN_EXTENSION) {
            if (gamepad2.dpad_up)
                extensionMotor.setPower(EXTENSION_POWER);
            else if (gamepad2.dpad_down)
                extensionMotor.setPower(-EXTENSION_POWER);
            else
                extensionMotor.setPower(0.0);
        } else
            extensionMotor.setPower(0.0);
        if (gamepad2.dpad_up)
            extensionMotor.setPower(EXTENSION_POWER);
        else if (gamepad2.dpad_down)
            extensionMotor.setPower(-EXTENSION_POWER);
        else
            extensionMotor.setPower(0.0);
        // Telemetry output for driver assistance
        telemetry.addData(">", "Press Stop to end program." );
        telemetry.addData(">", "linear slide encoder value: " + extensionMotor.getCurrentPosition());
        telemetry.addData(">", "rotation encoder value: " + rotationMotor.getCurrentPosition());
        telemetry.addData(">", "right motor position: " + rightRear.getCurrentPosition());
        telemetry.addData(">", "left motor position: " + leftRear.getCurrentPosition());
    */

    }

}