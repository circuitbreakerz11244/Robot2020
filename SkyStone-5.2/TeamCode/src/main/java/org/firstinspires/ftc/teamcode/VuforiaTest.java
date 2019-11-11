package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

@Autonomous(name = "vuforia test")
public class VuforiaTest extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        vcb.initVuforia();

        waitForStart();

        CameraDevice.getInstance().setFlashTorchMode(true);

        while(opModeIsActive()) {

            boolean visible = vcb.targetVisible();

            telemetry.addData(">", "x pose = " + vcb.getX());
            telemetry.addData(">", "y pose = " + vcb.getY());
            telemetry.addData(">", "target visible: " + visible);
            telemetry.update();
        }

        CameraDevice.getInstance().setFlashTorchMode(false);

    }
}
