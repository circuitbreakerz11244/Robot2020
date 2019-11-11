package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

@Autonomous(name = "Red Channel 1 Skystone", group = "autonomous")
public class RedCH1Skystone extends CBAutonomousBase {

    String inputColor = "red";

    @Override
    public void runOpMode() {

        initialization();
        waitForStart();

        encoderDrive("MB", 25, 0.7, 0,false);
        sleep(1000);
        util.updateStatus(">", "Searching for Skystone...");

        CameraDevice.getInstance().setFlashTorchMode(true);

        encoderDrive("MR", 36, 0.2, 0,true);
        util.updateStatus(">", "Skystone found! navigating...");

        CameraDevice.getInstance().setFlashTorchMode(false);

        sleep(1000);

        if (vcb.targetVisible()) {
            vuforiaNavigate(0.0, "red");
        }
        sleep(2000);

        //TBD
        // pick up skystone

        util.updateStatus(">", "Skystone Delivered!");

        encoderDrive("ML", 72, 0.8, 0,false, true);
        sleep(1000);
        encoderDrive("ML", 24, 0.5, 0,false);
        sleep(1000);
        encoderDrive("MR", 24, 0.5, 0,false, true);

        //TBD
        //drop the skystone


    }
}