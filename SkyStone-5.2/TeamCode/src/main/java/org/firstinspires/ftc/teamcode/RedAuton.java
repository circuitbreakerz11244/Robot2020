package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "red auton")
public class RedAuton extends CBAutonomousBase {

    @Override
    public void runOpMode() {

        initialization();

        waitForStart();

        encoderDrive("MB", 30, 0.5, false);

        sleep(1000);

        telemetry.addData(">", "searching for skystone...");
        telemetry.update();

        encoderDrive("MR", 36, 0.2, true);

        telemetry.addData(">", "skystone found! navigating...");
        telemetry.update();

        sleep(1000);

        if (vcb.targetVisible()) {
            vuforiaNavigate(0.0);
        }

        sleep(2000);

        // pick up skystone

        telemetry.addData(">", "skystone delivered!");

        encoderDrive("MF", 24, 0.5, false);

        // go sideways to park
//        strafe(1, 0.8, 0);






    }

}
