package org.firstinspires.ftc.teamcode;

public class RedAuton extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        encoderDrive(36, 0.8, 0);

        while(!vcb.targetVisible())  {
            strafe(-1, 0.2, 0);
            telemetry.addData(">", "searching for skystone...");
            telemetry.update();
        }

        telemetry.addData(">", "skystone found! navigating...");
        telemetry.update();

        Thread.sleep(1000);

        if (vcb.targetVisible()) {
            vuforiaNavigate(0.0);
        }

        Thread.sleep(2000);

        // pick up skystone
        grabStone();

        telemetry.addData(">", "skystone delivered!");

        encoderDrive(-36, 0.8, 0);

        // go sideways to park
//        strafe(1, 0.8, 0);






    }

}
