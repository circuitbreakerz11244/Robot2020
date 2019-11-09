package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "AutonomousV1", group = "autonomous")
public class CBAutoTest1 extends CBAutonomousBase {

    @Override
    public void runOpMode() {

        initialization();
        waitForStart();

        encoderDrive("MF",12,0.7);
        util.displayDriveEncoderValues();
        sleep(3000);
        encoderDrive("MB",12,0.7);
        util.displayDriveEncoderValues();
        sleep(3000);
        encoderDrive("MR",12,0.4);
        util.displayDriveEncoderValues();
        sleep(3000);
        encoderDrive("ML",12,0.4);
        util.displayDriveEncoderValues();
        sleep(100000);
    }


}
