package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ServoPositionTest extends OpMode {

    Servo testServo;
    double servoPos = 0.5;

    @Override
    public void init() {
        testServo = hardwareMap.servo.get("servoName");
        testServo.setPosition(servoPos);
    }

    @Override
    public void loop() {
        Range.clip(servoPos, 0, 1);

        if (gamepad1.a) {
            servoPos += 0.01;
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (gamepad1.b) {
            servoPos -= 0.01;
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }        }
        testServo.setPosition(servoPos);
        telemetry.addData(">"," Current Servo Pos: " + servoPos);
    }
}
