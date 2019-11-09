package org.firstinspires.ftc.teamcode.kavitha;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Declare all Hardware Details here
public class CBRobotHardware {

    //Mecanum wheel Motors. Power Range for all motors is -1 to +1
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    //Arm Motor
    DcMotor arm;

    //Servo Range 0 to 1. 0 to 180 Degree (0 degree --> 0, 90 Degree --> 0.5, 180 Degree --> 1.0)
    //Servo for Claw
    Servo claw;

    //Servo for pull foundation
    Servo pullServo;

    final double CONSTANT_MOVE_POWER   = 0.71;
    final double CONSTANT_ROTATE_POWER = 1.00;

    final double CLAW_OPEN  = 0.3;
    final double CLAW_CLOSE = 0.8;

    final double SERVO_INITIAL = 0.0; //move to   0 degree
    final double SERVO_MIDDLE  = 0.5; //move to  90 degrees
    final double SERVO_FINAL   = 1.0; //move to 180 degrees

    boolean bHWInitialized = false;

    CBRobotHardware() {
    }

    CBRobotHardware(HardwareMap hardwareMap) {
        this.bHWInitialized = initializeHW(hardwareMap);
    }

    public boolean initializeHW(HardwareMap hardwareMap) {

        //Initialize all Hardware configured in the Robot
        // Mecanum Wheels
        leftFront   = hardwareMap.get(DcMotor.class,  "leftFront"  );
        rightFront  = hardwareMap.get(DcMotor.class,  "rightFront" );
        leftRear    = hardwareMap.get(DcMotor.class,  "leftRear"   );
        rightRear   = hardwareMap.get(DcMotor.class,  "rightRear"  );

        //Arm Motor
        arm  = hardwareMap.get(DcMotor.class,  "arm"  );

        //Servos
        claw = hardwareMap.servo.get("claw");
        pullServo = hardwareMap.servo.get("pullServo");

        //Make sure all Hardware are initialized
        if(leftFront != null && rightFront != null
                && leftRear != null && rightRear != null
                && arm != null
                && claw != null
                && pullServo != null
        ) {
            bHWInitialized = true;
        }

        return bHWInitialized;

    }

    public boolean getHWInitializationStatus() {
        return bHWInitialized;
    }

    public static void main(String args[]) throws Exception {
        System.out.println("CBRobotHardware...");
    }
}
