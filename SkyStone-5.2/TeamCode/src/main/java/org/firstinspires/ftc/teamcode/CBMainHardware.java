package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class CBMainHardware {

    CBMecanumDrive drive = null;
    CBIMU imu = null;
    CBRoboArmClaw roboArmClaw = null;
    //Include Vforia here

    boolean bRoboInitialized = false;

    CBMainHardware() {
    }

    CBMainHardware(HardwareMap hardwareMap) {
        this.bRoboInitialized = initializeRoboHW(hardwareMap);
    }

    CBMainHardware(HardwareMap hardwareMap, boolean bDriveEncodersOn) {
        this.bRoboInitialized = initializeRoboHW(hardwareMap, bDriveEncodersOn);
    }

    public boolean initializeRoboHW(HardwareMap hardwareMap) {
        return initializeRoboHW(hardwareMap ,false);
    }

    public boolean initializeRoboHW(HardwareMap hardwareMap, boolean bDriveEncodersOn) {

        drive = new CBMecanumDrive(hardwareMap, bDriveEncodersOn);
        imu   = new CBIMU(hardwareMap);
        roboArmClaw = new CBRoboArmClaw(hardwareMap);
        setRoboInitializationStatus(drive.getDriveInitializationStatus() && imu.isCalibrated() && roboArmClaw.getArmInitializationStatus());

        return bRoboInitialized;
    }

    public boolean initializeDrive(HardwareMap hardwareMap) {
        return initializeDrive(hardwareMap);
    }

    public boolean initializeDrive(HardwareMap hardwareMap, boolean bDriveEncodersOn) {
        drive = new CBMecanumDrive(hardwareMap, bDriveEncodersOn);
        return  drive.getDriveInitializationStatus();
    }

    public boolean initializeArmClaw(HardwareMap hardwareMap) {
        return initializeArmClaw(hardwareMap, true);
    }

    public boolean initializeArmClaw(HardwareMap hardwareMap, boolean bArmEncoderOn) {
        roboArmClaw = new CBRoboArmClaw(hardwareMap, bArmEncoderOn);
        return roboArmClaw.getArmInitializationStatus();
    }

    public boolean initializeArmClaw(HardwareMap hardwareMap, boolean bArmEncoderOn, boolean bClawEncodeOn) {
        roboArmClaw = new CBRoboArmClaw(hardwareMap, bArmEncoderOn, bClawEncodeOn);
        return roboArmClaw.getArmInitializationStatus();
    }

    public boolean getRoboInitializationStatus() {
        return bRoboInitialized;
    }

    public void setRoboInitializationStatus(boolean pInitialized) {
        this.bRoboInitialized = pInitialized;
    }
}
