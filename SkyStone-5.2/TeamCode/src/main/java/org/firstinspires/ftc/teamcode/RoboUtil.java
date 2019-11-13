package org.firstinspires.ftc.teamcode;

import java.text.DecimalFormat;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoboUtil {

    CBMainHardware robot;
    Telemetry telemetry;
    String mode = null;
    boolean bForceStop = false;

    DecimalFormat df = new DecimalFormat("0.00");

    int i = 1; //Counter for Msg
    StringBuffer strBuff = null;

    RoboUtil() {
    }

    RoboUtil(String pMode) {
        this.mode = pMode;
    }

    RoboUtil(String pMode, Telemetry pTelemetry) {
        this.mode = pMode;
        this.telemetry = pTelemetry;
        this.robot = new CBMainHardware();
    }

    RoboUtil(HardwareMap pHardwareMap, Telemetry pTelemetry) {
        robot = new CBMainHardware(pHardwareMap);
        this.telemetry = pTelemetry;
    }

    RoboUtil(String pMode, HardwareMap pHardwareMap, Telemetry pTelemetry) {
        this.mode = pMode;
        this.robot = new CBMainHardware(pHardwareMap);
        this.telemetry = pTelemetry;
    }

    RoboUtil(String pMode, HardwareMap pHardwareMap, Telemetry pTelemetry, boolean bDriveEncoderOn) {
        this.mode = pMode;
        this.telemetry = pTelemetry;
        this.robot = new CBMainHardware(pHardwareMap, bDriveEncoderOn);
    }

    public void setMode(String pMode) {
        this.mode = pMode;
    }

    public String getMode() {
        return this.mode;
    }

    public void moveForward() {
        moveForward(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveForward(double pDblPower) {
        addStatus("moveForward with power " + pDblPower);
        setPower(CBRoboConstants.moveForward, pDblPower);
    }

    public void moveBackward() {
        moveBackward(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveBackward(double pDblPower) {
        addStatus("moveBackward with power " + pDblPower);
        setPower(CBRoboConstants.moveBackward, pDblPower);
    }

    public void moveRight() {
        moveRight(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveRight(double pDblPower) {
        addStatus("moveRight with power " + pDblPower);
        setPower(CBRoboConstants.moveRight, pDblPower);
    }

    public void moveLeft() {
        moveLeft(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveLeft(double pDblPower) {
        addStatus("moveLeft with power " + pDblPower);
        setPower(CBRoboConstants.moveLeft, pDblPower);
    }

    public void rotateLeft() {
        rotateLeft(CBRoboConstants.DRIVE_MOTOR_ROTATE_POWER);
    }

    public void rotateLeft(double pDblPower) {
        addStatus("rotateLeft with power " + pDblPower);
        setPower(CBRoboConstants.rotateLeft, pDblPower);
    }

    public void rotateRight() {
        rotateRight(CBRoboConstants.DRIVE_MOTOR_ROTATE_POWER);
    }

    public void rotateRight(double pDblPower) {
        addStatus("rotateRight with power " + pDblPower);
        setPower(CBRoboConstants.rotateRight, pDblPower);
    }

    public void moveDiagonalQ1() {
        moveDiagonalQ1(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveDiagonalQ1(double pDblPower) {
        addStatus("move Diagonal Q1 with power " + pDblPower);
        setPower(CBRoboConstants.moveDiagonalQ1, pDblPower);
    }

    public void moveDiagonalQ2() {
        moveDiagonalQ2(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveDiagonalQ2(double pDblPower) {
        addStatus("move Diagonal Q2 with power " + pDblPower);
        setPower(CBRoboConstants.moveDiagonalQ2, pDblPower);
    }

    public void moveDiagonalQ3() {
        moveDiagonalQ3(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveDiagonalQ3(double pDblPower) {
        addStatus("move Diagonal Q3 with power " + pDblPower);
        setPower(CBRoboConstants.moveDiagonalQ3, pDblPower);
    }

    public void moveDiagonalQ4() {
        moveDiagonalQ4(CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void moveDiagonalQ4(double pDblPower) {
        addStatus("move Diagonal Q4 with power " + pDblPower);
        setPower(CBRoboConstants.moveDiagonalQ4, pDblPower);
    }

    public void setMovePower(double pd1, double pd2, double pd3, double pd4) {
        setPower(pd1, pd2, pd3, pd4, CBRoboConstants.DRIVE_MOTOR_MOVE_POWER);
    }

    public void setRotatePower(double pd1, double pd2, double pd3, double pd4) {
        setPower(pd1, pd2, pd3, pd4, CBRoboConstants.DRIVE_MOTOR_ROTATE_POWER);
    }

    public void setPower(double[] pDirectionVector, double pMultiplier) {
        double d1 = pDirectionVector[0];
        double d2 = pDirectionVector[1];
        double d3 = pDirectionVector[2];
        double d4 = pDirectionVector[3];
        setPower(d1, d2, d3, d4, pMultiplier );
    }

    public void setPower(double pd1, double pd2, double pd3, double pd4, double pMultiplier) {
        double v1 = pd1 * pMultiplier;
        double v2 = pd2 * pMultiplier;
        double v3 = pd3 * pMultiplier;
        double v4 = pd4 * pMultiplier;
        setPower(v1, v2, v3, v4);
    }

    //Set the power for all motors
    public void setPower(double v1, double v2, double v3, double v4) {

        // -v1 v2+ Front wheel Arrangement --> leftFront(v1 - port0)  rightFront(v2 - port1)
        // -v3 v4+ Back  wheel Arrangement --> leftRear (v3 - port2)  rightRear (v4 - port3)

        //Clip is used to trim the power for the given range -1 to 1
        v1 = Range.clip(v1, -1, 1);
        v2 = Range.clip(v2, -1, 1);
        v3 = Range.clip(v3, -1, 1);
        v4 = Range.clip(v4, -1, 1);

        //Set the power for the motors
        robot.drive.leftFront.setPower(v1);
        robot.drive.rightFront.setPower(v2);
        robot.drive.leftRear.setPower(v3);
        robot.drive.rightRear.setPower(v4);

        //Display the power settings
        strBuff = new StringBuffer();
        strBuff.append(" v1 " + df.format(v1));
        strBuff.append(" v2 " + df.format(v2));
        strBuff.append(" v3 " + df.format(v3));
        strBuff.append(" v4 " + df.format(v4));
        updateStatus(strBuff.toString());
    }

    public void clawOpen() {
        robot.roboArmClaw.clawOpen();
    }

    public void clawOpen(double pDouble) {
        robot.roboArmClaw.clawOpen(pDouble);
    }

    public void clawClose() {
        robot.roboArmClaw.clawClose();
    }

    public void clawClose(double pDouble) {
        robot.roboArmClaw.clawClose(pDouble);
    }

    public void pullServoOpen() {
        robot.roboArmClaw.pullServoOpen();
    }

    public void pullServoOpen(double pDouble) {
        robot.roboArmClaw.pullServoOpen(pDouble);
    }

    public void pullServoClose() {
        robot.roboArmClaw.pullServoClose();
    }

    public void pullServoClose(double pDouble) {
        robot.roboArmClaw.skystoneServoClose(pDouble);
    }

    public void skystoneServoOpen() {
        robot.roboArmClaw.skystoneServoOpen();
    }

    public void skystoneServoOpen(double pDouble) {
        robot.roboArmClaw.skystoneServoOpen(pDouble);
    }

    public void skystoneServoClose() {
        robot.roboArmClaw.skystoneServoClose();
    }

    public void skystoneServoClose(double pDouble) {
        robot.roboArmClaw.clawClose(pDouble);
    }

    public int getMotorEncoderValue(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public void displayCurrentEncoderValues() {

        int encCurrentValues[] = robot.drive.getDriveCurrentEncoderValues();

        StringBuffer strBuff = new StringBuffer();
        strBuff.append(" c1=").append(encCurrentValues[0]);
        strBuff.append(" c2=").append(encCurrentValues[1]);
        strBuff.append(" c3=").append(encCurrentValues[2]);
        strBuff.append(" c4=").append(encCurrentValues[3]);
        updateStatus("EncValues", strBuff.toString());
    }

    public void displayDriveEncoderValues() {

        int encPreviousValues[] = robot.drive.getDrivePreviousEncoderValues();
        int encDeltaValues[] = robot.drive.getDriveEncoderDeltaValues();

        StringBuffer strBuff = new StringBuffer();
        strBuff.append(" p1=").append(encPreviousValues[0]).append(" d1=").append(encDeltaValues[0]);
        strBuff.append(" p2=").append(encPreviousValues[1]).append(" d2=").append(encDeltaValues[1]);
        strBuff.append(" p3=").append(encPreviousValues[2]).append(" d3=").append(encDeltaValues[2]);
        strBuff.append(" p4=").append(encPreviousValues[3]).append(" d4=").append(encDeltaValues[3]);
        updateStatus("EncValues", strBuff.toString());
    }

    public void addStatus(String strMsg) {
        addStatus("Status", i + "." + strMsg);
    }

    public void addStatus(String strMsgKey, String strMsg) {
        telemetry.addData(strMsgKey, strMsg);
    }

    public void updateStatus(String strMsg) {
        updateStatus("Status", i + "." + strMsg);
    }

    public void updateStatus(String strMsgKey, String strMsg) {
        telemetry.addData(strMsgKey, strMsg);
        telemetry.update();
        i++;
    }



}
