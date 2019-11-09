package org.firstinspires.ftc.teamcode.kavitha;

import java.text.DecimalFormat;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CBRoboUtil {

    CBRobotHardware robot;
    Telemetry telemetry;
    String mode = null;

    DecimalFormat df = new DecimalFormat("0.00");

    int i = 1; //Counter for Msg
    StringBuffer strBuff = null;

    CBRoboUtil() {
    }

    CBRoboUtil(HardwareMap pHardwareMap, Telemetry pTelemetry) {
        robot = new CBRobotHardware(pHardwareMap);
        telemetry = pTelemetry;
    }

    CBRoboUtil(String pMode, HardwareMap pHardwareMap, Telemetry pTelemetry) {
        this.mode = pMode;
        this.robot = new CBRobotHardware(pHardwareMap);
        this.telemetry = pTelemetry;
    }

    public void setMode(String pMode) {
        this.mode = pMode;
    }

    public String getMode() {
        return this.mode;
    }

    public void moveForward() {
        addStatus("moveForward.");
        //0 1 0 ==> -1 1 -1 1 [Direction Vector]
        setMovePower(-1, 1, -1, 1);
    }

    public void moveBackward() {
        addStatus("moveBackward.");
        //0 -1 0 ==> 1 -1 1 -1 [Direction Vector]
        setMovePower(1, -1, 1, -1);
    }

    public void moveRight() {
        //1 0 0 ==> -1 -1 1 1 [Direction Vector]
        addStatus("moveRight.");
        setMovePower(-1, -1, 1, 1);
    }

    public void moveLeft() {
        addStatus("moveLeft.");
        //-1 0 0 ==> 1 1 -1 -1 [Direction Vector]
        setMovePower(1 ,1 ,-1, -1);
    }

    public void rotateLeft() {
        addStatus("rotateLeft.");
        //0 0 -1 ==> 1 1 1 1 [Direction Vector] rotate the robot in anti-clockwise direction
        setRotatePower(1, 1, 1, 1);
    }

    public void rotateRight() {
        addStatus("rotateRight.");
        //0 0 1 ==> -1 -1 -1 -1 [Direction Vector] rotate the robot in clockwise direction
        setRotatePower(-1, -1, -1, -1);
    }

    public void moveDiagonalQ1() {
        addStatus("move Diagonal Q1.");
        //0.71 0.71 0 ==> -1 0 0 1 [Direction Vector] move diagonally in Quad Q1
        setMovePower(-1, 0, 0, 1);
    }

    public void moveDiagonalQ2() {
        addStatus("move Diagonal Q2.");
        //-0.71 0.71 0 ==> 0 1 -1 0 [Direction Vector] move diagonally in Quad Q2
        setMovePower(0, 1, -1, 0);
    }

    public void moveDiagonalQ3() {
        addStatus("move Diagonal Q3.");
        //-0.71 -0.71 0 ==> 1 0 0 -1 [Direction Vector] move diagonally in Quad Q3
        setMovePower(1, 0, 0, -1);
    }

    public void moveDiagonalQ4() {
        addStatus("move Diagonal Q4.");
        //0.71 -0.71 0 ==> 0 -1 1 0 [Direction Vector] move diagonally in Quad Q4
        setMovePower(0, -1, 1, 0);
    }

    public void setMovePower(double pd1, double pd2, double pd3, double pd4) {
        setPower(pd1, pd2, pd3, pd4, robot.CONSTANT_MOVE_POWER);
    }

    public void setRotatePower(double pd1, double pd2, double pd3, double pd4) {
        setPower(pd1, pd2, pd3, pd4, robot.CONSTANT_ROTATE_POWER);
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
        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftRear.setPower(v3);
        robot.rightRear.setPower(v4);

        //Display the power settings
        strBuff = new StringBuffer();
        strBuff.append(" v1 " + df.format(v1));
        strBuff.append(" v2 " + df.format(v2));
        strBuff.append(" v3 " + df.format(v3));
        strBuff.append(" v4 " + df.format(v4));
        updateStatus(strBuff.toString());
    }

    public void clawOpen() {
        robot.claw.setPosition(robot.CLAW_OPEN);
    }

    public void clawOpen(double pDouble) {
        robot.claw.setPosition(pDouble);
    }

    public void clawClose() {
        robot.claw.setPosition(robot.CLAW_CLOSE);
    }

    public void clawClose(double pDouble) {
        robot.claw.setPosition(pDouble);
    }

    public void addStatus(String strMsg) {
        addStatus("Status", i + "." + strMsg);
    }

    public void addStatus(String strMsgKey, String strMsg) {
        telemetry.addData(strMsgKey, strMsg);
    }

    public void updateStatus(String strMsg) {
        updateStatus("Status", strMsg);
    }

    public void updateStatus(String strMsgKey, String strMsg) {
        telemetry.addData(strMsgKey, strMsg);
        telemetry.update();
        i++;
    }

}
