package org.firstinspires.ftc.teamcode;

import java.text.DecimalFormat;

public class AppUtil {

    public static double[] getVectorArrays(double leftX, double leftY, double rightX) {

        double r = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;

        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        double[] dbArray = {v1, v2, v3, v4};

        return dbArray;
    }

    public static void main(String args[]) throws Exception {
        System.out.println("AppUtil Initialized..");

        //Unit Test values
        double x = 0;
        double y = .5;
        double z = 1;

        double leftX  = 0.0;
        double leftY  = 0.0;
        double rightX = 0.0;

        DecimalFormat df = new DecimalFormat("0.00");
        if(args.length > 0) {
            leftX = Double.parseDouble(args[0]);
            leftY = Double.parseDouble(args[1]);
            rightX = Double.parseDouble(args[2]);
        } else {
            leftX = x;
            leftY = y;
            rightX = z;
        }
        double[] tempArray = getVectorArrays(leftX, leftY, rightX);


        for(int i=0; i<tempArray.length; i++) {
            if ((i % 2) == 0) {
                System.out.println("v" + (i+1) + " >> " + df.format(-tempArray[i]));
            } else {
                System.out.println("v" + (i+1) + " >> " + df.format(tempArray[i]));
            }

        }
    }
}
