package org.firstinspires.ftc.teamcode.kavitha;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Ch1 Skystone Auto", group = "autonomous")
public class CBRedChannel1SkystoneAuto extends CBAutoBase {

  private ElapsedTime runtime = new ElapsedTime();
  private double timeoutS=25; // we keep timeout in seconds and set it to 25; After this time, robot will go and Rest on the tape
  int i=0;


  @Override
  public void runOpMode() {
    runtime.reset();
    initialize();

  // Wait for the game to start (driver presses PLAY)
    waitForStart();

    moveStraight(1,0.8,36);
    while(true) {

      if (isSkystoneVisible()) {
        //lower arm and pick skystone
          telemetry.addData(">","skystone detected. Pick it with arm");
          i++;
          break;
      } else {
        //move left one stone
          moveSideways(1, 0.2, 8);
      }
    }
    //move to channel 1 back wards
    moveStraight(0,0.8,36);

    // move right to the building zone
    moveSideways(0,0.8,90);

    // drop the arm

    // move left to REST at the line
    moveSideways(1,0.8,18);

//    // if elapsed time is less than 15 seconds, detect one more skystone
//    if(runtime.seconds()<10){
//      //move left to reach the starting popsition
//      moveSideways()
//    }

  }



}