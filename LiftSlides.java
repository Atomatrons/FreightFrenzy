package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "LiftSlides (Blocks to Java)", group = "Test")
public class LiftSlides extends LinearOpMode {
  private DcMotor slides;
  private DcMotor duck;
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    slides = hardwareMap.get(DcMotor.class, "slides");
    duck = hardwareMap.get (DcMotor.class, "duck");
       
    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      LiftSlides();
    }
  }

  /**
   * Describe this function...
   */
  private void LiftSlides() {
    telemetry.addData("MyNameIs", "Mihir");
    telemetry.addData("velocity", 0.5);
    telemetry.update();
    
    duck.setPower(0.03);

    sleep(1000);
    slides.setPower(-0.7);
    sleep(4000);
    telemetry.addData("lifting up slides","lifting up by .7 power");
    slides.setPower(0);
    sleep(1000);
    telemetry.addData("slide motor activity", "IDLE");
    slides.setPower(0.7);
    sleep(4000);
    
    
    telemetry.addData("Carousel Spinning Direction", "Clockwise");
    
  }
}
