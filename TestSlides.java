package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestSlides (Blocks to Java)", group = "Test")
public class TestSlides extends LinearOpMode {

  private DcMotor slides;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    slides = hardwareMap.get(DcMotor.class, "slides");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      runTest();
    }
  }

  /**
   * Describe this function...
   */
  private void runTest() {
    telemetry.speak("start", null, null);
    telemetry.update();
    slides.setPower(0.6);
    sleep(2000);
    telemetry.speak("0.75", null, null);
    telemetry.update();
    slides.setPower(0.8);
    sleep(2000);
    slides.setPower(0.3);
    sleep(2000);
    slides.setPower(0);
  }
}
