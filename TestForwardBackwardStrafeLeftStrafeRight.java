package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "TestForwardBackwardStrafeLeftStrafeRight (Blocks to Java)", group = "Test")
public class TestForwardBackwardStrafeLeftStrafeRight extends LinearOpMode {

  private DcMotor front_right;
  private DcMotor back_right;
  private DcMotor front_left;
  private DcMotor back_left;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    back_left = hardwareMap.get(DcMotor.class, "back_left");

    // Put initialization blocks here.
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      drive_forward();
      drive_backward();
      strafe_left();
      strafe_right();
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void wheel_test_1() {
    front_left.setPower(1);
    sleep(1000);
    front_left.setPower(0);
    back_left.setPower(1);
    sleep(1000);
    back_left.setPower(0);
    front_right.setPower(1);
    sleep(1000);
    front_right.setPower(0);
    back_right.setPower(1);
    sleep(1000);
    back_right.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void drive_forward() {
    front_left.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    back_right.setPower(0.5);
    sleep(1000);
    front_left.setPower(0);
    back_left.setPower(0);
    front_right.setPower(0);
    back_right.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void strafe_left() {
    front_left.setPower(-0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    back_right.setPower(-0.5);
    sleep(1000);
    front_left.setPower(0);
    back_left.setPower(0);
    front_right.setPower(0);
    back_right.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void drive_backward() {
    front_left.setPower(-0.5);
    back_left.setPower(-0.5);
    front_right.setPower(-0.5);
    back_right.setPower(-0.5);
    sleep(1000);
    front_left.setPower(0);
    back_left.setPower(0);
    front_right.setPower(0);
    back_right.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void strafe_right() {
    front_left.setPower(0.5);
    back_left.setPower(-0.5);
    front_right.setPower(-0.5);
    back_right.setPower(0.5);
    sleep(1000);
    front_left.setPower(0);
    back_left.setPower(0);
    front_right.setPower(0);
    back_right.setPower(0);
  }
}
