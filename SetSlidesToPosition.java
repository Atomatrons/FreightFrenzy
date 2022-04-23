package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Set Slides To Position", group = "Active")
public class SetSlidesToPosition extends LinearOpMode{
  
    private ShivaRobot robot = new ShivaRobot();

    public void runOpMode()throws InterruptedException
    {
      robot.init(hardwareMap);
      
      waitForStart();
      
      robot.slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void moveSlides() throws InterruptedException
    {
     robot.slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      robot.slides_motor.setTargetPosition(0);
      robot.slides_motor.setPower(-1);
      while(robot.slides_motor.isBusy()){}
    }
}