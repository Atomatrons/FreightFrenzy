package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 Manav Sanghvi 1/23/2021
 */

@Autonomous(name="Drive Train Test", group="Inactive")
public class DriveTrainTestOp extends LinearOpMode 
{
    ShivaRobot robot = new ShivaRobot();
    DriveTrain driveTrain = new DriveTrain();
    Gyro gyro = new Gyro();
        
    public void runOpMode() throws InterruptedException
    {
      robot.init(hardwareMap);
      gyro.init(robot, telemetry);
      driveTrain.init(robot, gyro);
      
      waitForStart();

      driveTrain.forward(2, 1);
      driveTrain.backward(2, 1);
      driveTrain.strafeRight(2,1);
      driveTrain.strafeLeft(2,1);
    }
    
}
