package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 Mihir Sonthi, Dhruv Battula 1/26/2022
 */

@Autonomous(name="REDDUCKSHIPPING", group="Active")
public class REDDUCKSHIPPING extends LinearOpMode 
{
    ShivaRobot robot = new ShivaRobot();
    DriveTrain driveTrain = new DriveTrain();
    Gyro gyro = new Gyro();
    SlideySlides slides=new SlideySlides();
        
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        gyro.init(robot, telemetry);
        driveTrain.init(robot, gyro);
        slides.init(robot);
        

        waitForStart();
        driveTrain.strafeLeft(1.5,(float)0.7);
        driveTrain.turn(-15,(float)0.3);
        driveTrain.backward(2.2,(float)0.7);
        driveTrain.backward(0.7,(float)0.2);
       robot.duck_motor.setPower(1);
      
        Thread.sleep(3000);
        
        robot.duck_motor.setPower(0);
        
    }
}
