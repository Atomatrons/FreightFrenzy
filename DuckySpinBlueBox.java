package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 Mihir Sonthi, Dhruv Battula 1/26/2022
 */

@Autonomous(name="Ducky Spin Blue Box", group="Active")
public class DuckySpinBlueBox extends LinearOpMode 
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
        
        driveTrain.strafeRight(1.5,(float)0.7);
        driveTrain.turn(15,(float)0.3);
        driveTrain.backward(2.2,(float)0.7);
        driveTrain.backward(0.9,(float)0.2);
        
        robot.duck_motor.setPower(-1);
        Thread.sleep(3000);
        robot.duck_motor.setPower(0);
        
        driveTrain.forward(0.2,(float)0.3);
        driveTrain.strafeRight(2.3,(float)0.7);
        driveTrain.forward(0.3,(float)0.3);
        driveTrain.turn(10,(float)0.3);
        driveTrain.backward(0.5, (float)0.1);
       
    }
}
