package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 Mihir Sonthi, Dhruv Battula 1/26/2022
 */

@Autonomous(name="Blue Box Shipping Hub", group="Active")
public class BlueBoxShippingHub extends LinearOpMode 
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

        // Go to Ducks
        driveTrain.strafeRight(1.5,(float)0.7);
        driveTrain.turn(15,(float)0.3);
        driveTrain.backward(2.2,(float)0.7);
        driveTrain.backward(1.1,(float)0.2);
        
        // Spin Ducks
        robot.duck_motor.setPower(-1);
        Thread.sleep(3000);
        robot.duck_motor.setPower(0);
        
        // Park in Red Box
        driveTrain.forward(0.2,(float)0.3);
        driveTrain.strafeRight(2.5,(float)0.7);
        driveTrain.forward(0.3,(float)0.3);
        
        
        
        deliverCargo();
    }
    
    void deliverCargo() throws InterruptedException {
        moveToTower();
        dropOffCargo();
        moveBackFromTower();
    }
    
    void moveToTower() throws InterruptedException {
        driveTrain.strafeRight(1, (float)0.3);
        driveTrain.forward(2.2, (float) 0.7);
    }
    
    void moveBackFromTower() throws InterruptedException {
        driveTrain.backward(2.5, (float) 0.7);
        driveTrain.strafeLeft(1.1, (float)0.3);
        driveTrain.backward(1, (float)0.4);
    }
    
    void dropOffCargo() {
      slides.MoveSlides(-4600);
      
      robot.intake_spinner.setPower(-0.75);
      sleep(1000);
      robot.intake_spinner.setPower(0);
      
      slides.MoveSlides(0);
    }
}
