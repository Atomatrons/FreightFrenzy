package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "WarehouseParking", group = "Active")
public class WarehouseParking extends LinearOpMode{
  
    private ShivaRobot robot = new ShivaRobot();
    private Gyro gyro = new Gyro();
    private DriveTrain driveTrain = new DriveTrain();

    public void runOpMode() throws InterruptedException
    {
      robot.init(hardwareMap);
      gyro.init(robot, telemetry);
      driveTrain.init(robot, gyro);
      
      
      waitForStart();
      
     driveTrain.forward(3.5,(float)1);
     
    }
    
}
