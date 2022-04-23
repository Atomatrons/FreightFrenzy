package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Metal Mavericks", group = "Active")
public class MetalMavericks extends LinearOpMode{
  
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
     driveTrain.strafeRight(0.7, (float)0.4); 
    }
    
}
