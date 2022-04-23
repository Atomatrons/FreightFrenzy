package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
1/20/21 - Manav Sanghvi
 */

@TeleOp(name = "GyroTestOp", group = "Test OpMode")
public class GyroTestOp extends OpMode {
    
    //Declares variables for movement
    private int MOTOR_TICKS_PER_360 = 1120;
    private double GEAR_RATIO = 40/15.0;
    private double TICKS_PER_360 = MOTOR_TICKS_PER_360 * GEAR_RATIO;
    
    //Constructs new robot and gyro class.
    ShivaRobot robot = new ShivaRobot();
    Gyro gyro = new Gyro();
    
    //Initialize method
    public void init()
    {
        //initializes robot and gyro.
        robot.init(hardwareMap);
        gyro.init(robot, telemetry);
    }
    
    //Loops through driving and adding to telemetry.
    public void loop()
    {
        drive();
        gyro.getCurrentAngle();
    }    

    //                                     *****SUBSYSTEMS*****

    // DRIVE
    public void drive() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).

        double drive  = gamepad1.left_stick_y;
        double twist = -gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        
        if (gamepad1.y) {
            drive = 0.0;
            strafe = 0.5;
            twist = 0.0;
        } else if (gamepad1.x) {
            drive = 0.0;
            strafe = -0.5;
            twist = 0.0;
        }

        double[] speeds = {
            -(drive + strafe + twist),
            -(drive - strafe - twist),
            -(drive - strafe + twist),
            -(drive + strafe - twist)
        };

        //Normalizes values
        double max = Math.abs(speeds[0]);
        for(int i = 1; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        
        // Reverse meaning of joystick if back button pressed to make it easier to
        // navigate robot when dealing with the arm
        if (gamepad1.right_bumper) {
            for (int index = 0; index < 4; index++) {
                speeds[index] *= -1;
            }
        }
        
        // apply the calculated values to the motors.
        robot.front_left.setPower(speeds[0]);
        robot.front_right.setPower(speeds[1]);
        robot.back_left.setPower(speeds[2]);
        robot.back_right.setPower(speeds[3]);
        
        robot.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }
    
    
    public void telemetry() 
    {
        telemetry.addData("Running", "Loop");
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gyro", "Current Angle: " + gyro.getCurrentAngle());
        telemetry.update();
    }
    
}