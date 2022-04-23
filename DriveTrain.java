package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/*
 Manav Sanghvi 1/23/2021
 */
 

public class DriveTrain {
    
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor duck_motor = null;
    
    private Gyro gyro = null;
    private ShivaRobot robot = null;
    
    private float amplifier = 0.02f;
    
    public void init(ShivaRobot robot, Gyro newGyro)
    {
        front_left  = robot.front_left;
        front_right = robot.front_right;
        back_left   = robot.back_left;
        back_right  = robot.back_right;
        duck_motor = robot.duck_motor;
        gyro = newGyro;
        
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void forward(double rotationsToSpin, double power)
    {
        float startAngle = (float)gyro.getCurrentAngle();
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);
        
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        
        front_left.setTargetPosition(target_position);
        back_left.setTargetPosition(target_position);
        front_right.setTargetPosition(target_position);
        back_right.setTargetPosition(target_position);
        
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        front_left.setPower((float)power);
        back_left.setPower((float)power);
        front_right.setPower((float)power);
        back_right.setPower((float)power);
        
        while(front_left.isBusy())
        {
            adjust(-getCorrectionAngle(startAngle), (float)power);
        }
        
        reset();
    }
    //Carousel by Chris Wilford 11/18/21
    public void carousel (double rotationsToSpin, double power)
    {
         int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin * 4.233);
         
         duck_motor.setDirection(DcMotorSimple.Direction.FORWARD);
         
         duck_motor.setTargetPosition(target_position);
         
         duck_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         
         duck_motor.setPower((float)power);
         
         while(duck_motor.isBusy())
         {
             
         }
    }
    
    public void backward(double rotationsToSpin, double power)
    {   
        float startAngle = (float)gyro.getCurrentAngle();
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);
        
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        front_left.setTargetPosition(target_position);
        back_left.setTargetPosition(target_position);
        front_right.setTargetPosition(target_position);
        back_right.setTargetPosition(target_position);
        
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        front_left.setPower((float)power);
        back_left.setPower((float)power);
        front_right.setPower((float)power);
        back_right.setPower((float)power);

        while(back_left.isBusy())
        {
            adjust(getCorrectionAngle(startAngle), (float)power);
        }
        
        reset();
    }

    public void strafeRight(double rotationsToSpin, double power) throws InterruptedException
    {
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);
                
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        
        front_left.setTargetPosition(target_position);
        back_left.setTargetPosition(target_position);
        front_right.setTargetPosition(target_position);
        back_right.setTargetPosition(target_position);
        
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower((float)power);
        back_left.setPower((float)power);
        front_right.setPower((float)power);
        back_right.setPower((float)power);
        
        while(front_left.isBusy())
        {
                
        }

        reset();
    }
    
    public void strafeLeft(double rotationsToSpin, double power) throws InterruptedException
    {
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);
                
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        
        front_left.setTargetPosition(target_position);
        back_left.setTargetPosition(target_position);
        front_right.setTargetPosition(target_position);
        back_right.setTargetPosition(target_position);
        
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower((float)power);
        back_left.setPower((float)power);
        front_right.setPower((float)power);
        back_right.setPower((float)power);
        
        while(front_left.isBusy())
        {
                
        }

        reset();
    }
    
    public void turn(float compassPoint, float power)
    {
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        if (gyro.isClockwise(compassPoint))
        {
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
            back_left.setDirection(DcMotorSimple.Direction.FORWARD);
            front_right.setDirection(DcMotorSimple.Direction.FORWARD);
            back_right.setDirection(DcMotorSimple.Direction.FORWARD);
                
            front_left.setPower(power);
            back_left.setPower(power);
            front_right.setPower(power);
            back_right.setPower(power);
            while(gyro.getCurrentAngle() < compassPoint)
            {
                
            }
        }
        else if (!gyro.isClockwise(compassPoint))
        {
            front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            back_right.setDirection(DcMotorSimple.Direction.REVERSE);
                
            front_left.setPower(power);
            back_left.setPower(power);
            front_right.setPower(power);
            back_right.setPower(power);
            while(gyro.getCurrentAngle() > compassPoint)
            {
                
            }
        }
        
        reset();
        
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    
    public void adjust(float angle, float basePower)
    {
        float powerChange = Math.abs(angle * amplifier);
        
        if(angle > 0)
        {
            front_left.setPower(basePower - powerChange);
            back_left.setPower(basePower - powerChange);
            front_right.setPower(basePower + powerChange);
            back_right.setPower(basePower + powerChange);
        }
        else if(angle < 0)
        {
            front_left.setPower(basePower + powerChange);
            back_left.setPower(basePower + powerChange);
            front_right.setPower(basePower - powerChange);
            back_right.setPower(basePower - powerChange);
        }
    }
    
    public float getCorrectionAngle(float startAngle)
    {
        float correctionAngle = startAngle - (float)gyro.getCurrentAngle();
        return correctionAngle;
    }
    
    public void stop()
    {
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }
    
    public void reset()
    {
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
