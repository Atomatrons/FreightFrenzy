package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is the Shiva Atomobot
 */
public class ShivaRobot
{
    // Wheel motors
    public DcMotor front_left  = null;
    public DcMotor front_right = null;
    public DcMotor back_left   = null;
    public DcMotor back_right  = null;
    
    public DcMotor test_encoder = null;
    
    public DcMotor slides_motor = null;
    public DcMotor intake_spinner = null;

    public DcMotor duck_motor = null;
    
    // Arm motor & servos
    public DcMotorEx wobble_arm  = null;
    public Servo wobble_1 = null;
    public Servo wobble_2 = null;
    public Servo rings = null;
    
    // Intake motor
    public DcMotorEx intake = null;
    
    // Gyro
    public BNO055IMU imu = null;
    
    //Color Sensor
    public NormalizedColorSensor colorSensor = null;
    
    // Constants
    public static final double MOTOR_TICKS_PER_360 = 1120;
    public static final double ARM_GEAR_RATIO = 40 / 15.0;
    public static final double ARM_TICKS_PER_360 = MOTOR_TICKS_PER_360 * ARM_GEAR_RATIO;

    // local OpMode members
    HardwareMap hardwareMap    =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ShivaRobot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap inputMap) {
        // Save reference to Hardware map
        hardwareMap = inputMap; 
        
        // Initialize wheel motors
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class,"front_right");
        
        test_encoder = hardwareMap.get(DcMotor.class, "test_encoder");
        
        duck_motor = hardwareMap.get(DcMotor.class, "duck");
        slides_motor = hardwareMap.get(DcMotor.class, "slides");
        intake_spinner = hardwareMap.get(DcMotor.class, "intake_spinner");
        
        //slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    

        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        
        // Previous wobble arm code:

        /*
        wobble_arm = hardwareMap.get(DcMotorEx.class, "wobble_arm");
        wobble_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobble_arm.setTargetPosition(wobble_arm.getCurrentPosition());
        */

        // Previous wobble servo code:

        /*
        wobble_1 = hardwareMap.get(Servo.class, "wobble_1");
        wobble_2 = hardwareMap.get(Servo.class, "wobble_2");
        rings = hardwareMap.get(Servo.class, "rings");
        */
        
        // Previous intake code:

        /*
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        */
        
        // Initialize gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        // Intitialize color sensor
        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorsenv3");
    }
 }
