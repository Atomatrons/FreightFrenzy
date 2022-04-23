package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveHard", group = "Atomobot v2")
public class DriveHard extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    
    private int MOTOR_TICKS_PER_360 = 1120;
    private double GEAR_RATIO = 40/15.0;
    private double TICKS_PER_360 = MOTOR_TICKS_PER_360 * GEAR_RATIO;

    private ShivaRobot robot = new ShivaRobot();
    private Arm_Prime arm;
    
    private Boolean resetArm = false;
    private Boolean armIsBusy = false;
    private Boolean emergency = false;

    public double servoPosition;

    
    public void init() {
      robot.init(hardwareMap);      
    }

    public void loop() {
        drive();
        telemetry();
    }

    //                                     *****SUBSYSTEMS*****

    // DRIVE
    public void drive() {
        
        robot.slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).

        double drive  = gamepad1.left_stick_y;
        double twist = -gamepad1.right_stick_x / 2;
        double strafe = -gamepad1.left_stick_x / 2;
        
        if(gamepad1.dpad_up)
        {
            drive = -0.2;
        }
        if(gamepad1.dpad_down)
        {
            drive = 0.2;
        }
        if(gamepad1.dpad_left)
        {
            strafe = 0.4;
        }
        if(gamepad1.dpad_right)
        {
            strafe = -0.4;
        }

        double[] speeds = {
            -(drive + strafe + twist), //Front left power
            -(drive - strafe - twist), //Front right power
            -(drive - strafe + twist), //Back left power
            -(drive + strafe - twist) //Back right power
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

        robot.duck_motor.setPower(gamepad2.right_trigger);
        robot.duck_motor.setPower(-gamepad2.left_trigger);
        
        robot.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad2.right_stick_y > 0 && robot.slides_motor.getCurrentPosition() <= 0) 
        {
            robot.slides_motor.setPower(gamepad2.right_stick_y);
        }
    
        else if(gamepad2.right_stick_y < 0 && robot.slides_motor.getCurrentPosition() >= -4950) 
        {
            robot.slides_motor.setPower(gamepad2.right_stick_y);
        }
        else 
        {
            robot.slides_motor.setPower(0);
        }
        
        robot.intake_spinner.setPower(gamepad2.left_stick_y);
}
    
    // Previous arm method:
    public void arm() {
        servoPosition = 0;
        if(gamepad2.left_bumper == true) {
            servoPosition = 1.0;
        }
        robot.wobble_1.setPosition(servoPosition);

        
        // If the arm is still going where we told it to go, keep going
        if (robot.wobble_arm.isBusy() && armIsBusy) {
            return;
        }
        armIsBusy = false;
    
        // If the arm has completed going where we told it to go, stop the motor
        if (resetArm) {
            robot.wobble_arm.setPower(0);
            robot.wobble_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetArm = false;
        }
            
        // Respond to gamepad commands
        //  UP: put arm in position to pick up wobble goal
        if (gamepad2.y) {
            arm.moveWobbleArmDown();
            armIsBusy = true;
        }
            
        // DOWN: pull up the wobble goal
        if (gamepad2.a) {
            arm.moveToWobblePositionUpArm();
            armIsBusy = true;
        }
            
        // LEFT: Reset to home
        if (gamepad2.x) {
            arm.resetArm();
        }
            
        // RIGHT: Drop off Rings 
        if (gamepad2.b) {
            arm.ringDropoff();
            armIsBusy = true;
        }
        
        //if arm slips, can fix
        double emergencyArmPower = 0;
        if (gamepad2.left_trigger > 0) {
            emergencyArmPower = gamepad2.left_trigger;
            emergency = true;
        } else if (gamepad2.right_trigger > 0) {
            emergencyArmPower = -gamepad2.right_trigger;
            emergency = true;
        }
        if (emergency) {
            robot.wobble_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.wobble_arm.setPower(emergencyArmPower);
        }
    }
    
    // Previous intake method:

    public void drive_intake() {
        double drive = gamepad2.left_stick_y;

        // apply the calculated values to the motors.
        robot.intake.setPower(drive);

        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // TELEMETRY
    public void telemetry() {
        telemetry.addData("Running", "Loop");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        
        telemetry.addData("Slide Motor", "Position: " + robot.slides_motor.getCurrentPosition());
        telemetry.addData("Test Encoder", "Ticks: " + robot.test_encoder.getCurrentPosition());

        // Previous Arm and Servo telemetry:

        /*
        telemetry.addData("arm position", "" + robot.wobble_arm.getCurrentPosition());
        telemetry.addData("arm target", "" + robot.wobble_arm.getTargetPosition());
        telemetry.addData("arm busy", "" + robot.wobble_arm.isBusy());
        telemetry.addData("servo position", "" + servoPosition);
        */
    }

}