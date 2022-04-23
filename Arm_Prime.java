package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm_Prime {
    private ShivaRobot robot;
    private Boolean resetArm = false;
    private Boolean armIsBusy = false;
    
    void init(ShivaRobot jeff)
    {
        robot = jeff;
    }
    
    // Carry wobble goal position
    void moveToWobblePositionUpArm()
    {
        int newTargetPosition = 0 - (int)Math.round(ShivaRobot.MOTOR_TICKS_PER_360 / 3);
        robot.wobble_arm.setTargetPosition(newTargetPosition);
        robot.wobble_arm.setPower(0.1);
        robot.wobble_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armIsBusy = true;
    }
    
    // pick up wobble goal position
    void moveWobbleArmDown()
    {
        int newTargetPosition = 0 - (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 / 1.65);
        robot.wobble_arm.setTargetPosition(newTargetPosition);
        robot.wobble_arm.setPower(0.4);
        robot.wobble_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armIsBusy = true;
    }
    
    // fold arm back to lay on top of robot
    void resetArm()
    {
        robot.wobble_arm.setTargetPosition(0);
        robot.wobble_arm.setPower(0.2);
        robot.wobble_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetArm = true;
        armIsBusy = true;
    }
    
    void encoderReset()
    {
        robot.wobble_arm.setPower(0);
        robot.wobble_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetArm = false;
    }
    
    void ringDropoff()
    {
        int newTargetPosition = 0 - (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 / 2);
        robot.wobble_arm.setTargetPosition(newTargetPosition);
        robot.wobble_arm.setPower(0.1);
        robot.wobble_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armIsBusy = true;
    }
}
