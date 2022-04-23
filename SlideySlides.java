package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

// all code done entirely by (without ANY help) Christopher wilford
public class SlideySlides {

     private ShivaRobot robot = null;
     private DcMotor slides= null;
     
     public void init(ShivaRobot robot){
         this.robot = robot;
         slides = robot.slides_motor;
         robot.slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }
     public void MoveSlides(int targetPosition){
         
        if (targetPosition < -4950 || targetPosition > 0) {
            return;
        }
        
        slides.setTargetPosition(targetPosition);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (targetPosition < slides.getCurrentPosition())
        {
             slides.setPower(1);
        }
        else if (targetPosition > slides.getCurrentPosition())
        {
             slides.setPower(-1);
        }
        
        while(slides.isBusy())
        {
        }
     }
}