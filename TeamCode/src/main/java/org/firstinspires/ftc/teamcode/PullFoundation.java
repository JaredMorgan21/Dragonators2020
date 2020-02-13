package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "PullFoundation", group =  "auto")


public class PullFoundation extends LinearOpMode {
//declaring motor variables, BLM = Back left motor 
    private Gyroscope imu;
    private DcMotor BLM;
    private DcMotor BRM;
    private DcMotor FLM;
    private DcMotor FRM;
    private Blinker expansion_Hub_2;
    
    private CRServo claw;
    

    public void runOpMode() {
        //Getting Motor Name From Phone
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        // showing which direction the robot moves 
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
        // geting claw from phone
        claw = hardwareMap.crservo.get("claw");
        waitForStart();
        if (opModeIsActive()) {
            
         //the robot is moving towards the foundation
            sideways(0.5,1500);
         //the claw/arm is going down to pull the foundation  
            claw.setPower(1);
            sleep(1000);
         // the robot is moving backwards pulling the foundation towards the building zone   
            sideways(-0.5,3000);
            claw.setPower(-1);
            sleep(500);
         // the claw/arm resets    
            claw.setPower(0); 
            sleep(500);
         //the robot is backing up to get to pull the foundation into the building zone
            drive(-0.5,500);
            sleep(1000);
            claw.setPower(1);
            sleep(750);
         //the robot is giving the foundation an extra push if the foundation is not in 
            drive(0.25,500);
            sleep(500);
         //the robot is moving back after it detaches from the foundation
            drive(-0.5,750);
            sleep(500);
            claw.setPower(-1);
        
                      
        
        }
    }
    //Functions save us time and reduces space in the main code.
    
    // This function sets the power for all the motors to the same value. 
    public void drive(double power,int time) { 
        BLM.setPower(power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(power);
        sleep(time);
        Stop();
        
    }
    // For sideways motion. Negative power reverses the power for the motor.
    
    public void sideways(double power,int time){
        BLM.setPower(-power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(-power);
        sleep(time);
        Stop();
        
    }
    
    //For North east/South West diaginal motion. 
    public void diagonalFR(double power,int time){
        BRM.setPower(power);
        FLM.setPower(power);
        sleep(time);
        Stop();
        
        

    }
    
    // For North West/ South East diaginal motion. 
    public void diagonalFL(double power,int time){
        BLM.setPower(power);
        FRM.setPower(power);
        sleep(time);
        Stop();
        
    }

    public void turn(double power,int time){
        BLM.setPower(power);
        BRM.setPower(-power);
        FLM.setPower(power);
        FRM.setPower(-power);
        sleep(time);
        Stop();
    
    }
    public void Stop(){
        BLM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        FRM.setPower(0);
    }
    
}
