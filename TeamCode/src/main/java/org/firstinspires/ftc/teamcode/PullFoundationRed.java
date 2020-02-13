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

@Autonomous(name = "PullFoundationRed", group =  "auto")


public class PullFoundationRed extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor BLM;
    private DcMotor BRM;
    private DcMotor FLM;
    private DcMotor FRM;
    private Blinker expansion_Hub_2;
    
    private CRServo claw;
    

    public void runOpMode() {
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
        claw = hardwareMap.crservo.get("claw");
        waitForStart();
        if (opModeIsActive()) {
          /*  drive(.25,1000);
            claw.setPosition(0);
            sleep(250);
            drive(-.25,1000);*/
    
            //sideways(.75,1000);
            sideways(0.5,1500);
            claw.setPower(1);
            sleep(1000);
            sideways(-0.5,3000);
            claw.setPower(-1);
            sleep(500);
            claw.setPower(0); 
            sleep(500);
            drive(0.5,500);
            sleep(1000);
            claw.setPower(1);
            sleep(750);
            drive(-0.25,500);
            sleep(500);
            drive(0.5,750);
            sleep(500);
            claw.setPower(-1);
        
            /*drive(0.25,500);
            sleep(500);
            sideways(0.5,250);
            sleep(500);
            claw.setPower(1);
            sleep(500);
            sideways(-0.5,2000);
            sleep(250);
            drive(0.25,2000);
            claw.setPower(0);          
        */
        }
    }
    
    public void drive(double power,int time) { 
        BLM.setPower(power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(power);
        sleep(time);
        Stop();
        
    }
    public void sideways(double power,int time){
        BLM.setPower(-power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(-power);
        sleep(time);
        Stop();
        
    }
    
    public void diagonalFR(double power,int time){
        BRM.setPower(power);
        FLM.setPower(power);
        sleep(time);
        Stop();
        
        

    }
    
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
    public void encoderDrive (double power, int inches){
        int currentPosFRM = 0;
        int currentPosBLM = 0;
        int currentPosFLM = 0;
        int currentPosBRM = 0;
        int target = inches*60;
        if (opModeIsActive()){
            FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            FRM.setTargetPosition(target);
            FLM.setTargetPosition(target);
            
            
            BRM.setTargetPosition(target);
            BLM.setTargetPosition(target);
            
            FRM.setPower(power);
            FLM.setPower(power);
            BRM.setPower(power);
            BLM.setPower(power);
            
            if (power > 0){
                while (opModeIsActive() && (currentPosFLM < target && currentPosFRM < target && 
                currentPosBLM < target && currentPosBRM < target)){
                    currentPosFLM = FLM.getCurrentPosition();
                    currentPosFRM = FRM.getCurrentPosition();
                    currentPosBLM = BLM.getCurrentPosition();
                    currentPosBRM = BRM.getCurrentPosition();
                    
                    telemetry.addData("Running to: ", target);
                    telemetry.addData("Running at front left: ", currentPosFLM);
                    telemetry.addData("Running at front right: ", currentPosFRM);
                    telemetry.addData("Running at back left: ", currentPosBLM);
                    telemetry.addData("Running at back right: ", currentPosBRM);
                    telemetry.update();
            
                }
            }
            else {
                while (opModeIsActive() && (currentPosFLM > target && currentPosFRM > target && 
                currentPosBLM > target && currentPosBRM > target)){
                    currentPosFLM = FLM.getCurrentPosition();
                    currentPosFRM = FRM.getCurrentPosition();
                    currentPosBLM = BLM.getCurrentPosition();
                    currentPosBRM = BRM.getCurrentPosition();
                    
                    telemetry.addData("Running to: ", target);
                    telemetry.addData("Running at front left: ", currentPosFLM);
                    telemetry.addData("Running at front right: ", currentPosFRM);
                    telemetry.addData("Running at back left: ", currentPosBLM);
                    telemetry.addData("Running at back right: ", currentPosBRM);
                    telemetry.update();
                }
                
            }
            Stop();
            
            
            FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
    }
    
    
        
    

    
    
}
