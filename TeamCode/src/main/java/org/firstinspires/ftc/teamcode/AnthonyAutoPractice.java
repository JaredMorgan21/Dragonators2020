package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "AnthonyAutoPractice", group = "practice")

public class AnthonyAutoPractice extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor BLM;
    private DcMotor BRM;
    private DcMotor FLM;
    private DcMotor FRM;
    private Blinker expansion_Hub_2;
    private Servo claw;
   
    
   
   
    public void runOpMode() {
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        claw = hardwareMap.servo.get("claw");
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
          /*
            BLM.setPower(1);
            BRM.setPower(1);
            FLM.setPower(1);
            FRM.setPower(1);
            sleep(1000);
            BLM.setPower(0);
            BRM.setPower(0);
            FLM.setPower(0);
            FRM.setPower(0);
           */
            drive(0.5,500);
            drive(-0.5,500);
            sideways(1,5000);
            sideways(-1,5000);
            diaginalFR(0.5,500);
            diaginalFL(0.5,500);
            claw.setPosition(1);
            sleep(500);
            claw.setPosition(0);
            /*BRM.setPower(0.5);
            sleep(1000);
            BRM.setPower(0);
            BLM.setPower(0.5);
            sleep(1000);
            BLM.setPower(0);
            FRM.setPower(0.5);
            sleep(1000);
            FRM.setPower(0);
            FLM.setPower(0.5);
            sleep(1000);
            FLM.setPower(0);
        */
        } 
        
        
    } 
    
    public void drive(double power,int time) {
        BLM.setPower(power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(power);
        sleep(time);
        BLM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        FRM.setPower(0);
    }
    
    public void sideways(double power, int time) {
        BLM.setPower(-power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(-power);
        sleep(time);
        BLM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        FRM.setPower(0);
    }
    
    public void diaginalFR(double power, int time) {
        BRM.setPower(power);
        FLM.setPower(power);
        sleep(time);
        BRM.setPower(0);
        FLM.setPower(0);
    }
    
     public void diaginalFL(double power, int time) {
        BLM.setPower(power);
        FRM.setPower(power);
        sleep(time);
        BLM.setPower(0);
        FRM.setPower(0);
     }
     
     public void turn(double power, int time) {
        BLM.setPower(power);
        BRM.setPower(-power);
        FLM.setPower(power);
        FRM.setPower(-power);
        sleep(time);
        BLM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        FRM.setPower(0);
    }
}