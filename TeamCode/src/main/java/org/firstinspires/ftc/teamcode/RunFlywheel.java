package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Run flywheel", group =  "test")


public class RunFlywheel extends LinearOpMode {
//declaring motor variables, BLM = Back left motor 
    private Gyroscope imu;
    private DcMotor BLM;
    private DcMotor BRM;
    private DcMotor FLM;
    private DcMotor FRM;
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_3;
    private DcMotor flywheelL;
    private DcMotor flywheelR;
    private CRServo claw;
    private Orientation prevAngles = new Orientation();
    double globalAngle;
    double currentAngle;
    BNO055IMU Imu;
    
    @Override
    public void runOpMode() {
        //Getting Motor Name From Phone
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        flywheelL = hardwareMap.dcMotor.get("flywheelL");
        flywheelR = hardwareMap.dcMotor.get("flywheelR");
        
        // showing which direction the robot moves 
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //setting run mode
        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // getting claw from phone
        claw = hardwareMap.crservo.get("claw");
        
        //Main program, runs robot
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()){
                flywheelL.setPower(-1);
                flywheelR.setPower(1);
            }
        
        }
        
    }
  
}    