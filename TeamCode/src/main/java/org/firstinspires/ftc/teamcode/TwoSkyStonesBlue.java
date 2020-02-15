package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@Autonomous(name = "BlueTwoSkystones", group =  "blue full")


public class TwoSkyStonesBlue extends LinearOpMode {
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
    private Servo claw;
    private Orientation prevAngles = new Orientation();
    double globalAngle;
    double currentAngle;
    BNO055IMU Imu;
    private Servo push;
    private ElapsedTime runtime = new ElapsedTime();
    private TouchSensor touch;

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
        claw = hardwareMap.servo.get("claw");
        push = hardwareMap.servo.get("push");

        //TouchSensor touch
        touch = hardwareMap.touchSensor.get("touch");

        // setting parameters for gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // setting to run as an inertial measurement unit
        parameters.mode = BNO055IMU.SensorMode.IMU;

        // set to meausure in degrees
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // getting imu name from phone
        Imu = hardwareMap.get(BNO055IMU.class, "imu");
        Imu.initialize(parameters);
        telemetry.addData("Mode", "Calibrating...");
        telemetry.update();


        // make sure the imu gryo is calibrated before continuing
        while (!isStopRequested() && !Imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration status", Imu.getCalibrationStatus().toString());
        telemetry.update();


        waitForStart();
        if (opModeIsActive()) {
            sideways(0.5, 1250);

        }

    }
    //Functions save us time and reduces space in the main code.

    // This function sets the power for all the motors to the same value.
    public void drive(double power, int time) {
        BLM.setPower(power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(power);
        sleep(time);
        Stop();

    }
    // For sideways motion. Negative power reverses the power for the motor.

    public void sideways(double power, int time) {
        BLM.setPower(-power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(-power);
        sleep(time);
        Stop();

    }

    //positive is for North east diagonal motion, negative is for South West diagonal motion.
    public void diagonalFR(double power, int time) {
        BRM.setPower(power);
        FLM.setPower(power);
        sleep(time);
        Stop();


    }

    // positive is for North West diagonal motion, negative is for South East diaginal motion.
    public void diagonalFL(double power, int time) {
        BLM.setPower(power);
        FRM.setPower(power);
        sleep(time);
        Stop();

    }

    // positive is for clockwise motion, negative is counter-clockwise
    public void turn(double power, int time) {
        BLM.setPower(power);
        BRM.setPower(-power);
        FLM.setPower(power);
        FRM.setPower(-power);
        sleep(time);
        Stop();

    }

    public void Stop() {
        BLM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        FRM.setPower(0);
    }

    public void GyroTurn(double power, int degrees) {
        double currentAng = 0;
        resetAngle();
        //if the power is greater than zero and the current angle is less than we want it to turn to
        //turn right and tell the phone where we are going and where we are at
        if (power > 0) {
            //currentAng is negative because the IMU has clockwise as negative and we want it to be positive
            while (opModeIsActive() && -currentAng < degrees) {
                //updates current angle
                currentAng = getAngle();
                BLM.setPower(power);
                BRM.setPower(-power);
                FLM.setPower(power);
                FRM.setPower(-power);

                telemetry.addData("Angle: ", -currentAng);
                telemetry.addData("Target: ", degrees);
                telemetry.update();
            }
        }
        //if the power is less than zero and the current angle is greater than we want it to turn to
        //turn left and tell the phone where we are going and where we are at
        else {
            //currentAng is negative because the IMU has counter-clockwise as positive and we want it to be negative
            while (opModeIsActive() && -getAngle() > -degrees) {
                //updates current angle
                currentAng = getAngle();
                BLM.setPower(power);
                BRM.setPower(-power);
                FLM.setPower(power);
                FRM.setPower(-power);

                telemetry.addData("Angle: ", -currentAng);
                telemetry.addData("Target: ", -degrees);
                telemetry.update();
            }
        }
        Stop();
        sleep(100);
    }

    public void resetAngle() {
        //stores where the robot is facing as where the robot was previously facing
        prevAngles = Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // set where the robot is facing as 0
        globalAngle = 0;

    }

    public double getAngle() {
        //set current angles to where the robot is facing
        Orientation currAngles = Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //angle change is the difference between where the robot is facing currently and where it used to face
        double angleChange = currAngles.firstAngle - prevAngles.firstAngle;
        //turning more than 180 degrees in either direction is the equivelant of turning
        //360 - the turn in the other direction
        if (angleChange < -180)
            angleChange += 360;
        else if (angleChange > 180)
            angleChange -= 360;

        //add the turn amount to the global robot orientation
        globalAngle += angleChange;

        //set previous orientation to current orientation
        prevAngles = currAngles;
        //get angle getAngle()=globalAngle
        return globalAngle;
    }

    public void encoderDrive(double power, int inches) {
        int currentPosFRM = 0;
        int currentPosBLM = 0;
        int currentPosFLM = 0;
        int currentPosBRM = 0;
        int target = inches * 60;
        if (opModeIsActive()) {
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

            if (power > 0) {
                while (opModeIsActive() && (currentPosFLM < target && currentPosFRM < target &&
                        currentPosBLM < target && currentPosBRM < target)) {
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
            } else {
                while (opModeIsActive() && (currentPosFLM > target && currentPosFRM > target &&
                        currentPosBLM > target && currentPosBRM > target)) {
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