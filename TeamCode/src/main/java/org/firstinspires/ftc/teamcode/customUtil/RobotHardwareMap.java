package org.firstinspires.ftc.teamcode.customUtil;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RobotHardwareMap {
    //Create Motors
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor backLeftMotor = null;

    public DcMotor slideMotor = null;
    public DcMotor lazySusanSpinner = null;

//    public RevBlinkinLedDriver ledDriver = null;



//    public DistanceSensor slideHeightSensor = null;
//    public DistanceSensor coneSensorDistance = null;
//    public ColorSensor coneSensorColor = null;

    public Servo clawServo = null;


    public BNO055IMU imu = null;

    //Additional Variable
    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public RobotHardwareMap() {

    }

    public void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;

        //Connect Motor
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");

        slideMotor = hwMap.get(DcMotor.class, "slideMotor");
        lazySusanSpinner = hwMap.get(DcMotor.class, "lazySusanSpinner");

//        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");


        clawServo = hwMap.get(Servo.class, "clawServo");


//        coneSensorColor = hwMap.get(ColorSensor.class, "coneSensor");
//
//        coneSensorDistance = hwMap.get(DistanceSensor.class, "coneSensor");
//
//        slideHeightSensor = hwMap.get(DistanceSensor.class, "slideHeightSensor");

        //Set Up Motor Direction
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        lazySusanSpinner.setDirection(DcMotor.Direction.FORWARD);

        clawServo.setDirection(Servo.Direction.FORWARD);

        //Set ZERO POWER BEHAVIOR
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lazySusanSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motors to Use No Power
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        slideMotor.setPower(0);

        lazySusanSpinner.setPower(0);


        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.8);

        lazySusanSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lazySusanSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lazySusanSpinner.setTargetPosition(0);
        lazySusanSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lazySusanSpinner.setPower(0.8);











        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //return value of radians
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //gets imu from rev hardware map and connects it to code
        imu = hwMap.get(BNO055IMU.class, "imu");
        //sets the settings we declared above.
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


    }


}