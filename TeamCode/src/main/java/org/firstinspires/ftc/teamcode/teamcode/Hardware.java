package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    public boolean isAuto = false;
    public DcMotor[] motor = new DcMotor[4]; // motoarele rotilor

    public DcMotor mCremaliera;
    public DcMotorEx mGlisLeft, mGlisRight;

    // servo uri
    public Servo servoClaw, servo180, servoForebar;

    public CRServo servoMovement;

    //senzori
    public DigitalChannel upBorderMovement, downBorderMovement, sGlisLeft, sGlisRight;


    // parametrii imu
    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters;
    private HardwareMap hardwareMap = null;


    public ElapsedTime runTime = new ElapsedTime();

    public Hardware(HardwareMap hdMap, boolean auto) {
        this.isAuto = auto;
        initializare(hdMap);
    }

    private void initializare(HardwareMap hdMap) {
        this.hardwareMap = hdMap;

        initMotors();
        initServos();
        initImu();
        initSensors();
    }

    private void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    private DcMotor setDefaultStateMotor(DcMotor motor, String nume, DcMotorSimple.Direction direction) {
        motor = hardwareMap.get(DcMotor.class, nume);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        return motor;
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private void initMotors() {
        motor[0] = setDefaultStateMotor(motor[0], "mFrontRight", DcMotorSimple.Direction.FORWARD); //0
        motor[1] = setDefaultStateMotor(motor[1], "mFrontLeft", DcMotorSimple.Direction.FORWARD); //1
        motor[2] = setDefaultStateMotor(motor[2], "mBackRight", DcMotorSimple.Direction.FORWARD); //2
        motor[3] = setDefaultStateMotor(motor[3], "mBackLeft", DcMotorSimple.Direction.FORWARD); //3

        mGlisLeft = configureMotor("mGlisLeft", DcMotorSimple.Direction.FORWARD); //2
        mGlisRight = configureMotor("mGlisRight", DcMotorSimple.Direction.FORWARD); //3

        mCremaliera = setDefaultStateMotor(mCremaliera, "mCremaliera", DcMotorSimple.Direction.REVERSE); //0
        mCremaliera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mCremaliera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotorEx configureMotor(String motorName, DcMotorSimple.Direction direction)
    {
        DcMotorEx motorObj = hardwareMap.get(DcMotorEx.class, motorName);
        motorObj.setDirection(direction);
       // motorObj.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Specifications.newPIDF);
        motorObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motorObj;
    }

    private void initServos() {
        servoClaw = hardwareMap.get(Servo.class, "servoClaw"); //5
        servo180 = hardwareMap.get(Servo.class, "servo180"); //1
        servoMovement = hardwareMap.get(CRServo.class, "servoMovement"); //4
        servoForebar = hardwareMap.get(Servo.class, "servoForebar"); //0
    }

    private void initSensors(){
        upBorderMovement = hardwareMap.get(DigitalChannel.class, "upBorderMovement"); //3
        downBorderMovement = hardwareMap.get(DigitalChannel.class, "downBorderMovement"); //1
        sGlisLeft = hardwareMap.get(DigitalChannel.class, "sGlisLeft"); //1
        sGlisRight = hardwareMap.get(DigitalChannel.class, "sGlisRight"); //3
    }
}