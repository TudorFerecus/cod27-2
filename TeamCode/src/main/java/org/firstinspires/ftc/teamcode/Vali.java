package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Vali", group="Linear Opmode")
public class Vali extends LinearOpMode {

    private Servo servoLeft;
    private Servo servoRight;

    private DcMotor mLeft;
    private DcMotor mRight;

    private float posL = 0;
    private float posR = 0;
    private float power = 0.8f;

    boolean openMouth = false;

    void close()
    {
        servoLeft.setPosition(.05f);
        servoRight.setPosition(-.57f);
    }

    void open()
    {
        servoLeft.setPosition(0.4f);
        servoRight.setPosition(-.22f);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        mLeft = hardwareMap.get(DcMotor.class, "mLeft");
        mRight = hardwareMap.get(DcMotor.class, "mRight");

        mRight.setDirection(DcMotorSimple.Direction.REVERSE);
        servoLeft.setDirection(Servo.Direction.REVERSE);



        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.left_bumper)
            {
                close();
            }
            if(gamepad1.right_bumper)
            {
                open();
            }

            mRight.setPower(gamepad1.left_stick_x * power+gamepad1.left_stick_y * power);
            mLeft.setPower(gamepad1.left_stick_x * power-gamepad1.left_stick_y * power);


            telemetry.addData("ST" ,servoLeft.getPosition());
            telemetry.addData("DR" ,servoRight.getPosition());

            telemetry.update();

        }

    }
}
