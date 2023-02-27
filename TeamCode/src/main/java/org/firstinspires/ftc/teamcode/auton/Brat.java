package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Encoder;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="test brat", group="Autonomy")

public class Brat extends LinearOpMode {

    private  Hardware hardware;
    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime elapsedTime;

        hardware = new Hardware(hardwareMap, true);
        motor = hardwareMap.get(DcMotor.class, "m2");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("test", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
