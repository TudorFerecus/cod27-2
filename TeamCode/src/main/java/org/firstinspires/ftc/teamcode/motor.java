package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="motor", group="Linear Opmode")

public class motor extends LinearOpMode {

    private Servo s;


    @Override
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.get(Servo.class, "s");
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.left_bumper) s.setPosition(1);
            if(gamepad1.right_bumper) s.setPosition(0);
        }
    }
}