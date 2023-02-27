//package org.firstinspires.ftc.teamcode.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.util.Encoder;
//
//@Disabled
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//public class Test extends LinearOpMode {
//
//    private Encoder parallelEncoder;
//    private Encoder perpendicularEncoder;
//
//    //Hardware class instance
//    private Hardware hardware;
//
//
//    private void glisMovement(float upArm, float downArm)
//    {
//        if(upArm != 0)
//        {
//            hardware.mGlisRight.setPower(upArm * Specifications.arm_moving_speed_up);
//            hardware.mGlisLeft.setPower(upArm * Specifications.arm_moving_speed_up);
//        }
//        else if(downArm != 0)
//        {
//            hardware.mGlisRight.setPower(-downArm * Specifications.arm_moving_speed_down);
//            hardware.mGlisLeft.setPower(-downArm * Specifications.arm_moving_speed_down);
//        }
//        else
//        {
//
//            hardware.mGlisRight.setPower(0);
//            hardware.mGlisLeft.setPower(0);
//        }
//    }
//
//    private void shoulderMovement(boolean upShoulder, boolean downShoulder)
//    {
//        if(upShoulder)
//        {
//            hardware.m180.setPower(Specifications.shoudler_rotation_speed);
//        }
//        else if(downShoulder)
//        {
//            hardware.m180.setPower(-Specifications.shoudler_rotation_speed);
//        }
//        else
//        {
//
//            hardware.m180.setPower(0f);
//        }
//    }
//    private void coneGrab(float grab, float release)
//    {
//        if(grab != 0)
//        {
//            hardware.servoClaw.setPower(grab * Specifications.cone_grab_speed);
//        }
//        else if(release != 0)
//        {
//            hardware.servoClaw.setPower(-release * Specifications.cone_grab_speed);
//        }
//        else
//        {
//            hardware.servoClaw.setPower(0);
//        }
//    }
//
//
//    private void handMovement(boolean upHand, boolean downHand)
//    {
//        if(upHand)
//        {
//                hardware.servoArm.setPower(Specifications.servo_multiplier);
//        }
//        else if(downHand)
//        {
//            hardware.servoArm.setPower(-Specifications.servo_multiplier);
//        }
//        else
//        {
//            hardware.servoArm.setPower(0);
//        }
//    }
//
//
//    private void movement(float forward, float strafe, float rotation)
//    {
//        // power applied to the robot wheel by wheel
//        double[] power = new double[4];
//        rotation *= -1;
//        power[0] = (-forward + strafe + rotation) * Specifications.moving_speed_teleop;   //+
//        power[1] = (+forward + strafe + rotation) * Specifications.moving_speed_teleop;   //-
//        power[2] = (-forward - strafe + rotation) * Specifications.moving_speed_teleop;   //-
//        power[3] = (+forward - strafe + rotation) * Specifications.moving_speed_teleop;   //+
//
//        // applying the power
//        for(int i=0; i<4; i++)
//        {
//            hardware.motor[i].setPower(power[i]);
//        }
//    }
//
//    @Override
//    public void runOpMode()
//    {
//        hardware = new Hardware(hardwareMap, false);
//
//        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "m3"));
//        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "m2"));
//
//        waitForStart();
//
//        while(opModeIsActive())
//        {
//            float upArm = gamepad2.left_stick_y;
//            float downArm = gamepad2.left_stick_y;
//
//            boolean upShoulder = gamepad2.dpad_up;
//            boolean downShoulder = gamepad2.dpad_down;
//
//
//            float grabCone = gamepad2.right_trigger;
//            float releaseCone = gamepad2.left_trigger;
//
//            boolean rotateArmUp = gamepad2.right_bumper;
//            boolean rotateArmDown = gamepad2.left_bumper;
//
//            float fata = gamepad1.left_stick_y; // forward movement
//            float lateral = gamepad1.left_stick_x; // strafe movement
//            float rotire = gamepad1.right_stick_x; // rotate
//
//            shoulderMovement(upShoulder, downShoulder);
//            coneGrab(grabCone, releaseCone);
//            glisMovement(upArm, downArm);
//            handMovement(rotateArmUp, rotateArmDown);
//
//            movement(fata, lateral, rotire);
//
//            telemetry.addData("paralel: ", parallelEncoder.getCurrentPosition());
//            telemetry.addData("perpendicular: ", perpendicularEncoder.getCurrentPosition());
//
//            telemetry.update();
//
//        }
//
//    }
//}