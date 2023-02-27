package org.firstinspires.ftc.teamcode;

import android.widget.GridLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Specifications;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="MainTeleop")
public class Teleop extends OpMode {

    private boolean clawState = false;
    private boolean forebarState = false;
    private boolean xState = false;
    private boolean _180State = false;
    private boolean smState = true;
    private boolean cremState = false;

    private int opState = 0;
    private boolean systemRunningUp = false;
    private boolean systemRunningDown = false;
    private boolean goHigh = false;
    private boolean goMid = false;

    private Hardware hardware;

    private ElapsedTime elapsedTime;

    private double lastTimeY;
    private double lastTimeA;
    private double lastTimeB;
    private double lastTimeX;
    private double lastTimeDL;
    private double lastTimeDR;

    private double forebarDelay = 0;
    private double _180Delay = 0;
    private double _180FinishMovementDelay = 0;
    private double servoFinishForebarMovementDelay = 0;
   // private double motorFinishRaiseDelay = 0;

    private boolean resetY = false;
    private boolean resetA = false;
    private boolean resetB = false;
    private boolean resetX = false;
    private boolean resetDL = false;
    private boolean resetDR = false;

    @Override
    public void init() {

        hardware = new Hardware(hardwareMap, false);
        elapsedTime = new ElapsedTime();


    }

    @Override
    public void loop() {

        float forward = gamepad1.left_stick_y; // forward movement
        float strafe = -gamepad1.left_stick_x; // strafe movement
        float rotation = gamepad1.right_stick_x; // rotate
        boolean slowdown = gamepad1.right_bumper;

        float upGlis = gamepad2.right_trigger;
        float downGlis = gamepad2.left_trigger;
        boolean startAutoUpHigh = gamepad2.y;
        boolean startAutoUpMid = gamepad1.b;
        boolean startAutoDown = gamepad2.x;
        // boolean opCremMovement = gamepad2.b;
        boolean cremFwd = gamepad2.left_bumper;
        boolean cremRev = gamepad2.right_bumper;
        boolean servoMoveUp = gamepad2.dpad_up;
        boolean servoMoveDown = gamepad2.dpad_down;
        boolean forceStopSystem = gamepad2.b;

        if (resetY && elapsedTime.milliseconds() > lastTimeY + Specifications.button_delay) resetY = false;

        if (resetA && elapsedTime.milliseconds() > lastTimeA + Specifications.button_delay) resetA = false;

        if (resetB && elapsedTime.milliseconds() > lastTimeB + Specifications.button_delay) resetB = false;

        if(resetDL && elapsedTime.milliseconds() > lastTimeDL + Specifications.button_delay) resetDL = false;

        if(resetX && elapsedTime.milliseconds() > lastTimeX + Specifications.button_delay) resetX = false;

        if(resetDR && elapsedTime.milliseconds() > lastTimeDR + Specifications.button_delay) resetDR = false;

        movement(forward, strafe, rotation, slowdown);

        stop(forceStopSystem);
        autoUp(startAutoUpHigh, Specifications.glis_highj_ticks);
//        if(!systemRunningUp || goHigh){
//            autoUp(startAutoUpHigh, Specifications.glis_highj_ticks);
//        }
//        if(!systemRunningUp || goMid){
//            autoUp(startAutoUpMid, Specifications.glis_midj_ticks);
//        }
        autoDown(startAutoDown);


        if((hardware.mGlisRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                Math.abs(hardware.mGlisRight.getCurrentPosition() - hardware.mGlisRight.getTargetPosition()) < 10 &&
                Math.abs(hardware.mGlisLeft.getCurrentPosition() - hardware.mGlisLeft.getTargetPosition()) < 10)
                || hardware.mGlisLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER && !systemRunningUp && !systemRunningDown)
            glisOperation(downGlis, upGlis);

        clawOperation(gamepad2.a && !gamepad2.right_bumper);

        if(gamepad2.left_bumper && gamepad2.a)
        {
            hardware.mGlisLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.mGlisRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.mGlisLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.mGlisRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if(!systemRunningUp && !systemRunningDown) {
            forebarOperation(gamepad2.dpad_left);

               cremaliera(cremFwd, cremRev);

            if((servoFinishForebarMovementDelay < elapsedTime.milliseconds() - 1000)
                    && (_180FinishMovementDelay < elapsedTime.milliseconds() - 1000))
            {
                _180Operation(gamepad2.dpad_right);
                servoMovementOperation(servoMoveUp, servoMoveDown);
            }
        }
        //  if (gamepad2.b && !resetB) goToPos(Specifications.low_ticks);

        telemetry.addData("glis left", hardware.mGlisLeft.getCurrentPosition());
        telemetry.addData("glis right", hardware.mGlisRight.getCurrentPosition());
        telemetry.addData("servo180", hardware.servo180.getPosition());
        telemetry.addData("cremaliera",hardware.mCremaliera.getCurrentPosition());
        telemetry.addData("_180FinishMovementDelay", _180FinishMovementDelay);
        telemetry.addData("fata",hardware.upBorderMovement.getState());
        telemetry.addData("spate", hardware.downBorderMovement.getState());
        telemetry.update();

    }

    void glisOperation(float downGlis, float upGlis)
    {
        if(hardware.mGlisLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        {
            hardware.mGlisLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.mGlisRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (upGlis == 0 && downGlis == 0) {
            hardware.mGlisLeft.setPower(0);
            hardware.mGlisRight.setPower(0);

        } else if (upGlis != 0) {
            float power = Math.max(Specifications.glis_min_speed, Math.min(1, upGlis * Specifications.glis_speed_multiplier));
            hardware.mGlisLeft.setPower(power);
            hardware.mGlisRight.setPower(-power);

        }else {
            float power = -Math.max(Specifications.glis_min_speed, Math.min(1, downGlis * Specifications.glis_speed_multiplier));

            hardware.mGlisLeft.setPower(power);
            hardware.mGlisRight.setPower(-power);
        }
    }

    private void movement(float forward, float strafe, float rotation, boolean slowdown) {
        // power applied to the robot wheel by wheel
        double[] power = new double[4];
        rotation *= -1;

        float speedMulti = 1;
        if(slowdown) speedMulti = 0.65f;

        power[0] = (-forward + strafe + rotation) * Specifications.moving_speed_teleop * speedMulti;   //+
        power[1] = (+forward + strafe + rotation) * Specifications.moving_speed_teleop * speedMulti;   //-
        power[2] = (-forward - strafe + rotation) * Specifications.moving_speed_teleop * speedMulti;   //-
        power[3] = (+forward - strafe + rotation) * Specifications.moving_speed_teleop * speedMulti;   //+

        // applying the power
        for (int i = 0; i < 4; i++) {
            hardware.motor[i].setPower(power[i]);
        }
    }

    private void cremaliera(boolean cremFwd, boolean cremRev)
    {
        if(hardware.mCremaliera.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        {
            hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (cremFwd) {
            hardware.mCremaliera.setPower(Specifications.crem_speed);
        } else if (cremRev) {
            hardware.mCremaliera.setPower(-Specifications.crem_speed);
        } else {
            hardware.mCremaliera.setPower(0);
        }
    }

    private void clawOperation(boolean button) {
        if (!resetA && button) {
            if (clawState) hardware.servoClaw.setPosition(Specifications.servo_claw_pos_up);

            else hardware.servoClaw.setPosition(Specifications.servo_claw_pos_down);

            resetA = true;
            clawState = !clawState;
            lastTimeA = elapsedTime.milliseconds();
        }
    }

    private void forebarOperation(boolean button)
    {
        if(!resetDL && button)
        {
            if(!forebarState) hardware.servoForebar.setPosition(Specifications.servo_forebar_pos_up);
            else hardware.servoForebar.setPosition(Specifications.servo_forebar_pos_down);

            resetDL = true;
            forebarState = !forebarState;
            lastTimeDL = elapsedTime.milliseconds();

        }
    }

    private void servoMovementOperation(boolean up, boolean down)
    {
        if(up)
        {
            hardware.servoMovement.setPower(-Specifications.servo_movement_speed);
        }
        else if(down)
        {
            hardware.servoMovement.setPower(Specifications.servo_movement_speed);
        }
        else
        {
            hardware.servoMovement.setPower(0);
        }
    }

    private void goToPos(int posTicks)
    {
        hardware.mGlisRight.setTargetPosition(posTicks);
        hardware.mGlisLeft.setTargetPosition(-posTicks);

        hardware.mGlisRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.mGlisLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.mGlisLeft.setPower(Specifications.glis_speed_multiplier);
        hardware.mGlisRight.setPower(-Specifications.glis_speed_multiplier);


    }

    private void _180Operation(boolean button)
    {
        if(!resetDR && button)
        {
            if(_180State) hardware.servo180.setPosition(Specifications.servo_180_pos_up);
            else hardware.servo180.setPosition(Specifications.servo_180_pos_down);

            resetDR = true;
            _180State = !_180State;
            lastTimeDR = elapsedTime.milliseconds();
        }
    }

    private void _180OperationAutomatedUp(boolean state, boolean moveForebar)
    {

        if(state) hardware.servo180.setPosition(Specifications.servo_180_pos_down);
        else hardware.servo180.setPosition(Specifications.servo_180_pos_up);

        if(moveForebar) forebarDelay = elapsedTime.milliseconds();
        else systemRunningUp = false;


    }

    private void _180OperationAutomatedDown(boolean state)
    {
        if(state) hardware.servo180.setPosition(Specifications.servo_180_pos_up);
        _180FinishMovementDelay = elapsedTime.milliseconds();


    }

    private void forebarOperationAutomatedUp(boolean state)
    {
        if(state) hardware.servoForebar.setPosition(Specifications.servo_forebar_pos_up);
        servoFinishForebarMovementDelay = elapsedTime.milliseconds();
    }

    private void forebarOperationAutomatedDown(boolean state, boolean move180)
    {

        if(state) hardware.servoForebar.setPosition(Specifications.servo_forebar_pos_up);
        else hardware.servoForebar.setPosition(Specifications.servo_forebar_pos_down);

        if(move180) _180Delay = elapsedTime.milliseconds();
        else systemRunningDown = false;

    }

    private void autoUp(boolean button, int posTicks){
        if(!resetY && button && forebarDelay == 0){
            goToPos(Specifications.glis_180_space_ticks);
//            else{
//                hardware.servoMovement.setPower(-Specifications.servo_movement_speed);
//                hardware.mCremaliera.setTargetPosition(Specifications.upper_limit_crem_ticks);
//            }
//            if(posTicks==Specifications.glis_midj_ticks){
//                goMid = true;
//            }else if(posTicks==Specifications.glis_highj_ticks){
//                goHigh = true;
//            }
            smState = true;
            systemRunningUp = true;
            resetY =true;
            lastTimeY = elapsedTime.milliseconds();
        }



        if(systemRunningUp && forebarDelay == 0)
        {
            if(smState && getDiffTargeCurrentGlisLeft() < 100 && getDiffTargeCurrentGlisRight() < 100)
            {
                goToPos(posTicks);
                hardware.servoMovement.setPower(Specifications.servo_movement_speed);
                hardware.mCremaliera.setTargetPosition(Specifications.lower_limit_crem_ticks);
                hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.mCremaliera.setPower(Specifications.crem_speed_auto);

            }

        }
        if(systemRunningUp && smState && !hardware.downBorderMovement.getState() && forebarDelay == 0){
            hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.mCremaliera.setPower(0);
            hardware.servoMovement.setPower(0);
            _180OperationAutomatedUp(true, true);
        }

        if(systemRunningUp && forebarDelay != 0 && forebarDelay < elapsedTime.milliseconds() - 500)
        {

            systemRunningUp = false;
           // goMid = false;
           // goHigh = false;
            forebarDelay = 0;
            forebarOperationAutomatedUp(true);
        }
    }

    private void autoDown(boolean button){
        if(!resetX && button && _180Delay == 0){
            goToPos(Specifications.glis_180_space_ticks);
            forebarOperationAutomatedDown(false, true);
            systemRunningDown = true;
            resetX =false;
            smState=false;
            lastTimeX = elapsedTime.milliseconds();
        }

        if(systemRunningDown && _180Delay != 0 && _180Delay < elapsedTime.milliseconds() - 200)
        {
            _180OperationAutomatedDown(true);
            _180Delay = 0;

        }

        if(systemRunningDown)
        {
            if(!smState && getDiffTargeCurrentGlisLeft() < 300 && getDiffTargeCurrentGlisRight() < 300
                    && _180FinishMovementDelay < elapsedTime.milliseconds() - 3000)
            {
                hardware.servoMovement.setPower(-Specifications.servo_movement_speed);
                hardware.mCremaliera.setTargetPosition(Specifications.upper_limit_crem_ticks-30);
                hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.mCremaliera.setPower(Specifications.crem_speed_auto);
            }
            if(!smState && !hardware.upBorderMovement.getState() && _180Delay == 0)
            {
                hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.mCremaliera.setPower(0);
                hardware.servoMovement.setPower(0);
                systemRunningDown = false;
                goToPos(1500);
                forebarDelay = 0;
            }

        }

    }

    private void setCramHome()
    {
        hardware.mCremaliera.setTargetPosition(0);
        hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private int getDiffTargeCurrentGlisLeft()
    {
        return Math.abs(hardware.mGlisLeft.getCurrentPosition() - hardware.mGlisLeft.getTargetPosition());
    }

    private int getDiffTargeCurrentGlisRight()
    {
        return Math.abs(hardware.mGlisRight.getCurrentPosition() - hardware.mGlisRight.getTargetPosition());
    }

    private void stop(boolean forceStop){
        if(forceStop && (systemRunningUp || systemRunningDown)){
            systemRunningDown = false;
            systemRunningUp = false;
           // goMid = false;
           // goHigh = false;
            hardware.mGlisRight.setPower(0);
            hardware.mGlisLeft.setPower(0);
            hardware.mCremaliera.setPower(0);
            hardware.servoMovement.setPower(0);
            hardware.servo180.setPosition(Specifications.servo_180_pos_up);
        }
    }
}
/*
gr     gl
7702   -7654
11400  -11000



 */