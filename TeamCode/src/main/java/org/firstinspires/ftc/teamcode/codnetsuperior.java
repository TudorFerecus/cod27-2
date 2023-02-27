package org.firstinspires.ftc.teamcode;

import android.widget.GridLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Specifications;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test573", group="Test")
public class codnetsuperior extends OpMode {

    private Hardware hardware;

    private ElapsedTime elapsedTime;

    private int glisStationaryTicks = 0;
    private int glisRightOffset = 0;
    private int glisLeftOffset = 0;

    private boolean clawState = false;
    private boolean forebarState = false;
    private boolean _180State = false;
    boolean finishedMove = false;

   // private boolean systemRunningUp = false;

   // private boolean systemRunningDown = false;

    private double lastTimeY;
    private double lastTimeA;
    private double lastTimeB;
    private double lastTimeX;
    private double lastTimeDL;
    private double lastTimeDR;
    private double forebar_delay;
    private double _180_delay;

    private boolean resetY = false;
    private boolean resetA = false;
    private boolean resetB = false;
    private boolean resetX = false;
    private boolean resetDL = false;
    private boolean resetDR = false;

    private enum STATE_MACHINE
    {
        DRIVER_CONTROLLED,
        AUTOMID,
        AUTOHIGH,
        AUTODOWN
    }
    private STATE_MACHINE currentState = STATE_MACHINE.DRIVER_CONTROLLED;

    private enum MOVEMENT_STATE
    {
        UP1,
        UP2,
        UP3,
        DOWN1,
        DOWN2,
        DOWN3,
        FINISH
    }
    private MOVEMENT_STATE currentMovement = MOVEMENT_STATE.UP1;

    @Override
    public void init() {

        hardware = new Hardware(hardwareMap, false);
        elapsedTime = new ElapsedTime();

    }

    @Override
    public void loop() {

        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float rotation = gamepad1.right_stick_x;
        boolean slowdown = gamepad1.right_bumper;

        float glis_up = gamepad2.left_trigger;
        float glis_down = gamepad2.right_trigger;

        boolean crem_fwd = gamepad2.left_bumper;
        boolean crem_rev = gamepad2.right_bumper;

        boolean servo_move_up = gamepad2.dpad_up;
        boolean servo_move_down = gamepad2.dpad_down;

        boolean servo_fb = gamepad2.dpad_right;
        boolean servo_180 = gamepad2.dpad_left;
        boolean servo_claw = gamepad2.a;

        boolean force_stop_state_machine = gamepad1.x;
        boolean force_stop_glis = gamepad1.b;
        boolean force_stop_crem = gamepad1.y;

        timersControl();

        forceStop(force_stop_state_machine, force_stop_glis, force_stop_crem);
        movement(forward, strafe, rotation, slowdown);

        telemetry.addData("RunMode", hardware.mGlisRight.getMode());
        telemetry.addData("RIGHTPOS", hardware.mGlisRight.getCurrentPosition());
        telemetry.addData("LEFTTPOS", hardware.mGlisLeft.getCurrentPosition());
        telemetry.addData("crempos" , hardware.mCremaliera.getCurrentPosition());
        telemetry.addData("state", currentMovement);
        telemetry.addData("senzor down", hardware.downBorderMovement.getState());
        telemetry.addData("senzor up", hardware.upBorderMovement.getState());
        telemetry.addData("crem", hardware.mCremaliera.isBusy());
        telemetry.update();

        switch (currentState)
        {
            case DRIVER_CONTROLLED:
                if(!force_stop_glis) glisOperation(glis_up, glis_down);
                if(!force_stop_crem) cremOperation(crem_fwd, crem_rev);
                forebarOperation(servo_fb);
                clawOperation(servo_claw);
                _180Operation(servo_180);
                servoMovementOperation(servo_move_up, servo_move_down);
                if(gamepad2.x) currentState = STATE_MACHINE.AUTOMID;
                if(gamepad2.y) currentState = STATE_MACHINE.AUTOHIGH;
                if(gamepad2.b) {
                    _180_delay = elapsedTime.milliseconds();
                    currentMovement = MOVEMENT_STATE.DOWN1;
                    currentState = STATE_MACHINE.AUTOMID;
                }
                break;

            case AUTOMID:
                autoUp(1400);
                break;

            case AUTOHIGH:
                autoUp(2000);
                break;
        }


    }

    private void autoUp(int finalPosTicks)
    {
        switch (currentMovement)
        {
            case UP1:
                goToPos(Specifications.glis_treshold_move, 0);
                if(Specifications.glis_treshold_move < Math.abs(hardware.mGlisRight.getCurrentPosition())
                        && Specifications.glis_treshold_move < Math.abs(hardware.mGlisLeft.getCurrentPosition()))
                    currentMovement = MOVEMENT_STATE.UP2;
                break;

            case UP2:
                goToPos(finalPosTicks, 0);
                if(!finishedMove) finishedMove = moveCrane(1);
                moveCremaliera(Specifications.lower_limit_crem_ticks);
                forebar_delay = elapsedTime.milliseconds();
                if(hardware.mCremaliera.getCurrentPosition()<Specifications.lower_limit_crem_ticks + 50 && finishedMove
                        && finalPosTicks < Math.abs(hardware.mGlisRight.getCurrentPosition())
                        && finalPosTicks < Math.abs(hardware.mGlisLeft.getCurrentPosition()))
                {
                    hardware.mCremaliera.setPower(0);
                    hardware.servoMovement.setPower(0);
                    hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    currentMovement = MOVEMENT_STATE.UP3;
                }
                break;

            case UP3:
                finishedMove = false;
                hardware.servo180.setPosition(Specifications.servo_180_down);
                if(forebar_delay+254 < elapsedTime.milliseconds()){
                    hardware.servoForebar.setPosition(Specifications.servo_forebar_up);
                    currentState = STATE_MACHINE.DRIVER_CONTROLLED;
                }
                break;

            case DOWN1:
                hardware.servoForebar.setPosition(Specifications.servo_forebar_down);
                hardware.servo180.setPosition(Specifications.servo_180_up);
                if(_180_delay+700 < elapsedTime.milliseconds())
                    currentMovement = MOVEMENT_STATE.DOWN2;
                break;

            case DOWN2:
                moveCremaliera(Specifications.upper_limit_crem_ticks);
                if(!finishedMove) finishedMove = moveCrane(0);
                goToPos(Specifications.glis_treshold_move, 1);
                if(hardware.mCremaliera.getCurrentPosition()>Specifications.upper_limit_crem_ticks - 50 && !hardware.mGlisLeft.isBusy() && !hardware.mGlisRight.isBusy())
                    currentMovement = MOVEMENT_STATE.DOWN3;
                break;

            case DOWN3:
                finishedMove = false;
                goToPos(0, 1);
                break;

            case FINISH:
                break;
        }
    }

    private void timersControl() {
        if(resetY && elapsedTime.milliseconds() > lastTimeY + Specifications.button_delay) resetY = false;

        if(resetA && elapsedTime.milliseconds() > lastTimeA + Specifications.button_delay) resetA = false;

        if(resetB && elapsedTime.milliseconds() > lastTimeB + Specifications.button_delay) resetB = false;

        if(resetX && elapsedTime.milliseconds() > lastTimeX + Specifications.button_delay) resetX = false;

        if(resetDL && elapsedTime.milliseconds() > lastTimeDL + Specifications.button_delay) resetDL = false;

        if(resetDR && elapsedTime.milliseconds() > lastTimeDR + Specifications.button_delay) resetDR = false;
    }

    private void movement(float forward, float strafe, float rotation, boolean slowdown){
        // power applied to the robot wheel by wheel
        double[] power = new double[4];
        rotation *= -1;

        if(!slowdown){
            power[0] = (-forward + strafe + rotation) * Specifications.movement_speed_fast;   //+
            power[1] = (+forward + strafe + rotation) * Specifications.movement_speed_fast;   //-
            power[2] = (-forward - strafe + rotation) * Specifications.movement_speed_fast;   //-
            power[3] = (+forward - strafe + rotation) * Specifications.movement_speed_fast;   //+
        }else{
            power[0] = (-forward + strafe + rotation) * Specifications.movement_speed_slow;   //+
            power[1] = (+forward + strafe + rotation) * Specifications.movement_speed_slow;   //-
            power[2] = (-forward - strafe + rotation) * Specifications.movement_speed_slow;   //-
            power[3] = (+forward - strafe + rotation) * Specifications.movement_speed_slow;   //+
        }
        // applying the power
        for (int i = 0; i < 4; i++) {
            hardware.motor[i].setPower(power[i]);
        }
    }

    private void glisOperation(float up, float down){

        if(!hardware.sGlisLeft.getState())
            glisLeftOffset = -hardware.mGlisLeft.getCurrentPosition();
        if(!hardware.sGlisRight.getState())
            glisRightOffset = -hardware.mGlisRight.getCurrentPosition();

        if(up == 0 && down == 0){

            if(Math.abs(hardware.mGlisRight.getCurrentPosition()) > Specifications.glis_threshold_stationary
                    && Math.abs(hardware.mGlisLeft.getCurrentPosition()) > Specifications.glis_threshold_stationary) {
                hardware.mGlisLeft.setTargetPosition(glisStationaryTicks);
                hardware.mGlisRight.setTargetPosition(-glisStationaryTicks);

                //hardware.mGlisLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Specifications.runToPosPIDF);
                //hardware.mGlisRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Specifications.runToPosPIDF);

                hardware.mGlisRight.setPower(-Specifications.glis_speed_stationary);
                hardware.mGlisLeft.setPower(Specifications.glis_speed_stationary);
            }else{
                if(hardware.mGlisRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
                   // hardware.mGlisLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Specifications.newPIDF);
                  //hardware.mGlisRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Specifications.newPIDF);
                }

                hardware.mGlisLeft.setPower(0);
                hardware.mGlisRight.setPower(0);
            }

        }else if(up != 0){

            if(hardware.mGlisRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
              //  hardware.mGlisLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Specifications.newPIDF);
             //   hardware.mGlisRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Specifications.newPIDF);
            }

            hardware.mGlisRight.setPower(-up * Specifications.glis_speed_extension);
            hardware.mGlisLeft.setPower(up * Specifications.glis_speed_extension);
            glisStationaryTicks = hardware.mGlisLeft.getCurrentPosition() + glisLeftOffset;

        }else if(down !=0){

            if(hardware.mGlisRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            }

            if(hardware.sGlisRight.getState()){
                if(Math.abs(getPos(hardware.mGlisRight))<Specifications.glis_threshold_gradual_slowdown){
                    float power = Math.abs(getPos(hardware.mGlisRight))*Specifications.glis_speed_threshold_multi;
                    hardware.mGlisRight.setPower(Math.max(power, Specifications.glis_speed_min_retraction));
                }else
                    hardware.mGlisRight.setPower(down * Specifications.glis_speed_fast_retraction);
            }

            if(hardware.sGlisLeft.getState()){
                if(Math.abs(getPos(hardware.mGlisLeft))<Specifications.glis_threshold_gradual_slowdown){
                    float power = Math.abs(getPos(hardware.mGlisLeft))*Specifications.glis_speed_threshold_multi;
                    hardware.mGlisLeft.setPower(-Math.max(power, Specifications.glis_speed_min_retraction));
                }else
                    hardware.mGlisLeft
                            .setPower(down * -Specifications.glis_speed_fast_retraction);
            }

            glisStationaryTicks = hardware.mGlisLeft.getCurrentPosition() + glisLeftOffset;
        }
    }

    private void forebarOperation(boolean button){
        if(!resetDR && button){
            if(!forebarState) hardware.servoForebar.setPosition(Specifications.servo_forebar_up);
            else hardware.servoForebar.setPosition(Specifications.servo_forebar_down);

            resetDR = true;
            forebarState = !forebarState;
            lastTimeDR = elapsedTime.milliseconds();
        }
    }

    private void clawOperation(boolean button){
        if (!resetA && button){
            if (clawState) hardware.servoClaw.setPosition(Specifications.servo_claw_closed);
            else hardware.servoClaw.setPosition(Specifications.servo_claw_open);

            resetA = true;
            clawState = !clawState;
            lastTimeA = elapsedTime.milliseconds();
        }
    }

    private void _180Operation(boolean button){
        if(!resetDL && button){
            if(_180State) hardware.servo180.setPosition(Specifications.servo_180_up);
            else hardware.servo180.setPosition(Specifications.servo_180_down);

            resetDL = true;
            _180State = !_180State;
            lastTimeDL = elapsedTime.milliseconds();
        }
    }

    private void servoMovementOperation(boolean up, boolean down) {
        if(up)
            hardware.servoMovement.setPower(-Specifications.servo_movement_ms);
        else if(down)
            hardware.servoMovement.setPower(Specifications.servo_movement_ms);
        else
            hardware.servoMovement.setPower(0);
    }

    private void cremOperation(boolean crem_fwd, boolean crem_rev){
        if(hardware.mCremaliera.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (crem_fwd)
            hardware.mCremaliera.setPower(Specifications.crem_speed);
        else if (crem_rev)
            hardware.mCremaliera.setPower(-Specifications.crem_speed);
        else
            hardware.mCremaliera.setPower(0);
    }

    private int getPos(DcMotorEx motor){
        if(motor == hardware.mGlisLeft)
            return hardware.mGlisLeft.getCurrentPosition() + glisLeftOffset;
        else
            return hardware.mGlisRight.getCurrentPosition() + glisRightOffset;
    }

    private void goToPos(int posTicks, int direction)
    {
        if(!hardware.sGlisLeft.getState() && direction==1){
            hardware.mGlisLeft.setPower(0);
            hardware.mGlisRight.setPower(0);
            hardware.mGlisLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.mGlisRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.mGlisLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.mGlisRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currentState = STATE_MACHINE.DRIVER_CONTROLLED;
            currentMovement = MOVEMENT_STATE.UP1;
            return;
        }
        if(direction == 0) {
            if(posTicks > Math.abs(hardware.mGlisRight.getCurrentPosition()) && posTicks > Math.abs(hardware.mGlisLeft.getCurrentPosition()))
            {
                hardware.mGlisLeft.setPower(1);
                hardware.mGlisRight.setPower(-1);
            }
        }
        if(direction == 1)
        {
            if(posTicks < Math.abs(hardware.mGlisRight.getCurrentPosition()) && posTicks < Math.abs(hardware.mGlisLeft.getCurrentPosition()))
            {
                hardware.mGlisLeft.setPower(-0.2);
                hardware.mGlisRight.setPower(0.2);
            }

        }
        if(posTicks < Math.abs(hardware.mGlisRight.getCurrentPosition()) && posTicks < Math.abs(hardware.mGlisLeft.getCurrentPosition()) && direction == 0)
        {
            hardware.mGlisLeft.setPower(0.2);
            hardware.mGlisRight.setPower(-0.2);
        }
        if(posTicks > Math.abs(hardware.mGlisRight.getCurrentPosition()) && posTicks > Math.abs(hardware.mGlisLeft.getCurrentPosition()) && direction == 1)
        {
            hardware.mGlisLeft.setPower(0.2);
            hardware.mGlisRight.setPower(-0.2);
        }
    }



    private boolean moveCrane(int direction)
    {
        if(direction == 0)
        {
            hardware.servoMovement.setPower(-Specifications.servo_movement_ms);
            if(!hardware.upBorderMovement.getState())
            {
                hardware.servoMovement.setPower(0);
                return true;
            }
        }
        if(direction == 1)
        {
            hardware.servoMovement.setPower(Specifications.servo_movement_ms);
            if(!hardware.downBorderMovement.getState())
            {
                hardware.servoMovement.setPower(0);
                return true;
            }
        }
        return false;
    }

    private void moveCremaliera(int ticks)
    {
        hardware.mCremaliera.setTargetPosition(ticks);
        hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.mCremaliera.setPower(Specifications.crem_speed_auto);

    }

    private void forceStop(boolean button, boolean glisbutton, boolean crembutton){
        if(button){
            currentState = STATE_MACHINE.DRIVER_CONTROLLED;
            currentMovement = MOVEMENT_STATE.UP1;
            hardware.servoMovement.setPower(0);
        }
        if(glisbutton || Math.abs(hardware.mGlisLeft.getCurrentPosition())>2300){
            hardware.mGlisRight.setPower(0);
            hardware.mGlisLeft.setPower(0);
        }
        if(crembutton || hardware.mCremaliera.getCurrentPosition() > 750 || hardware.mCremaliera.getCurrentPosition() < -1270) hardware.mCremaliera.setPower(0);
    }

}
