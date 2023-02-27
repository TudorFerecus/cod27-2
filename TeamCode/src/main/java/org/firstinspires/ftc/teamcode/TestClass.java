package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test", group="Test")
public class TestClass extends OpMode {

    private PIDFCoefficients newPid;
    private PIDFCoefficients runToPosPIDF;
    private DcMotorEx mGlisLeft;
    private DcMotorEx mGlisRight;
    private DigitalChannel sGlisLeft;
    private DigitalChannel sGlisRight;
    private int glLeftTicks = 0;
    private int glRightTicks = 0;

    private int glisRightOffset = 0;
    private int glisLeftOffset = 0;


    @Override
    public void init() {

        newPid = new PIDFCoefficients(12.5, 3.0, 0.0, 12.0);
        runToPosPIDF = new PIDFCoefficients(1, 0, 0.0, 0);

        mGlisLeft = configureMotor("em2", DcMotorSimple.Direction.FORWARD);
        mGlisRight = configureMotor("em3", DcMotorSimple.Direction.FORWARD);

        sGlisLeft = hardwareMap.get(DigitalChannel.class, "cs1");
        sGlisRight = hardwareMap.get(DigitalChannel.class, "cs3");

    }

    @Override
    public void loop() {
        float up = gamepad1.left_trigger;
        float down = gamepad1.right_trigger;


        if(!sGlisLeft.getState())
        {
            glisLeftOffset = -mGlisLeft.getCurrentPosition();
        }
        if(!sGlisRight.getState())
        {


            glisRightOffset = -mGlisRight.getCurrentPosition();
        }

        if(up == 0 && down == 0)
        {
            if(Math.abs(mGlisRight.getCurrentPosition()) > 100 && Math.abs(mGlisLeft.getCurrentPosition()) > 100)
            {
                mGlisLeft.setTargetPosition(glLeftTicks);
                mGlisRight.setTargetPosition(-glLeftTicks);

                mGlisLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, runToPosPIDF);
                mGlisRight.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, runToPosPIDF);

                mGlisRight.setPower(-0.2f);
                mGlisLeft.setPower(0.2f);
            }
            else
            {
                if(mGlisRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                {
                    mGlisLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
                    mGlisRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
                }
                mGlisLeft.setPower(0);
                mGlisRight.setPower(0);
           }
        }
        else if(up != 0)
        {
            if(mGlisRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            {
                mGlisLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
                mGlisRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
            }

            mGlisRight.setPower(-up);
            mGlisLeft.setPower(up);
            glLeftTicks = mGlisLeft.getCurrentPosition() + glisLeftOffset;
            glRightTicks = mGlisRight.getCurrentPosition() + glisRightOffset;
        }
        else
        {
            if(mGlisRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            {
                mGlisLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
                mGlisRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
            }
            if(sGlisRight.getState())
            {

                if(Math.abs(getPos(mGlisRight)) < 400) {
                    if (Math.abs(getPos(mGlisRight)) < 100)
                        mGlisRight.setPower(0.02);
                    else mGlisRight.setPower(0.05f);
                }
                else {
                    if(Math.abs(getPos(mGlisRight)) < 1500)
                    {
                        float power = Math.abs(getPos(mGlisRight)) * 0.00008f;
                        mGlisRight.setPower(power);
                    }
                    else mGlisRight.setPower(down * 0.15f);
                }
            }
            if(sGlisLeft.getState())
            {
                if(Math.abs(getPos(mGlisLeft)) < 400) {
                    if (Math.abs(getPos(mGlisLeft)) < 100)
                        mGlisLeft.setPower(-0.02);
                    else mGlisLeft.setPower(-0.05f);
                }
                else
                {
                    if(Math.abs(getPos(mGlisLeft)) < 1500)
                    {
                        float power = Math.abs(getPos(mGlisLeft)) * 0.00004f;
                        mGlisLeft.setPower(-power);
                    }
                    mGlisLeft.setPower(down * -0.15f);
                }
            }
            glLeftTicks = mGlisLeft.getCurrentPosition() + glisLeftOffset;
            glRightTicks = mGlisRight.getCurrentPosition() + glisRightOffset;
        }

        telemetry.addData("RIGHTPOS", getPos(mGlisRight));
        telemetry.addData("LEFTTPOS", getPos(mGlisLeft));
        telemetry.update();

    }

    public DcMotorEx configureMotor(String motorName, DcMotorSimple.Direction direction)
    {
        DcMotorEx motorObj = hardwareMap.get(DcMotorEx.class, motorName);
        motorObj.setDirection(direction);
        motorObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorObj.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
        return motorObj;
    }

    public int getPos(DcMotorEx motor)
    {
        if(motor == mGlisLeft)
            return mGlisLeft.getCurrentPosition() + glisLeftOffset;
        else
            return mGlisRight.getCurrentPosition() + glisRightOffset;
    }


}
