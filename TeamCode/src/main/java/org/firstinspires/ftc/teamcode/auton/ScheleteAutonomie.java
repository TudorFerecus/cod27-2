package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Specifications;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Disabled

public abstract class ScheleteAutonomie extends LinearOpMode
{


    OpenCvCamera camera;
    Hardware hardware;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    AprilTagDetection autonomyCase = null;

    public int[] cremOffset = new int[5];
    public float[] fbOffset = new float[5];

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    //protected Hardware hardware = new Hardware(hardwareMap, true);
    protected ElapsedTime runTime = new ElapsedTime();

    protected AprilTagDetection tagOfInterest = null;

    protected void init_auto()
    {
        hardware = new Hardware(hardwareMap, true);
        hardware.servoForebar.setPosition(Specifications.servo_forbar_start_auton);
        hardware.servoClaw.setPosition(Specifications.servo_claw_pos_down);
        runTime = new ElapsedTime();

        cremOffset[0] = 0;
        cremOffset[1] = 215;
        cremOffset[2] = 305;
        cremOffset[3] = 450;
        cremOffset[4] = 580;

        fbOffset[0]=0.1f;
        fbOffset[1]=0.2f;
        fbOffset[2]=0.3f;

    }


    protected void AprilTagInit()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    protected void AprilTagInitLoop()
    {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();

    }

    protected void coneRaise(int amount)
    {

        hardware.mGlisLeft.setTargetPosition(amount);
        hardware.mGlisRight.setTargetPosition(amount);

        hardware.mGlisRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.mGlisLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //hardware.mArmRight.setPower(0.5f);
        //hardware.mArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //hardware.mArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    protected void goToPos(int posTicks)
    {
        hardware.mGlisRight.setTargetPosition(posTicks);
        hardware.mGlisLeft.setTargetPosition(-posTicks);

        hardware.mGlisRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.mGlisLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.mGlisLeft.setPower(Specifications.glis_speed_multiplier);
        hardware.mGlisRight.setPower(-Specifications.glis_speed_multiplier);


    }

    protected void cremalieraMovement(int posTicks)
    {
        hardware.mCremaliera.setTargetPosition(posTicks);
        hardware.mCremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.mCremaliera.setPower(Specifications.crem_speed_auto);

    }

    protected int getDiffTargeCurrentGlisLeft()
    {
        return Math.abs(hardware.mGlisLeft.getCurrentPosition() - hardware.mGlisLeft.getTargetPosition());
    }

    protected int getDiffTargeCurrentGlisRight()
    {
        return Math.abs(hardware.mGlisRight.getCurrentPosition() - hardware.mGlisRight.getTargetPosition());
    }



}