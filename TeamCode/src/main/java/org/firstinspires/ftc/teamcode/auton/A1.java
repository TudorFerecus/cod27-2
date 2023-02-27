package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.Specifications;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Albastru 1", group="Autonomy")
public class A1 extends ScheleteAutonomie {
    ElapsedTime elapsedTime;
    @Override
    public void runOpMode() {
        init_auto();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(-36, 65, Math.toRadians(-90));

        drive.setPoseEstimate(startPos);

        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-36, 13.8, Math.toRadians(-90)))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-36, 13.8, Math.toRadians(180)))
                .forward(10)
                .build();

        Trajectory traj2_2 = drive.trajectoryBuilder(traj1.end())
                .back(11)
                .build();

        Trajectory traj2_1 = drive.trajectoryBuilder(traj1.end())
                .back(35)
                .build();

        Trajectory traj2_3 = drive.trajectoryBuilder(traj1.end())
                .forward(8)
                .build();

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        AprilTagInit();

        while(!isStarted() && !isStopRequested()){
            AprilTagInitLoop();
        }

        if (tagOfInterest != null) {
            telemetry.addData("Autonomy Case", tagOfInterest.id);
            telemetry.update();
        }

        drive.followTrajectory(traj0);

        drive.turn(Math.toRadians(-101));

        sleep(100);

        drive.followTrajectory(traj1);

        sleep(400);


        double delay = runTime.milliseconds();

        waitForStart();

        runTime.reset();

        goToPos(4100);

        while(delay > runTime.milliseconds() - Specifications.delayAutonAction - 1000 && hardware.upBorderMovement.getState())
        {
            hardware.servoMovement.setPower(-Specifications.servo_movement_speed);
            if(delay < runTime.milliseconds() - 2000)
                cremalieraMovement(-450);
        }
        hardware.servoMovement.setPower(0);

        hardware.servo180.setPosition(Specifications.servo_180_auto_drop + 0.03f);

        sleep(1400);

        hardware.servoClaw.setPosition(Specifications.servo_claw_pos_up);

        sleep(300);

        //** start loop

//        for(int i=1; i<=3 && runTime.seconds()<25; i++){
//            sleep(300);
//            hardware.servo180.setPosition(Specifications.servo_180_pos_up);
//
//            sleep(1000);
//
//            goToPos(250);
//            cremalieraMovement(665);
//
//            while(getDiffTargeCurrentGlisLeft() > 30 && getDiffTargeCurrentGlisRight() > 30
//                    && hardware.mCremaliera.getCurrentPosition() < hardware.mCremaliera.getTargetPosition() - 10) ;
//
//            hardware.servoForebar.setPosition(Specifications.servo_forbar_start_auton);
//
//            sleep(800);
//
//            hardware.servoClaw.setPosition(Specifications.servo_claw_pos_down);
//
//            sleep(1400);
//
//            goToPos(4100);
//
//            hardware.servoForebar.setPosition(Specifications.servo_forbar_start_auton);
//            sleep(200);
//
//            sleep(1000);
//
//            //hardware.servo180.setPosition(Specifications.servo_180_auto_drop);
//
//            cremalieraMovement(-450);
//
//            while(hardware.mCremaliera.getTargetPosition() > hardware.mCremaliera.getCurrentPosition() + 10) ;
//
//            sleep(1400);
//
//            hardware.servoClaw.setPosition(Specifications.servo_claw_pos_up);
//        }
//
        if(tagOfInterest.id==1){
            drive.followTrajectory(traj2_1);
        }else if(tagOfInterest.id==2){
            drive.followTrajectory(traj2_2);
        }else{
            drive.followTrajectory(traj2_3);
        }
    }

}
/*
todo move to stack
todo find loop sequence
todo park
 */

