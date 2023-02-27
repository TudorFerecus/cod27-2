package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.Specifications;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Rosu 2 C1", group="Autonomy")
public class R2C1 extends ScheleteAutonomie {
    ElapsedTime elapsedTime;
    @Override
    public void runOpMode() {
        init_auto();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(-36, -65, Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-36, -36.5, Math.toRadians(90)))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-36, -36.5, Math.toRadians(90)))
                .back(7f)
                .build();

        Trajectory traj2_2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(24)
                .build();

        Trajectory traj2_1 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(24)
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

        sleep(400);

        waitForStart();

        drive.followTrajectory(traj0);

        runTime.reset();

        goToPos(5900);

        sleep(100);

        while(hardware.upBorderMovement.getState()) hardware.servoMovement.setPower(-1);
        hardware.servoMovement.setPower(0);

        while(getDiffTargeCurrentGlisLeft() > 10 && getDiffTargeCurrentGlisRight() > 10) ;

        hardware.servoForebar.setPosition(0);

        sleep(1000);

        drive.turn(Math.toRadians(56));

        sleep(100);

        // drive.followTrajectory(traj1);

        sleep(2000);

        hardware.servoClaw.setPosition(Specifications.servo_claw_pos_up);

        sleep(1000);

        goToPos(900);

        while(getDiffTargeCurrentGlisLeft() > 10 && getDiffTargeCurrentGlisRight() > 10) ;

        sleep(500);

        hardware.servoForebar.setPosition(1);

        sleep(1000);

        drive.turn(Math.toRadians(-56));

        sleep(1000);

        drive.followTrajectory(traj1);

        sleep(1000);

        if(tagOfInterest.id==1){
            drive.followTrajectory(traj2_2);
        }else if(tagOfInterest.id==3){
            drive.followTrajectory(traj2_1);
        }
    }

}
/*
todo move to stack
todo find loop sequence
todo park
 */

