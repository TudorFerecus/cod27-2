package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.Specifications;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Rosu 2", group="Autonomy")
public class R2 extends ScheleteAutonomie {
    ElapsedTime elapsedTime;


    @Override
    public void runOpMode()
    {
        init_auto();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(36, -65, Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(36, -13.8, Math.toRadians(90)))
                .build();

        Trajectory traj2_1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(60, -13.8, Math.toRadians(90)))
                .build();

        Trajectory traj2_3 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(12, -13.8, Math.toRadians(90)))
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

        sleep(200);

        if(tagOfInterest.id==3){
            drive.followTrajectory(traj2_1);
        }else if(tagOfInterest.id==1){
            drive.followTrajectory(traj2_3);
        }

    }

}
