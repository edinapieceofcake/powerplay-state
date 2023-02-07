package edu.edina.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveOdo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class DriveToPickup extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveOdo drive = new SampleMecanumDriveOdo(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(28, 10, Math.toRadians(245)));
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(28, 10, Math.toRadians(245)))
                .lineToSplineHeading(new Pose2d(56, 12, Math.toRadians(180)))
                .build();


        drive.followTrajectorySequence(traj);
    }
}
