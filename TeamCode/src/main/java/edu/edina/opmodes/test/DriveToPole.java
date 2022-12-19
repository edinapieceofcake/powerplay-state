package edu.edina.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import edu.edina.library.util.PoseStorage;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class DriveToPole extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(270)));
        TrajectorySequence traj =  drive.trajectorySequenceBuilder(new Pose2d(36, 64, Math.toRadians(270)))
                .splineTo(new Vector2d(28, 10), Math.toRadians(245))
                .build();

        drive.followTrajectorySequence(traj);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
