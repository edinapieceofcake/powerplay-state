package edu.edina.opmodes.test;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveOdo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class StrafeToColor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveOdo drive = new SampleMecanumDriveOdo(hardwareMap);
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors;

        drive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(270)));
        TrajectorySequence strafe = drive.trajectorySequenceBuilder(new Pose2d(36, 64, Math.toRadians(270)))
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        // return 20;
                        return 45;
                    }
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        // return 20;
                        return 45;
                    }
                })
                .strafeLeft(10).build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(strafe);
        while (!Thread.currentThread().isInterrupted()) {
            colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (hsvValues[0] > 100) {
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                break;
            }

            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.update();
            drive.update();
            idle();
        }

        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .resetConstraints()
                .forward(10)
                .build();

        drive.followTrajectorySequence(forward);

        while (!isStopRequested()) {
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Hue1", "%.3f", hsvValues[0])
                    .addData("Saturation1", "%.3f", hsvValues[1])
                    .addData("Value1", "%.3f", hsvValues[2]);
            telemetry.update();
            idle();
        }
    }
}
