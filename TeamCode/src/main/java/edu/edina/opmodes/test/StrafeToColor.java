package edu.edina.opmodes.test;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import edu.edina.library.util.PoseStorage;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
//@Disabled
public class StrafeToColor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        NormalizedColorSensor backColor = hardwareMap.get(NormalizedColorSensor.class, "backColor");
        NormalizedColorSensor frontColor = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors;

        backColor.setGain(2);
        frontColor.setGain(2);
        drive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(270)));
        TrajectorySequence strafe = drive.trajectorySequenceBuilder(new Pose2d(36, 64, Math.toRadians(270)))
                .strafeLeft(10).build();

        while (!isStarted()) {
            colors = backColor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0])
                    .addData("Back Saturation", "%.3f", hsvValues[1])
                    .addData("back Value", "%.3f", hsvValues[2]);

            colors = frontColor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Front Hue", "%.3f", hsvValues[0])
                    .addData("Front Saturation", "%.3f", hsvValues[1])
                    .addData("Front Value", "%.3f", hsvValues[2]);

            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(strafe);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            colors = frontColor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            if ((hsvValues[0] < 100) || (hsvValues[0] > 160)){
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                break;
            }

            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0])
                    .addData("Back Saturation", "%.3f", hsvValues[1])
                    .addData("Back Value", "%.3f", hsvValues[2]);
            telemetry.update();
            drive.update();
            idle();
        }

        double distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH)-3.75;
        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .resetConstraints()
                .forward(distanceToTravel)
                .build();

        drive.followTrajectorySequence(forward);

        while (!isStopRequested()) {
            colors = backColor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0])
                    .addData("Back Saturation", "%.3f", hsvValues[1])
                    .addData("back Value", "%.3f", hsvValues[2]);

            colors = frontColor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Front Hue", "%.3f", hsvValues[0])
                    .addData("Front Saturation", "%.3f", hsvValues[1])
                    .addData("Front Value", "%.3f", hsvValues[2]);

            telemetry.update();
            idle();
        }
    }
}
