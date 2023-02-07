package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveOdo;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
//@Disabled
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    private DcMotorEx leftEncoder;
    private DcMotorEx rightEncoder;
    private DcMotorEx centerEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        centerEncoder = hardwareMap.get(DcMotorEx.class, "centerEncoder");

        Servo leftServo = hardwareMap.get(Servo.class, "leftPodServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightPodServo");
        Servo centerServo = hardwareMap.get(Servo.class, "centerPodServo");
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        centerServo.setPosition(1);

        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        SampleMecanumDriveOdo drive = new SampleMecanumDriveOdo(hardwareMap);

        int leftStart = leftEncoder.getCurrentPosition();
        int rightStart = rightEncoder.getCurrentPosition();
        int centerStart = centerEncoder.getCurrentPosition();


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        while (!isStopRequested() && opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("Left Start", leftStart);
            telemetry.addData("Right Start", rightStart);
            telemetry.addData("Center Start", centerStart);
            telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
            telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder Diff", leftStart - leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder Diff", rightStart - rightEncoder.getCurrentPosition());
            telemetry.addData("Center Encoder Diff", centerStart - centerEncoder.getCurrentPosition());
            telemetry.addData("Left/Right Encoder Diff", (leftStart - leftEncoder.getCurrentPosition()) - (centerStart - centerEncoder.getCurrentPosition()) );

            telemetry.update();
        }
    }
}
