package edu.edina.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import edu.edina.library.vision.AprilTagDetectionPipeline;

@Autonomous
@Config
@Disabled
public class GoAroundSixStack extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double dropTime = .5;
    double pickupTime = .5;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int detectionId = 6; // middle

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public static double PICKUPY = -8.5;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        double dropoffX1 = 25;
        double dropoffX2 = 25;
        double dropoffX3 = 25;
        double dropoffX4 = 25;
        double dropoffX5 = 25;
        double dropoffX6 = 25;
        double dropoffY = -4;
        double pickupX = 64;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(35, -65, Math.toRadians(90)));
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(35, -65, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(12, -57), Math.toRadians(90))
                .forward(45)
                .splineTo(new Vector2d(dropoffX1, dropoffY), Math.toRadians(0))
                .waitSeconds(dropTime) // drop first cone
                .strafeTo(new Vector2d(pickupX, PICKUPY))
                .waitSeconds(pickupTime) // grab second cone
                .strafeTo(new Vector2d(dropoffX2, dropoffY))
                .waitSeconds(dropTime) // drop second cone
                .strafeTo(new Vector2d(pickupX, PICKUPY))
                .waitSeconds(pickupTime) // grab third cone
                .strafeTo(new Vector2d(dropoffX3, dropoffY))
                .waitSeconds(dropTime) // drop third cone
                .strafeTo(new Vector2d(pickupX, PICKUPY))
                .waitSeconds(pickupTime) // grab fourth cone
                .strafeTo(new Vector2d(dropoffX4, dropoffY))
                .waitSeconds(dropTime) // drop fourth cone
                .strafeTo(new Vector2d(pickupX, PICKUPY))
                .waitSeconds(pickupTime) // grab fifth cone
                .strafeTo(new Vector2d(dropoffX5, dropoffY))
                .waitSeconds(dropTime) // drop fifth cone
                .strafeTo(new Vector2d(pickupX, PICKUPY))
                .waitSeconds(pickupTime) // grab sixth cone
                .strafeTo(new Vector2d(dropoffX6, dropoffY))
                .waitSeconds(dropTime) // drop sixth cone
                .strafeTo(new Vector2d(pickupX, PICKUPY))
                .build();

        waitForStart();

        drive.followTrajectorySequence(trajectory);


    }
}
