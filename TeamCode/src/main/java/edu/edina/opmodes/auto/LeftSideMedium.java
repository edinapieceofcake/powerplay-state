package edu.edina.opmodes.auto;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.RobotState;
import edu.edina.library.vision.AprilTagDetectionPipeline;

@Autonomous
@Config
//@Disabled
public class LeftSideMedium extends LinearOpMode {

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

    private DcMotorEx liftMotor;
    private RobotState robotState;
    private Servo armServo;
    private Servo clawServo;
    private Servo slicerServo;
    private ColorSensor frontColor;
    private DistanceSensor frontDistance;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "rightCamera"), cameraMonitorViewId);
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

        robotState = new RobotState();
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        slicerServo = hardwareMap.get(Servo.class, "slicerServo");
        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        final float[] hsvValues = new float[3];

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setPower(1);
        robotState.FutureTargetPosition = 0;
        liftMotor.setTargetPosition(robotState.FutureTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;

        armServo.setPosition(robotState.ARMBACKPOSITION);
        robotState.ArmServoPosition = ArmServoPosition.Back;

        slicerServo.setPosition(robotState.SLICERSTORAGEPOSITION);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-33, -65, Math.toRadians(-180)));

        // cone one drop off
        TrajectorySequence start = drive.trajectorySequenceBuilder(new Pose2d(-33, -65, Math.toRadians(-180)))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);})
                .addTemporalMarker(1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONLOW); })
                .addTemporalMarker(2.3, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .strafeTo(new Vector2d(-33, -19))
                .build();

        // cone two pickup
        TrajectorySequence backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeRight(10)
                .forward(22.5)
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION5);
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .build();

        TrajectorySequence strafe = drive.trajectorySequenceBuilder(backToPickup1.end())
                .strafeLeft(10).build();

        // cone two drop off
        TrajectorySequence backToDropOff1 = drive.trajectorySequenceBuilder(new Pose2d(-60, -10, Math.toRadians(-180)))
                .strafeTo(new Vector2d(-23, -10))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONLOW);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    } )
                .addTemporalMarker(2.1, () -> {
                    clawServo.setPosition(robotState.CLAWOPENFORDROPOFF);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone three pickup
        TrajectorySequence backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeTo(new Vector2d(-60, -10))
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION4);
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone three drop off
        TrajectorySequence backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .strafeTo(new Vector2d(-22.5, -10))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONLOW);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone four pickup
        TrajectorySequence backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .strafeTo(new Vector2d(-60, -10))
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION3);
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone four drop off
        TrajectorySequence backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .strafeTo(new Vector2d(-22.5, -10))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONLOW);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone five pickup
        TrajectorySequence backToPickup4 = drive.trajectorySequenceBuilder(backToDropOff3.end())
                .strafeTo(new Vector2d(-60, -10))
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION2);
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone five drop off
        TrajectorySequence backToDropOff4 = drive.trajectorySequenceBuilder(backToPickup4.end())
                .strafeTo(new Vector2d(-22.5, -10))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONLOW);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone six pickup
        TrajectorySequence backToPickup5 = drive.trajectorySequenceBuilder(backToDropOff4.end())
                .strafeTo(new Vector2d(-60, -10))
                .addTemporalMarker(.7, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION1);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone six drop off
        TrajectorySequence backToDropOff5 = drive.trajectorySequenceBuilder(backToPickup5.end())
                .strafeTo(new Vector2d(-22.5, -10))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONLOW);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // park
        TrajectorySequence backToPickup6_1 = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .forward(35)
                .build();

        TrajectorySequence backToPickup6_2 = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .forward(7)
                .build();

        TrajectorySequence backToPickup6_3 = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(15)
                .build();

        while (!isStarted() && !isStopRequested())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            telemetry.addData("Make sure claw is in the back and low camera is facing field.", "");
            telemetry.addData("Cone should always be on side with medium pole", "");

            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        detectionId = detection.id;
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }

        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });

        if (opModeIsActive()) {
            liftMotor.setTargetPosition(-50);
            liftMotor.setPower(1);

            drive.followTrajectorySequence(start); // cone one

            sleep(100);

            drive.followTrajectorySequence(backToPickup1);

            drive.followTrajectorySequenceAsync(strafe);
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                Color.RGBToHSV(frontColor.red() * 8, frontColor.green() * 8, frontColor.blue() * 8, hsvValues);
                if ((hsvValues[0] < 100) || (hsvValues[0] > 180)){
                    drive.breakFollowing();
                    drive.setDrivePower(new Pose2d());
                    break;
                }

                drive.update();
                idle();
            }

            double distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);
            if (distanceToTravel > 4) {
                distanceToTravel = 4;
            }
            TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .resetConstraints()
                    .forward(distanceToTravel)
                    .build();

            drive.followTrajectorySequence(forward);

            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            sleep(200);

            drive.setPoseEstimate(new Pose2d(new Vector2d(-60, -10), Math.toRadians(-180)));

            drive.followTrajectorySequence(backToDropOff1); // cone two

            drive.followTrajectorySequence(backToPickup2);

            drive.followTrajectorySequence(backToDropOff2); // cone three

            drive.followTrajectorySequence(backToPickup3);

            drive.followTrajectorySequence(backToDropOff3); // cone four

            drive.followTrajectorySequence(backToPickup4);

            drive.followTrajectorySequence(backToDropOff4); // cone five

            drive.followTrajectorySequence(backToPickup5);

            drive.followTrajectorySequence(backToDropOff5); // cone six

            if (detectionId == 3) {
                drive.followTrajectorySequence(backToPickup6_3);
            } else if (detectionId == 6) {
                drive.followTrajectorySequence(backToPickup6_2);
            } else {
                drive.followTrajectorySequence(backToPickup6_1);
            }

            while (opModeIsActive()) {
                Color.RGBToHSV(frontColor.red() * 8, frontColor.green() * 8, frontColor.blue() * 8, hsvValues);
                distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);
                telemetry.addData("Pose", drive.getPoseEstimate());
                telemetry.addLine()
                        .addData("Front Hue", "%.3f", hsvValues[0])
                        .addData("Distance To Travel", distanceToTravel);

                telemetry.update();
                sleep(20);
            }
        }
    }
}
