package edu.edina.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.RobotState;
import edu.edina.library.vision.AprilTagDetectionPipeline;

@Autonomous
@Config
@Disabled
public class LeftSide extends LinearOpMode {

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
    private DigitalChannel liftSwitch;

    @Override
    public void runOpMode() {
/*        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
*/
        double dropoffX1 = -22;
        double dropoffX2 = -25;
        double dropoffX3 = -25;
        double dropoffX4 = -25;
        double dropoffX5 = -25;
        double dropoffX6 = -25;
        double dropoffY = -8;
        double pickupX = -58;

        robotState = new RobotState();
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");

        liftSwitch.setMode(DigitalChannel.Mode.INPUT);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setPower(1);
        robotState.FutureTargetPosition = 0;
        liftMotor.setTargetPosition(robotState.FutureTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;

        armServo.setPosition(robotState.ARMFRONTPOSITION);
        robotState.ArmServoPosition = ArmServoPosition.Front;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-35, -65, Math.toRadians(90)));
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-59, -57), Math.toRadians(90))
                .addTemporalMarker(2, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONHIGH); })
                .forward(40)
                .splineTo(new Vector2d(dropoffX1, dropoffY), Math.toRadians(7))
                /*
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

                 */
                .build();

        waitForStart();

        liftMotor.setTargetPosition(-200);
        liftMotor.setPower(1);
        drive.followTrajectorySequence(trajectory);

        armServo.setPosition(robotState.ARMSIDEPOSITION);
        robotState.ArmServoPosition = ArmServoPosition.Side;

        sleep(750);

        liftMotor.setTargetPosition(robotState.POLEPOSITIONHIGH + 100);

        clawServo.setPosition(robotState.CLAWOPENPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Open;

        sleep (750);

        armServo.setPosition(robotState.ARMFRONTPOSITION);
        robotState.ArmServoPosition = ArmServoPosition.Front;

        while (opModeIsActive()) {sleep(20);}
    }
}
