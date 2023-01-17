package edu.edina.library.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import edu.edina.library.util.PoseStorage;
import edu.edina.library.util.RobotState;

public class MecanumDriveRR extends Subsystem{
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private boolean dPadUp;
    private SampleMecanumDrive drive;
    private double speedMultiplier = .75;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    public MecanumDriveRR(HardwareMap map, RobotState robotState){
        try {
            drive = new SampleMecanumDrive(map);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.setPoseEstimate(PoseStorage.currentPose);
            robotState.DriveSuccessfullySetup = true;
        } catch (Exception ex) {
            robotState.DriveSuccessfullySetup = false;
        }
    }

    @Override
    public void update() {

        if (currentMode == Mode.DRIVER_CONTROL) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -leftStickY,
                    -leftStickX
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -rightStickX
                    )
            );

            if (dPadUp) {
                currentMode = Mode.AUTOMATIC_CONTROL;
                Vector2d endPoint = new Vector2d(poseEstimate.getX() + 10, poseEstimate.getY());
                Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                        .lineTo(endPoint)
                        .build();
                drive.followTrajectoryAsync(traj1);
            }
        } else {
            if (!drive.isBusy()) {
                currentMode = Mode.DRIVER_CONTROL;
            }
        }
        // Update everything. Odometry. Etc.
        drive.update();

    }

    public void setDriveProperties(double leftStickX, double leftStickY, double rightStickX){
        this.leftStickX = leftStickX * speedMultiplier;
        this.leftStickY = leftStickY * speedMultiplier;
        this.rightStickX = rightStickX * speedMultiplier;

        if ((leftStickY != 0) || (leftStickX != 0) || (rightStickX != 0)) {
            if (currentMode == Mode.AUTOMATIC_CONTROL){
                drive.breakFollowing();
            }
        } else {
            this.dPadUp = dPadUp;
        }
        currentMode = Mode.DRIVER_CONTROL;
    }
}
