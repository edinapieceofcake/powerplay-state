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
    private SampleMecanumDrive drive;
    //To change the speed of the robot change the speedMultiplier variable
    private double speedMultiplier = .5;
    private RobotState robotState;

    public MecanumDriveRR(HardwareMap map, RobotState robotState){
        try {
            drive = new SampleMecanumDrive(map);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.setPoseEstimate(PoseStorage.currentPose);
            robotState.SpeedMultiplier = speedMultiplier;
            this.robotState = robotState;
            robotState.DriveSuccessfullySetup = true;
        } catch (Exception ex) {
            robotState.DriveSuccessfullySetup = false;
        }
    }

    @Override
    public void update() {
        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        // To change to Robot centric change leftSticks to ____
                        // To change to Robot centric change leftSticks to ____
                        -leftStickY,
                        -leftStickX,
                        -rightStickX
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();
    }

    public void setDriveProperties(double leftStickX, double leftStickY, double rightStickX, boolean dPadDown){
        if (dPadDown) {
            if (speedMultiplier == 0.5) {
                speedMultiplier = 0.6;
            }

            if (speedMultiplier == 0.6) {
                speedMultiplier = 0.5;
            }
        }

        robotState.SpeedMultiplier = speedMultiplier;
        this.leftStickX = leftStickX * speedMultiplier;
        this.leftStickY = leftStickY * speedMultiplier;
        this.rightStickX = rightStickX * speedMultiplier;
    }
}
