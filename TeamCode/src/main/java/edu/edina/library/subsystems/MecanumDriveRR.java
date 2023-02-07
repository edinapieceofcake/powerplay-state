package edu.edina.library.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveOdo;

import edu.edina.library.util.PoseStorage;
import edu.edina.library.util.RobotState;

public class MecanumDriveRR extends Subsystem{
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private SampleMecanumDriveOdo drive;
    private RobotState robotState;

    public MecanumDriveRR(HardwareMap map, RobotState robotState){
        try {
            drive = new SampleMecanumDriveOdo(map);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.setPoseEstimate(PoseStorage.currentPose);
            robotState.SpeedMultiplier = robotState.LowSpeedMultiplier;
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
            if (robotState.SpeedMultiplier == robotState.LowSpeedMultiplier) {
                robotState.SpeedMultiplier = robotState.HighSpeedMultiplier;
            }

            if (robotState.SpeedMultiplier == robotState.HighSpeedMultiplier) {
                robotState.SpeedMultiplier = robotState.LowSpeedMultiplier;
            }
        }

        this.leftStickX = leftStickX * robotState.SpeedMultiplier;
        this.leftStickY = leftStickY * robotState.SpeedMultiplier;
        this.rightStickX = rightStickX * robotState.SpeedMultiplier;
    }
}
