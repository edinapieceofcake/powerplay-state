package edu.edina.library.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.library.util.RobotState;

//Constructor matches class name eg. MecanumDriveLib

public class MecanumDriveLib extends Subsystem{
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;
    private RevIMU imu;

    public MecanumDriveLib(HardwareMap map, RobotState RobotState){
        Motor fL = new Motor(map, "leftFront", Motor.GoBILDA.RPM_312);
        Motor fR = new Motor(map, "rightFront", Motor.GoBILDA.RPM_312);
        Motor bL = new Motor(map, "leftRear", Motor.GoBILDA.RPM_312);
        Motor bR = new Motor(map, "rightRear", Motor.GoBILDA.RPM_312);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(fL, fR, bL, bR);
        imu = new RevIMU(map);
        imu.init();

    }

    @Override
    public void update() {
        drive.driveFieldCentric(
                leftStickX,
                -leftStickY,
                -rightStickX,
                //Added 180 because the control hub was facing the opposite direction of what we wanted so the robot would move opposite to the supposed direction
                imu.getRotation2d().getDegrees() + 180,   // gyro value passed in here must be in degrees
                false
        );
    }

    public void setDriveProperties(double leftStickX, double leftStickY, double rightStickX){
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
    }
}
