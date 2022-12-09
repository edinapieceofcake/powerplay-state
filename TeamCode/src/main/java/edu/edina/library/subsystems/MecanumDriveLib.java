package edu.edina.library.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.edina.library.util.RobotState;

//Constructor matches class name eg. MecanumDriveLib

public class MecanumDriveLib extends Subsystem{
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private MecanumDrive drive;

    public MecanumDriveLib(HardwareMap map, RobotState RobotState){
        Motor fL = new Motor(map, "leftFront", Motor.GoBILDA.RPM_312);
        Motor fR = new Motor(map, "rightFront", Motor.GoBILDA.RPM_312);
        Motor bL = new Motor(map, "LeftBack", Motor.GoBILDA.RPM_312);
        Motor bR = new Motor(map, "LeftFront", Motor.GoBILDA.RPM_312);

        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    @Override
    public void update() {
        drive.driveRobotCentric(
                leftStickX,
                leftStickY,
                rightStickX
        );
    }

    public void setDriveProperties(double leftStickX, double leftStickY, double rightStickX){
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
    }
}
