package edu.edina.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import edu.edina.library.subsystems.Lift;
import edu.edina.library.subsystems.MecanumDriveLib;
import edu.edina.library.subsystems.Subsystem;
import edu.edina.library.util.RobotState;

public class NoThreadRobotLib {
    private List<Subsystem> subsystems;
    private Telemetry telemetry;
    public MecanumDriveLib driveLib;
    public Lift lift;
    public RobotState robotState = new RobotState();

    public void update() {
        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) continue;
                subsystem.update();
        }
    }

    public NoThreadRobotLib(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;

        subsystems = new ArrayList<>();

        driveLib = new MecanumDriveLib(opMode.hardwareMap, robotState);
        subsystems.add(driveLib);

        lift = new Lift(opMode.hardwareMap, robotState);
        subsystems.add(lift);
    }

    public void telemetry()
    {
        robotState.telemetry(telemetry);
        telemetry.update();
    }
}
