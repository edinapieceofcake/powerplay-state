package edu.edina.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import edu.edina.library.subsystems.Intake2;
import edu.edina.library.subsystems.Lift2;
import edu.edina.library.subsystems.MecanumDrive;
import edu.edina.library.subsystems.MecanumDriveLib;
import edu.edina.library.subsystems.MecanumDriveRR;
import edu.edina.library.subsystems.Subsystem;
import edu.edina.library.util.RobotState;

public class NoThreadRobotRR {
    private List<Subsystem> subsystems;
    private Telemetry telemetry;
    public MecanumDriveRR driveRR;
    public Lift2 lift;
    public Intake2 intake;
    public RobotState robotState = new RobotState();

    public void update() {
        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) continue;
                subsystem.update();
        }
    }

    public NoThreadRobotRR(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;

        subsystems = new ArrayList<>();

        driveRR = new MecanumDriveRR(opMode.hardwareMap, robotState);
        subsystems.add(driveRR);

        //lift = new Lift2(opMode.hardwareMap, robotState);
        //subsystems.add(lift);

        //intake = new Intake2(opMode.hardwareMap, robotState);
        //subsystems.add(intake);
    }

    public void telemetry()
    {
        robotState.telemetry(telemetry);
        telemetry.update();
    }
}
