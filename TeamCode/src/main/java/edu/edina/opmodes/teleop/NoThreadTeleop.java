package edu.edina.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.edina.library.util.Stickygamepad;

@TeleOp(name = "DriveMe", group = "teleop")
@Disabled
public class NoThreadTeleop extends OpMode {
    private NoThreadRobot robot;
    private Stickygamepad _gamepad1;
    private Stickygamepad _gamepad2;


    public void init() {
        _gamepad1 = new Stickygamepad(gamepad1);
        _gamepad2 = new Stickygamepad(gamepad2);
        robot = new NoThreadRobot(this, telemetry);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        _gamepad1.update();
        _gamepad2.update();

        // set things into the robot from the gamepad or other sensors

        robot.drive.setDriveProperties(gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, _gamepad1.dpad_left, _gamepad1.dpad_up, _gamepad1.dpad_right);

        /*
        robot.lift.setLiftProperties(gamepad2.right_stick_y, _gamepad2.dpad_left, _gamepad2.dpad_right,
                gamepad2.left_bumper, gamepad2.right_bumper, _gamepad2.dpad_up, _gamepad2.dpad_down,
                _gamepad2.x, _gamepad2.y, _gamepad2.b, _gamepad2.a);

        robot.intake.setIntakeProperties(_gamepad1.x, (gamepad1.left_trigger != 0),
                (gamepad1.right_trigger != 0), gamepad1.right_bumper, gamepad1.left_bumper,
                _gamepad1.a, _gamepad1.y);
         */

        robot.update();

        robot.telemetry();
    }

    @Override
    public  void stop() {

    }
}
