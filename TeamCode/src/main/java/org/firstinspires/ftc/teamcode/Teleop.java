package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends CommandOpMode {
    private MecanumCommand mecanumCommand;
    CommandScheduler scheduler = CommandScheduler.getInstance();
    Main robot;

    @Override
    public void initialize() {
        robot = new Main("tele", hardwareMap, telemetry);
        mecanumCommand = new MecanumCommand(robot.driveSubsystem, gamepad1, .5);

        waitForStart();
        schedule(mecanumCommand);
    }
}

