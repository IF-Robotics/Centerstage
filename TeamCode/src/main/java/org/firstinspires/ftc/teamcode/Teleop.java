package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.TeledriveCommand;

@TeleOp
public class Teleop extends CommandOpMode {
    private TeledriveCommand mecanumCommand;
    CommandScheduler scheduler = CommandScheduler.getInstance();
    Main robot;

    @Override
    public void initialize() {
        robot = new Main("tele", hardwareMap, telemetry);
        mecanumCommand = new TeledriveCommand(robot.driveSubsystem, gamepad1, .5);
    }

    @Override
    public void run() {
        super.run();
        robot.driveSubsystem.setDefaultCommand(mecanumCommand);
        //1 button for intake: spins & lowers intake, then when released it outtakes and comes up again
        //button for pickiing up pixels and setting up for scoring
        //button to go up 1 pixel every time
        //also have dump mode
        //hold down button to go up, when let go goes down to pull robot up (but not all the way)
        //reset button in case you accidentally hit the slides during match
        //single button to shoot drone -> 2 for safety or maybe endgame mode?
        //rumble for endgame
        //trigger or buttons to let go of pixels?


        //field centric arcade gm0 for dt
        // 3 speeds: normal (full speed), slow speed for scoring (left trigger), medium for intaking (right trigger)
    }
}

