package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.ClimbSubsystem;

public class ClimbUpCommand extends CommandBase {
    ClimbSubsystem climb;
    public ClimbUpCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.setPower(1);
    }

    @Override
    public boolean isFinished() {
        return climb.getPosition() > 2100;
    }

    @Override
    public void end(boolean interrupted) {;
    }
}
