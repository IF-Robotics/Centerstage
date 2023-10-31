package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.ClimbSubsystem;

public class ClimbDownCommand extends CommandBase {
    private ClimbSubsystem climb;

    public ClimbDownCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setPower(-1);
    }

    @Override
    public boolean isFinished() {
        return climb.getPosition() < 900;
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            //manual control has been taken over
        } else {
            climb.setPower(-.2);
        }
    }
}
