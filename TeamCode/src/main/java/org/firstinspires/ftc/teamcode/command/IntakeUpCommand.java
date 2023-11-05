package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeUpCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    ElapsedTime timer = new ElapsedTime();
    public IntakeUpCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        intakeSubsystem.setPower(-.6);
    }

    @Override
    public void execute() {
        if (timer.time(TimeUnit.MILLISECONDS) > 1000) {
            intakeSubsystem.setPosition(intakeSubsystem.up);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 2000;
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.setPower(0);
    }
}
