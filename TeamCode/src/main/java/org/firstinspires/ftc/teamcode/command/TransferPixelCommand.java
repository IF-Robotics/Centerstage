package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class TransferPixelCommand extends CommandBase {
    private Telemetry telemetry;
    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    public TransferPixelCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, Telemetry telemtry) {
        this.telemetry = telemtry;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;

        addRequirements(intakeSubsystem, intakeSubsystem, clawSubsystem);
    }

    @Override
    public void initialize() {
        //TODO: add setup stuff here
    }

    @Override
    public void execute() {

    }
}
