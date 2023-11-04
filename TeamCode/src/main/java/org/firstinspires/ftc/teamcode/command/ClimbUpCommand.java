package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.ClimbSubsystem;

public class ClimbUpCommand extends CommandBase {
    ClimbSubsystem climb;
    public ClimbUpCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
        climb.motor.setTargetPosition(2100);
    }

    @Override
    public void initialize() {
        climb.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climb.setPower(1);
    }

    @Override
    public void execute() {
//        climb.setPower(1);
    }

    @Override
    public boolean isFinished() {
        return false;//climb.getPosition() > 2000;
    }

    @Override
    public void end(boolean interrupted) {
        climb.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climb.setPower(0);
    }
}
