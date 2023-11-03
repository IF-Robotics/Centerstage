package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class ArmFlipCommand extends CommandBase {
    ArmSubsystem arm;
    ElapsedTime timer = new ElapsedTime();
    boolean isFlipped = false, isWristDown = true;
    Telemetry telemetry;

    public ArmFlipCommand(ArmSubsystem arm, Telemetry telemetry) {
        this.arm = arm;
        addRequirements(arm);
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.setArm(arm.armNeutral);
        isFlipped = false;
    }

    @Override
    public void execute() {
        if((timer.milliseconds() > 300) && !isFlipped) {
            isFlipped = true;
            if(arm.getWrist() > .5) {
                arm.setWrist(arm.wristFlip);
                isWristDown = false;
            } else {
                arm.setWrist(arm.wristDown);
                isWristDown = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 600;
    }

    @Override
    public void end(boolean isInterrupted) {
        if(!isInterrupted) {
            if(isWristDown) {
                arm.setArm(arm.armDown);
            } else {
                arm.setArm(arm.armFlip);
            }
        }
    }
}
