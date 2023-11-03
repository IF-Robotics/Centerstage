package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class ArmFlipCommand extends CommandBase {
    ArmSubsystem arm;
    ClawSubsystem claw;
    ElapsedTime timer = new ElapsedTime();
    boolean isFlipped = false, isWristDown = true;
    Telemetry telemetry;

    public ArmFlipCommand(ArmSubsystem arm, ClawSubsystem claw, Telemetry telemetry) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm, claw);
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

        if(timer.milliseconds() > 600) {
            if(isWristDown) {
                arm.setArm(arm.armDown);
            } else {
                arm.setArm(arm.armFlip);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 900;
    }

    @Override
    public void end(boolean isInterrupted) {
        if(isWristDown) {
            claw.setLower(claw.openL);
            claw.setUpper(claw.openU);
        } else {
            claw.setLower(claw.closeL);
            claw.setUpper(claw.closeU);
        }
    }
}
