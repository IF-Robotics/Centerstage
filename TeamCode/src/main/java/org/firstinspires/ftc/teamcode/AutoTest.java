package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoTest extends CommandOpMode {
Main robot;
PropPipeline.Position position = PropPipeline.Position.unknown;
    @Override
    public void initialize() {
        robot = new Main("auto", hardwareMap, telemetry);
        telemetry.addData("position", robot.aPipe.getPosition());
        telemetry.addData("x-value", robot.aPipe.getX());
        telemetry.addData("y-value", robot.aPipe.getY());
        telemetry.addData("max area", robot.aPipe.getMaxContour());
        telemetry.update();
        configureCommands();
    }

    @Override
    public void run() {
        position = robot.aPipe.getPosition();

    }

    private void configureCommands() {
        //create your commands here
        //for example, to drive, for powers of .7 with multiplier of 1
//        robot.driveSubsystem.setAllPower(.7, .7, .7, .7, 1);

        //to set runmode use
//        robot.driveSubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
