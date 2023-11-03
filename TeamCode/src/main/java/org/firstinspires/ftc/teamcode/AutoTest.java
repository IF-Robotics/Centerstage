package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
    }

    @Override
    public void run() {
        position = robot.aPipe.getPosition();
    }
}
