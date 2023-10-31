package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.command.ClimbDownCommand;
import org.firstinspires.ftc.teamcode.command.ClimbUpCommand;
import org.firstinspires.ftc.teamcode.command.ReadyScoreCommand;
import org.firstinspires.ftc.teamcode.command.TeledriveCommand;
import org.firstinspires.ftc.teamcode.command.TransferPixelCommand;

@TeleOp
public class Teleop extends CommandOpMode {
    private TeledriveCommand mecanumCommand;
    private ReadyScoreCommand readyScore;
    private Command intakeDown, intakeUp, shootPlane, resetIMU, climbUp, climbDown, climbDefault;
    CommandScheduler scheduler = CommandScheduler.getInstance();
    private Button up, down, a1, x2, lb2, rb2;
    private Trigger rt2;
    private GamepadEx gpad2;
    private GamepadEx gpad1;
    private TransferPixelCommand transfer;
    Main robot;

    @Override
    public void initialize() {
        gpad2 = new GamepadEx(gamepad2);
        gpad1 = new GamepadEx(gamepad1);
        configureButtons();
        robot = new Main("tele", hardwareMap, telemetry);
        configureCommands();
    }

    @Override
    public void run() {
        super.run();
        robot.driveSubsystem.setDefaultCommand(mecanumCommand);
//        robot.climbSubsystem.setDefaultCommand(climbDefault);

        //1 button for intake: spins & lowers intake, then when released it outtakes and comes up again
        x2.whenPressed(intakeDown);
        x2.whenReleased(intakeUp);

        //reset IMU
        a1.whenPressed(resetIMU);

        //climb
        rb2.whileHeld(climbUp);
        rb2.whenReleased(climbDown);
        rt2.whileActiveContinuous(climbDefault);

        //rumble for endgame
        if(getRuntime() > 90) {
            gamepad2.rumble(1000);
            gamepad1.rumble(1000);
        }

        //button for picking up pixels and setting up for scoring
        /*down.whenPressed(transfer);

        //button to go up 1 pixel every time
        //also have dump mode
        up.whenPressed(readyScore);

        //hold down button to go up, when let go goes down to pull robot up (but not all the way)
        //reset button in case you accidentally hit the slides during match
        //single button to shoot drone -> 2 for safety or maybe endgame mode?
        rb2.whenPressed(shootPlane);

        //trigger or buttons to let go of pixels?
//        lb2.whenPressed() TODO: fix the fact that there is multiple claws

        //field centric arcade gm0 for dt
        // 3 speeds: normal (full speed), slow speed for scoring (left trigger), medium for intaking (right trigger) --> integrated in
    */}

    public void configureCommands() {
        mecanumCommand = new TeledriveCommand(robot.driveSubsystem, hardwareMap, gamepad1);
        intakeDown = new InstantCommand(() -> { robot.intakeSubsystem.setPosition(robot.intakeSubsystem.down);
            robot.intakeSubsystem.setPower(1);}, robot.intakeSubsystem);
        intakeUp = new InstantCommand(() -> {robot.intakeSubsystem.setPosition(robot.intakeSubsystem.up); robot.intakeSubsystem.setPower(0);},
            robot.intakeSubsystem);
        shootPlane = new InstantCommand(() -> {robot.airplaneSubsystem.setPosition(robot.airplaneSubsystem.shoot);}, robot.airplaneSubsystem);
        transfer = new TransferPixelCommand(robot.intakeSubsystem, robot.armSubsystem, robot.clawSubsystem, telemetry);
        resetIMU = new InstantCommand(() -> robot.driveSubsystem.resetIMU());
        climbDefault = new RunCommand(() -> robot.climbSubsystem.setPower(1, gamepad2), robot.climbSubsystem);
        climbUp = new ClimbUpCommand(robot.climbSubsystem);
        climbDown = new ClimbDownCommand(robot.climbSubsystem);
    }

    public void configureButtons() {
        up = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_UP);
        down = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_DOWN);
        a1 = new GamepadButton(gpad1, GamepadKeys.Button.A);
        x2 = new GamepadButton(gpad2, GamepadKeys.Button.X);
        lb2 = new GamepadButton(gpad2, GamepadKeys.Button.LEFT_BUMPER);
        rb2 = new GamepadButton(gpad2, GamepadKeys.Button.RIGHT_BUMPER);
        rt2 = new Trigger(() -> (gamepad2.right_trigger > .3));
    }
}

