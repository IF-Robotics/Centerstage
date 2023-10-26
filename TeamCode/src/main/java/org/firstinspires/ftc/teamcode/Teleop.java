package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.ReadyScoreCommand;
import org.firstinspires.ftc.teamcode.command.TeledriveCommand;
import org.firstinspires.ftc.teamcode.command.TransferPixelCommand;
import org.firstinspires.ftc.teamcode.subsystem.AirplaneSubsystem;

@TeleOp
public class Teleop extends CommandOpMode {
    private TeledriveCommand mecanumCommand;
    private ReadyScoreCommand readyScore;
    private Command intakeDown, intakeUp, shootPlane;
    CommandScheduler scheduler = CommandScheduler.getInstance();
    private Button up, down, x2, lb2, rb2;
    private GamepadEx gpad2;
    private TransferPixelCommand transfer;
    Main robot;

    @Override
    public void initialize() {
        gpad2 = new GamepadEx(gamepad2);
        configureButtons();
        robot = new Main("tele", hardwareMap, telemetry);
        configureCommands();
    }

    @Override
    public void run() {
        super.run();
        robot.driveSubsystem.setDefaultCommand(mecanumCommand);

        //1 button for intake: spins & lowers intake, then when released it outtakes and comes up again
        /*x2.whenPressed(intakeDown);
        x2.whenReleased(intakeUp);

        //button for picking up pixels and setting up for scoring
        down.whenPressed(transfer);

        //button to go up 1 pixel every time
        //also have dump mode
        up.whenPressed(readyScore);

        //hold down button to go up, when let go goes down to pull robot up (but not all the way)
        //reset button in case you accidentally hit the slides during match
        //single button to shoot drone -> 2 for safety or maybe endgame mode?
        rb2.whenPressed(shootPlane);

        //rumble for endgame
        if(getRuntime() > 90) {
            gamepad2.rumble(1000);
            gamepad1.rumble(1000);
        }

        //trigger or buttons to let go of pixels?
//        lb2.whenPressed() TODO: fix the fact that there is multiple claws

        //field centric arcade gm0 for dt
        // 3 speeds: normal (full speed), slow speed for scoring (left trigger), medium for intaking (right trigger) --> integrated in
    */}

    public void configureCommands() {
        mecanumCommand = new TeledriveCommand(robot.driveSubsystem, gamepad1);
        intakeDown = new InstantCommand(() -> { robot.intakeSubsystem.setPosition(robot.intakeSubsystem.down);
            robot.intakeSubsystem.setPower(.5);});
        intakeUp = new InstantCommand(() -> {robot.intakeSubsystem.setPosition(robot.intakeSubsystem.up); robot.intakeSubsystem.setPower(0);});
        shootPlane = new InstantCommand(() -> {robot.airplaneSubsystem.setPosition(robot.airplaneSubsystem.shoot);});
        transfer = new TransferPixelCommand(robot.intakeSubsystem, robot.armSubsystem, robot.clawSubsystem, telemetry);
    }

    public void configureButtons() {
        up = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_UP);
        down = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_DOWN);
        x2 = new GamepadButton(gpad2, GamepadKeys.Button.X);
        lb2 = new GamepadButton(gpad2, GamepadKeys.Button.LEFT_BUMPER);
        rb2 = new GamepadButton(gpad2, GamepadKeys.Button.RIGHT_BUMPER);
    }
}

