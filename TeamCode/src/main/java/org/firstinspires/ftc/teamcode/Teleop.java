package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.ArmDownCommand;
import org.firstinspires.ftc.teamcode.command.ArmFlipCommand;
import org.firstinspires.ftc.teamcode.command.ArmUpCommand;
import org.firstinspires.ftc.teamcode.command.ClimbDownCommand;
import org.firstinspires.ftc.teamcode.command.ClimbUpCommand;
import org.firstinspires.ftc.teamcode.command.IntakeUpCommand;
import org.firstinspires.ftc.teamcode.command.ReadyScoreCommand;
import org.firstinspires.ftc.teamcode.command.TeledriveCommand;
import org.firstinspires.ftc.teamcode.command.TransferPixelCommand;

@Config
@TeleOp
public class Teleop extends CommandOpMode {
    private TeledriveCommand mecanumCommand;
    private ReadyScoreCommand readyScore;
    private ArmDownCommand armDown;
    private Command intakeDown, intakeUp, closeClaw, openClaw, closeLower, closeUpper, armUp, shootPlane, flipCommand, manualSlide, resetIMU, climbUp, climbDown, climbDefault;
    CommandScheduler scheduler = CommandScheduler.getInstance();
    private Button up2, down2, up1, a1, a2, b2, x2, y2, lb2, rb2, l2, r2;
    private Trigger lt2, rt2;
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
        down2.whenPressed(intakeDown);
        down2.whenReleased(intakeUp);
        //claw toggle
        up2.whenPressed(closeClaw);
        b2.whenPressed(openClaw);
        l2.whenHeld(closeLower);
        r2.whenHeld(closeUpper);

        //slides
        lt2.whileActiveContinuous(manualSlide);

        //arm
        y2.whenPressed(armUp);
        a2.whenPressed(armDown);
        x2.whenPressed(flipCommand);
        telemetry.addData("arm Command", robot.armSubsystem.getCurrentCommand());

        //reset IMU
        a1.whenPressed(resetIMU);

        //climb
        rb2.whileHeld(climbUp);
        rb2.whenReleased(climbDown);
        rt2.whileActiveContinuous(climbDefault);

        //airplane
        up1.whenPressed(shootPlane);

        //rumble for endgame
        if(getRuntime() >  80 && getRuntime() < 81) {
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
        intakeUp = new IntakeUpCommand(robot.intakeSubsystem);
        closeClaw = new InstantCommand(() -> {robot.clawSubsystem.close();}, robot.clawSubsystem);
        openClaw = new InstantCommand(() -> {robot.clawSubsystem.open();}, robot.clawSubsystem);
        closeLower = new InstantCommand(()-> robot.clawSubsystem.setLower(robot.clawSubsystem.closeL));
        closeUpper = new InstantCommand(()-> robot.clawSubsystem.setUpper(robot.clawSubsystem.closeU));
        armUp = new ArmUpCommand(robot.armSubsystem, 600, robot.armSubsystem.armUp, robot.armSubsystem.wristUp, 200);
        armDown = new ArmDownCommand(robot.armSubsystem, robot.clawSubsystem, 0, robot.armSubsystem.armFlip, robot.armSubsystem.wristFlip, 600);
        flipCommand = new ArmFlipCommand(robot.armSubsystem, robot.clawSubsystem, telemetry);
        shootPlane = new InstantCommand(() -> {robot.airplaneSubsystem.setPosition(robot.airplaneSubsystem.shoot);}, robot.airplaneSubsystem);
        manualSlide = new InstantCommand(() -> {robot.armSubsystem.addSlidePosition( (int) (-10 * gamepad2.left_stick_y));});
        transfer = new TransferPixelCommand(robot.intakeSubsystem, robot.armSubsystem, robot.clawSubsystem, telemetry);
        resetIMU = new InstantCommand(() -> robot.driveSubsystem.resetIMU());
        climbDefault = new RunCommand(() -> robot.climbSubsystem.setPower(1, gamepad2), robot.climbSubsystem);
        climbUp = new ClimbUpCommand(robot.climbSubsystem);
        climbDown = new ClimbDownCommand(robot.climbSubsystem);
        shootPlane = new InstantCommand(()-> robot.airplaneSubsystem.setPosition(robot.airplaneSubsystem.shoot), robot.airplaneSubsystem);
    }

    public void configureButtons() {
        up2 = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_UP);
        down2 = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_DOWN);
        a1 = new GamepadButton(gpad1, GamepadKeys.Button.A);
        a2 = new GamepadButton(gpad2, GamepadKeys.Button.A);
        b2 = new GamepadButton(gpad2, GamepadKeys.Button.B);
        x2 = new GamepadButton(gpad2, GamepadKeys.Button.X);
        y2 = new GamepadButton(gpad2, GamepadKeys.Button.Y);
        lb2 = new GamepadButton(gpad2, GamepadKeys.Button.LEFT_BUMPER);
        rb2 = new GamepadButton(gpad2, GamepadKeys.Button.RIGHT_BUMPER);
        lt2 = new Trigger(() -> (gamepad2.left_trigger > .3));
        rt2 = new Trigger(() -> (gamepad2.right_trigger > .3));
        l2 = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_LEFT);
        r2 = new GamepadButton(gpad2, GamepadKeys.Button.DPAD_RIGHT);
        up1 = new GamepadButton(gpad1, GamepadKeys.Button.DPAD_UP);
    }
}

