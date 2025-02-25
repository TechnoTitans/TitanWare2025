package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoOption;
import frc.robot.auto.Autos;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.state.GamepieceState;
import frc.robot.state.ReefState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.elevator.ElevatorArm;
import frc.robot.subsystems.superstructure.arm.intake.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Robot extends LoggedRobot {
    private static final String AKitLogPath = "/U/logs";
    private static final String HootLogPath = "/U/logs";

    public static final BooleanSupplier IsRedAlliance = () -> {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    };

    public final PowerDistribution powerDistribution = new PowerDistribution(
            RobotMap.PowerDistributionHub, PowerDistribution.ModuleType.kRev
    );

    public final Swerve swerve = new Swerve(
            Constants.CURRENT_MODE,
            HardwareConstants.GYRO,
            SwerveConstants.FrontLeftModule,
            SwerveConstants.FrontRightModule,
            SwerveConstants.BackLeftModule,
            SwerveConstants.BackRightModule
    );

    public final PhotonVision photonVision = new PhotonVision(
            Constants.CURRENT_MODE,
            swerve,
            swerve.getPoseEstimator()
    );

    public final Elevator elevator = new Elevator(
            Constants.CURRENT_MODE,
            HardwareConstants.ELEVATOR
    );
    public final ElevatorArm elevatorArm = new ElevatorArm(
            Constants.CURRENT_MODE,
            HardwareConstants.ELEVATOR_ARM
    );
    public final IntakeArm intakeArm = new IntakeArm(
            Constants.CURRENT_MODE,
            HardwareConstants.INTAKE_ARM
    );
    public final Superstructure superstructure = new Superstructure(elevator, elevatorArm, intakeArm);

    public final Intake intake = new Intake(
            Constants.CURRENT_MODE,
            HardwareConstants.INTAKE
    );

    public final ReefState reefState = new ReefState();
    public final GamepieceState gamePieceState = new GamepieceState(Constants.CURRENT_MODE, intake);
    public final ScoreCommands scoreCommands = new ScoreCommands(swerve, superstructure, intake, gamePieceState);

    public final Autos autos = new Autos(
            swerve,
            superstructure,
            intake,
            photonVision,
            scoreCommands,
            gamePieceState,
            reefState
    );
    public final AutoChooser autoChooser = new AutoChooser(
            new AutoOption(
                    "DoNothing",
                    autos::doNothing,
                    Constants.CompetitionType.COMPETITION
            )
    );

    public final CommandXboxController driverController = new CommandXboxController(RobotMap.MainController);
    public final CommandXboxController coController = new CommandXboxController(RobotMap.CoController);
    public final Alert driverControllerDisconnected = new Alert(
            "Driver controller not connected!",
            Alert.AlertType.kWarning
    );
    public final Alert coControllerDisconnected = new Alert(
            "Co controller not connected!",
            Alert.AlertType.kWarning
    );

    private final EventLoop teleopEventLoop = new EventLoop();
    private final EventLoop testEventLoop = new EventLoop();

    private final Trigger autonomousEnabled = RobotModeTriggers.autonomous();
    private final Trigger endgameTrigger = new Trigger(() -> DriverStation.getMatchTime() <= 20)
            .and(DriverStation::isFMSAttached)
            .and(RobotModeTriggers.teleop());

    final Supplier<ScoreCommands.ScorePosition> driverScorePositionSupplier =
                scoreCommands.getScorePositionSupplier(driverController);

    final Supplier<ScoreCommands.ScorePosition> coDriverScorePositionSupplier =
            scoreCommands.getScorePositionSupplier(coController);


    @Override
    public void robotInit() {
        if ((RobotBase.isReal() && Constants.CURRENT_MODE != Constants.RobotMode.REAL) ||
                (RobotBase.isSimulation() && Constants.CURRENT_MODE == Constants.RobotMode.REAL)) {
            DriverStation.reportWarning(
                    String.format(
                            "Potentially incorrect CURRENT_MODE \"%s\" specified, robot is running \"%s\"",
                            Constants.CURRENT_MODE,
                            RobotBase.getRuntimeType().toString()
                    ),
                    true
            );

            throw new RuntimeException("Incorrect CURRENT_MODE specified!");
        }

        // we never use LiveWindow, and apparently this causes loop overruns so disable it
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // register shutdown hook
        ToClose.hook();

        // disable joystick not found warnings when in sim
        DriverStation.silenceJoystickConnectionWarning(Constants.CURRENT_MODE == Constants.RobotMode.SIM);

        // record git metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        // no need to inspect this here because BuildConstants is a dynamically changing file upon compilation
        //noinspection RedundantSuppression
        switch (BuildConstants.DIRTY) {
            //noinspection DataFlowIssue,RedundantSuppression
            case 0 -> //noinspection UnreachableCode
                    Logger.recordMetadata("GitDirty", "All changes committed");
            //noinspection DataFlowIssue,RedundantSuppression
            case 1 -> //noinspection UnreachableCode
                    Logger.recordMetadata("GitDirty", "Uncommitted changes");
            //noinspection DataFlowIssue,RedundantSuppression
            default -> //noinspection UnreachableCode
                    Logger.recordMetadata("GitDirty", "Unknown");
        }

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                try {
                    Files.createDirectories(Paths.get(HootLogPath));
                    SignalLogger.setPath(HootLogPath);
                } catch (final IOException ioException) {
                    SignalLogger.setPath("/U");
                    DriverStation.reportError(
                            String.format(
                                    "Failed to create .hoot log path at \"%s\"! Falling back to default.\n%s",
                                    HootLogPath,
                                    ioException
                            ),
                            false
                    );
                }

                Logger.addDataReceiver(new WPILOGWriter(AKitLogPath));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                // log to working directory when running sim
                // setPath doesn't seem to work in sim (path is ignored and hoot files are always sent to /logs)
//                SignalLogger.setPath("/logs");
                Logger.addDataReceiver(new WPILOGWriter(""));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                setUseTiming(false);

                final String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(
                                LogFileUtil.addPathSuffix(logPath, "_sim"),
                                WPILOGWriter.AdvantageScopeOpenBehavior.AUTO)
                );
            }
        }

        powerDistribution.clearStickyFaults();
        powerDistribution.setSwitchableChannel(true);

        configureStateTriggers();
        configureAutos();
        configureButtonBindings(teleopEventLoop);

        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();
        ToClose.add(SignalLogger::stop);

        CommandScheduler.getInstance().onCommandInitialize(
                command -> Logger.recordOutput("Commands/Initialized", command.getName())
        );
        CommandScheduler.getInstance().onCommandFinish(
                command -> Logger.recordOutput("Commands/Finished", command.getName())
        );

        CommandScheduler.getInstance().onCommandInterrupt((interrupted, interrupting) -> {
            Logger.recordOutput(
                    "Commands/Interrupted",
                    interrupted.getName()
            );
            Logger.recordOutput(
                    "Commands/InterruptedRequirements",
                    LogUtils.getRequirementsFromSubsystems(interrupted.getRequirements())
            );
            Logger.recordOutput(
                    "Commands/Interrupter",
                    interrupting.isPresent() ? interrupting.get().getName() : "None"
            );
            Logger.recordOutput(
                    "Commands/InterrupterRequirements",
                    LogUtils.getRequirementsFromSubsystems(
                            interrupting.isPresent() ? interrupting.get().getRequirements() : Set.of()
                    )
            );
        });

        Logger.start();

        Logger.recordOutput("EmptyPose", new Pose3d());
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        VirtualSubsystem.run();

        driverControllerDisconnected.set(!driverController.getHID().isConnected());
        coControllerDisconnected.set(!coController.getHID().isConnected());

        Logger.recordOutput("ScorePosition/Driver", driverScorePositionSupplier.get());
        Logger.recordOutput("ScorePosition/CoDriver", coDriverScorePositionSupplier.get());

        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        //noinspection SuspiciousNameCombination
        swerve.setDefaultCommand(
                swerve.teleopDriveCommand(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX,
                        IsRedAlliance
                )
        );

        superstructure.toInstantSuperstructureGoal(Superstructure.Goal.STOW);
    }

    @Override
    public void teleopPeriodic() {
        teleopEventLoop.poll();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        driverController.leftBumper(testEventLoop).onTrue(Commands.runOnce(SignalLogger::stop));

        driverController.y(testEventLoop).whileTrue(
                elevatorArm.voltageSysIdCommand()
        );

//        driverController.y(testEventLoop).whileTrue(
//                swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward)
//        );
        driverController.a(testEventLoop).whileTrue(
                swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse)
        );
        driverController.b(testEventLoop).whileTrue(
                swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kForward)
        );
        driverController.x(testEventLoop).whileTrue(
                swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kReverse)
        );

        driverController.povDown().onTrue(
                Commands.sequence(
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L4).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L4).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L4).withTimeout(2.5),
                        superstructure.runSuperstructureGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.toInstantSuperstructureGoal(Superstructure.Goal.STOW)
                )
        );
    }

    @Override
    public void testPeriodic() {
        testEventLoop.poll();
    }

    @Override
    public void simulationPeriodic() {}

    public void configureStateTriggers() {
        endgameTrigger.onTrue(ControllerUtils.rumbleForDurationCommand(
                driverController.getHID(), GenericHID.RumbleType.kBothRumble, 0.5, 1)
        );

        intake.isCoralPresent.onTrue(ControllerUtils.rumbleForDurationCommand(
                driverController.getHID(), GenericHID.RumbleType.kBothRumble, 0.5, 1)
        );
    }

    public void configureAutos() {
        autonomousEnabled.whileTrue(Commands.deferredProxy(() -> autoChooser.getSelected().cmd()));

        autoChooser.addAutoOption(new AutoOption(
                "Cage0ToReef4",
                autos::cage0ToReef4,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "TwoPieceCage0ToReef5",
                autos::twoPieceCage0ToReef5,
                Constants.CompetitionType.COMPETITION
        ));
    }

    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        this.driverController.y(teleopEventLoop).onTrue(swerve.zeroRotationCommand());
        this.driverController.rightBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.FAST),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ));
        this.driverController.leftBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SLOW),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ));

        this.driverController.leftTrigger(0.5, teleopEventLoop).whileTrue(
                scoreCommands.intakeFacingClosestCoralStation(driverController::getLeftY, driverController::getLeftX)
        );

        this.driverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(scoreCommands.readyScoreAtPosition(driverScorePositionSupplier))
                .onFalse(scoreCommands.scoreAtPosition(driverScorePositionSupplier));

//        this.driverController.a(teleopEventLoop)
//                .whileTrue(scoreCommands.readyScoreNet(driverController::getLeftX))
//                .onFalse(scoreCommands.scoreNet());

//        this.driverController.b(teleopEventLoop)
//                .whileTrue(scoreCommands.readyScoreProcessor())
//                .onFalse(scoreCommands.scoreProcessor());

        this.driverController.x(teleopEventLoop)
                .whileTrue(superstructure.runSuperstructureGoal(Superstructure.Goal.CLIMB))
                .onFalse(superstructure.toInstantSuperstructureGoal(Superstructure.Goal.CLIMB_DOWN));

        this.coController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(scoreCommands.readyScoreAtPositionNoLineup(coDriverScorePositionSupplier))
                .onFalse(scoreCommands.scoreAtPosition(coDriverScorePositionSupplier));

        this.coController.x(teleopEventLoop).whileTrue(scoreCommands.intakeAlgaeFromGround());

        this.coController.y(teleopEventLoop).whileTrue(scoreCommands.readyIntakeUpperAlgae());

        this.coController.a(teleopEventLoop).whileTrue(scoreCommands.intakeLowerAlgae());
    }
}
