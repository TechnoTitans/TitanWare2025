package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoOption;
import frc.robot.auto.Autos;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.selector.BranchSelector;
import frc.robot.state.GamepieceState;
import frc.robot.state.ReefState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.distal.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.proximal.ElevatorArm;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.Container;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.logging.LoggedCommandScheduler;
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

    public final ElevatorArm elevatorArm = new ElevatorArm(
            Constants.CURRENT_MODE,
            HardwareConstants.ELEVATOR_ARM
    );

    public final Elevator elevator = new Elevator(
            Constants.CURRENT_MODE,
            HardwareConstants.ELEVATOR
    );

    public final IntakeArm intakeArm = new IntakeArm(
            Constants.CURRENT_MODE,
            HardwareConstants.INTAKE_ARM
    );

    public final Superstructure superstructure = new Superstructure(elevatorArm, elevator, intakeArm);

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

    public final BranchSelector branchSelector = new BranchSelector();

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

    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger teleopEnabled = RobotModeTriggers.teleop();
    private final Trigger autonomousEnabled = RobotModeTriggers.autonomous();
    private final Trigger endgameTrigger = new Trigger(() -> DriverStation.getMatchTime() <= 20)
            .and(DriverStation::isFMSAttached)
            .and(RobotModeTriggers.teleop());

    final Supplier<ScoreCommands.ScorePosition> rawScorePositionSupplier = branchSelector::getSelected;
    final Supplier<ScoreCommands.ScorePosition> scorePositionSupplier = () -> {
            final ScoreCommands.ScorePosition rawScorePosition = rawScorePositionSupplier.get();
            if (!gamePieceState.hasCoral.getAsBoolean()) {
                return new ScoreCommands.ScorePosition(rawScorePosition.side(), ScoreCommands.Level.L1);
            }
            return rawScorePosition;
    };

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

                DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
                DriverStationSim.notifyNewData();

                autonomousEnabled.whileTrue(
                        Commands.waitSeconds(15)
                                .andThen(() -> {
                                    DriverStationSim.setEnabled(false);
                                    DriverStationSim.notifyNewData();
                                })
                );
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

        LoggedCommandScheduler.init(CommandScheduler.getInstance());

        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();
        ToClose.add(SignalLogger::stop);

        Logger.start();

        Logger.recordOutput("EmptyPose", Pose3d.kZero);
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        RefreshAll.refreshAll();

        CommandScheduler.getInstance().run();
        VirtualSubsystem.run();

        driverControllerDisconnected.set(!driverController.getHID().isConnected());
        coControllerDisconnected.set(!coController.getHID().isConnected());

        LoggedCommandScheduler.periodic();

        Logger.recordOutput("RawScorePosition", rawScorePositionSupplier.get());
        Logger.recordOutput("ScorePosition", scorePositionSupplier.get());
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
    }

    @Override
    public void teleopPeriodic() {
        teleopEventLoop.poll();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        driverController.leftBumper(testEventLoop).onTrue(Commands.runOnce(SignalLogger::stop));

//        driverController.a(testEventLoop).whileTrue(
//                swerve.wheelRadiusCharacterization()
//        );

        final Container<Pose2d> pose2dContainer = Container.empty();
        driverController.x(testEventLoop).whileTrue(
                Commands.sequence(
                        pose2dContainer.set(() -> swerve.getPose().transformBy(
                                new Transform2d(Units.feetToMeters(15), 0, Rotation2d.kZero)
                        )),
                        Commands.parallel(
                                Commands.run(() -> Logger.recordOutput("NewDist", pose2dContainer.value.getTranslation().getNorm())),
                                Commands.run(() -> Logger.recordOutput("Robot", swerve.getPose().getTranslation().getNorm())),
                                swerve.runToPose(pose2dContainer)
                        )
                )
        );

        driverController.b().whileTrue(swerve.wheelRadiusCharacterization());

        driverController.y(testEventLoop).whileTrue(
                intakeArm.pivotVoltageSysIdCommand()
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );

//        driverController.y(testEventLoop).whileTrue(
//                swerve.angularVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward)
//        );
//        driverController.a(testEventLoop).whileTrue(
//                swerve.angularVoltageSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse)
//        );
//        driverController.b(testEventLoop).whileTrue(
//                swerve.angularVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kForward)
//        );
//        driverController.x(testEventLoop).whileTrue(
//                swerve.angularVoltageSysIdDynamicCommand(SysIdRoutine.Direction.kReverse)
//        );

        driverController.povDown().onTrue(
                Commands.sequence(
                        superstructure.runGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L4).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L4).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L2).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L3).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L4).withTimeout(2.5),
                        superstructure.runGoal(Superstructure.Goal.L1).withTimeout(2.5),
                        superstructure.toInstantGoal(Superstructure.Goal.STOW)
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

        disabled.onTrue(swerve.stopCommand());
        teleopEnabled.onTrue(superstructure.forceGoal(Superstructure.Goal.STOW));
    }

    public void configureAutos() {
        autonomousEnabled.whileTrue(Commands.deferredProxy(() -> autoChooser.getSelected().cmd()));

        autoChooser.addAutoOption(new AutoOption(
                "TwoPieceCage0",
                autos::twoPieceCage0,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "TwoPieceCage1",
                autos::twoPieceCage1,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "TwoPieceCage2",
                autos::twoPieceCage2,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "TwoPieceCage3",
                autos::twoPieceCage3,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "TwoPieceCage4",
                autos::twoPieceCage4,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "TwoPieceCage5",
                autos::twoPieceCage5,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "Straight",
                autos::straight,
                Constants.CompetitionType.TESTING
        ));

        autoChooser.addAutoOption(new AutoOption(
                "ThreePieceCage1ToReef4And5",
                autos::threePieceCage1ToReef4And5,
                Constants.CompetitionType.TESTING
        ));

        autoChooser.addAutoOption(new AutoOption(
                "ThreePieceCage4ToReef2And1",
                autos::threePieceCage4ToReef2And1,
                Constants.CompetitionType.TESTING
        ));
    }

    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        this.driverController.rightBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.FAST),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ).withName("SwerveSpeedFast"));

        this.driverController.leftBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SLOW),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ).withName("SwerveSpeedSlow"));

        this.driverController.leftTrigger(0.5, teleopEventLoop).whileTrue(
                scoreCommands.intakeFacingClosestCoralStation(driverController::getLeftY, driverController::getLeftX)
        );

        this.driverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(scoreCommands.scoreAtFixedPosition(scorePositionSupplier));

        this.driverController.y(teleopEventLoop).whileTrue(scoreCommands.descoreUpperAlgae());

        this.driverController.a(teleopEventLoop).whileTrue(scoreCommands.descoreLowerAlgae());

        this.driverController.x(teleopEventLoop).whileTrue(scoreCommands.scoreBarge());

        this.driverController.b(teleopEventLoop)
                .whileTrue(scoreCommands.readyClimb(driverController::getLeftY, driverController::getLeftX))
                .onFalse(scoreCommands.climb());

        this.driverController.start().whileTrue(scoreCommands.processor());

        this.coController.rightBumper(teleopEventLoop)
                .whileTrue(superstructure.runGoal(Superstructure.Goal.CLIMB))
                .onFalse(superstructure.toInstantGoal(Superstructure.Goal.CLIMB_DOWN));

        this.coController.leftBumper(teleopEventLoop).whileTrue(intake.scoreAlgae());

        this.coController.start(teleopEventLoop)
                .whileTrue(superstructure.toGoal(Superstructure.Goal.CLIMB));

        this.coController.rightTrigger(0.5, teleopEventLoop)
                .negate()
                .and(coController.povDown())
                .whileTrue(intake.ejectCoral())
                .onFalse(intake.instantStopCommand());

        this.coController.b(teleopEventLoop)
                .whileTrue(scoreCommands.readyScoreProcessor())
                .onFalse(scoreCommands.scoreProcessor());

        this.coController.leftTrigger(0.5, teleopEventLoop)
                .whileTrue(superstructure.toGoal(Superstructure.Goal.HP)
                        .alongWith(intake.intakeCoralHP())
                );

        this.coController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(scoreCommands.readyScoreAtPositionNoLineup(rawScorePositionSupplier))
                .onFalse(scoreCommands.scoreAtPosition(rawScorePositionSupplier));

        this.coController.x(teleopEventLoop).whileTrue(scoreCommands.intakeAlgaeFromGround());

        this.coController.y(teleopEventLoop).whileTrue(scoreCommands.intakeUpperAlgae());

        this.coController.a(teleopEventLoop).whileTrue(scoreCommands.intakeLowerAlgae());
    }
}
