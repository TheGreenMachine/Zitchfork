package com.team1816.season;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.controlboard.ActionManager;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.SubsystemLooper;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.Util;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.DrivetrainTargets;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Objects;

import static com.team1816.lib.controlboard.ControlUtils.createAction;
import static com.team1816.lib.controlboard.ControlUtils.createHoldAction;

public class Robot extends TimedRobot {

    /**
     * Looper
     */
    private final Looper enabledLoop;
    private final Looper disabledLoop;

    /**
     * Controls
     */
    private IControlBoard controlBoard;
    private ActionManager actionManager;

    private final Infrastructure infrastructure;
    private final SubsystemLooper subsystemManager;

    /**
     * State Managers
     */
    private final Orchestrator orchestrator;
    private final RobotState robotState;

    /**
     * Subsystems
     */
    private final Drive drive;

    private final Camera camera;

    private final Collector collector;

    private final Elevator elevator;

    /**
     * Factory
     */
    private static RobotFactory factory;

    /**
     * Autonomous
     */
    private final AutoModeManager autoModeManager;

    private Thread autoTargetAlignThread;

    /**
     * Timing
     */
    private double loopStart;
    public static double looperDt;
    public static double robotDt;
    public static double autoStart;
    public static double teleopStart;

    private DoubleLogEntry robotLoopLogger;
    private DoubleLogEntry looperLogger;

    /**
     * Properties
     */
    private boolean faulted;


    /**
     * Instantiates the Robot by injecting all systems and creating the enabled and disabled loopers
     */
    Robot() {
        super();
        // initialize injector
        Injector.registerModule(new SeasonModule());
        enabledLoop = new Looper(this);
        disabledLoop = new Looper(this);
        drive = (Injector.get(Drive.Factory.class)).getInstance();
        collector = Injector.get(Collector.class);
        elevator = Injector.get(Elevator.class);
        camera = Injector.get(Camera.class);
        robotState = Injector.get(RobotState.class);
        orchestrator = Injector.get(Orchestrator.class);
        infrastructure = Injector.get(Infrastructure.class);
        subsystemManager = Injector.get(SubsystemLooper.class);
        autoModeManager = Injector.get(AutoModeManager.class);

        if (Constants.kLoggingRobot) {
            robotLoopLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/Robot");
            looperLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/RobotState");
        }
    }

    /**
     * Returns the static factory instance of the Robot
     *
     * @return RobotFactory
     */
    public static RobotFactory getFactory() {
        if (factory == null) factory = Injector.get(RobotFactory.class);
        return factory;
    }

    /**
     * Returns the length of the last loop that the Robot was on
     *
     * @return duration (ms)
     */
    public Double getLastRobotLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    /**
     * Returns the duration of the last enabled loop
     *
     * @return duration (ms)
     * @see Looper#getLastLoop()
     */
    public Double getLastSubsystemLoop() {
        return enabledLoop.isRunning() ? enabledLoop.getLastLoop() : disabledLoop.getLastLoop();
    }

    /**
     * Actions to perform when the robot has just begun being powered and is done booting up.
     * Initializes the robot by injecting the controlboard, and registering all subsystems.
     */
    @Override
    public void robotInit() {
        try {
            /** Register All Subsystems */

            // Remember to register our subsystems below! The subsystem manager deals with calling
            // readFromHardware and writeToHardware on a loop, but it can only call read/write it if it
            // can recognize the subsystem. To recognize your subsystem, just add it alongside the
            // drive, ledManager, and camera parameters.
            subsystemManager.setSubsystems(drive, camera, collector, elevator);

            /** Logging */
            if (Constants.kLoggingRobot) {
                var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
                var robotName = System.getenv("ROBOT_NAME");
                if (robotName == null) robotName = "default";
                var logFileDir = "/home/lvuser/";
                // if there is a USB drive use it
                if (Files.exists(Path.of("/media/sda1"))) {
                    logFileDir = "/media/sda1/";
                }
                if (RobotBase.isSimulation()) {
                    if (System.getProperty("os.name").toLowerCase().contains("win")) {
                        logFileDir = System.getenv("temp") + "\\";
                    } else {
                        logFileDir = System.getProperty("user.dir") + "/";
                    }
                }
                // start logging
                DataLogManager.start(logFileDir, "", Constants.kLooperDt);
                if (RobotBase.isReal()) {
                    Util.cleanLogFiles();
                }
                DriverStation.startDataLog(DataLogManager.getLog(), false);
            }

            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);

            // zeroing ypr - (-90) b/c our pigeon is mounted with the "y" axis facing forward.
            // Might change later for different robots.
            infrastructure.resetPigeon(Rotation2d.fromDegrees(-90));
            subsystemManager.zeroSensors();

            /** [Specific subsystem] not zeroed on boot up - letting ppl know */
            faulted = true;

            /** Register ControlBoard */
            controlBoard = Injector.get(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    createAction(
                        () -> controlBoard.getAsBool("zeroPose"),
                        () -> {
                            drive.zeroSensors(robotState.allianceColor == Color.BLUE ? Constants.kDefaultZeroingPose : Constants.kFlippedZeroingPose);
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("brakeMode"),
                        drive::setBraking
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("slowMode"),
                        drive::setSlowMode
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("intakeCob"),
                        (pressed) -> {
                            if (pressed) {
                                collector.setDesiredState(Collector.COLLECTOR_STATE.INTAKE);
                                controlBoard.setFullRumble(IControlBoard.CONTROLLER.OPERATOR, 0.75);
                            } else {
                                collector.setDesiredState(Collector.COLLECTOR_STATE.STOP);
                                controlBoard.setFullRumble(IControlBoard.CONTROLLER.OPERATOR, 0);
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("collectPos"),
                        () -> {
                            elevator.setDesiredElevatorHeightState(Elevator.HEIGHT_STATE.HUMAN_COLLECT);
                        }
                    ),
                    // Operator Gamepad
                    createHoldAction(
                        () -> controlBoard.getAsBool("outtake"),
                        (pressed) -> {
                            if (pressed) {
                                collector.setDesiredState(Collector.COLLECTOR_STATE.OUTTAKE);
                                controlBoard.setFullRumble(IControlBoard.CONTROLLER.OPERATOR, 0.75);
                            } else {
                                collector.setDesiredState(Collector.COLLECTOR_STATE.STOP);
                                controlBoard.setFullRumble(IControlBoard.CONTROLLER.OPERATOR, 0);
                            }
                        }
                     ),
                    createAction(
                        () -> controlBoard.getAsBool("siloPos"),
                        () -> {
                            elevator.setDesiredElevatorHeightState(Elevator.HEIGHT_STATE.SILO_DROP);
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoScore1"),
                        () -> {
                            orchestrator.autoScore(1);
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoScore2"),
                        () -> {
                            orchestrator.autoScore(2);
                        }
                    )
//                    createHoldAction(
//                        () -> controlBoard.getAsBool("emergencyJolt"),
//                        (pressed) -> {
//                            if (pressed) {
//                                elevator.setDesiredElevatorHeightState(Elevator.HEIGHT_STATE.HUMAN_COLLECT);
//                            } else {
//                                elevator.setDesiredElevatorHeightState(Elevator.HEIGHT_STATE.JOLT);
//                            }
//                            elevator.setDownReleased(pressed);
//                        }
                    //)
                    // Button Board Gamepad
                );
        } catch (Throwable t) {
            faulted = true;
        }
    }

    /**
     * Actions to perform when the robot has entered the disabled period
     */
    @Override
    public void disabledInit() {
        try {
            orchestrator.clearThreads();

            enabledLoop.stop();
            // Stop any running autos
            autoModeManager.stopAuto();

            if (autoModeManager.getSelectedAuto() == null) {
                autoModeManager.reset();
            }

            subsystemManager.stop();

            robotState.resetAllStates();
            drive.zeroSensors();

            disabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the autonomous period
     */
    @Override
    public void autonomousInit() {
        disabledLoop.stop();

        drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());

        collector.setDesiredState(Collector.COLLECTOR_STATE.STOP);
        elevator.setDesiredElevatorHeightState(Elevator.HEIGHT_STATE.STOP);
        // TODO: Set up subsystem states

        drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
        autoModeManager.startAuto();

        autoStart = Timer.getFPGATimestamp();
        enabledLoop.start();
    }

    /**
     * Actions to perform when the robot has entered the teleoperated period
     */
    @Override
    public void teleopInit() {
        try {
            disabledLoop.stop();

            infrastructure.startCompressor();
            //elevator.setDesiredElevatorHeightState(Elevator.HEIGHT_STATE.HUMAN_COLLECT); TODO Uncomment once elevator working!

            teleopStart = Timer.getFPGATimestamp();
            enabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the test period
     */
    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();

            enabledLoop.stop();
            disabledLoop.start();
            drive.zeroSensors();

            if (subsystemManager.testSubsystems()) {
                GreenLogger.log("ALL SYSTEMS PASSED");
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform periodically on the robot when the robot is powered
     */
    @Override
    public void robotPeriodic() {
        try {
            // updating loop timers
            Robot.looperDt = getLastSubsystemLoop();
            Robot.robotDt = getLastRobotLoop();
            loopStart = Timer.getFPGATimestamp();

            if (Constants.kLoggingRobot) {
                looperLogger.append(looperDt);
                robotLoopLogger.append(robotDt);
            }

            subsystemManager.outputToSmartDashboard(); // update shuffleboard for subsystem values
            robotState.outputToSmartDashboard(); // update robot state on field for Field2D widget
            autoModeManager.outputToSmartDashboard(); // update shuffleboard selected auto mode
        } catch (Throwable t) {
            faulted = true;
            GreenLogger.log(t.getMessage());
        }
    }

    /**
     * Actions to perform periodically when the robot is in the disabled period
     */
    @Override
    public void disabledPeriodic() {
        try {
            if (RobotController.getUserButton()) {
                drive.zeroSensors(Constants.kDefaultZeroingPose);
            } else {
                if (faulted) {
                    // Logic if falted
                }
            }

            if (RobotBase.isReal()) {
                // TODO: Logic for if the robot is not a simulation
                // TODO: Also don't forget to add logic to make faulted false.
            }

            // Periodically check if drivers changed desired auto - if yes, then update the robot's position on the field
            if (autoModeManager.update()) {
                drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());
                robotState.field
                    .getObject("Trajectory")
                    .setTrajectory(
                        autoModeManager.getSelectedAuto().getCurrentTrajectory()
                    );
            }

            if (drive.isDemoMode()) { // Demo-mode
                drive.update();
            }

        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform periodically when the robot is in the autonomous period
     */
    @Override
    public void autonomousPeriodic() {
        robotState.field
            .getObject("Trajectory")
            .setTrajectory(autoModeManager.getSelectedAuto().getCurrentTrajectory());
    }

    /**
     * Actions to perform periodically when the robot is in the teleoperated period
     */
    @Override
    public void teleopPeriodic() {
        try {
            manualControl();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Sets manual inputs for subsystems like the drivetrain when criteria met
     */
    public void manualControl() {
        actionManager.update();

        double rotation = controlBoard.getAsDouble("rotation");
        double rotateFix = controlBoard.getAsDouble("rotateFix") * 0.5;
        if (rotation > 0.1) {
            rotation += rotateFix;
        } else if (rotation < -0.1) {
            rotation -= rotateFix;
        }

        drive.setTeleopInputs(
            -controlBoard.getAsDouble("throttle"),
            -controlBoard.getAsDouble("strafe"),
            rotation
        );
    }

    /**
     * Actions to perform periodically when the robot is in the test period
     */
    @Override
    public void testPeriodic() {
    }
}
