package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

@Singleton
public class Collector extends Subsystem {

    //documentation please!! :))
    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param name String
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    private static final String NAME = "collector";
    private final IGreenMotor intakeMotor;
    private COLLECTOR_STATE desiredState = COLLECTOR_STATE.STOP;
    private boolean outputsChanged = false;
    public final double intakePower;
    public final double outtakePower;
    public Collector(String name, Infrastructure inf, RobotState rs) {
        super(name, inf, rs);
        intakeMotor = factory.getMotor(NAME, "intakeMotor");
        intakePower = factory.getConstant(NAME, "intakePower", 0.70);
        outtakePower = factory.getConstant(NAME, "outtakePower", 0.45);
    }
    public void setDesiredState(COLLECTOR_STATE desiredState) {
        this.desiredState = desiredState;
        outputsChanged = true;
    }
    public void outtakeGamePiece(boolean outtaking) {
        if (outtaking) {
            setDesiredState(COLLECTOR_STATE.OUTTAKE);
        } else {
            setDesiredState(COLLECTOR_STATE.STOP);
        }
    }

    @Override
    public void readFromHardware() {
        if (robotState.actualCollectorState != desiredState) {
            robotState.actualCollectorState = desiredState;
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP -> {
                    intakeMotor.set(ControlMode.Velocity, 0);
                }
                case INTAKE -> {
                    intakeMotor.set(ControlMode.Velocity, intakePower);
                }
                case OUTTAKE -> {
                    intakeMotor.set(ControlMode.Velocity, outtakePower);
                }
            }
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean testSubsystem() {
        return false;
    }
    public enum COLLECTOR_STATE {
        STOP,
        INTAKE,
        OUTTAKE
    }
}
