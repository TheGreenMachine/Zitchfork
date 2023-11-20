package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Elevator;

public class ElevatorAction implements AutoAction {

    private final RobotState robotState;
    private final Elevator elevator;

    private final Elevator.HEIGHT_STATE desiredHeightState;

    public ElevatorAction(Elevator.HEIGHT_STATE height) {
        robotState = Injector.get(RobotState.class);
        elevator = Injector.get(Elevator.class);

        desiredHeightState = height;
    }

    @Override
    public void start() {
        GreenLogger.log("Setting elevator to height:  " + desiredHeightState.name());
        elevator.setDesiredElevatorHeightState(desiredHeightState);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return robotState.actualElevatorHeightState == desiredHeightState;
    }

    @Override
    public void done() {
        GreenLogger.log("Elevator action completed: Elevator height at: " + desiredHeightState.name());
    }
}
