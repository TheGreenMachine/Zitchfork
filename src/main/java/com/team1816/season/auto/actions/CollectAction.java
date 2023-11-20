package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Collector;

public class CollectAction implements AutoAction {

    private RobotState robotState;
    private Collector collector;

    private Collector.COLLECTOR_STATE desiredCollectorState;

    public CollectAction(Collector.COLLECTOR_STATE collectorState) {
        robotState = Injector.get(RobotState.class);
        collector = Injector.get(Collector.class);
        this.desiredCollectorState = collectorState;
    }

    @Override
    public void start() {
        if (desiredCollectorState != null) {
            GreenLogger.log("Setting collector to state: " + desiredCollectorState.name());
            collector.setDesiredState(desiredCollectorState);
        }
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return robotState.actualCollectorState == desiredCollectorState;
    }

    @Override
    public void done() {
        GreenLogger.log("Collect action completed");
    }
}
