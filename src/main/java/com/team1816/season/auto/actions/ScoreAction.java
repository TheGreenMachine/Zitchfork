package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class ScoreAction extends SeriesAction {

    public ScoreAction(int cobs) {
        super(
            new SeriesAction(
                new CollectAction(Collector.COLLECTOR_STATE.STOP),
                new ElevatorAction(Elevator.HEIGHT_STATE.SILO_DROP),
                new CollectAction(Collector.COLLECTOR_STATE.OUTTAKE),
                new WaitAction(cobs == 1 ? 0.2 : 0.3), // Tune these by seeing how long it takes to outtake one cob
                //May need 2 collect-wait blocks
                // resetting elevator / collector to starting states
                new CollectAction(Collector.COLLECTOR_STATE.STOP),
                new ElevatorAction(Elevator.HEIGHT_STATE.STOW)
            )
        );
    }

    @Override
    public void start() {
        GreenLogger.log("Started Score Action Sequence");
        super.start();
    }

    @Override
    public void done() {
        GreenLogger.log("Score Action Sequence Completed");
        super.done();
    }
}
