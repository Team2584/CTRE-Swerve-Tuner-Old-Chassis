package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine firsttest() {
        final AutoRoutine routine = m_factory.newRoutine("test");
        final AutoTrajectory test = routine.trajectory("test");

        routine.active().onTrue(
            test.resetOdometry()
                .andThen(test.cmd())
        );
        return routine;
    }

    
}