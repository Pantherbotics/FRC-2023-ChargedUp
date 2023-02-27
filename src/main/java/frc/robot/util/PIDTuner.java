package frc.robot.util;

public interface PIDTuner {

    public void alterP(double val);

    public void alterI(double val);

    public void alterD(double val);

    public String getIdentifier();
}

enum TuningMode {
    FocusP,
    FocusI,
    FocusD
}