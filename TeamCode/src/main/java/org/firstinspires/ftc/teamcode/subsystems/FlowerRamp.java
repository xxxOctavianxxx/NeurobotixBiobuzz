package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Rampa de la intake (se duce sub bilele de la floare): STOWED -> DEPLOYING -> DEPLOYED -> STOWING. Mișcare lină. */
public class FlowerRamp {
    public enum State { STOWED, DEPLOYING, DEPLOYED, STOWING }

    private final Servo servo;
    private State state = State.STOWED;
    private double position = RobotConfig.RAMP_STOWED_POS;
    private final ElapsedTime dt = new ElapsedTime();

    public FlowerRamp(HardwareMap hw) {
        servo = hw.get(Servo.class, RobotConfig.FLOWER_RAMP);
        if (RobotConfig.RAMP_REVERSED) servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(position);
        dt.reset();
    }

    public void deploy() { if (state != State.DEPLOYED) state = State.DEPLOYING; }
    public void stow()   { if (state != State.STOWED)   state = State.STOWING; }

    /** Coboară dacă e ridicată, ridică dacă e coborâtă (sau în mișcare spre jos). */
    public void toggle() {
        if (state == State.STOWED || state == State.STOWING) deploy();
        else stow();
    }

    public State state() { return state; }
    public double position() { return position; }

    private static double approach(double current, double goal, double maxStep) {
        double delta = goal - current;
        if (Math.abs(delta) <= maxStep) return goal;
        return current + Math.signum(delta) * maxStep;
    }

    public void update() {
        double elapsed = dt.seconds();
        dt.reset();
        double step = RobotConfig.RAMP_SPEED_PER_SECOND * elapsed;

        if (state == State.DEPLOYING) {
            position = approach(position, RobotConfig.RAMP_DEPLOYED_POS, step);
            if (position == RobotConfig.RAMP_DEPLOYED_POS) state = State.DEPLOYED;
        } else if (state == State.STOWING) {
            position = approach(position, RobotConfig.RAMP_STOWED_POS, step);
            if (position == RobotConfig.RAMP_STOWED_POS) state = State.STOWED;
        }
        servo.setPosition(position);
    }
}
