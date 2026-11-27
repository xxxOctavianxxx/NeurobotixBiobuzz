package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Flywheel pe viteza reala: feedforward + PID. Nu se opreste niciodata, merge la viteza de repaus. */
public class Flywheel {
    public enum State { SPINNING_UP, READY }

    private final DcMotorEx motor;
    private State state = State.SPINNING_UP;
    private double targetVelocity = RobotConfig.FLYWHEEL_IDLE_VELOCITY;
    private double integral = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stableTimer = new ElapsedTime();

    public Flywheel(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, RobotConfig.FLYWHEEL);
        motor.setDirection(RobotConfig.dir(RobotConfig.FLYWHEEL_REVERSED));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        loopTimer.reset();
        stableTimer.reset();
    }

    /** Cere o viteza (ticks/s). Daca se schimba mult, restabilizeaza. */
    public void spinUp(double velocity) {
        boolean changed = Math.abs(velocity - targetVelocity) > RobotConfig.FLYWHEEL_RETARGET_TOLERANCE;
        targetVelocity = velocity;
        if (changed) {
            state = State.SPINNING_UP;
            stableTimer.reset();
        }
    }

    public State state() { return state; }
    public boolean isReady() { return state == State.READY; }
    public double velocity() { return motor.getVelocity(); }

    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double error = targetVelocity - motor.getVelocity();

        integral += error * dt;
        if (RobotConfig.FLYWHEEL_KI > 0) {
            double iMax = RobotConfig.FLYWHEEL_I_MAX / RobotConfig.FLYWHEEL_KI;
            integral = Math.max(-iMax, Math.min(iMax, integral));
        }
        double derivative = dt > 0 ? (error - lastError) / dt : 0.0;
        lastError = error;

        double output = RobotConfig.FLYWHEEL_KF * targetVelocity
                + RobotConfig.FLYWHEEL_KP * error
                + RobotConfig.FLYWHEEL_KI * integral
                + RobotConfig.FLYWHEEL_KD * derivative;
        output = Math.max(0.0, Math.min(1.0, output));
        motor.setPower(output);

        if (Math.abs(error) <= RobotConfig.FLYWHEEL_VELOCITY_TOLERANCE) {
            if (state == State.SPINNING_UP && stableTimer.seconds() >= RobotConfig.FLYWHEEL_STABLE_SECONDS) {
                state = State.READY;
            }
        } else {
            stableTimer.reset();
            if (state == State.READY && Math.abs(error) > 2 * RobotConfig.FLYWHEEL_VELOCITY_TOLERANCE) {
                state = State.SPINNING_UP;
            }
        }
    }
}