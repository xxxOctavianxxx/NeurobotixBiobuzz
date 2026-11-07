package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Flywheel: state machine OFF -> SPINNING_UP -> READY.
 * Mod putere (implicit): READY după FLYWHEEL_SPINUP_SECONDS.
 * Mod viteză (FLYWHEEL_USE_VELOCITY): READY când viteza reală e în toleranță un timp.
 */
public class Flywheel {
    public enum State { OFF, SPINNING_UP, READY }

    private final DcMotorEx motor;
    private State state = State.OFF;
    private double targetPower = 0.0, targetVelocity = 0.0, refPower = 0.0;
    private final ElapsedTime spinTimer = new ElapsedTime();
    private final ElapsedTime stableTimer = new ElapsedTime();

    public Flywheel(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, RobotConfig.FLYWHEEL);
        motor.setDirection(RobotConfig.dir(RobotConfig.FLYWHEEL_REVERSED));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // se oprește lin
        motor.setMode(RobotConfig.FLYWHEEL_USE_VELOCITY
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** porneste / menține flywheelu la ținta. daca tinta se schimba ff mult, restabilizeaza. */
    public void spinUp(double power, double velocity) {
        targetPower = power;
        targetVelocity = velocity;
        boolean changed = Math.abs(power - refPower) > RobotConfig.FLYWHEEL_RETARGET_POWER_DELTA;
        if (state == State.OFF || changed) {
            state = State.SPINNING_UP;
            refPower = power;
            spinTimer.reset();
            stableTimer.reset();
        }
    }

    public void stop() { state = State.OFF; }
    public State state() { return state; }
    public boolean isReady() { return state == State.READY; }
    public double velocity() { return motor.getVelocity(); }

    public void update() {
        if (state == State.OFF) {
            motor.setPower(0.0);
            return;
        }
        if (RobotConfig.FLYWHEEL_USE_VELOCITY) {
            motor.setVelocity(targetVelocity);
            double error = Math.abs(motor.getVelocity() - targetVelocity);
            if (error > RobotConfig.FLYWHEEL_VELOCITY_TOLERANCE) {
                stableTimer.reset();
                if (error > RobotConfig.FLYWHEEL_VELOCITY_TOLERANCE * 2) state = State.SPINNING_UP;
            } else if (state == State.SPINNING_UP
                    && stableTimer.seconds() >= RobotConfig.FLYWHEEL_STABLE_SECONDS) {
                state = State.READY;
            }
        } else {
            motor.setPower(targetPower);
            if (state == State.SPINNING_UP && spinTimer.seconds() >= RobotConfig.FLYWHEEL_SPINUP_SECONDS) {
                state = State.READY;
            }
        }
    }
}
