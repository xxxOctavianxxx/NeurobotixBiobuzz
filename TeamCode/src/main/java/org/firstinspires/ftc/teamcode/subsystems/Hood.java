package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Hood (servo): goto target; isReady() = a ajuns  */
public class Hood {
    private final Servo servo;
    private double position, target;
    private final ElapsedTime dt = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    public Hood(HardwareMap hw) {
        servo = hw.get(Servo.class, RobotConfig.HOOD_SERVO);
        position = clamp(RobotConfig.HOOD_START);
        target = position;
        servo.setPosition(position);
        dt.reset();
        settleTimer.reset();
    }

    private static double clamp(double v) {
        return Math.max(RobotConfig.HOOD_MIN, Math.min(RobotConfig.HOOD_MAX, v));
    }

    public void setTarget(double t) { target = clamp(t); }
    public double position() { return position; }
    public double target() { return target; }

    private boolean atTarget() { return Math.abs(position - target) <= RobotConfig.HOOD_TOLERANCE; }
    public boolean isReady() { return atTarget() && settleTimer.seconds() >= RobotConfig.HOOD_SETTLE_SECONDS; }

    public void update() {
        double elapsed = dt.seconds();
        dt.reset();
        double maxStep = RobotConfig.HOOD_SPEED_PER_SECOND * elapsed;
        double delta = target - position;
        position += Math.max(-maxStep, Math.min(maxStep, delta));
        servo.setPosition(position);
        if (!atTarget()) settleTimer.reset();
    }
}
