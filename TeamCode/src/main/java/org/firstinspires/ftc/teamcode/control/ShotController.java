package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRamp;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * Secvența de tragere, state machien:
 *   IDLE -> PREPARING (aim + porneste flywheel + set hood)
 *        -> FEEDING (transfer) -> RECOVERING -> IDLE
 *
 */
public class ShotController {
    public enum State { IDLE, PREPARING, FEEDING, RECOVERING }

    private final Localizer localizer;
    private final Flywheel flywheel;
    private final Transfer transfer;
    private final FlowerRamp ramp;

    private State state = State.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime aimStableTimer = new ElapsedTime();
    private final ElapsedTime idleTimer = new ElapsedTime();

    private double targetX = 0.0, targetY = 0.0;
    private boolean targetConfigured = false;
    private boolean preSpin = false;

    private boolean autoAim = false, aimed = true, turnControl = false;
    private double distance = RobotConfig.DEFAULT_SHOT_DISTANCE_IN;
    private double aimError = 0.0;
    private ShotTable.Params params = ShotTable.lookup(RobotConfig.DEFAULT_SHOT_DISTANCE_IN);
    private ShotTable.Params lockedParams = params;
    private String lastResult = "-";

    public ShotController(Localizer localizer, Flywheel flywheel, Transfer transfer, FlowerRamp ramp) {
        this.localizer = localizer;
        this.flywheel = flywheel;
        this.transfer = transfer;
        this.ramp = ramp;
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
        targetConfigured = !(x == 0.0 && y == 0.0);
    }

    //Prespin
    public void setPreSpin(boolean on) {
        preSpin = on;
        if (!on && state == State.IDLE) flywheel.stop();
    }
    public boolean preSpin() { return preSpin; }

    public void request() {
        if (state != State.IDLE) return;
        if (RobotConfig.RAMP_STOW_ON_SHOT) ramp.stow();
        state = State.PREPARING;
        stateTimer.reset();
        aimStableTimer.reset();
        lastResult = "în curs";
    }

    public void cancel() {
        if (state != State.IDLE) lastResult = "anulat";
        enterIdle();
    }

    private void enterIdle() {
        state = State.IDLE;
        turnControl = false;
        idleTimer.reset();
    }

    public State state() { return state; }
    public boolean isActive() { return state != State.IDLE; }
    public boolean controllingTurn() { return turnControl; }
    public double aimError() { return aimError; }
    public double distance() { return distance; }
    public boolean isAimed() { return aimed; }
    public boolean targetConfigured() { return targetConfigured; }
    public boolean flywheelReady() { return flywheel.isReady(); }
    public String lastResult() { return lastResult; }

    /** Puterea de rotire ceruta de auto-aim (+ = sens orar). */
    public double turnPower() {
        if (!turnControl || Math.abs(aimError) < RobotConfig.AIM_TOLERANCE_DEG) return 0.0;
        double turn = -aimError * RobotConfig.KP_AIM;
        if (Math.abs(turn) < RobotConfig.MIN_AIM_POWER) turn = Math.signum(turn) * RobotConfig.MIN_AIM_POWER;
        return Math.max(-RobotConfig.MAX_AIM_POWER, Math.min(RobotConfig.MAX_AIM_POWER, turn));
    }

    public void update() {
        // --- geometrie
        if (localizer.isAvailable() && targetConfigured) {
            double dx = targetX - localizer.x();
            double dy = targetY - localizer.y();
            distance = Math.hypot(dx, dy);
            double angleToTarget = Math.toDegrees(Math.atan2(dy, dx)) - RobotConfig.SHOOTER_ANGLE_OFFSET_DEG;
            aimError = AngleUnit.normalizeDegrees(angleToTarget - localizer.heading());
            autoAim = true;
        } else {
            distance = RobotConfig.DEFAULT_SHOT_DISTANCE_IN;
            aimError = 0.0;
            autoAim = false;
        }
        boolean tooClose = autoAim && distance < RobotConfig.MIN_AIM_DISTANCE_IN;
        boolean aimActive = autoAim && !tooClose;
        aimed = !aimActive || Math.abs(aimError) < RobotConfig.AIM_TOLERANCE_DEG;
        params = ShotTable.lookup(distance);

        // --- state---
        switch (state) {
            case IDLE: {
                turnControl = false;
                if (preSpin) {
                    flywheel.spinUp(params.power, params.velocity);
                    idleTimer.reset();
                } else if (flywheel.state() != Flywheel.State.OFF
                        && idleTimer.seconds() > RobotConfig.FLYWHEEL_KEEP_ALIVE_SECONDS) {
                    flywheel.stop();
                }
                break;
            }
            case PREPARING: {
                turnControl = aimActive;
                flywheel.spinUp(params.power, params.velocity);

                if (!aimed) aimStableTimer.reset();
                boolean aimOk = aimed && aimStableTimer.seconds() >= RobotConfig.AIM_STABLE_SECONDS;
                if (aimOk && flywheel.isReady()) {
                    lockedParams = params; // freeze vlaue
                    state = State.FEEDING;
                    stateTimer.reset();
                } else if (stateTimer.seconds() > RobotConfig.PREPARE_TIMEOUT_SECONDS) {
                    lastResult = "timeout (aim / flywheel / hood)";
                    enterIdle();
                }
                break;
            }
            case FEEDING: {
                turnControl = aimActive;
                flywheel.spinUp(lockedParams.power, lockedParams.velocity);
                if (stateTimer.seconds() >= RobotConfig.FEED_SECONDS) {
                    state = State.RECOVERING;
                    stateTimer.reset();
                }
                break;
            }
            case RECOVERING: {
                turnControl = false;
                if (stateTimer.seconds() >= RobotConfig.RECOVER_SECONDS) {
                    lastResult = "tras";
                    enterIdle();
                }
                break;
            }
        }

        transfer.setFeeding(state == State.FEEDING);
    }
}
