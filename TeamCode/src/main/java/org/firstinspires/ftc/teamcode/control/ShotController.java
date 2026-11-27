package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRamp;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * Secventa de tragere, state machine:
 *   IDLE -> PREPARING (aim + flywheel la viteza) -> FEEDING (transfer) -> RECOVERING -> IDLE
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

    private double targetX = 0.0, targetY = 0.0;
    private boolean targetConfigured = false;
    private boolean tracking = false;   // tinta blocata: rotatie continua spre stup
    private boolean preSpin = false;    // flywheel la viteza tabelului cat tinta e blocata

    private boolean autoAim = false, aimed = true, turnControl = false;
    private double distance = RobotConfig.DEFAULT_SHOT_DISTANCE_IN;
    private double aimError = 0.0;
    private ShotTable.Params params = ShotTable.lookup(RobotConfig.DEFAULT_SHOT_DISTANCE_IN);
    private ShotTable.Params lockedParams = params;
    private String lastResult = "-";

    // offset manual pentru calibrarea ShotTable din D-pad
    private double velocityOffset = 0.0;

    // filtru + histerezis + derivativ pentru rotatia de aim
    private double filteredErr = 0.0;
    private double aimRate = 0.0;       // cat s-a schimbat eroarea filtrata intr-o bucla
    private boolean filterInit = false;
    private boolean turnActive = false;

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

    public void setTracking(boolean on) { tracking = on; }

    public void setPreSpin(boolean on) { preSpin = on; }
    public boolean preSpin() { return preSpin; }

    public void adjustVelocityOffset(double delta) { velocityOffset += delta; }
    public void resetVelocityOffset() { velocityOffset = 0.0; }
    public double velocityOffset() { return velocityOffset; }
    public double targetVelocity() { return params.velocity; }

    public void request() {
        if (state != State.IDLE) return;
        if (RobotConfig.RAMP_STOW_ON_SHOT) ramp.stow();
        state = State.PREPARING;
        stateTimer.reset();
        aimStableTimer.reset();
        lastResult = "in curs";
    }

    public void cancel() {
        if (state != State.IDLE) lastResult = "anulat";
        enterIdle();
    }

    private void enterIdle() {
        state = State.IDLE;
        turnControl = false;
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
        if (!turnControl) {
            turnActive = false;
            return 0.0;
        }
        double err = filteredErr;
        double absErr = Math.abs(err);
        double tol = RobotConfig.AIM_TOLERANCE_DEG;

        // histerezis: porneste cand eroarea depaseste 2x toleranta, se opreste sub toleranta
        if (turnActive && absErr < tol) turnActive = false;
        else if (!turnActive && absErr > 2 * tol) turnActive = true;
        if (!turnActive) return 0.0;

        // P + D cu semn inversat pentru Drivetrain (turn + = sens orar)
        double turn = -(err * RobotConfig.KP_AIM + aimRate * RobotConfig.KD_AIM);

        // putere minima doar la erori mari, ca sa nu tremure aproape de tinta
        if (absErr > 3 * tol && Math.abs(turn) < RobotConfig.MIN_AIM_POWER) {
            turn = Math.signum(turn) * RobotConfig.MIN_AIM_POWER;
        }
        return Math.max(-RobotConfig.MAX_AIM_POWER, Math.min(RobotConfig.MAX_AIM_POWER, turn));
    }

    public void update() {
        // --- geometrie (INCH)
        if (localizer.isAvailable() && targetConfigured) {
            double dx = targetX - localizer.x();
            double dy = targetY - localizer.y();
            distance = Math.hypot(dx, dy);
            double angleToTarget = Math.toDegrees(Math.atan2(dy, dx)) - RobotConfig.SHOOTER_ANGLE_OFFSET_DEG;
            aimError = AngleUnit.normalizeDegrees(angleToTarget - localizer.heading());
            autoAim = true;

            // filtru exponential pe eroare, ca zgomotul Pinpoint-ului sa nu declanseze rotatia
            double prev = filteredErr;
            if (!filterInit) {
                filteredErr = aimError;
                filterInit = true;
            } else {
                filteredErr += 0.5 * (aimError - filteredErr);
            }
            aimRate = filteredErr - prev;
        } else {
            distance = RobotConfig.DEFAULT_SHOT_DISTANCE_IN;
            aimError = 0.0;
            autoAim = false;
            filterInit = false;
            aimRate = 0.0;
        }
        boolean tooClose = autoAim && distance < RobotConfig.MIN_AIM_DISTANCE_IN;
        boolean aimActive = autoAim && !tooClose;
        aimed = !aimActive || Math.abs(aimError) < RobotConfig.AIM_TOLERANCE_DEG;

        double baseVel = ShotTable.lookup(distance).velocity;
        params = new ShotTable.Params(Math.max(0.0, baseVel + velocityOffset));

        // --- state
        switch (state) {
            case IDLE: {
                turnControl = tracking && aimActive;
                flywheel.spinUp(preSpin ? params.velocity : RobotConfig.FLYWHEEL_IDLE_VELOCITY);
                break;
            }
            case PREPARING: {
                turnControl = aimActive;
                flywheel.spinUp(params.velocity);

                if (!aimed) aimStableTimer.reset();
                boolean aimOk = aimed && aimStableTimer.seconds() >= RobotConfig.AIM_STABLE_SECONDS;
                if (aimOk && flywheel.isReady()) {
                    lockedParams = params; // freeze valoarea
                    state = State.FEEDING;
                    stateTimer.reset();
                } else if (stateTimer.seconds() > RobotConfig.PREPARE_TIMEOUT_SECONDS) {
                    lastResult = "timeout (aim / flywheel)";
                    enterIdle();
                }
                break;
            }
            case FEEDING: {
                turnControl = aimActive;
                flywheel.spinUp(lockedParams.velocity);
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