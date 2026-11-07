package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * .
 * setTarget(...) o data apoi update(x, y, heading) la fiecare bucla citesti forward()/strafe()/turn().
 */
public class PoseController {
    private double targetX, targetY, targetH;
    private double maxPower = RobotConfig.MAX_DRIVE_POWER;
    private boolean done = true;
    private double forward, strafe, turn, distance, headingError;

    public void setTarget(double x, double y, double headingDeg, double maxPower) {
        targetX = x;
        targetY = y;
        targetH = headingDeg;
        this.maxPower = maxPower;
        done = false;
    }

    public void cancel() {
        done = true;
        forward = strafe = turn = 0.0;
    }

    public boolean isDone() { return done; }
    public double forward() { return forward; }
    public double strafe() { return strafe; }
    public double turn() { return turn; }
    public double distance() { return distance; }
    public double headingError() { return headingError; }

    public void update(double x, double y, double headingDeg) {
        double errX = targetX - x;
        double errY = targetY - y;
        double errH = AngleUnit.normalizeDegrees(targetH - headingDeg);
        distance = Math.hypot(errX, errY);
        headingError = errH;

        boolean positionOk = distance < RobotConfig.POS_TOLERANCE_IN;
        boolean headingOk = Math.abs(errH) < RobotConfig.HEADING_TOLERANCE_DEG;
        if (positionOk && headingOk) {
            done = true;
            forward = strafe = turn = 0.0;
            return;
        }
        done = false;

        // Din coordonate de TEREN în coordonate de ROBOT
        double hRad = Math.toRadians(headingDeg);
        double forwardErr =  errX * Math.cos(hRad) + errY * Math.sin(hRad);
        double leftErr    = -errX * Math.sin(hRad) + errY * Math.cos(hRad);

        double f = forwardErr * RobotConfig.KP_DRIVE;
        double s = -leftErr * RobotConfig.KP_DRIVE;   // strafe + = dreapta,
        double t = -errH * RobotConfig.KP_TURN;       // turn + = sens orar,

        // --- Limite pentru DEPLASARE ---
        double minPower = Math.min(RobotConfig.MIN_DRIVE_POWER, maxPower);
        double speed = Math.hypot(f, s);
        if (positionOk) {
            f = 0;
            s = 0;
        } else if (speed > maxPower) {
            f = f / speed * maxPower;
            s = s / speed * maxPower;
        } else if (speed > 0 && speed < minPower) {
            f = f / speed * minPower;
            s = s / speed * minPower;
        }

        // --- Limite pentru ROTIRE ---
        if (headingOk) {
            t = 0;
        } else if (Math.abs(t) < RobotConfig.MIN_TURN_POWER) {
            t = Math.signum(t) * RobotConfig.MIN_TURN_POWER;
        }
        t = Math.max(-RobotConfig.MAX_TURN_POWER, Math.min(RobotConfig.MAX_TURN_POWER, t));

        forward = f;
        strafe = s;
        turn = t;
    }
}
