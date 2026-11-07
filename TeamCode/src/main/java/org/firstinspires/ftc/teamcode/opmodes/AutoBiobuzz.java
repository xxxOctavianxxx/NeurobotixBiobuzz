package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.PoseController;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 *
 *   1. merge la punctul de aruncare (flywheel-ul se încălzește pe drum)
 *   2. aruncă preload-ul
 *   3. merge la floare, coboară rampa
 *   4. colectează polen (intake pornit, înaintare lentă)
 *   5. se întoarce și aruncă și polenul colectat
 *
 * Coordonatele sunt în RobotConfig.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto BIOBUZZ", group = "Auto", preselectTeleOp = "TeleOp BIOBUZZ")
public class AutoBiobuzz extends LinearOpMode {

    private enum Step {
        DRIVE_TO_SCORE, SHOOT_PRELOAD,
        DRIVE_TO_FLOWER, COLLECT, COLLECT_EXTRA,
        RETURN_TO_APPROACH, RETURN_TO_SCORE, SHOOT_COLLECTED,
        DONE
    }

    private Robot robot;
    private final PoseController pose = new PoseController();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private Step step = Step.DONE;
    private boolean usePose = false;

    private static final double SHOOT_TIMEOUT_SECONDS =
            RobotConfig.PREPARE_TIMEOUT_SECONDS + RobotConfig.FEED_SECONDS + RobotConfig.RECOVER_SECONDS + 1.0;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        while (opModeInInit()) {
            robot.localizer.update();
            telemetry.addData("Status", "Gata - Auto BIOBUZZ");
            if (!robot.localizer.isAvailable()) {
                telemetry.addLine("EROARE: Pinpoint lipsă - autonomia nu poate rula");
            }
            telemetry.addData("Poziție", "X:%.1f  Y:%.1f  H:%.1f°",
                    robot.localizer.x(), robot.localizer.y(), robot.localizer.heading());
            telemetry.update();
        }
        if (isStopRequested() || !robot.localizer.isAvailable()) return;

        ElapsedTime totalTimer = new ElapsedTime();
        robot.shot.setTarget(RobotConfig.TARGET_X[RobotConfig.AUTO_TARGET_INDEX],
                             RobotConfig.TARGET_Y[RobotConfig.AUTO_TARGET_INDEX]);
        enter(Step.DRIVE_TO_SCORE);

        while (opModeIsActive()) {
            robot.update();
            runStep();

            // ---------- Comanda de drive ----------
            double forward = 0, strafe = 0, turn = 0;
            if (usePose) {
                pose.update(robot.localizer.x(), robot.localizer.y(), robot.localizer.heading());
                forward = pose.forward();
                strafe = pose.strafe();
                turn = pose.turn();
            }
            if (robot.shot.controllingTurn()) turn = robot.shot.turnPower();
            robot.drivetrain.drive(forward, strafe, turn);

            // ---------- Telemetrie ----------
            telemetry.addData("Pas", step);
            telemetry.addData("Timp", "%.1f s  (pas: %.1f s)", totalTimer.seconds(), stepTimer.seconds());
            telemetry.addData("Poziție", "X:%.1f  Y:%.1f  H:%.1f°",
                    robot.localizer.x(), robot.localizer.y(), robot.localizer.heading());
            if (usePose) {
                telemetry.addData("Distanță rămasă", "%.1f in   eroare unghi %.1f°", pose.distance(), pose.headingError());
            }
            telemetry.addData("Tragere", "%s  (%s)", robot.shot.state(), robot.shot.lastResult());
            telemetry.update();
        }
    }

    // Intra intr-un pas nou
    private void enter(Step next) {
        step = next;
        stepTimer.reset();

        switch (next) {
            case DRIVE_TO_SCORE:
                robot.shot.setPreSpin(true); // flywheelul se angrewaza pe drum
                goTo(RobotConfig.AUTO_SCORE_X, RobotConfig.AUTO_SCORE_Y, RobotConfig.AUTO_SCORE_H,
                        RobotConfig.MAX_DRIVE_POWER);
                break;
            case SHOOT_PRELOAD:
            case SHOOT_COLLECTED:
                usePose = false;
                pose.cancel();
                robot.shot.request();
                break;
            case DRIVE_TO_FLOWER:
                robot.shot.setPreSpin(false);
                robot.ramp.deploy();
                goTo(RobotConfig.AUTO_FLOWER_APPROACH_X, RobotConfig.AUTO_FLOWER_APPROACH_Y,
                        RobotConfig.AUTO_FLOWER_APPROACH_H, RobotConfig.MAX_DRIVE_POWER);
                break;
            case COLLECT:
                robot.intake.command(Intake.State.INTAKING);
                goTo(RobotConfig.AUTO_FLOWER_COLLECT_X, RobotConfig.AUTO_FLOWER_COLLECT_Y,
                        RobotConfig.AUTO_FLOWER_COLLECT_H, RobotConfig.COLLECT_POWER);
                break;
            case COLLECT_EXTRA:
                usePose = false;
                pose.cancel();
                break;
            case RETURN_TO_APPROACH:
                robot.intake.command(Intake.State.OFF);
                robot.ramp.stow();
                robot.shot.setPreSpin(true);
                goTo(RobotConfig.AUTO_FLOWER_APPROACH_X, RobotConfig.AUTO_FLOWER_APPROACH_Y,
                        RobotConfig.AUTO_FLOWER_APPROACH_H, RobotConfig.MAX_DRIVE_POWER);
                break;
            case RETURN_TO_SCORE:
                goTo(RobotConfig.AUTO_SCORE_X, RobotConfig.AUTO_SCORE_Y, RobotConfig.AUTO_SCORE_H,
                        RobotConfig.MAX_DRIVE_POWER);
                break;
            case DONE:
                usePose = false;
                pose.cancel();
                robot.intake.command(Intake.State.OFF);
                robot.shot.setPreSpin(false);
                break;
        }
    }

    private void goTo(double x, double y, double headingDeg, double maxPower) {
        pose.setTarget(x, y, headingDeg, maxPower);
        usePose = true;
    }

    private boolean moveFinished() {
        return pose.isDone() || stepTimer.seconds() > RobotConfig.AUTO_MOVE_TIMEOUT_SECONDS;
    }

    private boolean shotFinished() {
        if (!robot.shot.isActive()) return true;
        if (stepTimer.seconds() > SHOOT_TIMEOUT_SECONDS) {
            robot.shot.cancel();
            return true;
        }
        return false;
    }

    /** el state machien */
    private void runStep() {
        switch (step) {
            case DRIVE_TO_SCORE:
                if (moveFinished()) enter(Step.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (shotFinished()) enter(Step.DRIVE_TO_FLOWER);
                break;
            case DRIVE_TO_FLOWER:
                if (moveFinished()) enter(Step.COLLECT);
                break;
            case COLLECT:
                if (moveFinished()) enter(Step.COLLECT_EXTRA);
                break;
            case COLLECT_EXTRA:
                if (stepTimer.seconds() >= RobotConfig.AUTO_INTAKE_EXTRA_SECONDS) {
                    if (RobotConfig.AUTO_SHOOT_COLLECTED) enter(Step.RETURN_TO_APPROACH);
                    else enter(Step.DONE);
                }
                break;
            case RETURN_TO_APPROACH:
                if (moveFinished()) enter(Step.RETURN_TO_SCORE);
                break;
            case RETURN_TO_SCORE:
                if (moveFinished()) enter(Step.SHOOT_COLLECTED);
                break;
            case SHOOT_COLLECTED:
                if (shotFinished()) enter(Step.DONE);
                break;
            case DONE:
                break;
        }
    }
}
