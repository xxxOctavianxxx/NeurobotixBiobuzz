package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 - TeleOp BIOBUZZ. Controler: gamepad1.
 *
 -   Stick stanga    = deplasare   | Stick dreapta (stanga/dreapta) = rotatie
 -   SHARE           apasa       = switch Field-Centric / Robot-Centric
 -   OPTIONS         apasa       = seteaza directia curenta a robotului ca "inainte" pt Field-Centric
 -   (patrat)        apasa       = tinta 1 (stup jos), auto-aim blocat (apasa din nou = oprit)
 -   (cerc)          apasa       = tinta 2 (stup sus), auto-aim blocat (apasa din nou = oprit)
 -   (triunghi)      apasa       = rampa floare: coboara / ridica
 -   (X)             tine        = EVACUARE intake
 -   R1              apasa       = TRAGE (secventa automata; apasa din nou = anuleaza)
 -   R2              tine        = TRANSFER asistenta spre tureta
 -   L1              apasa       = toggle viteza drive 40% (Slow Mode) / 100%
 -   L2              tine        = INTAKE
 -   D-pad Sus/Jos   apasa       = +/- 100 ticks/s viteza flywheel
 -   D-pad Dr/St     apasa       = +/- 25 ticks/s viteza flywheel (reglaj fin)
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp BIOBUZZ", group = "TeleOp")
public class TeleOpBiobuzz extends LinearOpMode {
    private static class RisingEdge {
        private boolean last = false;
        boolean pressed(boolean now) {
            boolean result = now && !last;
            last = now;
            return result;
        }
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        RisingEdge sqEdge        = new RisingEdge();
        RisingEdge ciEdge        = new RisingEdge();
        RisingEdge triEdge       = new RisingEdge();
        RisingEdge r1Edge        = new RisingEdge();
        RisingEdge l1Edge        = new RisingEdge();
        RisingEdge shareEdge     = new RisingEdge();
        RisingEdge optionsEdge   = new RisingEdge();
        RisingEdge dpadUpEdge    = new RisingEdge();
        RisingEdge dpadDownEdge  = new RisingEdge();
        RisingEdge dpadRightEdge = new RisingEdge();
        RisingEdge dpadLeftEdge  = new RisingEdge();

        int target = 0;                 // 0 = tinta 1 (patrat), 1 = tinta 2 (cerc)
        boolean aimLatched = false;
        boolean fieldCentric = true;    // implicit Field-Centric
        boolean slowMode = false;       // L1 toggle: true = 40% viteza
        double fieldZeroDeg = RobotConfig.FIELD_CENTRIC_ZERO_DEG;

        robot.shot.setTarget(RobotConfig.TARGET_X[target], RobotConfig.TARGET_Y[target]);

        while (opModeInInit()) {
            robot.localizer.update();
            telemetry.addData("Pinpoint", robot.localizer.status());
            if (robot.localizer.isAvailable()) {
                telemetry.addData("Poziție start", "X:%.1f in  Y:%.1f in  H:%.1f°",
                        robot.localizer.x(), robot.localizer.y(), robot.localizer.heading());
            }
            telemetry.update();
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean sq    = gamepad1.square;
            boolean ci    = gamepad1.circle;
            boolean tri   = gamepad1.triangle;
            boolean eject = gamepad1.cross;              // X = evacuare intake
            boolean r1    = gamepad1.right_bumper;
            boolean l1    = gamepad1.left_bumper;
            boolean l2    = gamepad1.left_trigger > 0.05;
            boolean r2    = gamepad1.right_trigger > 0.05;

            // ---------- switch Field-Centric / Robot-Centric (SHARE) ----------
            if (shareEdge.pressed(gamepad1.share)) {
                fieldCentric = !fieldCentric;
                gamepad1.rumbleBlips(fieldCentric ? 2 : 1);
            }
            // ---------- reset directie "inainte" pt Field-Centric (OPTIONS) ----------
            if (optionsEdge.pressed(gamepad1.options) && robot.localizer.isAvailable()) {
                fieldZeroDeg = robot.localizer.heading();
                gamepad1.rumbleBlips(1);
            }
            // ---------- toggle Slow Mode 40% (L1) ----------
            if (l1Edge.pressed(l1)) {
                slowMode = !slowMode;
                gamepad1.rumbleBlips(slowMode ? 1 : 2);
            }

            // ---------- EVACUARE (X) / INTAKE (L2) / TRANSFER spre tureta (R2) ----------
            if (eject) {
                robot.shot.cancel();
                robot.intake.command(Intake.State.EJECTING);
            } else {
                robot.intake.command(l2 ? Intake.State.INTAKING : Intake.State.OFF);
            }
            robot.transfer.setReversing(false);
            robot.setAssistRequested(r2 && !eject);

            // ---------- rampa (triunghi) ----------
            if (triEdge.pressed(tri)) robot.ramp.toggle();

            // ---------- reglaj viteza flywheel (D-pad) ----------
            if (dpadUpEdge.pressed(gamepad1.dpad_up))       robot.shot.adjustVelocityOffset(100.0);
            if (dpadDownEdge.pressed(gamepad1.dpad_down))   robot.shot.adjustVelocityOffset(-100.0);
            if (dpadRightEdge.pressed(gamepad1.dpad_right)) robot.shot.adjustVelocityOffset(25.0);
            if (dpadLeftEdge.pressed(gamepad1.dpad_left))   robot.shot.adjustVelocityOffset(-25.0);

            // ---------- tinta (patrat = 1, cerc = 2, toggle) ----------
            int pick = -1;
            if (sqEdge.pressed(sq)) pick = 0;
            if (ciEdge.pressed(ci)) pick = 1;
            if (pick >= 0) {
                if (aimLatched && target == pick) {
                    aimLatched = false;                     // apasat din nou = oprit
                } else {
                    target = pick;
                    aimLatched = true;
                    robot.shot.setTarget(RobotConfig.TARGET_X[target], RobotConfig.TARGET_Y[target]);
                }
                gamepad1.rumbleBlips(aimLatched ? target + 1 : 0);
            }
            robot.shot.setTracking(aimLatched);
            robot.shot.setPreSpin(aimLatched);              // flywheel pornit cat tinta e blocata

            // ---------- tragere (R1) ----------
            if (r1Edge.pressed(r1)) {
                if (robot.shot.isActive()) robot.shot.cancel();
                else robot.shot.request();
            }

            robot.setGateOpenRequested(aimLatched);
            robot.update();

            // ---------- DRIVE (Field-Centric sau Robot-Centric) ----------
            double driveScale = slowMode ? 0.4 : 1.0;
            double inputFwd   = -gamepad1.left_stick_y  * driveScale;   // inainte = stick in sus
            double inputRight =  gamepad1.left_stick_x  * driveScale;   // dreapta = stick in dreapta
            double turn       =  gamepad1.right_stick_x * driveScale;

            double forward = inputFwd;
            double strafe  = inputRight;
            boolean usingFieldCentric = fieldCentric && robot.localizer.isAvailable();
            if (usingFieldCentric) {
                double hRad = Math.toRadians(robot.localizer.heading() - fieldZeroDeg);
                double cosH = Math.cos(hRad);
                double sinH = Math.sin(hRad);
                forward = inputFwd * cosH - inputRight * sinH;
                strafe  = inputFwd * sinH + inputRight * cosH;
            }

            if (robot.shot.controllingTurn() && Math.abs(gamepad1.right_stick_x) < 0.05) {
                turn = robot.shot.turnPower();
            }
            robot.drivetrain.drive(forward, strafe, turn);

            // ---------- tele ----------
            if (!robot.localizer.isAvailable()) {
                telemetry.addLine("! Pinpoint lipsă - Robot-Centric, fără auto-aim");
            }
            if (!RobotConfig.SHOT_TABLE_CALIBRATED) {
                telemetry.addLine("! ShotTable NEMĂSURAT - valori de umplutură");
            }

            telemetry.addData("Drive (SHARE)", usingFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Viteză (L1)", slowMode ? "SLOW (40%)" : "NORMAL (100%)");
            telemetry.addData("--- TRAGERE (R1) ---", "");
            telemetry.addData("Stare", robot.shot.state());
            telemetry.addData("Țintă", "#%d   distanță %.1f in   eroare unghi %.1f°",
                    target + 1, robot.shot.distance(), robot.shot.aimError());
            telemetry.addData("Gata?", "aim:%s  flywheel:%s",
                    robot.shot.isAimed() ? "DA" : "nu",
                    robot.shot.flywheelReady() ? "DA" : "nu");
            telemetry.addData("Ultimul rezultat", robot.shot.lastResult());
            telemetry.addData("Tinta blocata", aimLatched ? "DA" : "nu");
            telemetry.addData("CMD fwd strafe turn", "%.2f %.2f %.2f", forward, strafe, turn);
            telemetry.addData("--- SISTEME ---", "");
            telemetry.addData("Intake", robot.intake.state());
            telemetry.addData("Transfer", robot.transfer.state());
            telemetry.addData("Rampă (Triunghi)", "%s (%.2f)", robot.ramp.state(), robot.ramp.position());
            telemetry.addData("Flywheel", "%s   țintă %.0f (offset %+.0f)   reală %.0f ticks/s",
                    robot.flywheel.state(), robot.shot.targetVelocity(), robot.shot.velocityOffset(), robot.flywheel.velocity());

            if (robot.localizer.isAvailable()) {
                telemetry.addData("Poziție", "X:%.1f in  Y:%.1f in  H:%.1f°",
                        robot.localizer.x(), robot.localizer.y(), robot.localizer.heading());
            }
            telemetry.update();
        }
    }
}