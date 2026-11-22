package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * TeleOp BIOBUZZ - 6 butoane (+ stick-uri). Controler: gamepad1.
 *
 *   Stick stânga    = deplasare   | Stick dreapta (stânga/dreapta) = rotație
 *   (X)  ține        = INTAKE
 *   (□)         apasă       = rampa floare: coboara ridica
 *   (△)         apasă       = TRAGE (secvență automată; apasă din nou = anulează)
 *  (cerc)         apasă       = schimbă ținta 1 <-> 2 (vibrează 1x / 2x)
 *   L1             ține        = EVACUARE intake + transfer invers
 *   R1             apasă       = PRE-SPIN flywheel pornit / oprit
 *
 * Tragerea e automata
 *
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp BIOBUZZ", group = "TeleOp")
public class  TeleOpBiobuzz extends LinearOpMode {
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

        RisingEdge xEdge = new RisingEdge();
        RisingEdge yEdge = new RisingEdge();
        RisingEdge bEdge = new RisingEdge();
        RisingEdge r1Edge = new RisingEdge();

        int target = 0; // 0 = ținta 1, 1 = ținta 2
        robot.shot.setTarget(RobotConfig.TARGET_X[target], RobotConfig.TARGET_Y[target]);

        while (opModeInInit()) {
            robot.localizer.update();
            telemetry.addData("Status", "Gata - TeleOp BIOBUZZ");
            telemetry.addData("Pinpoint", robot.localizer.isAvailable() ? "găsit" : "LIPSĂ (fără auto-aim)");
            telemetry.update();
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean a = gamepad1.a, x = gamepad1.x, y = gamepad1.y, b = gamepad1.b;
            boolean l1 = gamepad1.left_bumper, r1 = gamepad1.right_bumper;

            // ---------- EVACUARE (L1) / INTAKE ----------
            if (l1) {
                robot.shot.cancel();
                robot.intake.command(Intake.State.EJECTING);
                robot.transfer.setReversing(true);
            } else {
                robot.transfer.setReversing(false);
                robot.intake.command(a ? Intake.State.INTAKING : Intake.State.OFF);
            }

            // ---------- rampa (patrat) ----------
            if (xEdge.pressed(x)) robot.ramp.toggle();

            // ---------- shoot (triunghi) ----------
            if (yEdge.pressed(y)) {
                if (robot.shot.isActive()) robot.shot.cancel();
                else robot.shot.request();
            }

            // ---------- tinta (cerc) ----------
            if (bEdge.pressed(b) && !robot.shot.isActive()) {
                target = 1 - target;
                robot.shot.setTarget(RobotConfig.TARGET_X[target], RobotConfig.TARGET_Y[target]);
                gamepad1.rumbleBlips(target + 1);
            }

            // ---------- prespin (R1) ----------
            if (r1Edge.pressed(r1)) {
                robot.shot.setPreSpin(!robot.shot.preSpin());
                gamepad1.rumble(150);
            }


            robot.update();

            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;
            if (robot.shot.controllingTurn()) turn = robot.shot.turnPower();
            robot.drivetrain.drive(forward, strafe, turn);

            // ---------- tele ----------
            if (!robot.localizer.isAvailable()) {
                telemetry.addLine("! Pinpoint lipsă - tragi fără auto-aim");
            }
            if (!RobotConfig.SHOT_TABLE_CALIBRATED) {
                telemetry.addLine("! ShotTable NEMĂSURAT - valori de umplutură");
            }

            telemetry.addData("--- TRAGERE (Y) ---", "");
            telemetry.addData("Stare", robot.shot.state());
            telemetry.addData("Țintă", "#%d   distanță %.0f in   eroare unghi %.1f°",
                    target + 1, robot.shot.distance(), robot.shot.aimError());
            telemetry.addData("Gata?", "aim:%s  flywheel:%s  hood:%s",
                    robot.shot.isAimed() ? "DA" : "nu",
                    robot.shot.flywheelReady() ? "DA" : "nu");
            telemetry.addData("Ultimul rezultat", robot.shot.lastResult());
            telemetry.addData("Pre-spin (R1)", robot.shot.preSpin() ? "PORNIT" : "oprit");

            telemetry.addData("--- SISTEME ---", "");
            telemetry.addData("Intake", robot.intake.state());
            telemetry.addData("Transfer", robot.transfer.state());
            telemetry.addData("Rampă (X)", "%s (%.2f)", robot.ramp.state(), robot.ramp.position());
            telemetry.addData("Flywheel", "%s   viteză reală %.0f ticks/s",
                    robot.flywheel.state(), robot.flywheel.velocity());

            if (robot.localizer.isAvailable()) {
                telemetry.addData("Poziție", "X:%.1f  Y:%.1f  H:%.1f°",
                        robot.localizer.x(), robot.localizer.y(), robot.localizer.heading());
            }
            telemetry.update();
        }
    }
}
