package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Poziția robotului de la Pinpoint (în INCH).
 * Dacă Pinpoint-ul lipsește, isAvailable() e false și restul robotului merge oricum.
 */
public class Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private double x = RobotConfig.START_X_IN;
    private double y = RobotConfig.START_Y_IN;
    private double headingDeg = RobotConfig.START_HEADING_DEG;
    private boolean startPoseApplied = false;

    public Localizer(HardwareMap hw) {
        pinpoint = hw.tryGet(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
        if (pinpoint != null) {
            pinpoint.setOffsets(RobotConfig.POD_X_OFFSET_MM, RobotConfig.POD_Y_OFFSET_MM, DistanceUnit.MM);
            // ATENTIE: verifica sa ai fizic poduri 4-Bar (roata 48mm). Pentru Swingarm (32mm), pune goBILDA_SWINGARM_POD.
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    RobotConfig.POD_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    RobotConfig.POD_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU(); // calibreaza IMU (~0.25s) si reseteaza la 0
        }
    }

    public boolean isAvailable() { return pinpoint != null; }

    /** True cand Pinpoint-ul a terminat calibrarea IMU si raporteaza pozitii valide. */
    public boolean isReady() {
        return pinpoint != null
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public String status() {
        return pinpoint == null ? "LIPSĂ" : pinpoint.getDeviceStatus().name();
    }

    /** Seteaza manual pozitia robotului pe teren (in INCH si GRADE). */
    public void setPose(double xIn, double yIn, double headingDeg) {
        this.x = xIn;
        this.y = yIn;
        this.headingDeg = headingDeg;
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg));
            startPoseApplied = true;
        }
    }

    public void update() {
        if (pinpoint == null) return;
        pinpoint.update();

        // Aplica pozitia de start din RobotConfig imediat ce IMU-ul termina calibrarea
        if (!startPoseApplied && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    RobotConfig.START_X_IN,
                    RobotConfig.START_Y_IN,
                    AngleUnit.DEGREES,
                    RobotConfig.START_HEADING_DEG
            ));
            startPoseApplied = true;
            pinpoint.update();
        }

        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double x() { return x; }
    public double y() { return y; }
    public double heading() { return headingDeg; }
}