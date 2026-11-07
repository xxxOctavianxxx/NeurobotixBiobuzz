package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Poziția robotului de la Pinpoint. Dacă Pinpoint-ul lipsește, isAvailable() e false și restul robotului merge oricum. */
public class Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private double x, y, headingDeg;

    public Localizer(HardwareMap hw) {
        pinpoint = hw.tryGet(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
        if (pinpoint != null) {
            pinpoint.setOffsets(RobotConfig.POD_X_OFFSET_MM, RobotConfig.POD_Y_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    RobotConfig.POD_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                                               : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    RobotConfig.POD_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                                               : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU(); // start
        }
    }

    public boolean isAvailable() { return pinpoint != null; }

    public void update() {
        if (pinpoint == null) return;
        pinpoint.update();
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double x() { return x; }
    public double y() { return y; }
    public double heading() { return headingDeg; }
}
