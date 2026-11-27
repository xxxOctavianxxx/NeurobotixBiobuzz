package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.ShotController;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRamp;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/** Aduna toate sistemele robotului. update() se apeleaza o data pe bucla (TeleOp si Autonomie). */
public class Robot {
    public final Localizer localizer;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Transfer transfer;
    public final Flywheel flywheel;
    public final FlowerRamp ramp;
    public final Stopper stopper;
    public final ShotController shot;

    private boolean assistRequested = false;   // transfer spre tureta cat e tinut R2
    private boolean gateOpenRequested = false; // gate deschis din TeleOp (patrat / cerc)

    public Robot(HardwareMap hw) {
        localizer  = new Localizer(hw);
        drivetrain = new Drivetrain(hw);
        intake     = new Intake(hw);
        transfer   = new Transfer(hw);
        flywheel   = new Flywheel(hw);
        ramp       = new FlowerRamp(hw);
        stopper    = new Stopper(hw);
        shot       = new ShotController(localizer, flywheel, transfer, ramp);
    }

    /** Apeleaza din TeleOp inainte de update(). true = transferul duce bilele spre tureta. */
    public void setAssistRequested(boolean on) { assistRequested = on; }

    /** Apeleaza din TeleOp inainte de update(). true = deschide gate-ul (stopper). */
    public void setGateOpenRequested(boolean on) { gateOpenRequested = on; }

    public void update() {
        localizer.update();
        shot.update();                       // decide ce cere flywheel / transfer la tragere
        transfer.setAssist(assistRequested);
        intake.update();
        // stopper: inchis implicit; deschis doar la tragere (R1 / autonomie) sau tinta blocata (patrat / cerc)
        stopper.setOpen(shot.isActive() || gateOpenRequested);
        stopper.update();
        transfer.update();
        flywheel.update();
        ramp.update();
    }
}