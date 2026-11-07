package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.ShotController;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRamp;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/** Adună toate sistemele robotului. update() se apelează o dată pe buclă (TeleOp și Autonomie). */
public class Robot {
    public final Localizer localizer;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Transfer transfer;
    public final Flywheel flywheel;
    public final Hood hood;
    public final FlowerRamp ramp;
    public final ShotController shot;

    public Robot(HardwareMap hw) {
        localizer  = new Localizer(hw);
        drivetrain = new Drivetrain(hw);
        intake     = new Intake(hw);
        transfer   = new Transfer(hw);
        flywheel   = new Flywheel(hw);
        hood       = new Hood(hw);
        ramp       = new FlowerRamp(hw);
        shot       = new ShotController(localizer, flywheel, hood, transfer, ramp);
    }

    public void update() {
        localizer.update();
        shot.update();                       // decide ce cer flywheel / hood / transfer
        transfer.setAssist(intake.state() == Intake.State.INTAKING);
        intake.update();
        transfer.update();
        flywheel.update();
        hood.update();
        ramp.update();
    }
}
