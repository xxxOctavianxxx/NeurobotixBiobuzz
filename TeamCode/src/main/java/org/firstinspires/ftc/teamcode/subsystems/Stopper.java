package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Stopper servo: inchis = blocheaza bilele, deschis = le lasa spre tureta. */
public class Stopper {
    private final Servo servo;
    private boolean open = false;

    public Stopper(HardwareMap hw) {
        servo = hw.get(Servo.class, RobotConfig.STOPPER);
        if (RobotConfig.STOPPER_REVERSED) servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(RobotConfig.STOPPER_CLOSED_POS);
    }

    public void setOpen(boolean v) { open = v; }
    public boolean isOpen() { return open; }

    public void update() {
        servo.setPosition(open ? RobotConfig.STOPPER_OPEN_POS : RobotConfig.STOPPER_CLOSED_POS);
    }
}