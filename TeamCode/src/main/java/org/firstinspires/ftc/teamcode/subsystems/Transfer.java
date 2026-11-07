package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Transfer (motorul care duce bilele spre flywheel): IDLE / FEEDING / REVERSING. FEEDING are prioritate. */
public class Transfer {
    public enum State { IDLE, FEEDING, REVERSING }

    private final DcMotorEx motor;
    private boolean feeding = false, reversing = false, assist = false;

    public Transfer(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, RobotConfig.TRANSFER);
        motor.setDirection(RobotConfig.dir(RobotConfig.TRANSFER_REVERSED));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFeeding(boolean v)   { feeding = v; }
    public void setReversing(boolean v) { reversing = v; }
    public void setAssist(boolean v)    { assist = v; }

    public State state() {
        if (feeding) return State.FEEDING;
        if (reversing) return State.REVERSING;
        return State.IDLE;
    }

    public void update() {
        switch (state()) {
            case FEEDING:   motor.setPower(RobotConfig.TRANSFER_FEED_POWER);    break;
            case REVERSING: motor.setPower(RobotConfig.TRANSFER_REVERSE_POWER); break;
            default:        motor.setPower(assist ? RobotConfig.TRANSFER_ASSIST_POWER : 0.0); break;
        }
    }
}
