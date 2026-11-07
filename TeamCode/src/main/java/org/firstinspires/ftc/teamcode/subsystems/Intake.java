package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Intake: state machine OFF / INTAKING / EJECTING. */
public class Intake {
    public enum State { OFF, INTAKING, EJECTING }

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, RobotConfig.INTAKE);
        motor.setDirection(RobotConfig.dir(RobotConfig.INTAKE_REVERSED));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void command(State newState) { state = newState; }
    public State state() { return state; }

    public void update() {
        switch (state) {
            case INTAKING: motor.setPower(RobotConfig.INTAKE_IN_POWER);  break;
            case EJECTING: motor.setPower(RobotConfig.INTAKE_OUT_POWER); break;
            default:       motor.setPower(0.0);                          break;
        }
    }
}
