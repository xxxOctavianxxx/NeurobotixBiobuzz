package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Șasiu Mecanum. drive(): forward + = înainte, strafe + = dreapta, turn + = sens orar. */
public class Drivetrain {
    private final DcMotorEx lf, rf, lb, rb;

    public Drivetrain(HardwareMap hw) {
        lf = hw.get(DcMotorEx.class, RobotConfig.LEFT_FRONT);
        rf = hw.get(DcMotorEx.class, RobotConfig.RIGHT_FRONT);
        lb = hw.get(DcMotorEx.class, RobotConfig.LEFT_BACK);
        rb = hw.get(DcMotorEx.class, RobotConfig.RIGHT_BACK);

        lf.setDirection(RobotConfig.dir(RobotConfig.LEFT_FRONT_REVERSED));
        lb.setDirection(RobotConfig.dir(RobotConfig.LEFT_BACK_REVERSED));
        rf.setDirection(RobotConfig.dir(RobotConfig.RIGHT_FRONT_REVERSED));
        rb.setDirection(RobotConfig.dir(RobotConfig.RIGHT_BACK_REVERSED));

        for (DcMotorEx m : new DcMotorEx[]{lf, rf, lb, rb}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void drive(double forward, double strafe, double turn) {
        double x = strafe * RobotConfig.STRAFE_MULTIPLIER;
        double denominator = Math.max(Math.abs(forward) + Math.abs(x) + Math.abs(turn), 1.0);
        double frontLeft  = (forward + x + turn) / denominator;
        double backLeft   = (forward - x + turn) / denominator;
        double frontRight = (forward - x - turn) / denominator;
        double backRight  = (forward + x - turn) / denominator;

        lf.setPower(frontLeft);
        lb.setPower(backLeft);
        if (RobotConfig.RIGHT_SIDE_FRONT_BACK_SWAPPED) {
            rf.setPower(backRight);
            rb.setPower(frontRight);
        } else {
            rf.setPower(frontRight);
            rb.setPower(backRight);
        }
    }

    public void stop() { drive(0, 0, 0); }
}
