package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * pt debug wallahi
 */
public final class RobotConfig {
    private RobotConfig() {}

    public static DcMotorSimple.Direction dir(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    // =====================================================================
    // decl hardware si config
    // =====================================================================
    public static final String LEFT_FRONT  = "leftFront";
    public static final String RIGHT_FRONT = "rightFront";
    public static final String LEFT_BACK   = "leftBack";
    public static final String RIGHT_BACK  = "rightBack";
    public static final String INTAKE      = "intake";
    public static final String TRANSFER    = "turret";
    public static final String FLYWHEEL    = "flywheel";
    public static final String HOOD_SERVO  = "hood";
    public static final String FLOWER_RAMP = "flowerRamp";
    public static final String PINPOINT    = "pinpoint";

    // =====================================================================
    // DRIVE (valori verificate pe robot)
    // =====================================================================
    public static final boolean LEFT_FRONT_REVERSED  = true;
    public static final boolean LEFT_BACK_REVERSED   = false;
    public static final boolean RIGHT_FRONT_REVERSED = true;
    public static final boolean RIGHT_BACK_REVERSED  = true;
    // daca inverseaza stefanut motoarele ca data trecuta
    public static final boolean RIGHT_SIDE_FRONT_BACK_SWAPPED = false;
    public static final double  STRAFE_MULTIPLIER = 1.1;

    // =====================================================================
    // el pinpoint (offset-uri in mm fd de centrul robotului stanga = +X, fata = +Y)
    // =====================================================================
    public static final double  POD_X_OFFSET_MM = 115.0;   // ver: pod-urile sunt la ~70 mm de stanga allegedly
    public static final double  POD_Y_OFFSET_MM = -115.0;  // la 70 mm de muchia din spate (robot 370x370)
    public static final boolean POD_X_REVERSED  = true;    // la test, X mergea negativ inainte
    public static final boolean POD_Y_REVERSED  = false;

    // =====================================================================
    //  stupuri (inchi, relativ la poziția de start). Index 0 = tinta 1, index 1 = tinta 2.
    // =====================================================================
    public static final double[] TARGET_X = {94.0, 44.0};
    public static final double[] TARGET_Y = {-1.0, -10.0};

    // =====================================================================
    // AUTO-AIM
    // =====================================================================
    // 0 = fata, 180 = spate, 90 = st, -90 = dr
    public static final double SHOOTER_ANGLE_OFFSET_DEG = 0.0;
    public static final double KP_AIM              = 0.03;
    public static final double MIN_AIM_POWER       = 0.12;
    public static final double MAX_AIM_POWER       = 0.6;
    public static final double AIM_TOLERANCE_DEG   = 1.5;
    public static final double AIM_STABLE_SECONDS  = 0.15;  // cd inainte de tragere
    public static final double MIN_AIM_DISTANCE_IN = 2.0;   // lim angle
    // fara pinpoint (shoot and pray)
    public static final double DEFAULT_SHOT_DISTANCE_IN = 45.0;

    // =====================================================================
    // INTAKE
    // =====================================================================
    public static final boolean INTAKE_REVERSED  = false;
    public static final double  INTAKE_IN_POWER  = 1.0;
    public static final double  INTAKE_OUT_POWER = -1.0;

    // =====================================================================
    // TRANSFER
    // =====================================================================
    public static final boolean TRANSFER_REVERSED     = false;
    public static final double  TRANSFER_FEED_POWER   = 0.8;   // la tragere
    public static final double  TRANSFER_REVERSE_POWER = -0.8; // la evacuare
    public static final double  TRANSFER_ASSIST_POWER = 0.0;   // 0 = oprit

    // =====================================================================
    // FLYWHEEL
    // =====================================================================
    public static final boolean FLYWHEEL_REVERSED = false;
    public static final boolean FLYWHEEL_USE_VELOCITY = false;
    public static final double  FLYWHEEL_SPINUP_SECONDS       = 1.2;
    public static final double  FLYWHEEL_VELOCITY_TOLERANCE   = 50.0;
    public static final double  FLYWHEEL_STABLE_SECONDS       = 0.2;
    public static final double  FLYWHEEL_RETARGET_POWER_DELTA = 0.05;
    public static final double  FLYWHEEL_KEEP_ALIVE_SECONDS   = 4.0;

    // =====================================================================
    // HOOD
    // =====================================================================
    public static final double HOOD_START            = 0.5;
    public static final double HOOD_MIN              = 0.0;   // lim
    public static final double HOOD_MAX              = 1.0;
    public static final double HOOD_SPEED_PER_SECOND = 1.0;
    public static final double HOOD_SETTLE_SECONDS   = 0.15;
    public static final double HOOD_TOLERANCE        = 0.005;

    // =====================================================================
    // RAMPA DE LA FLOARE
    // =====================================================================
    public static final boolean RAMP_REVERSED          = false;
    public static final double  RAMP_STOWED_POS        = 0.0;  // ridiciat
    public static final double  RAMP_DEPLOYED_POS      = 0.5;  // sub bile
    public static final double  RAMP_SPEED_PER_SECOND  = 2.0;
    public static final boolean RAMP_STOW_ON_SHOT      = false; // ridica la tragere

    // =====================================================================
    // SECVENȚA DE TRAGERE
    // =====================================================================
    public static final double PREPARE_TIMEOUT_SECONDS = 3.5;  // delta t gen
    public static final double FEED_SECONDS            = 1.5;  // el feedo transfer
    public static final double RECOVER_SECONDS         = 0.3;
    //  true dupa ce fac tableu
    public static final boolean SHOT_TABLE_CALIBRATED  = false;

    // =====================================================================
    // AUTONOMIE
    // =====================================================================
    public static final double AUTO_SCORE_X = 50.0,  AUTO_SCORE_Y = -4.0, AUTO_SCORE_H = 0.0;
    public static final double AUTO_FLOWER_APPROACH_X = 82.0, AUTO_FLOWER_APPROACH_Y = 0.0,  AUTO_FLOWER_APPROACH_H = 91.0;
    public static final double AUTO_FLOWER_COLLECT_X  = 83.0, AUTO_FLOWER_COLLECT_Y  = 28.0, AUTO_FLOWER_COLLECT_H  = 90.0;
    public static final int     AUTO_TARGET_INDEX     = 0;      // care hive
    public static final boolean AUTO_SHOOT_COLLECTED  = true;   //go back
    public static final double  AUTO_INTAKE_EXTRA_SECONDS = 1.0;
    public static final double  AUTO_MOVE_TIMEOUT_SECONDS = 5.0;

    // Controlerul de pozitie
    public static final double KP_DRIVE        = 0.05;
    public static final double KP_TURN         = 0.03;
    public static final double MAX_DRIVE_POWER = 0.4;
    public static final double COLLECT_POWER   = 0.2;   // slow la polen
    public static final double MAX_TURN_POWER  = 0.5;
    public static final double MIN_DRIVE_POWER = 0.15;
    public static final double MIN_TURN_POWER  = 0.15;
    public static final double POS_TOLERANCE_IN    = 1.0;
    public static final double HEADING_TOLERANCE_DEG = 3.0;
}
