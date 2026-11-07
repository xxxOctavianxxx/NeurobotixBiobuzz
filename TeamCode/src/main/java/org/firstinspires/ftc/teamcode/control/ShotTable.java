package org.firstinspires.ftc.teamcode.control;

/**
 * Tabel de tragere: pentru fiecare distanță până la țintă, ce putere/viteză de flywheel
 * și ce poziție de hood intră mingea. Între rânduri se interpolează liniar.
 *
 * !!! VALORILE DE MAI JOS SUNT BULLSHIT, NU SUNT MĂSURATE !!!
 *
 * (putere sau viteză + hood) nimerește. Apoi pui RobotConfig.SHOT_TABLE_CALIBRATED = true.
 */
public final class ShotTable {
    private ShotTable() {}

    public static final class Params {
        public final double power, velocity, hood;
        public Params(double power, double velocity, double hood) {
            this.power = power;
            this.velocity = velocity;
            this.hood = hood;
        }
    }

    // { distanță (inchi), putere flywheel (0-1), viteză flywheel (ticks/s), poziție hood }
    // Distanțele trebuie să fie în ordine crescătoare.
    private static final double[][] DATA = {
            { 30.0, 0.60, 1200.0, 0.30},
            { 45.0, 0.70, 1450.0, 0.40},
            { 60.0, 0.75, 1600.0, 0.50},
            { 90.0, 0.85, 1900.0, 0.65},
    };

    private static Params row(double[] r) { return new Params(r[1], r[2], r[3]); }
    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }

    public static Params lookup(double distance) {
        int last = DATA.length - 1;
        if (distance <= DATA[0][0]) return row(DATA[0]);
        if (distance >= DATA[last][0]) return row(DATA[last]);
        for (int i = 0; i < last; i++) {
            double[] a = DATA[i], b = DATA[i + 1];
            if (distance <= b[0]) {
                double t = (distance - a[0]) / (b[0] - a[0]);
                return new Params(lerp(a[1], b[1], t), lerp(a[2], b[2], t), lerp(a[3], b[3], t));
            }
        }
        return row(DATA[last]);
    }
}
