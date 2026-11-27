package org.firstinspires.ftc.teamcode.control;

/**
 * Tabel de tragere: pentru fiecare distanta (INCH) pana la tinta, viteza flywheel-ului (ticks/s).
 * Intre randuri se interpoleaza liniar.
 *
 * Dupa calibrare, pune RobotConfig.SHOT_TABLE_CALIBRATED = true.
 */
public final class ShotTable {
    private ShotTable() {}

    public static final class Params {
        public final double velocity;
        public Params(double velocity) { this.velocity = velocity; }
    }

    // { distanta (INCH), viteza flywheel (ticks/s) }. Distantele in ordine crescatoare.
    private static final double[][] DATA = {
            { 31.5, 2750.0 },   // ~80 cm
            { 47.2, 3350.0 },   // ~120 cm
            { 59.1, 3550.0 },   // ~150 cm
            { 90.6, 4000.0 },   // ~230 cm
    };

    public static Params lookup(double distanceIn) {
        int last = DATA.length - 1;
        if (distanceIn <= DATA[0][0]) return new Params(DATA[0][1]);
        if (distanceIn >= DATA[last][0]) return new Params(DATA[last][1]);
        for (int i = 0; i < last; i++) {
            double[] a = DATA[i], b = DATA[i + 1];
            if (distanceIn <= b[0]) {
                double t = (distanceIn - a[0]) / (b[0] - a[0]);
                return new Params(a[1] + (b[1] - a[1]) * t);
            }
        }
        return new Params(DATA[last][1]);
    }
}