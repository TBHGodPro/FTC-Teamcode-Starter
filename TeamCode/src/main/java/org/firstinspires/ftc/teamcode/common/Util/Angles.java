package org.firstinspires.ftc.teamcode.common.Util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Angles {
    public static double normalizeDegrees(double degrees) {
        return AngleUnit.normalizeDegrees(degrees);
    }

    public static double normalizeRadians(double radians) {
        return AngleUnit.normalizeRadians(radians);
    }

    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }
}
