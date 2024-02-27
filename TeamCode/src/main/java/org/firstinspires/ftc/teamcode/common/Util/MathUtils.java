package org.firstinspires.ftc.teamcode.common.Util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MathUtils {
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

    public static int clamp(int num, int min, int max) {
        return Math.max(min, Math.min(num, max));
    }

    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    public static int max(int... nums) {
        int max = 0;

        for (int num : nums) {
            if (num > max) max = num;
        }

        return max;
    }

    public static double max(double... nums) {
        double max = 0;

        for (double num : nums) {
            if (num > max) max = num;
        }

        return max;
    }

    public static int min(int... nums) {
        int min = 0;

        for (int num : nums) {
            if (num < min) min = num;
        }

        return min;
    }

    public static double min(double... nums) {
        double min = 0;

        for (double num : nums) {
            if (num < min) min = num;
        }

        return min;
    }
}
