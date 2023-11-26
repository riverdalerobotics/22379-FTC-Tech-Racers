package org.firstinspires.ftc.teamcode.TERbot;

public class HelperMethods {


    public static double ticksToRotation(int ticks, int encoderTickRatio) {
        return ticks / encoderTickRatio;
    }

    public static int clipInteger(int inputInt, int min, int max) {
        if (inputInt < min) {
            return min;
        } else if (inputInt > max) {
            return max;
        } else {
            return inputInt;
        }
    }

    public static double clipDouble(double inputDouble, double min, double max) {
        if (inputDouble < min) {
            return min;
        } else if (inputDouble > max) {
            return max;
        } else {
            return inputDouble;
        }
    }
}
