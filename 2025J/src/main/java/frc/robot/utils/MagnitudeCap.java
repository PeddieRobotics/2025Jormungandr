package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class MagnitudeCap {
    public static Translation2d capMagnitude(Translation2d vec, double cap) {
        double norm = vec.getNorm();
        if (norm <= cap) 
            return vec;

        Translation2d unit = vec.div(norm);
        return unit.times(cap);
    }
}
