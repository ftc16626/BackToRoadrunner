package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class ArmPIDClass {

    private static PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static final double ticks_in_degrees = 1993.6 / 180.0;

    static double power;

    public static double returnArmPID(double target, double armPos) {

        controller = new PIDController(p,i,d);

        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        power = pid + ff;

        return power;
    }
}
