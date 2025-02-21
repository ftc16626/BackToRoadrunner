package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class ArmPIDClass {

    private static PIDController controller;

    public static double p = 0.06, i = 0, d = 0.03;
    public static double f = 0.001;

    public static final double ticks_in_degrees = 2570 / 100.0;


    static double power;

    public static double returnArmPID(double target, double armPos) {
        final double ZeroOffset = -750;

        controller = new PIDController(p,i,d);

        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians((-armPos + ZeroOffset) / ticks_in_degrees)) * f;

        power = pid + ff;

        return -power;
    }
}
