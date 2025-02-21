package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class ExtendPIDClass {

    private static PIDController controller;

    public static double p = 0.001, i = 0, d = 0.0005;

    public static final double ticks_in_degrees = 1993.6 / 180.0;


    public static double returnArmPID(double target, double armPos) {

        controller = new PIDController(p,i,d);

        double pid = controller.calculate(armPos, target);


        return pid;
    }
}
