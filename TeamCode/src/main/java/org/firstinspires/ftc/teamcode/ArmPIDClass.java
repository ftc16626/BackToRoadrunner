package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;

public class ArmPIDClass {

    private static PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0.001;

    public static final double ticks_in_degrees = 2520 / 113.0;
    public static InterpLUT lut = new InterpLUT();

    public ArmPIDClass(){
        lut.add(0, -.001);
        lut.add(359, -.1);
        lut.add(1694, -.35);
        lut.add(3003, -.35);
//generating final equation
        lut.createLUT();

    }

    static double power;

    public static double returnArmPID(int target, int armPos, int extpos) {
        final double ZeroOffset = -750;


        controller = new PIDController(p,i,d);

        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians((-armPos + ZeroOffset) / ticks_in_degrees)) * lut.get(extpos);

        power = pid + ff;

        return -power;
    }
}
