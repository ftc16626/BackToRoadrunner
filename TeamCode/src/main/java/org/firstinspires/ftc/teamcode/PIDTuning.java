package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PIDTune")
public class PIDTuning extends OpMode {
    private PIDController armcontroller;
    public static double Arm_p = 0.06, Arm_i = 0, Arm_d = 0.03;
    public static double f = 0.001;
    public static int target = -200;
    private final double ticks_in_degrees = 2520 / 113.0;
    private final double ZeroOffset = -750;
    private DcMotorEx rotateArm;
    private DcMotorEx extendArm1;
    InterpLUT lut = new InterpLUT();



    @Override
    public void init() {
        armcontroller = new PIDController (Arm_p,Arm_i,Arm_d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rotateArm = hardwareMap.get(DcMotorEx.class,"rotateArm");
        extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
        extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
        rotateArm.setDirection(DcMotor.Direction.REVERSE);
        //Adding each val with a key
        lut.add(0, -.001);
        lut.add(359, -.1);
        lut.add(1694, -.35);
        lut.add(3003, -.35);
//generating final equation
        lut.createLUT();


    }

    @Override
    public void loop() {
        int extpos = extendArm1.getCurrentPosition();
        armcontroller.setPID(Arm_p,Arm_i,Arm_d);
        int armpos = rotateArm.getCurrentPosition();
        double pid = armcontroller.calculate(armpos, target);
        double ff = Math.cos(Math.toRadians((-armpos + ZeroOffset) / ticks_in_degrees)) * lut.get(extpos);
        double power = pid + ff;
        rotateArm.setPower(-power);

        telemetry.addData("armpos", armpos);
        telemetry.addData("target", target);
        telemetry.addData("extpos",extpos);
        telemetry.update();



    }
}