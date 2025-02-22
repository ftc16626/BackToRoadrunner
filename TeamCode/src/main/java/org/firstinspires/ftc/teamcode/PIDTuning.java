package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    private final double ticks_in_degrees = 2570 / 100.0;
    private final double ZeroOffset = -750;
    private DcMotorEx rotateArm;


    @Override
    public void init() {
        armcontroller = new PIDController (Arm_p,Arm_i,Arm_d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rotateArm = hardwareMap.get(DcMotorEx.class,"rotateArm");
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateArm.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        armcontroller.setPID(Arm_p,Arm_i,Arm_d);
        int armpos = rotateArm.getCurrentPosition();
        double pid = armcontroller.calculate(armpos, target);
        double ff = Math.cos(Math.toRadians((-armpos + ZeroOffset) / ticks_in_degrees)) * f;
        double power = pid + ff;

        rotateArm.setPower(-power);

        telemetry.addData("armpos", armpos);
        telemetry.addData("target", target);
        telemetry.update();



    }
}