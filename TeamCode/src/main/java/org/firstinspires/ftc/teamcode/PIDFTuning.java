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
@TeleOp(name = "PIDFTuner")
public class PIDFTuning extends OpMode {
    private PIDFController armcontroller;
    public static double Arm_p = 0, Arm_i = 0, Arm_d = 0;
    public static double Arm_f = 0;
    public static int target = 0;
    private final double ticks_in_degrees = 1993.6 / 180.0;
    //private final double ZeroOffset = 950;
    private DcMotorEx rotateArm;
    int armpos = 0;


    @Override
    public void init() {
        armcontroller = new PIDFController (Arm_p,Arm_i,Arm_d,Arm_f,target,armpos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rotateArm = hardwareMap.get(DcMotorEx.class,"rotateArm");
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        armcontroller.setPIDF(Arm_p,Arm_i,Arm_d,Arm_f);
        int armpos = rotateArm.getCurrentPosition();
        double power = armcontroller.calculate(armpos, target);

        rotateArm.setPower(power);

        telemetry.addData("armpos", armpos);
        telemetry.addData("target", target);
        telemetry.update();



    }
}