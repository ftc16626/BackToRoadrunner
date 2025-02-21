package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "ExTTune")
public class ExtTuning extends OpMode {
    private PIDController armcontroller;
    public static double Arm_p = 0.001, Arm_i = 0, Arm_d = 0.0005;
    public static int target = 0;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;


    @Override
    public void init() {
        armcontroller = new PIDController (Arm_p,Arm_i,Arm_d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extendArm1 = hardwareMap.get(DcMotorEx.class,"extendArm1");
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm1.setDirection(DcMotor.Direction.REVERSE);
        extendArm2 = hardwareMap.get(DcMotorEx.class,"extendArm2");
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm2.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        armcontroller.setPID(Arm_p,Arm_i,Arm_d);
        int armpos = extendArm2.getCurrentPosition();
        double pid = -armcontroller.calculate(armpos, target);

        extendArm1.setPower(pid);
        extendArm2.setPower(pid);

        telemetry.addData("armpos", armpos);
        telemetry.addData("target", target);
        telemetry.update();



    }
}