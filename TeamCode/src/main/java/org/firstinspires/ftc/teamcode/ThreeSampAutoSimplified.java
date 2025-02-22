package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "PIDWITHENCODERS", group = "Autonomous")
public class ThreeSampAutoSimplified extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginpose = new Pose2d(-62,-9,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginpose);

        rotateArm arm = new rotateArm(hardwareMap);
        extendArm ext = new extendArm(hardwareMap);
        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(beginpose)
                .lineToX(-60)
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-52,-56))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d( -52.01, -56.01), Math.toRadians(116))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d( -52, -56), Math.toRadians(0))
                .strafeTo(new Vector2d(-52,-46))
                .strafeTo(new Vector2d(-40,-46))
                .waitSeconds(3)
                .afterTime(5,ext.SetPosition(-1000))
                .afterTime(5,arm.SetPosition(-250))
                .afterTime(5,ext.Intake(3,1,-1))
                .strafeToLinearHeading(new Vector2d( -40.01, -46.01), Math.toRadians(116))
                .strafeTo(new Vector2d(-52,-56));
                




        if(isStopRequested()) {return; }

            Actions.runBlocking(
                    new ParallelAction(move.build(),
                            arm.UpdatePID(),
                            ext.UpdatePID()
                    )
            );


    }


    public class rotateArm {
        public DcMotorEx rotateArm;
        public int setPosition;

        public rotateArm(HardwareMap hardwareMap) {

            rotateArm = hardwareMap.get(DcMotorEx.class, "rotateArm");
            rotateArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rotateArm.setDirection(DcMotor.Direction.REVERSE);
            rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class updatePID implements Action {
            public updatePID() {

            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotateArm.setPower(ArmPIDClass.returnArmPID(setPosition, rotateArm.getCurrentPosition()));
                return true;
            }
        }
        public Action UpdatePID() { return new updatePID();}

        public class setPosition implements Action {
            int set;
            private boolean initialized = false;

            public setPosition(int position) { set = position; }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setPosition = set;
                return false;
            }
        }
        public Action SetPosition(int pos) { return new setPosition(pos); }
    }
    public class extendArm {
        public DcMotorEx extendArm1;
        public DcMotorEx extendArm2;
        CRServo Wheel1;
        CRServo Wheel2;
        public int setPosition;
        public extendArm(HardwareMap hardwareMap) {
            Wheel1 = hardwareMap.get(CRServo.class,"Wheel1");
            Wheel1.resetDeviceConfigurationForOpMode();
            Wheel2 =  hardwareMap.get(CRServo.class,"Wheel2");
            Wheel2.resetDeviceConfigurationForOpMode();
            extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
            extendArm2 = hardwareMap.get(DcMotorEx.class, "extendArm2");
            extendArm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm2. setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm1.setDirection(DcMotorEx.Direction.REVERSE);
            extendArm2.setDirection(DcMotorEx.Direction.FORWARD);
            extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class updatePID implements Action {
            public updatePID() {

            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extendArm1.setPower(ExtendPIDClass.returnArmPID(setPosition, extendArm1.getCurrentPosition()));
                extendArm2.setPower(ExtendPIDClass.returnArmPID(setPosition, extendArm2.getCurrentPosition()));
                return true;
            }
        }
        public Action UpdatePID() { return new updatePID();}

        public class setPosition implements Action {
            int set;
            private boolean initialized = false;

            public setPosition(int position) { set = position; }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setPosition = set;
                return false;
            }
        }
        public Action SetPosition(int pos) { return new setPosition(pos); }
        public class Intake implements Action {
            double wheel1;
            double wheel2;
            ElapsedTime timer;
            double time;
            public Intake(double next, double lpower, double rpower) { time = next; wheel1 = lpower; wheel2 = rpower;}
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(timer == null) {
                    timer = new ElapsedTime();
                }
                Wheel1.setPower(wheel1);
                Wheel2.setPower(wheel2);
                return timer.seconds() < time;
            }
        }
        public Action Intake(double next, double lpower, double rpower) {return new Intake(next, lpower, rpower);}
    }
    }

