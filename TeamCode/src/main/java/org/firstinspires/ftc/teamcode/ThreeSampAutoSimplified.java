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
                .lineToX(-48)
                .afterTime(1,ext.SetPosition(-200))
                .afterTime(1, arm.SetPosition(-1800))
                .afterTime(3, ext.Intake(5))
                .strafeTo(new Vector2d(-48,-51))
                .turnTo(113);





        if(isStopRequested()) {return; }

            Actions.runBlocking(
                    new ParallelAction(move.build(),
                            arm.UpdatePID()
                    )

            );


    }

    public class rotateArm {
        public DcMotorEx rotateArm;
        public int setPosition;

        public rotateArm(HardwareMap hardwareMap) {

            rotateArm = hardwareMap.get(DcMotorEx.class, "rotateArm");
            rotateArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotateArm.setDirection(DcMotor.Direction.REVERSE);
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
            CRServo Wheel1 = hardwareMap.crservo.get("Wheel1");
            Wheel1.resetDeviceConfigurationForOpMode();
            CRServo Wheel2 =  hardwareMap.crservo.get("Wheel2");
            Wheel2.resetDeviceConfigurationForOpMode();
            extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
            extendArm2 = hardwareMap.get(DcMotorEx.class, "extendArm2");
            extendArm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm2. setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm1.setDirection(DcMotorEx.Direction.REVERSE);
            extendArm2.setDirection(DcMotorEx.Direction.FORWARD);
            extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArm1.setDirection(DcMotor.Direction.REVERSE);
            extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArm2.setDirection(DcMotor.Direction.FORWARD);
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
            public setPosition(int position) { set = position; }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setPosition = set;
                return false;
            }
        }
        public Action SetPosition(int pos) { return new setPosition(pos); }
        public class Intake implements Action {
            double lpower;
            double rpower;
            ElapsedTime timer;
            double next;
            public Intake(double time) { time = next;}
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(timer == null) {
                    timer = new ElapsedTime();
                }
                Wheel1.setPower(lpower);
                Wheel2.setPower(rpower);
                return timer.seconds() < next;
            }
        }
        public Action Intake(double next) {return new Intake(next);}
    }
    }

