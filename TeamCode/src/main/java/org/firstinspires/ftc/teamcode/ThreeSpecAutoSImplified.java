package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
public class ThreeSpecAutoSImplified extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginpose = new Pose2d(-72,8,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginpose);

        rotateArm arm = new rotateArm(hardwareMap);
        extendArm ext = new extendArm(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(beginpose)
                .afterTime(3,arm.SetPosition(150))
                .strafeToLinearHeading(new Vector2d( -58, -58), Math.toRadians(235)) // Zero Strafe Score
                .afterTime(3, arm.SetPosition(600))
                .strafeToLinearHeading(new Vector2d(-51.5,-50.5), Math.toRadians(88));


        if(isStopRequested()) {return; }

            Actions.runBlocking(
                    new ParallelAction(
                            move.build(),
                            arm.UpdatePID()
                    )

            );


    }

    public class Intake implements Action {
        CRServo Wheel1;
        CRServo Wheel2;
        double lpower;
        double rpower;
        ElapsedTime timer;
        double next;


        public Intake(CRServo ls, CRServo rs, double lp, double rp, double stop) {
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.next = stop;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
            }
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);
            return timer.seconds() < next;

        }
    }
    public class rotateArm {
        public DcMotorEx rotateArm;
        public int setPosition;

        public rotateArm(HardwareMap hardwareMap) {

            rotateArm = hardwareMap.get(DcMotorEx.class, "rotateArm");
            rotateArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rotateArm.setDirection(DcMotorEx.Direction.REVERSE);
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
        public int setPosition;

        public extendArm(HardwareMap hardwareMap) {

            extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
            extendArm2 = hardwareMap.get(DcMotorEx.class, "extendArm2");
            extendArm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm2. setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm1.setDirection(DcMotorEx.Direction.REVERSE);
            extendArm2.setDirection(DcMotorEx.Direction.FORWARD);
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

        public extendArm(HardwareMap hardwareMap) {
            extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
            extendArm2 = hardwareMap.get(DcMotorEx.class, "extendArm2");
            extendArm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            extendArm1.setDirection(DcMotorEx.Direction.REVERSE);
            extendArm2.setDirection(DcMotorEx.Direction.FORWARD);
        }
        public class ArmOut implements Action {
            double Aposition;
            private boolean initialized = false;
            ElapsedTime timer;
            double speed;
            double next;



            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer = new ElapsedTime();
                    final double COUNTS_PER_EXT_MOTOR_REV = 537.7;
                    final double PULLEY_DIAMETER_INCHES = 1.5;// For figuring circumference
                    final double COUNTS_PER_EXTINCH = (COUNTS_PER_EXT_MOTOR_REV) /
                            (PULLEY_DIAMETER_INCHES * 3.1415);
                    final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                    final double ROTATE_GEAR_REDUC = 2.0;
                    final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                    int newROTarget;
                    int newLEXTarget;
                    int newREXTarget;
                    newLEXTarget = extendArm1.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                    newREXTarget = extendArm2.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                    extendArm1.setTargetPosition(newLEXTarget);
                    extendArm2.setTargetPosition(newREXTarget);
                    extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extendArm1.setPower(Math.abs(speed));
                    extendArm2.setPower(Math.abs(speed));
                    initialized = true;
                }

                double pos = extendArm1.getCurrentPosition();
                double end = extendArm2.getTargetPosition();

                if (pos < end) {
                    return timer.seconds() < next;
                } else {
                    extendArm1.setPower(Math.abs(0));
                    extendArm2.setPower(Math.abs(0));
                    extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    return false;
                }
            }
        }

        }
    }

