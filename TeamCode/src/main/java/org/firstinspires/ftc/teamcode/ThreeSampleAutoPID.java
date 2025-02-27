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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "PIDTHREESAMP", group = "Autonomous")
public class ThreeSampleAutoPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginpose = new Pose2d(-72,8,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginpose);

        rotateArm arm = new rotateArm(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(beginpose)
                .afterTime(0,arm.SetPosition(150))
                .strafeToLinearHeading(new Vector2d( -58, -58), Math.toRadians(235)) // Zero Strafe Score
                .afterTime(0, arm.SetPosition(600))
                .strafeToLinearHeading(new Vector2d(-51.5,-50.5), Math.toRadians(88)); // Strafe First Sample

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
    public class extendArm {
        public DcMotorEx extendArm1;
        public DcMotorEx extendArm2;
        public int setPosition;

        public extendArm(HardwareMap hardwareMap) {

        extendArm1 = hardwareMap.get(DcMotorEx.class, "extendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "extendArm2");
        extendArm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendArm1.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.FORWARD);
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
    }
    public class rotateArm {
        public DcMotorEx rotateArm;
        public int setPosition;

        public rotateArm(HardwareMap hardwareMap) {

            rotateArm = hardwareMap.get(DcMotorEx.class, "rotateArm");
            rotateArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rotateArm.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public class updatePID implements Action {
            public updatePID() {

            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotateArm.setPower(ArmPIDClass.returnArmPID(setPosition, rotateArm.getCurrentPosition(),0));
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
    }
