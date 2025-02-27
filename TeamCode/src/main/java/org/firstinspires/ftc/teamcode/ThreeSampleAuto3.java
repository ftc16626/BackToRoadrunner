package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "BadControlHub", group = "Autonomous")
public class ThreeSampleAuto3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        CRServo Wheel1 = hardwareMap.crservo.get("Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        CRServo Wheel2 = hardwareMap.crservo.get("Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();
        DcMotor rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        DcMotor extendArm1 = hardwareMap.get(DcMotor.class, "extendArm1");
        DcMotor extendArm2 = hardwareMap.get(DcMotor.class, "extendArm2");

        rotateArm.setDirection(DcMotor.Direction.REVERSE);
        extendArm1.setDirection(DcMotor.Direction.REVERSE);
        extendArm2.setDirection(DcMotor.Direction.FORWARD);
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        rotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-62, -9, 0))
                       .lineToX(-60) // Zero Strafe Score
                       .strafeTo(new Vector2d(-50,-54))
                        .strafeToLinearHeading(new Vector2d( -50.01, -54.01), Math.toRadians(119))
                       .waitSeconds(.01)
                        .stopAndAdd(new RotateUp(rotateArm,260,3))
                        .stopAndAdd(new ExtendOut(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, 18,1,0,0,1.8))
                        .stopAndAdd(new Intake(Wheel1, Wheel2, -1,1,2)) // Score Zero
                        .stopAndAdd(new ExtendIn(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, -6,-1,0,0,1))
                        .stopAndAdd(new RotateDown(rotateArm,-200,2)) // Score First Sample Block

                        .strafeToLinearHeading(new Vector2d( -52, -56), Math.toRadians(0))
                       .strafeTo(new Vector2d(-52,-46))
                       .strafeTo(new Vector2d(-45,-46)) // Strafe Second Sample

                        .stopAndAdd(new ExtendOut(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, 18,.75,1,-1,1.5))
                        .stopAndAdd(new ExtendIn(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, -6,-1,0,0,1))
                        .stopAndAdd(new RotateUp(rotateArm,220,3)) // Grab Second Sample

                        .strafeToLinearHeading(new Vector2d( -35.01, -46.01), Math.toRadians(116))
                        .lineToYLinearHeading(-60,116) // Strafe Second Sample Score

                        .stopAndAdd(new ExtendOut(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, 18,1,0,0,1.5))
                        .stopAndAdd(new Intake(Wheel1, Wheel2, -1,1,1)) // Score Zero
                        .stopAndAdd(new ExtendIn(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, -6,-1,0,0,1))

                        .strafeToLinearHeading(new Vector2d( -51, -55), Math.toRadians(0))
                        .strafeTo(new Vector2d(-51,-50))
                        .strafeTo(new Vector2d(-40,-50))
                        .stopAndAdd(new RotateDown(rotateArm,-200,2)) // Don't Hit Wall


                        .stopAndAdd(new ExtendOut(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, 18,.75,-1,1,1.5))
                        .stopAndAdd(new ExtendIn(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, -6,-1,0,0,1))
                        .stopAndAdd(new RotateUp(rotateArm,260,3)) // Grab Third Sample


                        .stopAndAdd(new ExtendOut(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2, 18,1,0,0,1.5))
                        .stopAndAdd(new Intake(Wheel1, Wheel2, -1,1,1)) // Score Zero







//                        .waitSeconds(1)
//                        .strafeToLinearHeading(new Vector2d( -52, -56), Math.toRadians(0))
//                        .strafeTo(new Vector2d(-52,-46))
//                        .strafeTo(new Vector2d(-40,-46)) // Strafe First Sample

//                        .strafeToLinearHeading(new Vector2d( -40.01, -46.01), Math.toRadians(116))
//                        .strafeTo(new Vector2d(-52,-56)) // First Score Strafe

//                        .strafeToLinearHeading(new Vector2d(-67,-52), Math.toRadians(84)) // SecondSampleStrafe

//                        .strafeToLinearHeading(new Vector2d( -63, -67), Math.toRadians(240)) // SecondScoreStrafe

                        .build());

    }

    public class Intake implements Action {
        CRServo Wheel1;
        CRServo Wheel2;
        double lpower;
        double rpower;
        ElapsedTime timer;
        double next;
        private boolean initialized = false;


        public Intake(CRServo ls, CRServo rs, double lp, double rp, double stop) {
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.next = stop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                initialized = true;
            }
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (timer.seconds() < next) {
                Wheel1.setPower(1);
                Wheel2.setPower(-1);
                return true;

            } else {
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                return false;
            }
        }
    }
    public class RotateUp implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        DcMotor rotateArm;
        double Rposition;
        double next;

        public RotateUp(DcMotor ar, double rot, double stop) {
            this.rotateArm = ar;
            this.Rposition = rot;
            this.next = stop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double COUNTS_PER_DEGREE = COUNTS_PER_ROT_MOTOR_REV / 360;
                int newROTarget;
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateArm.setPower(1);
                initialized = true;
            }

            double pos = rotateArm.getCurrentPosition();
            double end = rotateArm.getTargetPosition();

            if (pos < end) {
                return timer.seconds() < next;
            } else {
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }
    }
    public class RotateDown implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        DcMotor rotateArm;
        double Rposition;
        double next;

        public RotateDown(DcMotor ar, double rot, double stop) {
            this.rotateArm = ar;
            this.Rposition = rot;
            this.next = stop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                final double COUNTS_PER_ROT_MOTOR_REV = 1993.6;
                final double COUNTS_PER_DEGREE = COUNTS_PER_ROT_MOTOR_REV / 360;
                int newROTarget;
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateArm.setPower(-1);
                initialized = true;
            }

            double pos = rotateArm.getCurrentPosition();
            double end = rotateArm.getTargetPosition();

            if (pos > end) {
                return timer.seconds() < next;
            } else {
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }
    }
    public class ExtendOut implements Action {
        DcMotor extendArm1;
        DcMotor extendArm2;
        double Aposition;
        private boolean initialized = false;
        ElapsedTime timer;
        double speed;
        CRServo Wheel1;
        CRServo Wheel2;
        double lpower;
        double rpower;
        DcMotor rotateArm;
        double next;

        public ExtendOut(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double speed, double lp, double rp, double stop) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.next = stop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                final double COUNTS_PER_EXT_MOTOR_REV = 537.7;
                final double PULLEY_DIAMETER_INCHES = 1.5;// For figuring circumference
                final double COUNTS_PER_EXTINCH = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                initialized = true;
            }

            double pos = extendArm2.getCurrentPosition();
            double end = extendArm2.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (timer.seconds() < next) {
                return true;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.05));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                return false;
            }
        }
    }
    public class ExtendIn implements Action {
        DcMotor extendArm1;
        DcMotor extendArm2;
        double Aposition;
        private boolean initialized = false;
        ElapsedTime timer;
        double speed;
        CRServo Wheel1;
        CRServo Wheel2;
        double lpower;
        double rpower;
        DcMotor rotateArm;
        double next;

        public ExtendIn(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double speed, double lp, double rp, double stop) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.next = stop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                final double COUNTS_PER_EXT_MOTOR_REV = 537.7;
                final double PULLEY_DIAMETER_INCHES = 1.5;// For figuring circumference
                final double COUNTS_PER_EXTINCH = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                initialized = true;
            }

            double pos = extendArm2.getCurrentPosition();
            double end = extendArm2.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (timer.seconds() < next) {
                return true;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.05));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                return false;
            }
        }
    }
}

