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


@Autonomous(name = "AgaBoga", group = "Autonomous")
public class ThreeSampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        CRServo Wheel1 = hardwareMap.crservo.get("Wheel1");
        Wheel1.resetDeviceConfigurationForOpMode();
        CRServo Wheel2 =  hardwareMap.crservo.get("Wheel2");
        Wheel2.resetDeviceConfigurationForOpMode();
        DcMotor rotateArm = hardwareMap.get(DcMotor.class, "rotateArm");
        DcMotor extendArm1 = hardwareMap.get(DcMotor.class, "extendArm1");
        DcMotor extendArm2 = hardwareMap.get(DcMotor.class, "extendArm2");

        rotateArm.setDirection(DcMotor.Direction.REVERSE);
        extendArm1.setDirection(DcMotor.Direction.FORWARD);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
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
                drive.actionBuilder(new Pose2d(-62,-9,0))
                        .lineToX(-60) // Zero Strafe Score
                        .strafeTo(new Vector2d(-51,-55))
                        .strafeToLinearHeading(new Vector2d( -51.01, -55.01), Math.toRadians(116))
                        .waitSeconds(.01)
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,0,70,0,0,0,.8)) // RotUP
                        .waitSeconds(.5)
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,16,0,1,0,0,2)) // ExtUp
                        .stopAndAdd(new Intake(Wheel1, Wheel2, -1,1,2))
                        .waitSeconds(1)
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-21,0,-1,0,0,3)) // ExtDown
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,0,-120,-1,0,0,1.3)) // RotDown
                        .strafeToLinearHeading(new Vector2d( -52, -56), Math.toRadians(0))
                        .strafeTo(new Vector2d(-52,-46))
                        .strafeTo(new Vector2d(-40,-46)) // Strafe First Sample
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,22,0,.85,-1,1,3)) // Grap Sample One
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-17,0,-1,0,0,1)) // Tuck In Arm
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,0,90,1,0,0,1.8)) // RotUp
                        .strafeToLinearHeading(new Vector2d( -40.01, -46.01), Math.toRadians(116))
                        .strafeTo(new Vector2d(-52,-56)) // First Score Strafe
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,5,44,1,0,0,1.2)) // RotUp
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,16.75,0,1,0,0,1.7)) // ExtUp
                        .stopAndAdd(new Intake(Wheel1, Wheel2, 1,-1,1)) // Score First Sample
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-19,0,-1,0,0,1.8)) // ExtDown
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-5,-139,-1,0,0,1.3)) // Rot Down
                        .strafeToLinearHeading(new Vector2d(-67,-52), Math.toRadians(84)) // SecondSampleStrafe
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,20,1,.85,-1,1,3)) // Grap Sample Two
                        .stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-16,0,-1,0,0,1)) // Tuck In Arm
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,0,98,1,0,0,1.5)) // RotUp
                        .strafeToLinearHeading(new Vector2d( -63, -67), Math.toRadians(240)) // SecondScoreStrafe
                        .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,20,24,1,0,0,2)) // ArmUP
                        .stopAndAdd(new Intake(Wheel1, Wheel2, 1,-1,1)) // Score Second Sample







                        // .stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,11,22,1,0,0 ))
                        //.stopAndAdd(new ArmInRD(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,-12,-92,-1,-1,1))
                        //.strafeToLinearHeading(new Vector2d( -48, -48 ), Math.toRadians(88))
                        //.stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,12,0,.6,-1,1))
                        //.stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,0,122,1,0,0))//.lineToY(-61)
                        //.stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,10,10,1,0,0))
                        //.stopAndAdd(new ArmOutRU(extendArm1, extendArm2, rotateArm, Wheel1, Wheel2,0,0,1,1,-1))
                        //.waitSeconds(.3)

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
    public class ArmOutRU implements Action {
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
        double Rposition;
        double next;

        public ArmOutRU(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp, double stop) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
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
                final double COUNTS_PER_ROT_MOTOR_REV = 8192;
                final double ROTATE_GEAR_REDUC = 1.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int) (Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos < end) {
                return timer.seconds() < next;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                return false;
            }
        }
    }
    public class ArmInRD implements Action {
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
        double Rposition;
        double next;

        public ArmInRD(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp, double stop) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
            this.next = stop;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                final double COUNTS_PER_ROT_MOTOR_REV = 8192;
                final double ROTATE_GEAR_REDUC = 1.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(-1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos > end) {
                return timer.seconds() < next;
            } else {
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                return false;
            }
        }


    }

    public class ArmInRU implements Action {
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
        double Rposition;

        public ArmInRU(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                final double COUNTS_PER_ROT_MOTOR_REV = 8192;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                rotateArm.setDirection(DcMotor.Direction.FORWARD);
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(-1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (timer.seconds() < 3) {
                return true;
            } else if (pos > end) {
                return true;
            } else {
                rotateArm.setDirection(DcMotor.Direction.REVERSE);
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }


        }


    }

    public class ArmOutRD implements Action {
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
        double Rposition;

        public ArmOutRD(DcMotor l, DcMotor r, DcMotor ar, CRServo ls, CRServo rs, double ext, double rot, double speed, double lp, double rp) {
            this.extendArm1 = l;
            this.extendArm2 = r;
            this.Aposition = ext;
            this.speed = speed;
            this.Wheel1 = ls;
            this.Wheel2 = rs;
            this.lpower = lp;
            this.rpower = rp;
            this.rotateArm = ar;
            this.Rposition = rot;
        }
        @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                final double     COUNTS_PER_EXT_MOTOR_REV    = 537.7;
                final double     PULLEY_DIAMETER_INCHES = 1.5 ;// For figuring circumference
                final double     COUNTS_PER_EXTINCH      = (COUNTS_PER_EXT_MOTOR_REV) /
                        (PULLEY_DIAMETER_INCHES * 3.1415);
                final double COUNTS_PER_ROT_MOTOR_REV = 8192;
                final double ROTATE_GEAR_REDUC = 2.0;
                final double COUNTS_PER_DEGREE = (COUNTS_PER_ROT_MOTOR_REV * ROTATE_GEAR_REDUC) / 360;
                extendArm1.setDirection(DcMotor.Direction.REVERSE);
                extendArm2.setDirection(DcMotor.Direction.FORWARD);
                int newROTarget;
                int newLEXTarget;
                int newREXTarget;
                newLEXTarget = extendArm1.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newREXTarget = extendArm2.getCurrentPosition() + (int)(Aposition * COUNTS_PER_EXTINCH);
                newROTarget = rotateArm.getCurrentPosition() + (int) (Rposition * COUNTS_PER_DEGREE);
                rotateArm.setTargetPosition(newROTarget);
                extendArm1.setTargetPosition(newLEXTarget);
                extendArm2.setTargetPosition(newREXTarget);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArm1.setPower(Math.abs(speed));
                extendArm2.setPower(Math.abs(speed));
                rotateArm.setPower(Math.abs(-1));
                initialized = true;
            }

            double pos = extendArm1.getCurrentPosition() + rotateArm.getCurrentPosition();
            double end = extendArm2.getTargetPosition() + rotateArm.getTargetPosition();
            Wheel1.setPower(lpower);
            Wheel2.setPower(rpower);

            if (pos > end) {
                return true;
            } else {
                extendArm1.setDirection(DcMotor.Direction.FORWARD);
                extendArm2.setDirection(DcMotor.Direction.REVERSE);
                extendArm1.setPower(Math.abs(0));
                extendArm2.setPower(Math.abs(0));
                rotateArm.setPower(Math.abs(.03));
                rotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }


    }

}
