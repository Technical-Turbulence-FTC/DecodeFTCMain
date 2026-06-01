package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class Robot {

    //Initialize Public Components

    public static boolean usingLimelight = true;
    public static boolean usingCamera = false;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    public PIDFCoefficients shooterPIDF;
    public static double shooterPIDF_P = 255;
    public static double shooterPIDF_I = 0.0;
    public static double shooterPIDF_D = 0.0;
    public static double shooterPIDF_F = 75;
//    public double[] shooterPIDF_StepSizes = {10.0, 1.0, 0.001, 0.0001};
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public Servo hood;
    public Servo transferServo;
    public Servo spindexBlocker;
    public Servo rapidFireBlocker;
    public Servo tilt1;
    public Servo tilt2;
    public Servo turr1;
    public Servo turr2;
    public Servo spin1;
    public Servo spin2;
    public TouchSensor beam1;
    public TouchSensor beam2;
    public TouchSensor beam3;
    public RevColorSensorV3 revSensor;

    public VoltageSensor voltage;

    // Below is disregarded
    public AnalogInput spin1Pos;
    public AnalogInput spin2Pos;
    public AnalogInput turr1Pos;
    public AnalogInput transferServoPos;
    public AprilTagProcessor aprilTagProcessor;
    public WebcamName webcam;
    public RevColorSensorV3 color1;
    public RevColorSensorV3 color2;
    public RevColorSensorV3 color3;
    public Limelight3A limelight;
    public Servo light;

    public Robot(HardwareMap hardwareMap) {

        //Define components w/ hardware map
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterPIDF = new PIDFCoefficients(shooterPIDF_P, shooterPIDF_I, shooterPIDF_D, shooterPIDF_F);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter1.setVelocity(0);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter2.setVelocity(0);

        hood = hardwareMap.get(Servo.class, "hood");

        turr1 = hardwareMap.get(Servo.class, "turr1");

        turr2 = hardwareMap.get(Servo.class, "turr2");

        spin1 = hardwareMap.get(Servo.class, "spin2");

        spin2 = hardwareMap.get(Servo.class, "spin1");

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        transferServo = hardwareMap.get(Servo.class, "transferServo");

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spindexBlocker = hardwareMap.get(Servo.class, "spinB");

        rapidFireBlocker = hardwareMap.get(Servo.class, "rapidB");

        tilt1 = hardwareMap.get(Servo.class, "tilt1");
        tilt2 = hardwareMap.get(Servo.class, "tilt2");

        beam1 = hardwareMap.get(TouchSensor.class, "beam1");
        beam2 = hardwareMap.get(TouchSensor.class, "beam2");
        beam3 = hardwareMap.get(TouchSensor.class, "beam3");

        revSensor = hardwareMap.get(RevColorSensorV3.class, "rev");

        // Below is disregarded

        turr1Pos = hardwareMap.get(AnalogInput.class, "t1Pos"); // Encoder of turret plugged in intake port

        spin1Pos = hardwareMap.get(AnalogInput.class, "spin1Pos");

        spin2Pos = hardwareMap.get(AnalogInput.class, "spin2Pos");

        transferServoPos = hardwareMap.get(AnalogInput.class, "tSPos");

        color1 = hardwareMap.get(RevColorSensorV3.class, "c1");

        color2 = hardwareMap.get(RevColorSensorV3.class, "c2");

        color3 = hardwareMap.get(RevColorSensorV3.class, "c3");

        if (usingLimelight) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } else if (usingCamera) {
            webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        }

        light = hardwareMap.get(Servo.class, "light");
        voltage = hardwareMap.voltageSensor.iterator().next();
    }

    // Voids below are used to minimize hardware calls to minimize loop times

    // Used to cut off digits that are negligible
    private final int maxDigits = 5;
    private final int roundingFactor = (int) Math.pow(10, maxDigits);

    private double prevFrontLeftPower = -10.501;
    public void setFrontLeftPower(double pow){
        pow = (double) Math.round(pow * roundingFactor) / roundingFactor;
        if (pow != prevFrontLeftPower){
            frontLeft.setPower(pow);
        }
        prevFrontLeftPower = pow;
    }

    private double prevFrontRightPower = -10.501;
    public void setFrontRightPower(double pow){
        pow = (double) Math.round(pow * roundingFactor) / roundingFactor;
        if (pow != prevFrontRightPower){
            frontRight.setPower(pow);
        }
        prevFrontRightPower = pow;
    }

    private double prevBackLeftPower = -10.501;
    public void setBackLeftPower(double pow){
        pow = (double) Math.round(pow * roundingFactor) / roundingFactor;
        if (pow != prevBackLeftPower){
            backLeft.setPower(pow);
        }
        prevBackLeftPower = pow;
    }

    private double prevBackRightPower = -10.501;
    public void setBackRightPower(double pow){
        pow = (double) Math.round(pow * roundingFactor) / roundingFactor;
        if (pow != prevBackRightPower){
            backRight.setPower(pow);
        }
        prevBackRightPower = pow;
    }

    private double prevIntakePower = -10.501;
    public void setIntakePower(double pow){
        pow = (double) Math.round(pow * roundingFactor) / roundingFactor;
        if (pow != prevIntakePower){
            intake.setPower(pow);
        }
        prevIntakePower = pow;
    }

    private double prevTransferPower = -10.501;
    public void setTransferPower(double pow){
        pow = (double) Math.round(pow * roundingFactor) / roundingFactor;
        if (pow != prevTransferPower){
            transfer.setPower(pow);
        }
        prevTransferPower = pow;
    }

    // shooter motors are done in separate class

    private double prevHoodPos = -10.501;
    public void setHoodPos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevHoodPos){
            hood.setPosition(pos);
        }
        prevHoodPos = pos;
    }

    private double prevTransferServoPos = -10.501;
    public void setTransferServoPos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevTransferServoPos){
            transferServo.setPosition(pos);
        }
        prevTransferServoPos = pos;
    }

    private double prevSpinPos = -10.501;
    public void setSpinPos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevSpinPos){
            spin1.setPosition(pos);
            spin2.setPosition(pos);
        }
        prevSpinPos = pos;
    }

    private double prevTurretPos = -10.501;
    public void setTurretPos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevTurretPos){
            turr1.setPosition(pos);
            turr2.setPosition(pos);
        }
        prevTurretPos = pos;
    }

    private double prevTilt1Pos = -10.501;
    public void setTilt1Pos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevTilt1Pos){
            tilt1.setPosition(pos);
        }
        prevTilt1Pos = pos;
    }

    private double prevTilt2Pos = -10.501;
    public void setTilt2Pos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevTilt2Pos){
            tilt2.setPosition(pos);
        }
        prevTilt2Pos = pos;
    }

    private double prevSpindexBlockerPos = -10.501;
    public void setSpindexBlockerPos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevSpindexBlockerPos){
            spindexBlocker.setPosition(pos);
        }
        prevSpindexBlockerPos = pos;
    }

    private double prevRapidFireBlockerPos = -10.501;
    public void setRapidFireBlockerPos(double pos){
        pos = (double) Math.round(pos * roundingFactor) / roundingFactor;
        if (pos != prevRapidFireBlockerPos){
            rapidFireBlocker.setPosition(pos);
        }
        prevRapidFireBlockerPos = pos;
    }
}
