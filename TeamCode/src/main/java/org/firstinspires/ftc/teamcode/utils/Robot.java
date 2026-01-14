package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Robot {

    //Initialize Public Components

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public Servo hood;
    public Servo transferServo;
    public CRServo turr1;
    public CRServo turr2;
    public CRServo spin1;
    public CRServo spin2;
    public AnalogInput spin1Pos;
    public AnalogInput spin2Pos;
    public DcMotorEx turr1Pos;
    public AnalogInput transferServoPos;
    public AprilTagProcessor aprilTagProcessor;
    public WebcamName webcam;
    public RevColorSensorV3 color1;
    public RevColorSensorV3 color2;
    public RevColorSensorV3 color3;
    public Limelight3A limelight;

    public static boolean usingLimelight = true;

    public static boolean usingCamera = true;

    public Robot(HardwareMap hardwareMap) {

        //Define components w/ hardware map
        //TODO: fix the configuration of these - I trust you to figure it out yourself @KeshavAnandCode
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
        //TODO: figure out which shooter motor is reversed using ShooterTest and change it in code @KeshavAnandCode
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hardwareMap.get(Servo.class, "hood");

        turr1 = hardwareMap.get(CRServo.class, "t1");

        turr2 = hardwareMap.get(CRServo.class, "t2");

        turr1Pos = intake; // Encoder of turret plugged in intake port

        //TODO: check spindexer configuration (both servo and analog input) - check comments in PositionalServoProgrammer
        spin1 = hardwareMap.get(CRServo.class, "spin1");

        spin1Pos = hardwareMap.get(AnalogInput.class, "spin1Pos");

        spin2 = hardwareMap.get(CRServo.class, "spin2");

        spin2Pos = hardwareMap.get(AnalogInput.class, "spin2Pos");

        spin1.setDirection(DcMotorSimple.Direction.REVERSE);
        spin2.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        transferServo = hardwareMap.get(Servo.class, "transferServo");

        transferServoPos = hardwareMap.get(AnalogInput.class, "tSPos");

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        color1 = hardwareMap.get(RevColorSensorV3.class, "c1");

        color2 = hardwareMap.get(RevColorSensorV3.class, "c2");

        color3 = hardwareMap.get(RevColorSensorV3.class, "c3");

        if (usingLimelight){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } else if (usingCamera){
            webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        }
    }
}
