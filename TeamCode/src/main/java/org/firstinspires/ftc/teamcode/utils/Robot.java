package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public Servo rejecter;
    public CRServo turr1;
    public CRServo turr2;
    public CRServo spin1;
    public CRServo spin2;
    public DigitalChannel pin0;
    public DigitalChannel pin1;
    public DigitalChannel pin2;
    public DigitalChannel pin3;
    public DigitalChannel pin4;
    public DigitalChannel pin5;
    public AnalogInput analogInput;
    public AnalogInput analogInput2;
    public AnalogInput spin1Pos;
    public AnalogInput spin2Pos;
    public AnalogInput hoodPos;
    public AnalogInput turr1Pos;
    public AnalogInput turr2Pos;
    public AnalogInput transferServoPos;
    public AprilTagProcessor aprilTagProcessor;
    public WebcamName webcam;
    public DcMotorEx shooterEncoder;
    public RevColorSensorV3 color1;
    public RevColorSensorV3 color2;
    public RevColorSensorV3 color3;
    public Limelight3A limelight;

    public static boolean usingLimelight = true;

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
        rejecter = hardwareMap.get(Servo.class, "rejecter");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterEncoder = shooter1;

        hood = hardwareMap.get(Servo.class, "hood");

        hoodPos = hardwareMap.get(AnalogInput.class, "hoodPos");

        turr1 = hardwareMap.get(CRServo.class, "t1");

        turr1Pos = hardwareMap.get(AnalogInput.class, "t1Pos");

        turr2 = hardwareMap.get(CRServo.class, "t2");

        turr2Pos = hardwareMap.get(AnalogInput.class, "t2Pos");

        spin1 = hardwareMap.get(CRServo.class, "spin1");

        spin1Pos = hardwareMap.get(AnalogInput.class, "spin1Pos");

        spin2 = hardwareMap.get(CRServo.class, "spin2");

        spin2Pos = hardwareMap.get(AnalogInput.class, "spin2Pos");

        spin1.setDirection(DcMotorSimple.Direction.REVERSE);
        spin2.setDirection(DcMotorSimple.Direction.REVERSE);

        pin0 = hardwareMap.get(DigitalChannel.class, "pin0");

        pin1 = hardwareMap.get(DigitalChannel.class, "pin1");

        pin2 = hardwareMap.get(DigitalChannel.class, "pin2");

        pin3 = hardwareMap.get(DigitalChannel.class, "pin3");

        pin4 = hardwareMap.get(DigitalChannel.class, "pin4");

        pin5 = hardwareMap.get(DigitalChannel.class, "pin5");

        analogInput = hardwareMap.get(AnalogInput.class, "analog");

        analogInput2 = hardwareMap.get(AnalogInput.class, "analog2");

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        transferServo = hardwareMap.get(Servo.class, "transferServo");

        transferServoPos = hardwareMap.get(AnalogInput.class, "tSPos");

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        color1 = hardwareMap.get(RevColorSensorV3.class, "c1");

        color2 = hardwareMap.get(RevColorSensorV3.class, "c2");

        color3 = hardwareMap.get(RevColorSensorV3.class, "c3");

        if (usingLimelight){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        }
    }
}
