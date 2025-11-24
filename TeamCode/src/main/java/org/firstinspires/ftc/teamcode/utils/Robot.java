package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

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

    public Servo turr1;

    public Servo turr2;

    public Servo spin1;

    public Servo spin2;

    public DigitalChannel pin0;

    public DigitalChannel pin1;
    public DigitalChannel pin2;
    public DigitalChannel pin3;
    public DigitalChannel pin4;

    public DigitalChannel pin5;

    public AnalogInput analogInput;

    public AnalogInput analogInput2;

    public AprilTagProcessor aprilTagProcessor;


    public WebcamName webcam;

    public DcMotorEx shooterEncoder;














    public Robot (HardwareMap hardwareMap) {

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

        hood = hardwareMap.get(Servo.class, "hood");

        turr1 = hardwareMap.get(Servo.class, "t1");

        turr2 = hardwareMap.get(Servo.class, "t2");

        spin1 = hardwareMap.get(Servo.class, "spin1");

        spin2 = hardwareMap.get(Servo.class, "spin2");

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

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();


        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");





    }
}
