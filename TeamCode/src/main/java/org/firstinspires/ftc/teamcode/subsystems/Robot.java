package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Robot {

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public WebcamName webcamName;

    public Robot(HardwareMap hardwareMap) {

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

}
