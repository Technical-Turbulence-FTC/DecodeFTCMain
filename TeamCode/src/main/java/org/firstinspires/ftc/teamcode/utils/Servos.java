package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Servos {
    //PID constants
    // TODO: get PIDF constants
    public static double spinP = 2.0, spinI = 0, spinD = 0.3, spinF = 0.02;
    public static double turrP = 1.1, turrI = 0.25, turrD = 0.0625, turrF = 0;
    public static double spin_scalar = 1.112;
    public static double spin_restPos = 0.155;
    public static double turret_scalar = 1.009;
    public static double turret_restPos = 0.0;
    Robot robot;
    PIDFController spinPID;
    PIDFController turretPID;

    private double prevSpinPos = 0.0;
    private boolean firstSpinPos = true;

    private double prevTransferPos = 0.0;
    private boolean firstTransferPos = true;

    private double prevHoodPos = 0.0;
    private boolean firstHoodPos = true;

    public Servos(HardwareMap hardwareMap) {
        robot = new Robot(hardwareMap);
        spinPID = new PIDFController(spinP, spinI, spinD, spinF);
        turretPID = new PIDFController(turrP, turrI, turrD, turrF);

        turretPID.setTolerance(0.001);
    }

    // In the code below, encoder = robot.servo.getVoltage()
    // TODO: set the restPos and scalar
    public double getSpinPos() {
        return spin_scalar * ((robot.spin1Pos.getVoltage() - spin_restPos) / 3.3);
    }

    public double getSpinCmdPos() {
        return prevSpinPos;
    }

    public static boolean servoPosEqual(double pos1, double pos2) {
        return (Math.abs(pos1 - pos2) < 0.005);
    }

    public double setTransferPos(double pos) {
        if (firstTransferPos || !servoPosEqual(pos, prevTransferPos)) {
            robot.transferServo.setPosition(pos);
            firstTransferPos = false;
        }

        prevTransferPos = pos;
        return pos;
    }

    public double setSpinPos(double pos) {
        if (firstSpinPos || !servoPosEqual(pos, prevSpinPos)) {
            robot.spin1.setPosition(pos);
            robot.spin2.setPosition(1-pos);
            firstSpinPos = false;
        }

        prevSpinPos = pos;
        return pos;
    }

    public double setHoodPos(double pos){
        if (firstHoodPos || !servoPosEqual(pos, prevHoodPos)) {
            robot.hood.setPosition(pos);
            firstHoodPos = false;
        }

        prevHoodPos = pos;
        return pos;
    }

    public boolean spinEqual(double pos) {
        return Math.abs(pos - this.getSpinPos()) < 0.03;
    }

    public double getTurrPos() {
        return 1.0;
    }

    public double setTurrPos(double pos) {
        return 1.0;
    }

    public boolean turretEqual(double pos) {
        return true;
    }
}
