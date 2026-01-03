package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
public class Servos {
    Robot robot;

    PIDFController spinPID;

    PIDFController turretPID;

    //PID constants
    public static double spinP = 2.85, spinI = 0.015, spinD = 0.09, spinF = 0.03;
    public static double turrP = 4.0, turrI = 0.0, turrD = 0.0, turrF;

    public static double spin_scalar = 1.011;
    public static double spin_restPos = 0.0;
    public static double turret_scalar = 1.009;
    public static double turret_restPos = 0.0;

    public Servos(HardwareMap hardwareMap) {
        robot = new Robot(hardwareMap);
        spinPID = new PIDFController(spinP, spinI, spinD, spinF);
        turretPID = new PIDFController(turrP, turrI, turrD, turrF);
    }

    // In the code below, encoder = robot.servo.getVoltage()

    public double getSpinPos() {
        return spin_scalar * ((robot.spin1Pos.getVoltage() - spin_restPos) / 3.3);
    }

    public double setSpinPos(double pos) {
        spinPID.setPIDF(spinP, spinI, spinD, spinF);

        return spinPID.calculate(this.getSpinPos(), pos);
    }

    public boolean spinEqual(double pos) {
        return Math.abs(pos - this.getSpinPos()) < 0.01;
    }

    public double getTurrPos() {
        return turret_scalar * ((robot.turr1Pos.getVoltage() - turret_restPos) / 3.3);
    }

    public double setTurrPos(double pos) {
        turretPID.setPIDF(turrP, turrI, turrD, turrF);

        return spinPID.calculate(this.getTurrPos(), pos);
    }

    public boolean turretEqual(double pos) {
        return Math.abs(pos - this.getTurrPos()) < 0.01;
    }
}
