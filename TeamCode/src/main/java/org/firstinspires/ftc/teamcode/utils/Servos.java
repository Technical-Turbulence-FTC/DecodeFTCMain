package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Servos {
    Robot robot;

    PIDFController spinPID;

    PIDFController turretPID;

    //PID constants
    // TODO: get PIDF constants
    public static double spinP = 3.3, spinI = 0, spinD = 0.1, spinF = 0.02;
    public static double turrP = 4.0, turrI = 0.0, turrD = 0.0, turrF = 0.0;

    public static double spin_scalar = 1.0086;
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
    //TODO: PID warp so 0 and 1 are usable positions
    public double setSpinPos(double pos) {
        spinPID.setPIDF(spinP, spinI, spinD, spinF);

        return spinPID.calculate(this.getSpinPos(), pos);
    }

    public boolean spinEqual(double pos) {
        return Math.abs(pos - this.getSpinPos()) < 0.02;
    }

    public double getTurrPos() {
        return robot.turr1Pos.getCurrentPosition();
    }

    public double setTurrPos(double pos) {
        turretPID.setPIDF(turrP, turrI, turrD, turrF);

        return spinPID.calculate(this.getTurrPos(), pos);
    }

    public boolean turretEqual(double pos) {
        return Math.abs(pos - this.getTurrPos()) < 0.01;
    }
}
