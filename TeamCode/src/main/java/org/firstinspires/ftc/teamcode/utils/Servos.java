package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

public class Servos {
    PIDController spinPID;

    PIDController turretPID;

    //PID constants
    public static double spinP = 0.0, spinI = 0.0, spinD = 0.0;
    public static double turrP = 0.0, turrI = 0.0, turrD = 0.0;

    public static double spin_scalar = 0.15;
    public static double spin_restPos = 1.112;
    public static double turret_scalar = 0.15;
    public static double turret_restPos = 1.112;

    public void initServos() {
        spinPID = new PIDController(spinP, spinI, spinD);
        turretPID = new PIDController(turrP, turrI, turrD);
    }

    // In the code below, encoder = robot.servo.getVoltage()

    public double getSpinPos(double encoder){
        return spin_scalar * ((encoder - spin_restPos) / 3.3);
    }
    public double setSpinPos(double pos, double encoder){
        spinPID.setPID(spinP, spinI, spinD);

        return spinPID.calculate(this.getSpinPos(encoder), pos);
    }
    public boolean spinEqual(double pos, double encoder){
        return Math.abs(pos - this.getSpinPos(encoder)) < 0.01;
    }

    public double getTurrPos(double encoder){
        return turret_scalar * ((encoder - turret_restPos) / 3.3);
    }
    public double setTurrPos(double pos, double encoder){
        turretPID.setPID(turrP, turrI, turrD);

        return spinPID.calculate(this.getTurrPos(encoder), pos);
    }
    public boolean turretEqual(double pos, double encoder){
        return Math.abs(pos - this.getTurrPos(encoder)) < 0.01;
    }
}
