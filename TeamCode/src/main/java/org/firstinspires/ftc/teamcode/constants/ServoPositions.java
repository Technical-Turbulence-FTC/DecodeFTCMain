package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ServoPositions {

    public static double spindexer_intakePos1 = 0;

    public static double spindexer_intakePos2 = 0.19;//0.5;

    public static double spindexer_intakePos3 = 0.38;//0.66;

    public static double spindexer_outtakeBall3 = 0.65;

    public static double spindexer_outtakeBall2 = 0.46;
    public static double spindexer_outtakeBall1 = 0.27;
    public static double spinStartPos = spindexer_outtakeBall1 - 0.08;


    public static double transferServo_out = 0.15;

    public static double transferServo_in = 0.38;

    public static double turret_range = 0.9;

    public static double hoodDefault = 0.6;

    public static double hoodAuto = 0.27;

    public static double hoodAutoFar = 0.5; //TODO: change this;

    public static double hoodHigh = 0.21; //TODO: change this;

    public static double hoodLow = 1.0; //TODO: change this;

    public static double turret_redClose = 0.42;
    public static double turret_blueClose = 0.38;
    public static double turret_redFar = 0.5; //TODO: change this
    public static double turret_blueFar = 0.5; // TODO: change this

    public static double turret_detectRedClose = 0.2;

    public static double turret_detectBlueClose = 0.6;
    public static double turrDefault = 0.4;

    public static double turrMin = 0.2;
    public static double turrMax = 0.8;


}
