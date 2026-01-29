package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ServoPositions {

    public static double spindexer_intakePos1 = 0.05; //0.13;

    public static double spindexer_intakePos2 = 0.24; //0.33;//0.5;

    public static double spindexer_intakePos3 = 0.43; //0.53;//0.66;

    public static double spindexer_outtakeBall3 = 0.71; //0.65; //0.24;

    public static double spindexer_outtakeBall2 = 0.53; //0.46; //0.6;
    public static double spindexer_outtakeBall1 = 0.35; //0.27; //0.4;
    public static double spinStartPos = spindexer_outtakeBall3 - 0.1;

    public static double shootAllAutoSpinStartPos = 0.2;
    public static double shootAllSpindexerSpeedIncrease = 0.02;
    public static double shootAllTime = 1.8;

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
