package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Color;
import org.firstinspires.ftc.teamcode.constants.StateEnums.LightState;


@Config
public final class Light {

    private static Light instance;
    public static  double ballColorCycleTime = 1000; //in ms
    public static double restingTime = 125; //in ms

    private Servo lightServo;
    private LightState state = LightState.DISABLED;

    // References to other systems (NOT static)
    private Spindexer spindexer;
    private Turret turret;
    private double manualLightColor = Color.Light0;

    private double lightColor = Color.Light0;
    private double previousLightColor = lightColor;

    private Light() {
    }

    public static synchronized Light getInstance() {
        if (instance == null) {
            instance = new Light();
        }
        return instance;
    }

    // Call once in OpMode init()
    public void init(
            Servo servo,
            Spindexer spin,
            Turret turr
    ) {
        this.lightServo = servo;
        this.spindexer = spin;
        this.turret = turr;
    }

    public void setManualLightColor(double value) {
        this.manualLightColor = value;
    }

    public void setState(LightState newState) {
        state = newState;
    }

    public void update() {
        if (lightServo == null) return;

        switch (state) {

            case BALL_COUNT:
                lightColor = spindexer.ballCounterLight();
                break;

            case BALL_COLOR:
                double currentTime = System.currentTimeMillis();
                if ((currentTime % ballColorCycleTime) < ((ballColorCycleTime / 3) - restingTime)) {
                    lightColor = spindexer.getRearCenterLight();
                } else if ((currentTime % ballColorCycleTime) < (ballColorCycleTime / 3)) {
                    lightColor = 0;
                } else if ((currentTime % ballColorCycleTime) < ((2 * ballColorCycleTime / 3) - restingTime)) {
                    lightColor = spindexer.getDriverLight();

                } else if ((currentTime % ballColorCycleTime) < (2 * ballColorCycleTime / 3)) {
                    lightColor = 0;
                } else if ((currentTime % ballColorCycleTime) < (ballColorCycleTime - restingTime)) {
                    lightColor = spindexer.getPassengerLight();

                } else {
                    lightColor = 0;
                }
                break;

            case GOAL_LOCK:
                lightColor = turret.getLightColor();
                break;

            case MANUAL:
                lightColor = manualLightColor;
                break;

            case DISABLED:
                break;

            case OFF:
                lightColor = 0;
                break;
        }

        if (lightColor != previousLightColor) {
            lightServo.setPosition(lightColor);
        }

        previousLightColor = lightColor;

    }
}