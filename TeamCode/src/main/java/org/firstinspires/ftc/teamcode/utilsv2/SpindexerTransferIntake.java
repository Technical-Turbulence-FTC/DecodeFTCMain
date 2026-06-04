package org.firstinspires.ftc.teamcode.utilsv2;

import android.health.connect.datatypes.units.Velocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.ServoPositions;

@Config
public class SpindexerTransferIntake {

    private final Robot robot;

    VelocityCommander commander;

    public SpindexerTransferIntake(Robot rob, MultipleTelemetry TELE, VelocityCommander com) {
        this.robot = rob;
        this.commander = com;
    }

    private final double sensorDistanceThreshold = 6.0;
    private final long pulseTime = 100; // ms

    public enum SpindexerMode {
        RAPID,
        SORTED
    }

    public enum RapidMode {
        INTAKE,
        TRANSFER_OFF,
        BEFORE_PULSE_OUT,
        PULSE_OUT,
        PULSE_IN,
        HOLD_BALLS,
        OPEN_GATE,
        SHOOT
    }

    private SpindexerMode mode = SpindexerMode.RAPID;
    private RapidMode rapidMode = RapidMode.INTAKE;

    /**
     * Time when current state was entered.
     */
    private long stateStartTime = System.currentTimeMillis();

    public void setRapidMode(RapidMode newMode) {
        if (rapidMode != newMode) {
            rapidMode = newMode;
            stateStartTime = System.currentTimeMillis();
        }
    }

    public void setSpindexerMode(SpindexerMode spindexerMode) {
        this.mode = spindexerMode;
    }

    public RapidMode getRapidState(){
        return this.rapidMode;
    }

    private long stateTime() {
        return System.currentTimeMillis() - stateStartTime;
    }

    public void update() {

        switch (mode) {

            case RAPID:

                robot.setSpindexBlockerPos(
                        ServoPositions.spindexBlocker_Open
                );

                switch (rapidMode) {

                    case INTAKE:

                        robot.setIntakePower(1);
                        robot.setRapidFireBlockerPos(
                                ServoPositions.rapidFireBlocker_Closed
                        );
                        robot.setSpinPos(
                                ServoPositions.spindexer_A2
                        );
                        robot.setTransferPower(-0.7);
                        robot.setTransferServoPos(
                                ServoPositions.transferServo_out
                        );

                        if (robot.insideBeam.isPressed() && robot.revSensor.getDistance(DistanceUnit.CM) < sensorDistanceThreshold) {

                            setRapidMode(RapidMode.TRANSFER_OFF);
                        }


                        break;

                    case TRANSFER_OFF:

                        if (robot.insideBeam.isPressed() && robot.outsideBeam.isPressed()) {
                            setRapidMode(RapidMode.BEFORE_PULSE_OUT);
                        }
                        robot.setTransferPower(-0.3);

                        break;

                    case BEFORE_PULSE_OUT:

                        robot.setIntakePower(1.0);

                        if (stateTime() >= 300) {
                            setRapidMode(RapidMode.PULSE_OUT);
                        }

                        break;

                    case PULSE_OUT:

                        robot.setIntakePower(-0.1);

                        if (stateTime() >= pulseTime) {
                            setRapidMode(RapidMode.PULSE_IN);
                        }

                        break;

                    case PULSE_IN:

                        robot.setIntakePower(1.0);

                        if (stateTime() >= 200) {
                            setRapidMode(RapidMode.HOLD_BALLS);
                        }


                        break;

                    case HOLD_BALLS:

                        if (robot.insideBeam.isPressed()
                                && robot.outsideBeam.isPressed()) {

                            robot.setIntakePower(0.1);

                        } else {

                            robot.setIntakePower(1);
                        }
                        break;

                    case OPEN_GATE:

                        robot.setRapidFireBlockerPos(
                                ServoPositions.rapidFireBlocker_Open
                        );

                        if (stateTime() >= 100) {
                            setRapidMode(RapidMode.SHOOT);
                        }

                        robot.setTransferPower(commander.getTransferPow());

                        break;

                    case SHOOT:

                        robot.setTransferServoPos(
                                ServoPositions.transferServo_in
                        );
                        if (stateTime() >= 400) {
                            setRapidMode(RapidMode.INTAKE);
                        }

                        robot.setTransferPower(commander.getTransferPow());

                        break;
                }
                break;

            case SORTED:

                // Future sorted-intake logic
                break;
        }
    }
}