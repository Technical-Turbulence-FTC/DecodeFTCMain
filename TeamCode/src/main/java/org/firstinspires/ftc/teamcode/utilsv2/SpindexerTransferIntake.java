package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Servo;


public class SpindexerTransferIntake {

    private Servo spin1;
    private Servo spin2;

    public SpindexerTransferIntake(Robot rob, MultipleTelemetry TELE) {
        this.spin1 = rob.spin1;
        this.spin2 = rob.spin2;

    }

    enum SpindexerMode {
        RAPID,
        SORTED
    }

    enum RapidMode {
        INTAKE,            // Normal intake operation
        TRANSFER_OFF,      // Slow transfer, waiting for rings to settle
        BEFORE_PULSE_OUT,  // Brief forward delay before pulse
        PULSE_OUT,         // Small reverse pulse to unjam/reposition rings
        PULSE_IN,          // Feed rings back forward
        HOLD_BALLS,        // Maintain ring position
        OPEN_GATE,         // Open shooter gate
        SHOOT              // Feed ring into shooter
    }

    private SpindexerMode mode = SpindexerMode.RAPID;

    private RapidMode rapidMode = RapidMode.INTAKE;

    public void setRapidMode (RapidMode rMode) {
        this.rapidMode = rMode;
    }

    public void setSpindexerMode (SpindexerMode spindexerMode){
        this.mode = spindexerMode;
    }

    public void update() {

        switch (mode) {

            case RAPID:

                switch (rapidMode) {

                    case INTAKE:

                        // Run front intake
                        // Run transfer intake
                        // Keep shooter gate closed
                        // Keep kicker retracted
                        // Keep spindexer in default position

                        // If first beam break sees ring:
                        // rapidMode = TRANSFER_OFF;

                        // If shoot button pressed:
                        // rapidMode = OPEN_GATE;

                        break;

                    case TRANSFER_OFF:

                        // Slow down transfer intake
                        // Allow rings to settle into storage

                        // If both beam breaks occupied:
                        // rapidMode = BEFORE_PULSE_OUT;

                        // If shoot button pressed:
                        // rapidMode = OPEN_GATE;

                        break;

                    case BEFORE_PULSE_OUT:

                        // Continue running intake forward

                        // After ~0.3 sec:
                        // rapidMode = PULSE_OUT;

                        break;

                    case PULSE_OUT:

                        // Reverse intake slightly
                        // Helps separate rings and prevent jams

                        // After pulse time:
                        // rapidMode = PULSE_IN;

                        break;

                    case PULSE_IN:

                        // Run intake forward again
                        // Re-seat rings after pulse

                        // After ~0.2 sec:
                        // rapidMode = HOLD_BALLS;

                        break;

                    case HOLD_BALLS:

                        // If both sensors occupied:
                        // hold intake at low power

                        // Else:
                        // run intake forward to pull rings back in

                        // If shoot button pressed:
                        // rapidMode = OPEN_GATE;

                        // If resume intake button pressed:
                        // rapidMode = INTAKE;

                        break;

                    case OPEN_GATE:

                        // Open upper gate
                        // Keep intake feeding

                        // After ~0.1 sec:
                        // rapidMode = SHOOT;

                        break;

                    case SHOOT:

                        // Activate kicker
                        // Feed ring into shooter

                        // After ~0.4 sec:
                        // rapidMode = INTAKE;

                        break;
                }

                break;

            case SORTED:

                // Future sorted-intake logic
                // Color sorting
                // Alliance filtering
                // Different spindexer behavior

                break;
        }
    }
}
