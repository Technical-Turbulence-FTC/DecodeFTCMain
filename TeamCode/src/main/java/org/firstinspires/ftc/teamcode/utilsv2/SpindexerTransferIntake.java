package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.ServoPositions;

public class SpindexerTransferIntake {

    private final Robot robot;

    VelocityCommander commander;

    public SpindexerTransferIntake(Robot rob, MultipleTelemetry TELE, VelocityCommander com) {
        this.robot = rob;
        this.commander = com;
    }

    public enum DesiredPattern {
        PPG,
        PGP,
        GPP
    }

    public enum SortedShootState {
        IDLE,

        MOVE_TO_1,
        WAIT_FOR_1,
        SHOOT_1,
        RETRACT_1,

        MOVE_TO_2,
        WAIT_FOR_2,
        SHOOT_2,
        RETRACT_2,

        MOVE_TO_3,
        WAIT_FOR_3,
        SHOOT_3,
        RETRACT_3,

        DONE
    }

    int[] shootOrder = {0,1,2};
    private final double sensorDistanceThreshold = 6.0;
    private final long pulseTime = 100; // ms

    private DesiredPattern desiredPattern = DesiredPattern.GPP;

    private SortedShootState shootState = SortedShootState.IDLE;
    private long shootStateStartTime = System.currentTimeMillis();

    private void setShootState(SortedShootState newState){
        shootState = newState;
        shootStateStartTime = System.currentTimeMillis();
    }

    private long shootStateTime(){
        return System.currentTimeMillis() - shootStateStartTime;
    }

    private int[] buildShootOrder(
            BallStates[] loaded,
            DesiredPattern desired) {

        BallStates[] target;

        switch (desired) {
            case PPG:
                target = new BallStates[]{
                        BallStates.PURPLE,
                        BallStates.PURPLE,
                        BallStates.GREEN
                };
                break;

            case PGP:
                target = new BallStates[]{
                        BallStates.PURPLE,
                        BallStates.GREEN,
                        BallStates.PURPLE
                };
                break;

            default: // GPP
                target = new BallStates[]{
                        BallStates.GREEN,
                        BallStates.PURPLE,
                        BallStates.PURPLE
                };
        }

        int[] order = new int[3];
        boolean[] used = new boolean[3];

        // first pass: exact color matches
        for (int i = 0; i < 3; i++) {

            order[i] = -1;

            for (int slot = 0; slot < 3; slot++) {

                if (!used[slot]
                        && loaded[slot] == target[i]) {

                    order[i] = slot;
                    used[slot] = true;
                    break;
                }
            }
        }

        // second pass: fill leftovers
        for (int i = 0; i < 3; i++) {

            if (order[i] != -1)
                continue;

            for (int slot = 0; slot < 3; slot++) {

                if (!used[slot]) {
                    order[i] = slot;
                    used[slot] = true;
                    break;
                }
            }
        }

        return order;
    }

    private void moveToSlot(int slot) {

        switch(slot) {

            case 0:
                robot.setSpinPos(
                        ServoPositions.spindexer_A1
                );
                break;

            case 1:
                robot.setSpinPos(
                        ServoPositions.spindexer_A2
                );
                break;

            case 2:
                robot.setSpinPos(
                        ServoPositions.spindexer_A3
                );
                break;
        }
    }

    public enum SortedIntakeStates {
        NOTHING,
        IDLE,
        INTAKE_1,
        DELAY_1,
        INTAKE_2,
        DELAY_2,
        INTAKE_3,
        REVERSE,

    }

    public enum SpindexerMode {
        RAPID,
        SORTED,
        SHOOT_SORTED
    }

    public enum BallStates {
        GREEN,
        PURPLE,
        UNKNOWN
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
    private SortedIntakeStates sortedIntakeStates = SortedIntakeStates.IDLE;
    private BallStates[] ballColors = {BallStates.UNKNOWN, BallStates.UNKNOWN, BallStates.UNKNOWN};
    private final double greenThresh = 0.40;
    private final double spinMovementTime = 250;

    /**
     * Time when current state was entered.
     */
    private long stateStartTime = System.currentTimeMillis();
    private long sortedStateStartTime = System.currentTimeMillis();

    public void setRapidMode(RapidMode newMode) {
        if (rapidMode != newMode) {
            rapidMode = newMode;
            stateStartTime = System.currentTimeMillis();
        }
    }

    public void setSortedIntakeMode(SortedIntakeStates newMode) {
        if (sortedIntakeStates != newMode) {
            sortedIntakeStates = newMode;
            sortedStateStartTime = System.currentTimeMillis();
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

    private long sortedStateTime() {
        return System.currentTimeMillis() - sortedStateStartTime;
    }

    public void update() {

        if (mode == SpindexerMode.RAPID && rapidMode == RapidMode.INTAKE){
            robot.setTransferPower(-0.7);
        }

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

                        break;

                    case SHOOT:

                        robot.setTransferServoPos(
                                ServoPositions.transferServo_in
                        );
                        if (stateTime() >= 400) {
                            setRapidMode(RapidMode.INTAKE);
                        }
                        break;
                }
                break;

            case SORTED:

                switch (sortedIntakeStates){
                    case NOTHING:
                        break;
                    case IDLE:
                        robot.setRapidFireBlockerPos(
                                ServoPositions.rapidFireBlocker_Open
                        );
                        robot.setSpindexBlockerPos(
                                ServoPositions.spindexBlocker_Closed
                        );
                        robot.setSpinPos(
                                ServoPositions.spindexer_A1
                        );
                        robot.setTransferServoPos(
                                ServoPositions.transferServo_out
                        );
                        robot.setIntakePower(1);
                        robot.setTransferPower(-1);
                        if (sortedStateTime() > 200) {
                            setSortedIntakeMode(SortedIntakeStates.INTAKE_1);
                        }
                        break;
                    case INTAKE_1:
                        robot.setIntakePower(1);
                        robot.setTransferPower(-1);
                        if (robot.insideBeam.isPressed() && robot.revSensor.getDistance(DistanceUnit.CM) < sensorDistanceThreshold){
                            NormalizedRGBA revColor = robot.revSensor.getNormalizedColors();
                            if ((revColor.green / (revColor.red + revColor.blue + revColor.green)) > greenThresh) {
                                ballColors[0] = BallStates.GREEN;
                            } else {
                                ballColors[0] = BallStates.PURPLE;
                            }
                            robot.setSpinPos(ServoPositions.spindexer_A2);
                            setSortedIntakeMode(SortedIntakeStates.DELAY_1);
                        }
                        break;
                    case DELAY_1:
                        robot.setSpinPos(ServoPositions.spindexer_A2);
                        if (sortedStateTime() > spinMovementTime){
                            setSortedIntakeMode(SortedIntakeStates.INTAKE_2);
                        }
                        break;
                    case INTAKE_2:
                        robot.setIntakePower(1);
                        robot.setTransferPower(-1);
                        if (robot.insideBeam.isPressed() && robot.revSensor.getDistance(DistanceUnit.CM) < sensorDistanceThreshold){
                            NormalizedRGBA revColor = robot.revSensor.getNormalizedColors();
                            if ((revColor.green / (revColor.red + revColor.blue + revColor.green)) > greenThresh) {
                                ballColors[1] = BallStates.GREEN;
                            } else {
                                ballColors[1] = BallStates.PURPLE;
                            }
                            robot.setSpinPos(ServoPositions.spindexer_A3);
                            setSortedIntakeMode(SortedIntakeStates.DELAY_2);
                        }
                        break;
                    case DELAY_2:

                        robot.setSpinPos(
                                ServoPositions.spindexer_A3
                        );

                        if (sortedStateTime() > spinMovementTime){
                            setSortedIntakeMode(
                                    SortedIntakeStates.INTAKE_3
                            );
                        }

                        break;
                    case INTAKE_3:
                        robot.setIntakePower(1);
                        robot.setTransferPower(-1);
                        if (robot.insideBeam.isPressed() && robot.revSensor.getDistance(DistanceUnit.CM) < sensorDistanceThreshold){
                            NormalizedRGBA revColor = robot.revSensor.getNormalizedColors();
                            if ((revColor.green / (revColor.red + revColor.blue + revColor.green)) > greenThresh) {
                                ballColors[2] = BallStates.GREEN;
                            } else {
                                ballColors[2] = BallStates.PURPLE;
                            }
                            setSortedIntakeMode(SortedIntakeStates.REVERSE);

                        }
                        break;
                    case REVERSE:
                        robot.setTransferPower(-0.3);
                        robot.setIntakePower(-0.1);
                }


                break;
            case SHOOT_SORTED:

                //TODO: ADD transfer intake powers here

                switch (shootState){
                    case IDLE:
                        shootOrder = buildShootOrder(
                                ballColors,
                                desiredPattern
                        );

                        setShootState(SortedShootState.MOVE_TO_1);
                        mode = SpindexerMode.SHOOT_SORTED;
                    case MOVE_TO_1:

                        moveToSlot(shootOrder[0]);
                        robot.setRapidFireBlockerPos(
                                ServoPositions.rapidFireBlocker_Open
                        );
                        robot.setSpindexBlockerPos(
                                ServoPositions.spindexBlocker_Closed
                        );


                        setShootState(
                                SortedShootState.WAIT_FOR_1
                        );

                        break;
                    case WAIT_FOR_1:

                        if(shootStateTime() > 250){

                            setShootState(
                                    SortedShootState.SHOOT_1
                            );
                        }

                        break;

                    case SHOOT_1:

                        robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Open);
                        robot.setTransferServoPos(ServoPositions.transferServo_in);


                        if(shootStateTime() > 300){

                            setShootState(
                                    SortedShootState.RETRACT_1
                            );
                        }

                        break;
                    case RETRACT_1:

                        robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Closed);
                        robot.setTransferServoPos(ServoPositions.transferServo_out);

                        if(shootStateTime() > 150){

                            setShootState(
                                    SortedShootState.MOVE_TO_2
                            );
                        }

                        break;
                    case MOVE_TO_2:

                        moveToSlot(shootOrder[1]);

                        setShootState(
                                SortedShootState.WAIT_FOR_2
                        );

                        break;
                    case WAIT_FOR_2:

                        if(shootStateTime() > 250){

                            setShootState(
                                    SortedShootState.SHOOT_2
                            );
                        }

                        break;
                    case SHOOT_2:

                        robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Open);
                        robot.setTransferServoPos(ServoPositions.transferServo_in);

                        if(shootStateTime() > 300){

                            setShootState(
                                    SortedShootState.RETRACT_2
                            );
                        }

                        break;
                    case RETRACT_2:

                        robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Closed);
                        robot.setTransferServoPos(ServoPositions.transferServo_out);



                        if(shootStateTime() > 150){

                            setShootState(
                                    SortedShootState.MOVE_TO_3
                            );
                        }

                        break;
                    case MOVE_TO_3:

                        moveToSlot(shootOrder[2]);

                        setShootState(
                                SortedShootState.WAIT_FOR_3
                        );

                        break;
                    case WAIT_FOR_3:

                        if(shootStateTime() > 250){

                            setShootState(
                                    SortedShootState.SHOOT_3
                            );
                        }

                        break;
                    case SHOOT_3:

                        robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Open);
                        robot.setTransferServoPos(ServoPositions.transferServo_in);

                        if(shootStateTime() > 300){

                            setShootState(
                                    SortedShootState.RETRACT_3
                            );
                        }

                        break;
                    case RETRACT_3:

                        robot.setTransferServoPos(ServoPositions.transferServo_out);

                        if(shootStateTime() > 150){

                            setShootState(
                                    SortedShootState.DONE
                            );
                        }

                        break;
                    case DONE:

                        robot.setRapidFireBlockerPos(
                                ServoPositions.rapidFireBlocker_Open
                        );
                        robot.setSpindexBlockerPos(
                                ServoPositions.spindexBlocker_Closed
                        );
                        robot.setSpinPos(
                                ServoPositions.spindexer_A1
                        );
                        robot.setTransferServoPos(
                                ServoPositions.transferServo_out
                        );
                        robot.setIntakePower(1);
                        robot.setTransferPower(-1);

                        setSortedIntakeMode(SortedIntakeStates.NOTHING);

                        mode = SpindexerMode.SORTED;

                        break;


                }



                break;
        }
    }
}