package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.constants.Color.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinD;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinF;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinI;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinP;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.Types;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;

public class Spindexer {

    Robot robot;
    Servos servos;
    Flywheel flywheel;
    MecanumDrive drive;
    double lastKnownSpinPos = 0.0;
    MultipleTelemetry TELE;

    PIDFController spinPID = new PIDFController(spinP, spinI, spinD, spinF);

    double spinCurrentPos = 0.0;

    public int commandedIntakePosition = 0;

    public double distanceRearCenter = 0.0;
    public double distanceFrontDriver = 0.0;
    public double distanceFrontPassenger = 0.0;

    private double prevPos = 0.0;

    public Types.Motif desiredMotif = Types.Motif.NONE;
    // For Use
    enum RotatedBallPositionNames {
        REARCENTER,
        FRONTDRIVER,
        FRONTPASSENGER
    }
    // Array of commandedIntakePositions with contents
    // {RearCenter, FrontDriver, FrontPassenger}
    static final int[][] RotatedBallPositions = {{0,2,1}, {1,0,2}, {2,1,0}};
    class spindexerBallRoatation {
        int rearCenter = 0; // aka commanded Position
        int frontDriver = 0;
        int frontPassenger = 0;
    }

    enum IntakeState {
        UNKNOWN_START,
        UNKNOWN_MOVE,
        UNKNOWN_DETECT,
        INTAKE,
        FINDNEXT,
        MOVING,
        FULL,
        SHOOTNEXT,
        SHOOTMOVING,
        SHOOTWAIT,
        SHOOT_ALL_PREP,
        SHOOT_ALL_READY
    }

    public IntakeState currentIntakeState = IntakeState.UNKNOWN_START;
    public int unknownColorDetect = 0;
    enum BallColor {
        UNKNOWN,
        GREEN,
        PURPLE
    }

    class BallPosition {
        boolean isEmpty = true;
        int foundEmpty = 0;
        BallColor ballColor = BallColor.UNKNOWN;
    }

    BallPosition[] ballPositions = new BallPosition[3];

    public boolean init () {
        return true;
    }

    public Spindexer(HardwareMap hardwareMap) {
        robot = new Robot(hardwareMap);
        servos = new Servos(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        //TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lastKnownSpinPos = servos.getSpinPos();

        ballPositions[0] = new BallPosition();
        ballPositions[1] = new BallPosition();
        ballPositions[2] = new BallPosition();


    }


    double[] outakePositions =
            {spindexer_outtakeBall1, spindexer_outtakeBall2, spindexer_outtakeBall3};

    double[] intakePositions =
            {spindexer_intakePos1, spindexer_intakePos2, spindexer_intakePos3};

    public int counter = 0;

//    private double getTimeSeconds ()
//    {
//        return (double) System.currentTimeMillis()/1000.0;
//    }

//    public double getPos() {
//        robot.spin1Pos.getVoltage();
//        robot.spin1Pos.getMaxVoltage();
//        return (robot.spin1Pos.getVoltage()/robot.spin1Pos.getMaxVoltage());
//    }

//    public void manageSpindexer() {
//
//    }

    public void resetBallPosition (int pos) {
        ballPositions[pos].isEmpty = true;
        ballPositions[pos].foundEmpty = 0;
        ballPositions[pos].ballColor = BallColor.UNKNOWN;
    }

    public void resetSpindexer () {
        for (int i = 0; i < 3; i++) {
            resetBallPosition(i);
        }
        currentIntakeState = IntakeState.UNKNOWN_START;
    }

    // Detects if a ball is found and what color.
    // Returns true is there was a new ball found in Position 1
    // FIXIT: Reduce number of times that we read the color sensors for loop times.
    public boolean detectBalls(boolean detectRearColor, boolean detectFrontColor) {

        boolean newPos1Detection = false;
        int spindexerBallPos = 0;

        // Read Distances
        distanceRearCenter = robot.color1.getDistance(DistanceUnit.MM);
        distanceFrontDriver = robot.color2.getDistance(DistanceUnit.MM);
        distanceFrontPassenger = robot.color3.getDistance(DistanceUnit.MM);

        // Position 1
        if (distanceRearCenter < 43) {

            // Mark Ball Found
            newPos1Detection = true;

            if (detectRearColor) {
                // Detect which color
                double green = robot.color1.getNormalizedColors().green;
                double red = robot.color1.getNormalizedColors().red;
                double blue = robot.color1.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                // FIXIT - Add filtering to improve accuracy.
                if (gP >= 0.4) {
                    ballPositions[commandedIntakePosition].ballColor = BallColor.PURPLE;  // purple
                } else {
                    ballPositions[commandedIntakePosition].ballColor = BallColor.GREEN;  // purple
                }
           }
        }
        // Position 2
        // Find which ball position this is in the spindexer
        spindexerBallPos = RotatedBallPositions[commandedIntakePosition][RotatedBallPositionNames.FRONTDRIVER.ordinal()];
        if (distanceFrontDriver < 60) {
            // reset FoundEmpty because looking for 3 in a row before reset
            ballPositions[spindexerBallPos].foundEmpty = 0;
            if (detectFrontColor) {
                double green = robot.color2.getNormalizedColors().green;
                double red = robot.color2.getNormalizedColors().red;
                double blue = robot.color2.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                if (gP >= 0.4) {
                    ballPositions[spindexerBallPos].ballColor = BallColor.PURPLE;  // purple
                } else {
                    ballPositions[spindexerBallPos].ballColor = BallColor.GREEN;  // purple
                }
            }
        } else {
            if (!ballPositions[spindexerBallPos].isEmpty) {
                if (ballPositions[spindexerBallPos].foundEmpty > 3) {
                    resetBallPosition(spindexerBallPos);
                }
                ballPositions[spindexerBallPos].foundEmpty++;
            }

        }

        // Position 3
        spindexerBallPos = RotatedBallPositions[commandedIntakePosition][RotatedBallPositionNames.FRONTPASSENGER.ordinal()];
        if (distanceFrontPassenger < 33) {

            // reset FoundEmpty because looking for 3 in a row before reset
            ballPositions[spindexerBallPos].foundEmpty = 0;
            if (detectFrontColor) {
                double green = robot.color3.getNormalizedColors().green;
                double red = robot.color3.getNormalizedColors().red;
                double blue = robot.color3.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                if (gP >= 0.4) {
                    ballPositions[spindexerBallPos].ballColor = BallColor.PURPLE;  // purple
                } else {
                    ballPositions[spindexerBallPos].ballColor = BallColor.GREEN;  // purple
                }
            }
        } else {
            if (!ballPositions[spindexerBallPos].isEmpty) {
                if (ballPositions[spindexerBallPos].foundEmpty > 3) {
                    resetBallPosition(spindexerBallPos);
                }
                ballPositions[spindexerBallPos].foundEmpty++;
            }
        }

//      TELE.addData("Velocity", velo);
//      TELE.addLine("Detecting");
//      TELE.addData("Distance 1", s1D);
//      TELE.addData("Distance 2", s2D);
//      TELE.addData("Distance 3", s3D);
//      TELE.addData("B1", b1);
//      TELE.addData("B2", b2);
//      TELE.addData("B3", b3);
//      TELE.update();

        return newPos1Detection;
    }
    // Has code to unjam spindexer
    private void moveSpindexerToPos(double pos) {
        robot.spin1.setPosition(pos);
        robot.spin2.setPosition(1-pos);
        double currentPos = servos.getSpinPos();
        if (!servos.spinEqual(pos) && Math.abs(prevPos - currentPos) <= 0){
            if (currentPos > pos){
                robot.spin1.setPosition(1);
                robot.spin2.setPosition(0);
            } else {
                robot.spin1.setPosition(0);
                robot.spin2.setPosition(1);
            }
        }
        prevPos = currentPos;
    }

    public void stopSpindexer() {

    }

    public void ballCounterLight(){
        int counter = 0;
        if (!ballPositions[0].isEmpty){
            counter++;
        }
        if (!ballPositions[1].isEmpty){
            counter++;
        }
        if (!ballPositions[2].isEmpty){
            counter++;
        }
        if (counter == 3){
            robot.light.setPosition(Light3);
        } else if (counter == 2){
            robot.light.setPosition(Light2);
        } else if (counter == 1){
            robot.light.setPosition(Light1);
        } else {
            robot.light.setPosition(Light0);
        }
    }

    public boolean isFull () {
        return (!ballPositions[0].isEmpty && !ballPositions[1].isEmpty && !ballPositions[2].isEmpty);
    }
    public boolean processIntake() {

        switch (currentIntakeState) {
            case UNKNOWN_START:
                // For now just set position ONE if UNKNOWN
                commandedIntakePosition = 0;
                servos.setSpinPos(intakePositions[0]);
                currentIntakeState = Spindexer.IntakeState.UNKNOWN_MOVE;
                break;
            case UNKNOWN_MOVE:
                // Stopping when we get to the new position
                if (servos.spinEqual(intakePositions[commandedIntakePosition])) {
                    currentIntakeState = Spindexer.IntakeState.UNKNOWN_DETECT;
                    stopSpindexer();
                    unknownColorDetect = 0;
                } else {
                    // Keep moving the spindexer
                    moveSpindexerToPos(intakePositions[commandedIntakePosition]);
                }
                break;
            case UNKNOWN_DETECT:
                if (unknownColorDetect >5) {
                    currentIntakeState = Spindexer.IntakeState.FINDNEXT;
                } else {
                    //detectBalls(true, true);
                    unknownColorDetect++;
                }
                break;
            case INTAKE:
                // Ready for intake and Detecting a New Ball
                if (detectBalls(true, false)) {
                    ballPositions[commandedIntakePosition].isEmpty = false;
                    currentIntakeState = Spindexer.IntakeState.FINDNEXT;
                } else {
                    // Maintain Position
                    moveSpindexerToPos(intakePositions[commandedIntakePosition]);
                }
                break;
            case FINDNEXT:
                // Find Next Open Position and start movement
                double currentSpindexerPos = servos.getSpinPos();
                double commandedtravelDistance = 2.0;
                //double proposedTravelDistance = Math.abs(intakePositions[0] - currentSpindexerPos);
                //if (ballPositions[0].isEmpty && (proposedTravelDistance < commandedtravelDistance)) {
                if (ballPositions[0].isEmpty) {
                    // Position 1
                    commandedIntakePosition = 0;
                    servos.setSpinPos(intakePositions[commandedIntakePosition]);
                    currentIntakeState = Spindexer.IntakeState.MOVING;
                }
                //proposedTravelDistance = Math.abs(intakePositions[1] - currentSpindexerPos);
                //if (ballPositions[1].isEmpty && (proposedTravelDistance < commandedtravelDistance)) {
                if (ballPositions[1].isEmpty) {
                    // Position 2
                    commandedIntakePosition = 1;
                    servos.setSpinPos(intakePositions[commandedIntakePosition]);
                    currentIntakeState = Spindexer.IntakeState.MOVING;
                }
                //proposedTravelDistance = Math.abs(intakePositions[2] - currentSpindexerPos);
                if (ballPositions[2].isEmpty) {
                    // Position 3
                    commandedIntakePosition = 2;
                    servos.setSpinPos(intakePositions[commandedIntakePosition]);
                    currentIntakeState = Spindexer.IntakeState.MOVING;
                }
                if (currentIntakeState != Spindexer.IntakeState.MOVING) {
                    // Full
                    //commandedIntakePosition = bestFitMotif();
                    currentIntakeState = Spindexer.IntakeState.FULL;
                }
                moveSpindexerToPos(intakePositions[commandedIntakePosition]);
                break;

            case MOVING:
                // Stopping when we get to the new position
                if (servos.spinEqual(intakePositions[commandedIntakePosition])) {
                    currentIntakeState = Spindexer.IntakeState.INTAKE;
                    stopSpindexer();
                    //detectBalls(false, false);
                } else {
                    // Keep moving the spindexer
                    moveSpindexerToPos(intakePositions[commandedIntakePosition]);
                }
                break;

            case FULL:
                // Double Check Colors
                detectBalls(false, false); // Minimize hardware calls
                if (ballPositions[0].isEmpty || ballPositions[1].isEmpty || ballPositions[2].isEmpty) {
                    // Error handling found an empty spot, get it ready for a ball
                    currentIntakeState = Spindexer.IntakeState.FINDNEXT;
                }
                // Maintain Position
                moveSpindexerToPos(intakePositions[commandedIntakePosition]);
                break;

            case SHOOT_ALL_PREP:
                // We get here with function call to prepareToShootMotif
                // Stopping when we get to the new position
                if (servos.spinEqual(intakePositions[commandedIntakePosition])) {
                    currentIntakeState = Spindexer.IntakeState.SHOOT_ALL_READY;
                } else {
                    // Keep moving the spindexer
                    moveSpindexerToPos(intakePositions[commandedIntakePosition]); // Possible error: should it be using "outakePositions" instead of "intakePositions"
                }
                break;

            case SHOOT_ALL_READY:
                // Double Check Colors
                //detectBalls(false, false); // Minimize hardware calls
                if (ballPositions[0].isEmpty && ballPositions[1].isEmpty && ballPositions[2].isEmpty) {
                    // All ball shot move to intake state
                    currentIntakeState = Spindexer.IntakeState.FINDNEXT;
                }
                // Maintain Position
                moveSpindexerToPos(intakePositions[commandedIntakePosition]);
                break;

            case SHOOTNEXT:
                // Find Next Open Position and start movement
                if (!ballPositions[0].isEmpty) {
                    // Position 1
                    commandedIntakePosition = 0;
                    servos.setSpinPos(outakePositions[commandedIntakePosition]);
                    currentIntakeState = Spindexer.IntakeState.SHOOTMOVING;
                } else if (ballPositions[1].isEmpty) { // Possible error: should it be !ballPosition[1].isEmpty?
                    // Position 2
                    commandedIntakePosition = 1;
                    servos.setSpinPos(outakePositions[commandedIntakePosition]);
                    currentIntakeState = Spindexer.IntakeState.SHOOTMOVING;
                } else if (ballPositions[2].isEmpty) { // Possible error: should it be !ballPosition[2].isEmpty?
                    // Position 3
                    commandedIntakePosition = 2;
                    servos.setSpinPos(intakePositions[commandedIntakePosition]); // Possible error: should it be using "outakePositions" instead of "intakePositions"
                    currentIntakeState = Spindexer.IntakeState.SHOOTMOVING;
                } else {
                    // Empty return to intake state
                    currentIntakeState = IntakeState.FINDNEXT;
                }
                moveSpindexerToPos(intakePositions[commandedIntakePosition]); // Possible error: should it be using "outakePositions" instead of "intakePositions"
                break;

            case SHOOTMOVING:
                // Stopping when we get to the new position
                if (servos.spinEqual(outakePositions[commandedIntakePosition])) {
                    currentIntakeState = Spindexer.IntakeState.SHOOTWAIT;
                } else {
                    // Keep moving the spindexer
                    moveSpindexerToPos(intakePositions[commandedIntakePosition]); // Possible error: should it be using "outakePositions" instead of "intakePositions"
                }
                break;

            case SHOOTWAIT:
                // Stopping when we get to the new position
                if (servos.spinEqual(intakePositions[commandedIntakePosition])) {
                    currentIntakeState = Spindexer.IntakeState.INTAKE;
                    stopSpindexer();
                    //detectBalls(true, false);
                } else {
                    // Keep moving the spindexer
                    moveSpindexerToPos(intakePositions[commandedIntakePosition]);
                }
                break;

            default:
                // Statements to execute if no case matches
        }
        //TELE.addData("commandedIntakePosition", commandedIntakePosition);
        //TELE.update();
        // Signal a successful intake
        return false;
    }

    public void setDesiredMotif (Types.Motif newMotif) {
        desiredMotif = newMotif;
    }

    // Returns the best fit for the motiff
    public int bestFitMotif () {
        switch (desiredMotif) {
            case GPP:
                if (ballPositions[0].ballColor == BallColor.GREEN) {
                    return 2;
                } else if (ballPositions[1].ballColor == BallColor.GREEN) {
                    return 0;
                } else {
                    return 1;
                }
                //break;
            case PGP:
                if (ballPositions[0].ballColor == BallColor.GREEN) {
                    return 0;
                } else if (ballPositions[1].ballColor == BallColor.GREEN) {
                    return 1;
                } else {
                    return 3;
                }
                //break;
            case PPG:
                if (ballPositions[0].ballColor == BallColor.GREEN) {
                    return 1;
                } else if (ballPositions[1].ballColor == BallColor.GREEN) {
                    return 0;
                } else {
                    return 2;
                }
                //break;
            case NONE:
                return 0;
                //break;
        }
        return 0;
    }

    void prepareToShootMotif () {
        commandedIntakePosition = bestFitMotif();
    }

    void shootAllToIntake () {
        currentIntakeState = Spindexer.IntakeState.FINDNEXT;
    }

    public void update()
    {
    }
}
