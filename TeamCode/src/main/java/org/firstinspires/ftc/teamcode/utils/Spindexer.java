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

    public double spindexerWiggle = 0.01;

    public double spindexerOuttakeWiggle = 0.01;
    private double prevPos = 0.0;
    public double spindexerPosOffset = 0.00;
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

    int shootWaitCount = 0;

    public IntakeState currentIntakeState = IntakeState.UNKNOWN_START;
    public IntakeState prevIntakeState = IntakeState.UNKNOWN_START;
    public int unknownColorDetect = 0;
    public enum BallColor {
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
            {spindexer_outtakeBall1+spindexerPosOffset,
                    spindexer_outtakeBall2+spindexerPosOffset,
                    spindexer_outtakeBall3+spindexerPosOffset};

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
        double dRearCenter = robot.color1.getDistance(DistanceUnit.MM);
        distanceRearCenter = (colorFilterAlpha * dRearCenter) + ((1-colorFilterAlpha) * distanceRearCenter);
        double dFrontDriver = robot.color2.getDistance(DistanceUnit.MM);
        distanceFrontDriver = (colorFilterAlpha * dFrontDriver) + ((1-colorFilterAlpha) * distanceFrontDriver);
        double dFrontPassenger = robot.color3.getDistance(DistanceUnit.MM);
        distanceFrontPassenger = (colorFilterAlpha * dFrontPassenger) + ((1-colorFilterAlpha) * distanceFrontPassenger);

        // Position 1
        if (distanceRearCenter < 60) {

            // Mark Ball Found
            newPos1Detection = true;

            if (detectRearColor) {
                // Detect which color
                double green = robot.color1.getNormalizedColors().green;
                double red = robot.color1.getNormalizedColors().red;
                double blue = robot.color1.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                // FIXIT - Add filtering to improve accuracy.
                if (gP >= 0.38) {
                    ballPositions[commandedIntakePosition].ballColor = BallColor.GREEN;  // green
                } else {
                    ballPositions[commandedIntakePosition].ballColor = BallColor.PURPLE;  // purple
                }
           }
        }
        // Position 2
        // Find which ball position this is in the spindexer
        spindexerBallPos = RotatedBallPositions[commandedIntakePosition][RotatedBallPositionNames.FRONTDRIVER.ordinal()];
        if (distanceFrontDriver < 50) {
            // reset FoundEmpty because looking for 3 in a row before reset
            ballPositions[spindexerBallPos].foundEmpty = 0;
            if (detectFrontColor) {
                double green = robot.color2.getNormalizedColors().green;
                double red = robot.color2.getNormalizedColors().red;
                double blue = robot.color2.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                if (gP >= 0.4) {
                    ballPositions[spindexerBallPos].ballColor = BallColor.GREEN;  // green
                } else {
                    ballPositions[spindexerBallPos].ballColor = BallColor.PURPLE;  // purple
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
        if (distanceFrontPassenger < 29) {

            // reset FoundEmpty because looking for 3 in a row before reset
            ballPositions[spindexerBallPos].foundEmpty = 0;
            if (detectFrontColor) {
                double green = robot.color3.getNormalizedColors().green;
                double red = robot.color3.getNormalizedColors().red;
                double blue = robot.color3.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                if (gP >= 0.42) {
                    ballPositions[spindexerBallPos].ballColor = BallColor.GREEN;  // green
                } else {
                    ballPositions[spindexerBallPos].ballColor = BallColor.PURPLE;  // purple
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
//            if (currentPos > pos){
//                robot.spin1.setPosition(servos.getSpinPos() + 0.05);
//                robot.spin2.setPosition(1 - servos.getSpinPos() - 0.05);
//            } else {
//                robot.spin1.setPosition(servos.getSpinPos() - 0.05);
//                robot.spin2.setPosition(1 - servos.getSpinPos() + 0.05);
//            }
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

    public boolean slotIsEmpty(int slot){
        return !ballPositions[slot].isEmpty;
    }

    public boolean isFull () {
        return (!ballPositions[0].isEmpty && !ballPositions[1].isEmpty && !ballPositions[2].isEmpty);
    }
    public boolean processIntake() {

        switch (currentIntakeState) {
            case UNKNOWN_START:
                // For now just set position ONE if UNKNOWN
                commandedIntakePosition = 0;
                moveSpindexerToPos(intakePositions[0]);
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
                    spindexerWiggle *= -1.0;
                    moveSpindexerToPos(intakePositions[commandedIntakePosition]+spindexerWiggle);
                }
                break;
            case FINDNEXT:
                // Find Next Open Position and start movement
                double currentSpindexerPos = servos.getSpinPos();
                double commandedtravelDistance = 2.0;
                double proposedTravelDistance = Math.abs(intakePositions[0] - currentSpindexerPos);
                //if (ballPositions[0].isEmpty && (proposedTravelDistance < commandedtravelDistance)) {
                if (ballPositions[0].isEmpty) {
                    // Position 1
                    commandedIntakePosition = 0;
                    currentIntakeState = Spindexer.IntakeState.MOVING;
                }
                proposedTravelDistance = Math.abs(intakePositions[1] - currentSpindexerPos);
                //if (ballPositions[1].isEmpty && (proposedTravelDistance < commandedtravelDistance)) {
                if (ballPositions[1].isEmpty) {
                    // Position 2
                    commandedIntakePosition = 1;
                    currentIntakeState = Spindexer.IntakeState.MOVING;
                }
                proposedTravelDistance = Math.abs(intakePositions[2] - currentSpindexerPos);
                if (ballPositions[2].isEmpty) {
                    // Position 3
                    commandedIntakePosition = 2;
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
                spindexerWiggle *= -1.0;
                moveSpindexerToPos(intakePositions[commandedIntakePosition]+spindexerWiggle);
                break;

            case SHOOT_ALL_PREP:
                // We get here with function call to prepareToShootMotif
                // Stopping when we get to the new position
                commandedIntakePosition = 0;
                if (!servos.spinEqual(outakePositions[commandedIntakePosition])) {
                    // Keep moving the spindexer
                    moveSpindexerToPos(outakePositions[commandedIntakePosition]); // Possible error: should it be using "outakePositions" instead of "intakePositions"
                }
                break;

            case SHOOT_ALL_READY: // Not used
                // Double Check Colors
                //detectBalls(false, false); // Minimize hardware calls
                if (ballPositions[0].isEmpty && ballPositions[1].isEmpty && ballPositions[2].isEmpty) {
                    // All ball shot move to intake state
                    currentIntakeState = Spindexer.IntakeState.SHOOTNEXT;
                }
                // Maintain Position
                moveSpindexerToPos(outakePositions[commandedIntakePosition]);
                break;

            case SHOOTNEXT:
                // Find Next Open Position and start movement
                if (!ballPositions[0].isEmpty) {
                    // Position 1
                    commandedIntakePosition = 0;
                    currentIntakeState = Spindexer.IntakeState.SHOOTMOVING;
                } else if (!ballPositions[1].isEmpty) {
                    // Position 2
                    commandedIntakePosition = 1;
                    currentIntakeState = Spindexer.IntakeState.SHOOTMOVING;
                } else if (!ballPositions[2].isEmpty) {
                    // Position 3
                    commandedIntakePosition = 2;
                    currentIntakeState = Spindexer.IntakeState.SHOOTMOVING;
                } else {
                    // Empty return to intake state
                    currentIntakeState = IntakeState.FINDNEXT;
                }
                moveSpindexerToPos(outakePositions[commandedIntakePosition]);
                break;

            case SHOOTMOVING:
                // Stopping when we get to the new position
                if (servos.spinEqual(outakePositions[commandedIntakePosition])) {
                    currentIntakeState = Spindexer.IntakeState.SHOOTWAIT;
                } else {
                    // Keep moving the spindexer
                    moveSpindexerToPos(outakePositions[commandedIntakePosition]);
                }
                break;

            case SHOOTWAIT:
                double shootWaitMax = 4;
                // Stopping when we get to the new position
                if (prevIntakeState != currentIntakeState) {
                    if (commandedIntakePosition==2) {
                        shootWaitMax = 5;
                    }
                    shootWaitCount = 0;
                } else {
                    shootWaitCount++;
                }
                // wait 10 cycles
                if (shootWaitCount > shootWaitMax) {
                    currentIntakeState = Spindexer.IntakeState.SHOOTNEXT;
                    ballPositions[commandedIntakePosition].isEmpty = true;
                    shootWaitCount = 0;
                    //stopSpindexer();
                    //detectBalls(true, false);
                }
                // Keep moving the spindexer
                spindexerOuttakeWiggle *= -1.01;
                moveSpindexerToPos(outakePositions[commandedIntakePosition]+spindexerOuttakeWiggle);
                break;

            default:
                // Statements to execute if no case matches
        }

        prevIntakeState = currentIntakeState;
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
                    return 2;
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

    public void prepareShootAll(){
        currentIntakeState = Spindexer.IntakeState.SHOOT_ALL_PREP;
    }
    public void shootAll () {
        ballPositions[0].isEmpty = false;
        ballPositions[1].isEmpty = false;
        ballPositions[2].isEmpty = false;
        currentIntakeState = Spindexer.IntakeState.SHOOTNEXT;
    }

    public boolean shootAllComplete ()
    {
        return ((currentIntakeState != Spindexer.IntakeState.SHOOT_ALL_PREP) &&
                (currentIntakeState != Spindexer.IntakeState.SHOOT_ALL_READY) &&
                (currentIntakeState != Spindexer.IntakeState.SHOOTMOVING) &&
                (currentIntakeState != Spindexer.IntakeState.SHOOTNEXT) &&
                (currentIntakeState != Spindexer.IntakeState.SHOOTWAIT));
    }

    void shootAllToIntake () {
        currentIntakeState = Spindexer.IntakeState.FINDNEXT;
    }

    public void update()
    {
    }

    public BallColor GetFrontDriverColor () {
        return ballPositions[RotatedBallPositions[commandedIntakePosition][RotatedBallPositionNames.FRONTDRIVER.ordinal()]].ballColor;
    }

    public BallColor GetFrontPassengerColor () {
        return ballPositions[RotatedBallPositions[commandedIntakePosition][RotatedBallPositionNames.FRONTPASSENGER.ordinal()]].ballColor;
    }

    public BallColor GetRearCenterColor () {
        return ballPositions[RotatedBallPositions[commandedIntakePosition][RotatedBallPositionNames.REARCENTER.ordinal()]].ballColor;
    }
}
