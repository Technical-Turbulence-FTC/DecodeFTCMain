package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.waitTransfer;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.waitTransferOut;
import static org.firstinspires.ftc.teamcode.utils.PositionalServoProgrammer.restPos;
import static org.firstinspires.ftc.teamcode.utils.PositionalServoProgrammer.scalar;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.util.ArrayList;
import java.util.List;

public class SpindexerSubsystem {
    private Robot robot;
    private MultipleTelemetry telemetry;
    private ColorSensorSubsystem colorSensor;

    private List<Integer> shootOrder = new ArrayList<>();
    private boolean shootAll = false;
    private double shootStamp = 0.0;
    private double shootStamp2 = 0.0;
    private boolean shootA = true;
    private boolean shootB = true;
    private boolean shootC = true;
    private boolean outtake1 = false;
    private boolean outtake2 = false;
    private boolean outtake3 = false;
    private double transferStamp = 0.0;
    private int tickerA = 1;
    private boolean transferIn = false;

    public SpindexerSubsystem(Robot robot, MultipleTelemetry telemetry, ColorSensorSubsystem colorSensor) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.colorSensor = colorSensor;
    }

    /**
     * Update spindexer shooting sequence
     */
    public void update(double runtime, AprilTagWebcam aprilTagWebcam) {
        if (!shootAll) {
            return;
        }

        telemetry.addData("shootOrder", shootOrder);

        if (!shootOrder.isEmpty() && (runtime - shootStamp < 12)) {
            int currentSlot = shootOrder.get(0);
            boolean shootingDone = false;

            // AprilTag tracking for turret override
            var d20 = aprilTagWebcam.getTagById(20);
            var d24 = aprilTagWebcam.getTagById(24);

            if (d20 != null) {
                double bearing = d20.ftcPose.bearing;
                double finalPos = robot.turr1.getPosition() - (bearing / 1300);
                robot.turr1.setPosition(finalPos);
                robot.turr2.setPosition(1 - finalPos);
                telemetry.addData("Bear", bearing);
            }

            if (d24 != null) {
                double bearing = d24.ftcPose.bearing;
                double finalPos = robot.turr1.getPosition() - (bearing / 1300);
                robot.turr1.setPosition(finalPos);
                robot.turr2.setPosition(1 - finalPos);
            }

            // Check if positions are reached
            if (!outtake1) {
                outtake1 = spindexPosEqual(spindexer_outtakeBall1);
            }
            if (!outtake2) {
                outtake2 = spindexPosEqual(spindexer_outtakeBall2);
            }
            if (!outtake3) {
                outtake3 = spindexPosEqual(spindexer_outtakeBall3);
            }

            // Execute shooting for current slot
            switch (currentSlot) {
                case 1:
                    shootA = shootTeleop(spindexer_outtakeBall1, outtake1, shootStamp2, runtime);
                    telemetry.addData("shootA", shootA);
                    if ((runtime - shootStamp) < 4 * (4 - shootOrder.size())) {
                        shootingDone = !shootA;
                    } else {
                        shootingDone = true;
                    }
                    break;
                case 2:
                    shootB = shootTeleop(spindexer_outtakeBall2, outtake2, shootStamp2, runtime);
                    telemetry.addData("shootB", shootB);
                    if ((runtime - shootStamp) < 4 * (4 - shootOrder.size())) {
                        shootingDone = !shootB;
                    } else {
                        shootingDone = true;
                    }
                    break;
                case 3:
                    shootC = shootTeleop(spindexer_outtakeBall3, outtake3, shootStamp2, runtime);
                    telemetry.addData("shootC", shootC);
                    if ((runtime - shootStamp) < 4 * (4 - shootOrder.size())) {
                        shootingDone = !shootC;
                    } else {
                        shootingDone = true;
                    }
                    break;
            }

            if (shootingDone) {
                shootOrder.remove(0);
                shootStamp2 = runtime;
            }
        } else {
            // Finished shooting all balls
            finishShooting();
        }
    }

    /**
     * Execute emergency mode (reject all balls)
     */
    public void updateEmergency(double runtime) {
        if (runtime % 3 > 1.5) {
            robot.spin1.setPosition(0);
            robot.spin2.setPosition(1);
        } else {
            robot.spin1.setPosition(1);
            robot.spin2.setPosition(0);
        }
        robot.transferServo.setPosition(transferServo_out);
        robot.transfer.setPower(1);
    }

    /**
     * Check if spindexer is at target position
     */
    private boolean spindexPosEqual(double spindexer) {
        return (scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) > spindexer - 0.01 &&
                scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) < spindexer + 0.01);
    }

    /**
     * Execute shooting for a single ball
     */
    private boolean shootTeleop(double spindexer, boolean spinOk, double stamp, double runtime) {
        robot.spin1.setPosition(spindexer);
        robot.spin2.setPosition(1 - spindexer);

        if (spinOk || runtime - stamp > 1.5) {
            if (tickerA == 1) {
                transferStamp = runtime;
                tickerA++;
                telemetry.addLine("tickerSet");
            }

            if (runtime - transferStamp > waitTransfer && !transferIn) {
                robot.transferServo.setPosition(transferServo_in);
                transferIn = true;
                telemetry.addLine("transferring");
                return true;
            } else if (runtime - transferStamp > waitTransfer + waitTransferOut && transferIn) {
                robot.transferServo.setPosition(transferServo_out);
                transferIn = false;
                tickerA = 1;
                transferStamp = 0.0;
                telemetry.addLine("shotFinished");
                return false;
            } else {
                telemetry.addLine("sIP");
                return true;
            }
        } else {
            robot.transferServo.setPosition(transferServo_out);
            tickerA = 1;
            transferStamp = runtime;
            transferIn = false;
            return true;
        }
    }

    /**
     * Start shooting sequence with odd ball first
     */
    public void startShootOddFirst(double runtime) {
        int greenCount = countGreenBalls();
        boolean oddBallColor = greenCount < 2;

        shootOrder.clear();
        addOddThenRest(shootOrder, oddBallColor);

        startShootingSequence(runtime);
        telemetry.addData("oddBall", oddBallColor);
    }

    /**
     * Start shooting sequence with odd ball in middle
     */
    public void startShootOddMiddle(double runtime) {
        int greenCount = countGreenBalls();
        boolean oddBallColor = greenCount < 2;

        shootOrder.clear();
        addOddInMiddle(shootOrder, oddBallColor);

        startShootingSequence(runtime);
        telemetry.addData("oddBall", oddBallColor);
    }

    /**
     * Start shooting sequence with odd ball last
     */
    public void startShootOddLast(double runtime) {
        int greenCount = countGreenBalls();
        boolean oddBallColor = greenCount < 2;

        shootOrder.clear();
        addOddLast(shootOrder, oddBallColor);

        startShootingSequence(runtime);
        telemetry.addData("oddBall", oddBallColor);
    }

    /**
     * Start fastest shooting sequence (ignore colors)
     */
    public void startShootFastest(double runtime) {
        shootOrder.clear();

        if (colorSensor.ballIn(3, runtime)) {
            shootOrder.add(3);
        }
        if (colorSensor.ballIn(2, runtime)) {
            shootOrder.add(2);
        }
        if (colorSensor.ballIn(1, runtime)) {
            shootOrder.add(1);
        }

        // Add missing slots
        if (!shootOrder.contains(3)) {
            shootOrder.add(3);
        }
        if (!shootOrder.contains(2)) {
            shootOrder.add(2);
        }
        if (!shootOrder.contains(1)) {
            shootOrder.add(1);
        }

        startShootingSequence(runtime);
    }

    private void startShootingSequence(double runtime) {
        shootStamp = runtime;
        shootStamp2 = runtime;
        outtake1 = false;
        outtake2 = false;
        outtake3 = false;
        shootAll = true;
    }

    private void finishShooting() {
        robot.spin1.setPosition(spindexer_intakePos1);
        robot.spin2.setPosition(1 - spindexer_intakePos1);
        shootA = true;
        shootB = true;
        shootC = true;
        shootAll = false;
        outtake1 = false;
        outtake2 = false;
        outtake3 = false;
    }

    private int countGreenBalls() {
        int count = 0;
        if (colorSensor.isGreen1()) count++;
        if (colorSensor.isGreen2()) count++;
        if (colorSensor.isGreen3()) count++;
        return count;
    }

    private void addOddThenRest(List<Integer> order, boolean oddColor) {
        for (int i = 1; i <= 3; i++) {
            if (colorSensor.getBallColor(i) == oddColor) order.add(i);
        }
        for (int i = 1; i <= 3; i++) {
            if (colorSensor.getBallColor(i) != oddColor) order.add(i);
        }
    }

    private void addOddInMiddle(List<Integer> order, boolean oddColor) {
        boolean[] used = new boolean[4];

        for (int i = 1; i <= 3; i++) {
            if (colorSensor.getBallColor(i) != oddColor) {
                order.add(i);
                used[i] = true;
                break;
            }
        }

        for (int i = 1; i <= 3; i++) {
            if (!used[i] && colorSensor.getBallColor(i) == oddColor) {
                order.add(i);
                used[i] = true;
                break;
            }
        }

        for (int i = 1; i <= 3; i++) {
            if (!used[i] && colorSensor.getBallColor(i) != oddColor) {
                order.add(i);
                used[i] = true;
                break;
            }
        }
    }

    private void addOddLast(List<Integer> order, boolean oddColor) {
        for (int i = 1; i <= 3; i++) {
            if (colorSensor.getBallColor(i) != oddColor) order.add(i);
        }
        for (int i = 1; i <= 3; i++) {
            if (colorSensor.getBallColor(i) == oddColor) order.add(i);
        }
    }

    public void stopShooting() {
        shootAll = false;
        finishShooting();
    }

    public boolean isShootingAll() {
        return shootAll;
    }

    public List<Integer> getShootOrder() {
        return shootOrder;
    }
}
