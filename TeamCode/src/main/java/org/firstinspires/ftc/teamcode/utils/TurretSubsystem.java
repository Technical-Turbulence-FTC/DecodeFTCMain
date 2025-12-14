package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.turrDefault;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class TurretSubsystem {
    private Robot robot;
    private MultipleTelemetry telemetry;
    
    private double manualOffset = 0.0;
    private boolean manualTurret = false;
    private boolean overrideTurr = false;
    private double desiredTurretAngle = 180;

    public TurretSubsystem(Robot robot, MultipleTelemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    /**
     * Update turret position to aim at goal
     * @param robotX Robot X position
     * @param robotY Robot Y position
     * @param robotHeading Robot heading in radians
     */
    public void updateAutoAim(double robotX, double robotY, double robotHeading) {
        double goalX = -10;
        double goalY = 0;

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        desiredTurretAngle = (Math.toDegrees(Math.atan2(dy, dx)) + 360) % 360;
        desiredTurretAngle += manualOffset;

        double offset = desiredTurretAngle - 180 - Math.toDegrees(robotHeading);

        if (offset > 135) {
            offset -= 360;
        }

        double pos = turrDefault;
        telemetry.addData("offset", offset);

        pos -= offset * (0.9 / 360);

        if (pos < 0.02) {
            pos = 0.02;
        } else if (pos > 0.97) {
            pos = 0.97;
        }

        if (manualTurret) {
            pos = turrDefault + (manualOffset / 100);
        }

        if (!overrideTurr) {
            robot.turr1.setPosition(pos);
            robot.turr2.setPosition(1 - pos);
        }
    }

    /**
     * Adjust turret with manual offset
     */
    public void adjustManualOffset(double delta) {
        manualOffset += delta;
    }

    /**
     * Set manual turret mode
     */
    public void setManualMode(boolean manual) {
        this.manualTurret = manual;
    }

    /**
     * Reset manual offset
     */
    public void resetManualOffset() {
        manualOffset = 0.0;
    }

    /**
     * Set turret override (for AprilTag tracking)
     */
    public void setOverride(boolean override) {
        this.overrideTurr = override;
    }

    /**
     * Manually set turret position (used for AprilTag tracking)
     */
    public void setPosition(double pos) {
        robot.turr1.setPosition(pos);
        robot.turr2.setPosition(1 - pos);
    }

    /**
     * Get current turret servo 1 position
     */
    public double getPosition() {
        return robot.turr1.getPosition();
    }

    public double getManualOffset() {
        return manualOffset;
    }

    public boolean isOverride() {
        return overrideTurr;
    }
}
