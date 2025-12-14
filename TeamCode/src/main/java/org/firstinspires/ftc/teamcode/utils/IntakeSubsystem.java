package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;

public class Intake {
    private Robot robot;
    private boolean isIntaking;
    private boolean isRejecting;

    public Intake(Robot robot) {
        this.robot = robot;
        this.isIntaking = false;
        this.isRejecting = false;
    }

    /**
     * Toggle intake on/off
     */
    public void toggleIntake() {
        isIntaking = !isIntaking;
        isRejecting = false;
    }

    /**
     * Set intake state directly
     */
    public void setIntake(boolean state) {
        isIntaking = state;
        if (state) {
            isRejecting = false;
        }
    }

    /**
     * Toggle reject mode
     */
    public void toggleReject() {
        isRejecting = !isRejecting;
        isIntaking = false;
    }

    /**
     * Set reject state directly
     */
    public void setReject(boolean state) {
        isRejecting = state;
        if (state) {
            isIntaking = false;
        }
    }

    /**
     * Update intake based on current state
     * @param runtime Current runtime in seconds for oscillating spindexer
     */
    public void update(double runtime) {
        if (isIntaking) {
            robot.intake.setPower(1);

            // Oscillate spindexer for better intake
            double position;
            if ((runtime % 0.3) > 0.15) {
                position = spindexer_intakePos1 + 0.015;
            } else {
                position = spindexer_intakePos1 - 0.015;
            }

            robot.spin1.setPosition(position);
            robot.spin2.setPosition(1 - position);
        } else if (isRejecting) {
            robot.intake.setPower(-1);
            double position = spindexer_intakePos1;
            robot.spin1.setPosition(position);
            robot.spin2.setPosition(1 - position);
        } else {
            robot.intake.setPower(0);
        }
    }

    public boolean isIntaking() {
        return isIntaking;
    }

    public boolean isRejecting() {
        return isRejecting;
    }

    public void stop() {
        isIntaking = false;
        isRejecting = false;
        robot.intake.setPower(0);
    }
}
