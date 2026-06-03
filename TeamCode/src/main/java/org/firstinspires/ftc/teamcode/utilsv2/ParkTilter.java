package org.firstinspires.ftc.teamcode.utilsv2;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.ServoPositions;

public class ParkTilter {
    Robot robot;
    public ParkTilter (Robot rob) {
        this.robot = rob;
    }

    public void park() {
        robot.setTilt1Pos(ServoPositions.tilt1_down);
        robot.setTilt2Pos(ServoPositions.tilt2_down);
    }

    public void unpark() {
        robot.setTilt1Pos(ServoPositions.tilt1_up);
        robot.setTilt2Pos(ServoPositions.tilt2_up);
    }
}
