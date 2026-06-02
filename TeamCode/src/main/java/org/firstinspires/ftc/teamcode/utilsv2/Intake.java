package org.firstinspires.ftc.teamcode.utilsv2;

import org.firstinspires.ftc.teamcode.utilsv2.Robot;

public class Intake {

    Robot rob;
    public Intake(Robot robot) {

        this.rob = robot;

    }

    public void setIntakePower(double pow){
        rob.intake.setPower(pow);
    }
    public void setTransferPower(double pow){
        rob.transfer.setPower(pow);
    }
}
