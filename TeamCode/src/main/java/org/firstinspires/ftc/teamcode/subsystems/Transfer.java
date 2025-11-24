package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Robot;

public class Transfer implements Subsystem{

    private final Servo servo;

    private final DcMotorEx transfer;

    private double motorPow = 0.0;

    private double servoPos = 0.501;

    public Transfer (Robot robot){

        this.servo = robot.transferServo;

        this.transfer = robot.transfer;

    }

    public void setTransferPosition(double pos){
        this.servoPos  = pos;
    }

    public void setTransferPower (double pow){
        this.motorPow = pow;
    }

    public void transferOut(){
        this.setTransferPosition(transferServo_out);
    }

    public void transferIn(){
        this.setTransferPosition(transferServo_in);
    }





    @Override
    public void update() {

        if (servoPos!=0.501){
            servo.setPosition(servoPos);
        }

        transfer.setPower(motorPow);

    }
}
