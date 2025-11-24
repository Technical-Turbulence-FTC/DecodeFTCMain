package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.utils.Robot;

public class Intake implements Subsystem {

    private GamepadEx gamepad;

    public MultipleTelemetry TELE;


    private DcMotorEx intake;

    private double intakePower = 1.0;

    private int intakeState = 0;


    public Intake (Robot robot){


        this.intake = robot.intake;


    }

    public int getIntakeState() {
        return intakeState;
    }

    public void toggle(){
        if (intakeState !=0){
            intakeState = 0;
        } else {
            intakeState = 1;
        }
    }

    public void setIntakePower(double pow){
        intakePower = pow;
    }

    public void intake(){
        intakeState =1;
    }

    public void reverse(){
        intakeState =-1;
    }


    public void stop(){
        intakeState =-1;
    }




    @Override
    public void update() {

        if (intakeState == 1){
            intake.setPower(intakePower);
        } else if (intakeState == -1){
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
        }

    }
}
