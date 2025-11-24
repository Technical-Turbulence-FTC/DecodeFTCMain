package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.ArrayList;

public class Spindexer implements Subsystem{

    private Servo s1;
    private Servo s2;

    private DigitalChannel p0;

    private DigitalChannel p1;
    private DigitalChannel p2;
    private DigitalChannel p3;
    private DigitalChannel p4;

    private DigitalChannel p5;

    private AnalogInput input;

    private AnalogInput input2;


    private MultipleTelemetry TELE;

    private double position = 0.501;

    private boolean telemetryOn = false;

    private boolean ball0 = false;

    private boolean ball1 = false;

    private boolean ball2 = false;

    private boolean green0 = false;

    private boolean green1 = false;

    private boolean green2 = false;




    public Spindexer (Robot robot, MultipleTelemetry tele){

        this.s1 = robot.spin1;
        this.s2 = robot.spin2;

        this.p0 = robot.pin0;
        this.p1 = robot.pin1;
        this.p2 = robot.pin2;
        this.p3 = robot.pin3;
        this.p4 = robot.pin4;
        this.p5 = robot.pin5;

        this.input = robot.analogInput;

        this.input2 = robot.analogInput2;

        this.TELE = tele;

    }

    public void setTelemetryOn(boolean state){
        telemetryOn = state;
    }

    public void colorSensorTelemetry() {


        TELE.addData("ball0", ball0);
        TELE.addData("ball1", ball1);
        TELE.addData("ball2", ball2);
        TELE.addData("green0", green0);
        TELE.addData("green1", green1);
        TELE.addData("green2", green2);



    }

    public void checkForBalls() {

        if (p0.getState()){
            ball0 = true;
            green0 = p1.getState();
        } else {
            ball0 = false;
        }

        if (p2.getState()){
            ball1 = true;
            green1 = p3.getState();
        } else {
            ball1 = false;
        }

        if (p4.getState()){
            ball2 = true;
            green2 = p5.getState();
        } else {
            ball2 = false;
        }
    }

    public void setPosition (double pos) {
        position = pos;
    }

    public void intake () {
        position = spindexer_intakePos;
    }

    public void intakeShake(double runtime) {
        if ((runtime % 0.25) >0.125) {
            position = spindexer_intakePos + 0.04;
        } else {
            position = spindexer_intakePos - 0.04;

        }
    }

    public void outtake3Shake(double runtime) {
        if ((runtime % 0.25) >0.125) {
            position = spindexer_outtakeBall3 + 0.04;
        } else {
            position = spindexer_outtakeBall3 - 0.04;

        }
    }


    public void outtake3 () {
        position = spindexer_outtakeBall3;
    }

    public void outtake2 () {
        position = spindexer_outtakeBall2;
    }

    public void outtake1 () {
        position = spindexer_outtakeBall1;
    }


    public int outtakeGreen(int secLast, int Last) {
        if (green2 && (secLast!=3) && (Last!=3)) {
            outtake3();
            return 3;
        } else if (green1 && (secLast!=2) && (Last!=2)){
            outtake2();
            return 2;
        } else if (green0 && (secLast!=1) && (Last!=1)) {
            outtake1();
            return 1;
        } else {

            if (secLast!=1 && Last!= 1){
                outtake1();
                return 1;
            } else if (secLast!=2 && Last!=2){
                outtake2();
                return 2;
            } else {
                outtake3();
                return 3;
            }

        }
    }



    public void outtakeGreenFs() {
        if (green0 && ball0) {
            outtake1();
        } else if (green1 && ball1){
            outtake2();
        } else if (green2 && ball2) {
            outtake3();
        }
    }

    public int greens() {
        int num = 0;

        if (green0){num++;}

        if (green1){num++;}


        if (green2){num++;}

        return num;


    }


    public int outtakePurple(int secLast, int Last) {
        if (!green2 && (secLast!=3) && (Last!=3)) {
            outtake3();
            return 3;
        } else if (!green1 && (secLast!=2) && (Last!=2)){
            outtake2();
            return 2;
        } else if (!green0 && (secLast!=1) && (Last!=1)) {
            outtake1();
            return 1;
        } else {

            if (secLast!=1 && Last!= 1){
                outtake1();
                return 1;
            } else if (secLast!=2 && Last!=2){
                outtake2();
                return 2;
            } else {
                outtake3();
                return 3;
            }


        }
    }




    @Override
    public void update() {

        if (position !=0.501) {

            s1.setPosition(position);
            s2.setPosition(1 - position);
        }


        if (telemetryOn) {
            colorSensorTelemetry();
        }



    }
}
