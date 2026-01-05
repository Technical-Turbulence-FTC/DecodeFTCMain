package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    Servos servo;

    public static int mode = 0; // 0 for teleop, 1 for auto
    public static double manualPow = 1.0;
    double stamp = 0;
    int ticker = 0;
    boolean b1 = false;
    boolean b2 = false;
    boolean b3 = false;
    boolean steadySpin = false;
    double powPID = 0.0;
    double spindexerPos = spindexer_intakePos1;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        servo = new Servos(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //TODO: test tele intake with new spindexer
            if (mode == 0) {
                if (gamepad1.cross) {
                    ticker = 0;
                    robot.spin1.setPower(manualPow);
                    robot.spin2.setPower(-manualPow);
                    robot.intake.setPower(1);
                } else {
                    robot.spin1.setPower(0);
                    robot.spin2.setPower(0);
                    if (ticker == 0) {
                        stamp = getRuntime();
                    }
                    ticker++;
                    if (getRuntime() - stamp < 0.5) {
                        robot.intake.setPower(-1);
                    } else {
                        robot.intake.setPower(0);
                    }
                }
            //TODO: test this monstrosity
            } else if (mode == 1) {

                if (gamepad1.cross){
                    robot.intake.setPower(1);
                } else if (gamepad1.circle){
                    robot.intake.setPower(-1);
                } else {
                    robot.intake.setPower(0);
                }

                colorDetect();
                spindexer();

                if (b1 && steadySpin && getRuntime() - stamp > 0.5){
                    if (!b2){
                        if (servo.spinEqual(spindexer_intakePos1)){
                            spindexerPos = spindexer_intakePos2;
                        } else if (servo.spinEqual(spindexer_intakePos2)){
                            spindexerPos = spindexer_intakePos3;
                        } else if (servo.spinEqual(spindexer_intakePos3)){
                            spindexerPos = spindexer_intakePos1;
                        }
                    } else if (!b3){
                        if (servo.spinEqual(spindexer_intakePos1)){
                            spindexerPos = spindexer_intakePos3;
                        } else if (servo.spinEqual(spindexer_intakePos2)){
                            spindexerPos = spindexer_intakePos1;
                        } else if (servo.spinEqual(spindexer_intakePos3)){
                            spindexerPos = spindexer_intakePos2;
                        }
                    }
                }

                powPID = servo.setSpinPos(spindexerPos);

            } else if (mode == 2){ // switch to this mode before switching modes
                powPID = 0;
                spindexerPos = spindexer_intakePos1;
                stamp = getRuntime();
                ticker = 0;
            }
            TELE.addData("Manual Power", manualPow);
            TELE.addData("PID Power", powPID);
            TELE.addData("B1", b1);
            TELE.addData("B2", b2);
            TELE.addData("B3", b3);
            TELE.addData("Spindex Pos", servo.getSpinPos());
        }
    }

    public void colorDetect() {
        // ----- COLOR 1 -----
        double green1 = robot.color1.getNormalizedColors().green;
        double blue1 = robot.color1.getNormalizedColors().blue;
        double red1 = robot.color1.getNormalizedColors().red;

        b1 = robot.color1.getDistance(DistanceUnit.MM) < 40;

        TELE.addData("Color1 toColor", robot.color1.getNormalizedColors().toColor());
        TELE.addData("Color1 green", green1 / (green1 + blue1 + red1));
        TELE.addData("Color1 distance (mm)", robot.color1.getDistance(DistanceUnit.MM));

// ----- COLOR 2 -----
        double green2 = robot.color2.getNormalizedColors().green;
        double blue2 = robot.color2.getNormalizedColors().blue;
        double red2 = robot.color2.getNormalizedColors().red;

        b2 = robot.color2.getDistance(DistanceUnit.MM) < 40;

        TELE.addData("Color2 toColor", robot.color2.getNormalizedColors().toColor());
        TELE.addData("Color2 green", green2 / (green2 + blue2 + red2));
        TELE.addData("Color2 distance (mm)", robot.color2.getDistance(DistanceUnit.MM));

// ----- COLOR 3 -----
        double green3 = robot.color3.getNormalizedColors().green;
        double blue3 = robot.color3.getNormalizedColors().blue;
        double red3 = robot.color3.getNormalizedColors().red;

        b3 = robot.color3.getDistance(DistanceUnit.MM) < 30;

        TELE.addData("Color3 toColor", robot.color3.getNormalizedColors().toColor());
        TELE.addData("Color3 green", green3 / (green3 + blue3 + red3));
        TELE.addData("Color3 distance (mm)", robot.color3.getDistance(DistanceUnit.MM));

        TELE.update();
    }

    public void spindexer(){
        if (!servo.spinEqual(spindexerPos)){
            robot.spin1.setPower(powPID);
            robot.spin2.setPower(-powPID);
            steadySpin = false;
            ticker = 0;
        } else{
            robot.spin1.setPower(0);
            robot.spin2.setPower(0);
            steadySpin = true;
            if (ticker == 0){
                stamp = getRuntime();
            }
            ticker++;
        }
    }
}
