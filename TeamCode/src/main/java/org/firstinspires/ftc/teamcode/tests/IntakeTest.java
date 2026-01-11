package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    Servos servo;
    public boolean green1 = false;
    public boolean green2 = false;
    public boolean green3 = false;
    List<Double> s1G = new ArrayList<>();
    List<Double> s2G = new ArrayList<>();
    List<Double> s3G = new ArrayList<>();
    List<Double> s1T = new ArrayList<>();
    List<Double> s2T = new ArrayList<>();
    List<Double> s3T = new ArrayList<>();
    List<Boolean> s1 = new ArrayList<>();
    List<Boolean> s2 = new ArrayList<>();
    List<Boolean> s3 = new ArrayList<>();
    public static int mode = 0; // 0 for teleop, 1 for auto
    public static double manualPow = 0.15;
    double stamp = 0;
    int ticker = 0;
    boolean steadySpin = false;
    double powPID = 0.0;
    boolean intake = true;
    double spindexerPos = spindexer_intakePos1;
    boolean wasMoving = false;
    double currentPos = 0.0;
    double initPos = 0.0;
    boolean reverse = false;
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        robot = new Robot(hardwareMap);
        servo = new Servos(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (mode == 0) {
                if (gamepad1.right_bumper) {
                    ticker++;
                    if (ticker % 16 == 0){
                        currentPos = servo.getSpinPos();
                        if (Math.abs(currentPos - initPos) == 0.0){
                            reverse = !reverse;
                        }
                        initPos = currentPos;
                    }
                    if (reverse){
                        robot.spin1.setPower(manualPow);
                        robot.spin2.setPower(-manualPow);
                    } else {
                        robot.spin1.setPower(-manualPow);
                        robot.spin2.setPower(manualPow);
                    }
                    robot.intake.setPower(1);
                    stamp = getRuntime();
                    TELE.addData("Reverse?", reverse);
                    TELE.update();
                } else {
                    robot.spin1.setPower(0);
                    robot.spin2.setPower(0);

                    if (getRuntime() - stamp < 0.5) {
                        robot.intake.setPower(-1);
                    } else {
                        robot.intake.setPower(0);
                    }

                    ticker = 0;
                }
            } else if (mode == 1) {

                if (gamepad1.right_bumper && intake){
                    robot.intake.setPower(1);
                } else if (gamepad1.left_bumper){
                    robot.intake.setPower(-1);
                } else {
                    robot.intake.setPower(0);
                }

                colorDetect();
                spindexer();

                if (ballIn(1) && steadySpin && intake && getRuntime() - stamp > 0.5){
                    if (!ballIn(2)){
                        if (servo.spinEqual(spindexer_intakePos1)){
                            spindexerPos = spindexer_intakePos2;
                        } else if (servo.spinEqual(spindexer_intakePos2)){
                            spindexerPos = spindexer_intakePos3;
                        } else if (servo.spinEqual(spindexer_intakePos3)){
                            spindexerPos = spindexer_intakePos1;
                        }
                    } else if (!ballIn(3)){
                        if (servo.spinEqual(spindexer_intakePos1)){
                            spindexerPos = spindexer_intakePos3;
                        } else if (servo.spinEqual(spindexer_intakePos2)){
                            spindexerPos = spindexer_intakePos1;
                        } else if (servo.spinEqual(spindexer_intakePos3)){
                            spindexerPos = spindexer_intakePos2;
                        }
                    }
                }

            } else if (mode == 2){ // switch to this mode before switching modes or to reset balls
                powPID = 0;
                spindexerPos = spindexer_intakePos1;
                stamp = getRuntime();
                ticker = 0;
                spindexer();
                intake = true;
            }
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);

            TELE.addData("Manual Power", manualPow);
            TELE.addData("PID Power", powPID);
            TELE.addData("B1", ballIn(1));
            TELE.addData("B2", ballIn(2));
            TELE.addData("B3", ballIn(3));
            TELE.addData("Spindex Pos", servo.getSpinPos());
            TELE.update();
        }
    }

    public void colorDetect() {
        double s1D = robot.color1.getDistance(DistanceUnit.MM);
        double s2D = robot.color2.getDistance(DistanceUnit.MM);
        double s3D = robot.color3.getDistance(DistanceUnit.MM);

        TELE.addData("Color 1 Distance", s1D);
        TELE.addData("Color 2 Distance", s2D);
        TELE.addData("Color 3 Distance", s3D);
        TELE.update();

        if (s1D < 43) {
            s1T.add(getRuntime());
        }

        if (s2D < 60) {
            s2T.add(getRuntime());
        }

        if (s3D < 33) {
            s3T.add(getRuntime());
        }
    }

    public void spindexer() {
        boolean atTarget = servo.spinEqual(spindexerPos);

        if (!atTarget) {
            powPID = servo.setSpinPos(spindexerPos);
            robot.spin1.setPower(powPID);
            robot.spin2.setPower(-powPID);

            steadySpin = false;
            wasMoving = true;   // remember we were moving
            stamp = getRuntime();
        } else {
            robot.spin1.setPower(0);
            robot.spin2.setPower(0);
            steadySpin = true;
            wasMoving = false;
        }
    }


    boolean ballIn(int slot) {
        List<Double> times;

        if (slot == 1) {times = s1T;}
        else if (slot == 2) {times = s2T;}
        else if (slot == 3) {times = s3T;}
        else return false;

        if (!times.isEmpty()){
            return times.get(times.size() - 1) > getRuntime() - 2;
        } else {
            return false;
        }
    }

}
