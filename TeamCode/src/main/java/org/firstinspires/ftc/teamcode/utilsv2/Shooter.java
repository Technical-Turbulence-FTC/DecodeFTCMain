package org.firstinspires.ftc.teamcode.utilsv2;

import static org.firstinspires.ftc.teamcode.utilsv2.Flywheel.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.constants.Color;

@Config
public class Shooter {

    Robot robot;
    Flywheel fly;
    Turret turr;
    VelocityCommander commander;

    double goalX = 0.0;
    double goalY = 0.0;
    double obeliskX = 72;
    double obeliskY = 144;
    double turretGoalX = 0;
    double turretGoalY = 0;

    private boolean red = true;
    public static boolean manualFlywheel = false;


    Follower follow;

    public Shooter(Robot rob, MultipleTelemetry TELE, Follower follower, boolean redAlliance, Turret turret, Flywheel flywheel, VelocityCommander com) {
        this.robot = rob;
        this.fly = flywheel;
        this.turr = turret;
        this.follow = follower;
        this.commander = com;

        setRedAlliance(redAlliance);

    }

    public void setRedAlliance(boolean input) {
        this.red = input;

        if (this.red) {
            goalX = 72;
            turretGoalX = 68;
        } else {
            goalX = -72;
            turretGoalX = -68;
        }
        goalY = 72;
        turretGoalY = 68;
    }

    private double flywheelVelocity = 0.0;
    private double turretPosition = 0.5;

    public enum ShooterState {
        READ_OBELISK,
        TRACK_GOAL,
        MANUAL_FLYWHEEL_TRACK_TURR,
        MANUAL_TURRET_TRACK_FLY,
        MANUAL,
        NOTHING
    }


    private ShooterState state = ShooterState.NOTHING;

    public void setState(ShooterState shooterState) {
        this.state = shooterState;
    }

    public ShooterState getState(){
        return state;
    }

    public void setTurretPosition(double input) {
        this.turretPosition = input;
    }

    public void setFlywheelVelocity(double input) {
        this.flywheelVelocity = input;
    }

    public int getObeliskID() {
        return turr.getObeliskID();
    }

    private final double shooterDistFromCenter = 1.545;
    public void update(double voltage) {
        setRedAlliance(Color.redAlliance);
        switch (state) {
            case NOTHING:
                break;
            case MANUAL:
                manualFlywheel = true;
                commander.getVeloPredictive(
                        (goalX - follow.getPose().getX() - shooterDistFromCenter*Math.cos(follow.getHeading())),
                        (goalY - follow.getPose().getY() - shooterDistFromCenter*Math.sin(follow.getHeading())),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent(),
                        voltage,
                        fly.getAverageVelocity()
                );

                fly.manageFlywheel(flywheelVelocity);
                fly.setPIDF(shooterPIDF_P, shooterPIDF_I, shooterPIDF_D, shooterPIDF_F / voltage);
                turr.manual(turretPosition);
                break;
            case TRACK_GOAL:
                manualFlywheel = false;
                turr.trackGoal(
                        (turretGoalX - follow.getPose().getX() - shooterDistFromCenter*Math.cos(follow.getHeading())),
                        (turretGoalY - follow.getPose().getY() - shooterDistFromCenter*Math.sin(follow.getHeading())),
                        follow.getHeading(),
                        follow.getAngularVelocity(),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                commander.getVeloPredictive(
                        (goalX - follow.getPose().getX() - shooterDistFromCenter*Math.cos(follow.getHeading())),
                        (goalY - follow.getPose().getY() - shooterDistFromCenter*Math.sin(follow.getHeading())),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent(),
                        voltage,
                        fly.getAverageVelocity()
                );

                flywheelVelocity = commander.getPredictedRPM();

                robot.setHoodPos(commander.getHoodPredicted());
                fly.manageFlywheel(flywheelVelocity);
                fly.setF(voltage);
                break;
            case READ_OBELISK:
                manualFlywheel = false;
                robot.setTurretPos(Turret.neutralPosition);

                turr.detectObelisk();

                commander.getVeloPredictive(
                        (goalX - follow.getPose().getX() - shooterDistFromCenter*Math.cos(follow.getHeading())),
                        (goalY - follow.getPose().getY() - shooterDistFromCenter*Math.sin(follow.getHeading())),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent(),
                        voltage,
                        fly.getAverageVelocity()
                );

                flywheelVelocity = commander.getPredictedRPM();

                fly.manageFlywheel(0);
                fly.setF(voltage);
                break;

            case MANUAL_TURRET_TRACK_FLY:
                manualFlywheel = false;
                turr.manual(turretPosition);
                commander.getVeloPredictive(
                        (goalX - follow.getPose().getX() - shooterDistFromCenter*Math.cos(follow.getHeading())),
                        (goalY - follow.getPose().getY() - shooterDistFromCenter*Math.sin(follow.getHeading())),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent(),
                        voltage,
                        fly.getAverageVelocity()
                );

                flywheelVelocity = commander.getPredictedRPM();
                robot.setHoodPos(commander.getHoodPredicted());

                fly.manageFlywheel(flywheelVelocity);
                break;

            case MANUAL_FLYWHEEL_TRACK_TURR:
                manualFlywheel = true;
                turr.trackGoal(
                        (turretGoalX - follow.getPose().getX() - shooterDistFromCenter*Math.cos(follow.getHeading())),
                        (turretGoalY - follow.getPose().getY() - shooterDistFromCenter*Math.sin(follow.getHeading())),
                        follow.getHeading(),
                        follow.getAngularVelocity(),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );
                commander.getVeloPredictive(
                        (goalX - follow.getPose().getX() - shooterDistFromCenter*Math.cos(follow.getHeading())),
                        (goalY - follow.getPose().getY() - shooterDistFromCenter*Math.sin(follow.getHeading())),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent(),
                        voltage,
                        fly.getAverageVelocity()
                );
                fly.manageFlywheel(flywheelVelocity);
                fly.setPIDF(shooterPIDF_P, shooterPIDF_I, shooterPIDF_D, shooterPIDF_F / voltage);
                fly.setF(voltage);
                break;

        }

    }

    public double getDistance(){return commander.getDistance();}


}
