package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class LimelightManager {
    private Limelight3A limelight;
    private LLResult lastResult;
    private int lastFiducialId = -1;
    private double lastBearing = 0.0;

    public static final int PIPELINE_DEFAULT = 1;
    public static final int PIPELINE_BLUE_DETECTION = 2;
    public static final int PIPELINE_RED_DETECTION = 3;

    public enum LimelightMode {
        OBELISK_DETECTION(PIPELINE_DEFAULT),
        BLUE_GOAL(PIPELINE_BLUE_DETECTION),
        RED_GOAL(PIPELINE_RED_DETECTION);

        public final int pipeline;

        LimelightMode(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    public LimelightManager(HardwareMap hardwareMap, boolean enabled) {
        if (enabled) {
            this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        }
    }

    public void init() {
        if (limelight != null) {
            limelight.start();
        }
    }

    public void switchMode(LimelightMode mode) {
        if (limelight != null) {
            limelight.pipelineSwitch(mode.pipeline);
        }
    }

    public void setPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    public void update() {
        if (limelight != null) {
            lastResult = limelight.getLatestResult();
            if (lastResult != null && lastResult.isValid()) {
                lastBearing = lastResult.getTx();
            }
        }
    }

    public double getBearing() {
        return lastBearing;
    }

    public int detectFiducial() {
        if (lastResult != null && lastResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = lastResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                lastFiducialId = fiducials.get(0).getFiducialId();
                return lastFiducialId;
            }
        }
        return -1;
    }

    public int getLastFiducialId() {
        return lastFiducialId;
    }

    public boolean isFiducialDetected(int id) {
        return lastFiducialId == id;
    }

    public LLResult getLatestResult() {
        return lastResult;
    }

    public boolean isConnected() {
        return limelight != null;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
