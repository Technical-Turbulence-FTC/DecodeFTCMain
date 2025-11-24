package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AprilTag implements Subsystem {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private MultipleTelemetry TELE;

    private boolean teleOn = false;

    private int detections = 0;

    List<AprilTagDetection> currentDetections;

    ArrayList<ArrayList<Double>> Data = new ArrayList<>();



    public AprilTag(Robot robot, MultipleTelemetry tele) {



        this.aprilTag = robot.aprilTagProcessor;


        this.TELE = tele;




    }

    @Override
    public void update() {

        currentDetections = aprilTag.getDetections();

        UpdateData();

        if(teleOn){
            tagTELE();
            initTelemetry();
        }




    }

    public void initTelemetry (){

        TELE.addData("Camera Preview", "Check Driver Station for stream");
        TELE.addData("Status", "Initialized - Press START");



    }

    public void tagTELE () {

        TELE.addData("# AprilTags Detected", detections);

        // Display info for each detected tag
        for (ArrayList<Double> detection : Data) {
            if (detection.get(0) ==1) {
                // Known AprilTag with metadata
                TELE.addLine(String.format("\n==== (ID %d) %s ====",
                        detection.get(1).intValue(), ""));

                TELE.addLine(String.format("XYZ: %6.1f %6.1f %6.1f  (inch)",
                        detection.get(2),
                        detection.get(3),
                        detection.get(4)));

                TELE.addData("Distance", getDistance(detection.get(1).intValue()));

                TELE.addLine(String.format("PRY: %6.1f %6.1f %6.1f  (deg)",
                        detection.get(5),
                        detection.get(6),
                        detection.get(7)));

                TELE.addLine(String.format("RBE: %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.get(8),
                        detection.get(9),
                        detection.get(10)));
            }
        }
    }


    public void turnTelemetryOn(boolean bool) {

        teleOn = bool;

    }



    public void UpdateData () {

        Data.clear();  // <--- THIS FIXES YOUR ISSUE


        detections = currentDetections.size();


        for (AprilTagDetection detection : currentDetections) {

            ArrayList<Double> detectionData = new ArrayList<Double>();



            if (detection.metadata != null) {

                detectionData.add(1.0);
                // Known AprilTag with metadata

                detectionData.add( (double) detection.id);



                detectionData.add(detection.ftcPose.x);
                detectionData.add(detection.ftcPose.y);
                detectionData.add(detection.ftcPose.z);



                detectionData.add(detection.ftcPose.pitch);
                detectionData.add(detection.ftcPose.roll);
                detectionData.add(detection.ftcPose.yaw);

                detectionData.add(detection.ftcPose.range);
                detectionData.add(detection.ftcPose.bearing);
                detectionData.add(detection.ftcPose.elevation);

            } else {

                detectionData.add(0, 0.0);

            }

            Data.add(detectionData);
        }

    }

    public int getDetectionCount() {

        return detections;

    }

    public boolean isDetected (int id){
        return (!filterID(Data, (double) id ).isEmpty());
    }



    public double getDistance(int id) {
        ArrayList<Double> d = filterID(Data, (double) id);
        if (d.size() >= 5) {
            double x = d.get(2);
            double y = d.get(3);
            double z = d.get(4);
            return Math.sqrt(x*x + y*y + z*z);
        }
        return -1; // tag not found
    }

    // Returns the position as [x, y, z]
    public List<Double> getPosition(int id) {
        ArrayList<Double> d = filterID(Data, (double) id);
        if (d.size() >= 5) {
            List<Double> pos = new ArrayList<>();
            pos.add(d.get(2));
            pos.add(d.get(3));
            pos.add(d.get(4));
            return pos;
        }
        return Collections.emptyList();
    }

    // Returns orientation as [pitch, roll, yaw]
    public List<Double> getOrientation(int id) {
        ArrayList<Double> d = filterID(Data, (double) id);
        if (d.size() >= 8) {
            List<Double> ori = new ArrayList<>();
            ori.add(d.get(5));
            ori.add(d.get(6));
            ori.add(d.get(7));
            return ori;
        }
        return Collections.emptyList();
    }

    // Returns range, bearing, elevation as [range, bearing, elevation]
    public List<Double> getRBE(int id) {
        ArrayList<Double> d = filterID(Data, (double) id);
        if (d.size() >= 11) {
            List<Double> rbe = new ArrayList<>();
            rbe.add(d.get(8));
            rbe.add(d.get(9));
            rbe.add(d.get(10));
            return rbe;
        }
        return Collections.emptyList();
    }

    // Returns full raw data for debugging or custom processing
    public ArrayList<Double> getRawData(int id) {
        return filterID(Data, (double) id);
    }

    public static ArrayList<Double> filterID(ArrayList<ArrayList<Double>> data, double x) {
        for (ArrayList<Double> innerList : data) {
            // Ensure it has a second element
            if (innerList.size() > 1 && Math.abs(innerList.get(1) - x) < 1e-9) {
                return innerList; // Return the first match
            }
        }
        // Return an empty ArrayList if no match found
        return new ArrayList<>();
    }

}
