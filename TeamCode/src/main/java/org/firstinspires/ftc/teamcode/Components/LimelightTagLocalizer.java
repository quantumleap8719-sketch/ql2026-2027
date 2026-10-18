package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Returns the robot's x / y / heading offset relative to a single AprilTag,
 * using the Limelight's per-tag "robot pose in target space".
 *
 * This does NOT need a field map, so it works with any 36h11 tag, but the
 * Limelight pipeline must have Full 3D turned on (web UI, Advanced tab), the
 * correct tag size, and the camera's position on the robot filled in.
 *
 * Output frame (relative to the tag):
 *   forward  = how far the robot center is out from the tag face (always positive)
 *   lateral  = how far the robot center is to the side of the tag's centerline
 *   heading  = 0 when the robot faces the tag squarely
 */
public class LimelightTagLocalizer {

    /** Which Pose3D angle is rotation about the vertical axis in target space. */
    public enum HeadingAxis { YAW, PITCH, ROLL }

    // ---- Set these after running "Limelight Tag Localization" (see its comments) ----
    public static HeadingAxis HEADING_AXIS = HeadingAxis.YAW;
    public static double LATERAL_SIGN = 1.0;   // flip to -1 if strafing right makes lateral go negative
    public static double HEADING_SIGN = 1.0;   // flip to -1 if turning left (CCW) makes heading go negative
    public static double MAX_STALENESS_MS = 100;

    public static class TagOffset {
        public final int tagId;
        public final double forwardM;
        public final double lateralM;
        public final double headingDeg;

        // Raw target-space values, for checking signs and axes.
        public final double rawX, rawY, rawZ;
        public final double rawYaw, rawPitch, rawRoll;

        TagOffset(int tagId, double forwardM, double lateralM, double headingDeg,
                  double rawX, double rawY, double rawZ,
                  double rawYaw, double rawPitch, double rawRoll) {
            this.tagId = tagId;
            this.forwardM = forwardM;
            this.lateralM = lateralM;
            this.headingDeg = headingDeg;
            this.rawX = rawX;
            this.rawY = rawY;
            this.rawZ = rawZ;
            this.rawYaw = rawYaw;
            this.rawPitch = rawPitch;
            this.rawRoll = rawRoll;
        }

        public double forwardIn() { return forwardM / 0.0254; }
        public double lateralIn() { return lateralM / 0.0254; }
        public double headingRad() { return Math.toRadians(headingDeg); }
    }

    private final Limelight3A limelight;
    private String status = "Not started";

    public LimelightTagLocalizer(HardwareMap hardwareMap, String deviceName, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
    }

    public void start() { limelight.start(); }

    public void stop() { limelight.stop(); }

    public Limelight3A getLimelight() { return limelight; }

    /** Why the last getOffset() call returned null (or "OK"). */
    public String getStatus() { return status; }

    /**
     * @param desiredTagId tag ID to use, or -1 for the closest tag in view
     * @return the offset, or null if there is no usable tag (see getStatus())
     */
    public TagOffset getOffset(int desiredTagId) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            status = "No tag in view";
            return null;
        }

        double staleness = result.getStaleness();
        if (staleness > MAX_STALENESS_MS) {
            status = "Result is stale (" + staleness + " ms)";
            return null;
        }

        LLResultTypes.FiducialResult best = null;
        Position bestPos = null;
        YawPitchRollAngles bestRot = null;
        double bestDist = Double.MAX_VALUE;
        boolean sawZeroPose = false;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (desiredTagId != -1 && tag.getFiducialId() != desiredTagId) continue;

            Pose3D pose = tag.getRobotPoseTargetSpace();
            if (pose == null) continue;

            Position pos = pose.getPosition().toUnit(DistanceUnit.METER);
            double dist = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

            // An all-zero pose means the pipeline isn't producing 3D data.
            if (dist < 1e-6) {
                sawZeroPose = true;
                continue;
            }

            if (dist < bestDist) {
                bestDist = dist;
                best = tag;
                bestPos = pos;
                bestRot = pose.getOrientation();
            }
        }

        if (best == null) {
            if (sawZeroPose) {
                status = "Tag seen but 3D pose is all zeros: turn on Full 3D "
                        + "(Advanced tab) and set the tag size and camera pose";
            } else if (desiredTagId != -1) {
                status = "Tag " + desiredTagId + " not in view";
            } else {
                status = "No tag in view";
            }
            return null;
        }

        double yaw = bestRot.getYaw(AngleUnit.DEGREES);
        double pitch = bestRot.getPitch(AngleUnit.DEGREES);
        double roll = bestRot.getRoll(AngleUnit.DEGREES);

        double rawHeading;
        switch (HEADING_AXIS) {
            case PITCH: rawHeading = pitch; break;
            case ROLL:  rawHeading = roll;  break;
            default:    rawHeading = yaw;   break;
        }

        // Target space: +X = tag's right, +Y = down, +Z = perpendicular to the tag face.
        // The robot can only see the tag from in front, so |Z| is the forward distance.
        double forward = Math.abs(bestPos.z);
        double lateral = LATERAL_SIGN * bestPos.x;
        double heading = normalizeDeg(HEADING_SIGN * rawHeading);

        status = "OK";
        return new TagOffset(best.getFiducialId(), forward, lateral, heading,
                bestPos.x, bestPos.y, bestPos.z, yaw, pitch, roll);
    }

    private static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}

