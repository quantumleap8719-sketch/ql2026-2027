package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

import static com.pedropathing.api.Paths.*;

@Autonomous(name = "Crazy")
public class BlueAuto extends LinearOpMode {

    // ---------------------------------------------------------------
    // Route: one entry per leg, run in order. Each leg goes through
    // the same three phases: FOLLOW -> WAIT -> PAUSE, then the next leg.
    // To reorder or drop a leg, just edit this enum.
    // ---------------------------------------------------------------
    private enum Leg {
        TO_FRONT_SHOOT,     // start        -> frontShoot  (line)   shoot preload
        TO_PRE_INTAKE,      // frontShoot   -> preIntake   (line)   intake
        RETURN_TO_SHOOT,    // preIntake    -> frontShoot  (line)   shoot
        TO_INT_FLOWER,      // frontShoot   -> intFlower   (CURVE)  intake
        TO_BACK_SHOOT,      // intFlower    -> backShoot   (line)   shoot
        TO_POST_INTAKE,     // backShoot    -> postIntake  (line)   finish
        DONE
    }

    private enum Phase { FOLLOW, WAIT, PAUSE }

    private Leg leg = Leg.TO_FRONT_SHOOT;
    private Phase phase = Phase.FOLLOW;

    private TelemetryManager telemetryM;
    private Follower follower;
    private Timer timer;

    private static final double PATH_TIMEOUT = 5.0;   // seconds per leg before giving up
    private static final double STATE_DELAY = 0.2;    // pause after each leg (shoot/intake goes here)

    // Fail-safes: move on when "close enough", even if follower.isBusy() is still true.
    private static final double POS_TOLERANCE = 1.5;                         // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(5);       // radians
    private static final double TIMEOUT_POS_TOLERANCE = 4.0;                 // inches; looser check at timeout

    private boolean timedOut = false;
    private String failedPath = "None";
    private String arriveReason = "-";

    // ---------------- Poses ----------------
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(85, 132, 270);
    private final Pose frontShoot = p.of(85, 107, 270);
    private final Pose preIntake = p.of(104.8, 133.6, 0);
    private final Pose postIntake = p.of(0, 24, 180);
    private final Pose intFlower = p.of(94, 10.6, 270);
    private final Pose firstFloControl = p.of(77, 54, 0);
    private final Pose secondFloControl = p.of(94, 29.1, 0);
    private final Pose backShoot = p.of(84.1, 33.5, 90);

    // ---------------- Paths (built once in init) ----------------
    private Path toFrontShoot, toPreIntake, returnToShoot, toIntFlower, toBackShoot, toPostIntake;

    private void buildPaths() {
        // Straight lines, heading turns smoothly from the start pose's heading to the end pose's.
        toFrontShoot  = line(start, frontShoot).linear(start, frontShoot);
        toPreIntake   = line(frontShoot, preIntake).linear(frontShoot, preIntake);
        returnToShoot = line(preIntake, frontShoot).linear(preIntake, frontShoot);

        // The only curve: Bezier through the two flower control points.
        toIntFlower   = curve(frontShoot, firstFloControl, secondFloControl, intFlower)
                .linear(frontShoot, intFlower);

        toBackShoot   = line(intFlower, backShoot).linear(intFlower, backShoot);
        toPostIntake  = line(backShoot, postIntake).linear(backShoot, postIntake);
    }

    private Path pathFor(Leg l) {
        switch (l) {
            case TO_FRONT_SHOOT:  return toFrontShoot;
            case TO_PRE_INTAKE:   return toPreIntake;
            case RETURN_TO_SHOOT: return returnToShoot;
            case TO_INT_FLOWER:   return toIntFlower;
            case TO_BACK_SHOOT:   return toBackShoot;
            case TO_POST_INTAKE:  return toPostIntake;
            default:              return null;
        }
    }

    /** Where each leg should end (used for the close-enough checks). */
    private Pose targetFor(Leg l) {
        switch (l) {
            case TO_FRONT_SHOOT:  return frontShoot;
            case TO_PRE_INTAKE:   return preIntake;
            case RETURN_TO_SHOOT: return frontShoot;
            case TO_INT_FLOWER:   return intFlower;
            case TO_BACK_SHOOT:   return backShoot;
            case TO_POST_INTAKE:  return postIntake;
            default:              return null;
        }
    }

    private double distanceTo(Pose target) {
        return follower.pose().distance(target);
    }

    private double headingErrorTo(Pose target) {
        return Math.abs(Angle.error(follower.pose().heading(), target.heading()));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        timer = new Timer();

        buildPaths();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();

        while (opModeIsActive()) {

            follower.update();

            if (leg == Leg.DONE) {
                follower.manual(0, 0, 0);
            } else {
                switch (phase) {

                    case FOLLOW:
                        follower.follow(pathFor(leg));
                        setPhase(Phase.WAIT);
                        break;

                    case WAIT: {
                        Pose target = targetFor(leg);
                        double dist = distanceTo(target);
                        boolean closeEnough = dist <= POS_TOLERANCE
                                && headingErrorTo(target) <= HEADING_TOLERANCE;
                        boolean timeUp = timer.get(TimeUnit.SECONDS) >= PATH_TIMEOUT;

                        if (!follower.isBusy()) {
                            arrive("follower done");
                        } else if (closeEnough) {
                            // Fail-safe 1: still "busy" but already at the target
                            arrive("close enough");
                        } else if (timeUp && dist <= TIMEOUT_POS_TOLERANCE) {
                            // Fail-safe 2: timed out, but near enough to keep the auto going
                            arrive("timeout, near target");
                        } else if (timeUp) {
                            // Fail-safe 3: timed out and far away, so stop safely
                            fail(leg.name());
                        }
                        break;
                    }

                    case PAUSE:
                        if (timer.get(TimeUnit.SECONDS) >= STATE_DELAY) {
                            nextLeg();
                        }
                        break;
                }
            }

            Pose current = follower.pose();
            telemetryM.addData("X", current.x());
            telemetryM.addData("Y", current.y());
            telemetryM.addData("Heading", current.heading());
            telemetryM.addData("Leg", leg);
            telemetryM.addData("Phase", phase);
            telemetryM.addData("Follower Busy", follower.isBusy());
            telemetryM.addData("Timer (s)", timer.get(TimeUnit.SECONDS));
            telemetryM.addData("Timed Out", timedOut);
            telemetryM.addData("Failed Path", failedPath);
            telemetryM.addData("Last Arrival", arriveReason);
            telemetryM.addData("Distance To Path End", follower.distanceToEndpoint());
            telemetryM.update();
        }
    }

    /** Hook for mechanism actions when a leg finishes (shooter, intake, etc.). */
    private void onArrive(Leg arrived) {
        switch (arrived) {
            case TO_FRONT_SHOOT:
            case RETURN_TO_SHOOT:
            case TO_BACK_SHOOT:
                // TODO: shoot
                break;
            case TO_PRE_INTAKE:
            case TO_INT_FLOWER:
                // TODO: intake
                break;
            default:
                break;
        }
    }

    private void arrive(String reason) {
        arriveReason = leg.name() + ": " + reason;
        onArrive(leg);
        setPhase(Phase.PAUSE);
    }

    private void nextLeg() {
        leg = Leg.values()[leg.ordinal() + 1];
        setPhase(Phase.FOLLOW);
    }

    private void setPhase(Phase newPhase) {
        phase = newPhase;
        timer.reset();
    }

    private void fail(String pathName) {
        timedOut = true;
        failedPath = pathName;
        leg = Leg.DONE;
        setPhase(Phase.FOLLOW);
    }
}


