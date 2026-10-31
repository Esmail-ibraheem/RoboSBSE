package org.robosbse.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.robosbse.core.Params;
import org.robosbse.geometry.Circle;
import org.robosbse.geometry.Rect;
import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;
import org.robosbse.simulation.controllers.Controller;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.simulation.controllers.ControllerSetup;

class SimulatorTest {

    private static final RobotSpec ROBOT = RobotSpec.DEFAULT;

    private static World world(List<Shape> obstacles) {
        return new World(new WorldMap(null, "t", null, 20, 10, new Pose(2, 5, 0), new Vec2(18, 5), 0.3, obstacles));
    }

    private static EpisodeResult run(World w, Controller c, Vec2 goal, EpisodeConfig cfg) {
        ControllerSetup setup = new ControllerSetup(ROBOT, cfg.dt(), goal, 0.3, List.of(goal), Params.empty());
        return Simulator.run(w, ROBOT, new Pose(2, 5, 0), goal, 0.3, c, setup, cfg);
    }

    /** Controller that always issues the same command. */
    private static Controller constant(double v, double omega) {
        return scripted(() -> Command.drive(v, omega));
    }

    private static Controller scripted(Supplier<Command> commands) {
        return new Controller() {
            @Override
            public String id() {
                return "scripted";
            }

            @Override
            public void reset(ControllerSetup setup) {
            }

            @Override
            public Command step(Observation observation) {
                return commands.get();
            }
        };
    }

    @Test
    void straightDriveReachesTheGoalWithKinematicLimits() {
        EpisodeResult r = run(world(List.of()), constant(1.0, 0), new Vec2(12, 5), EpisodeConfig.defaults().recording(1, false));
        assertEquals(Outcome.REACHED_GOAL, r.outcome());
        // 1 s to accelerate (0.5 m), then 1 m/s: reach x = 11.7 after about 9.7 s.
        assertEquals(10.2, r.time(), 0.06);
        Trajectory t = r.trajectory();
        double[] v = t.speeds();
        for (int i = 1; i < v.length; i++) {
            assertTrue(v[i] <= ROBOT.maxSpeed() + 1e-9);
            assertTrue(v[i] - v[i - 1] <= ROBOT.maxAcceleration() * 0.05 + 1e-9, "acceleration limit violated");
        }
        assertEquals(5.0, t.ys()[t.size() - 1], 1e-9, "no lateral drift on a straight drive");
    }

    @Test
    void collisionWithAnObstacleEndsTheEpisode() {
        EpisodeResult r = run(world(List.of(new Rect(6, 3, 1, 4))), constant(1.0, 0), new Vec2(18, 5), EpisodeConfig.defaults());
        assertEquals(Outcome.COLLISION, r.outcome());
        assertTrue(r.minObstacleClearance() < 0);
        assertEquals(1, r.collisions());
        assertEquals(5.75, r.finalPose().x(), 0.06, "stops where the body touches the box");
    }

    @Test
    void blockedModeCountsCollisionsWithoutEndingTheEpisode() {
        EpisodeConfig cfg = new EpisodeConfig(0.05, 400, true, false, 0, 0.1, 0, 0, 1, 1, false, 1, false);
        EpisodeResult r = run(world(List.of(new Rect(6, 3, 1, 4))), constant(1.0, 0), new Vec2(18, 5), cfg);
        assertEquals(Outcome.TIMEOUT, r.outcome());
        assertTrue(r.collisions() >= 1);
        assertTrue(r.finalPose().x() < 5.76);
    }

    @Test
    void leavingTheWorldIsDetected() {
        EpisodeResult r = run(world(List.of()), constant(1.0, 0), new Vec2(30, 5), EpisodeConfig.defaults());
        assertEquals(Outcome.OUT_OF_BOUNDS, r.outcome());
        assertTrue(r.minBorderClearance() < 0);
    }

    @Test
    void spinningInPlaceIsADeadlock() {
        EpisodeResult r = run(world(List.of()), constant(0, 1.0), new Vec2(18, 5), EpisodeConfig.defaults());
        assertEquals(Outcome.DEADLOCK, r.outcome());
        assertEquals(8.0, r.time(), 0.1);
    }

    @Test
    void batteryDepletionStopsTheRobot() {
        EpisodeResult r = run(world(List.of()), constant(1.0, 0), new Vec2(18, 5), EpisodeConfig.defaults().withBattery(0.01));
        assertEquals(Outcome.BATTERY_DEPLETED, r.outcome());
        assertEquals(15.0, r.energy(), 1.0, "1% of 1500 J, plus at most one step");
    }

    @Test
    void invalidCommandsAndExceptionsAreReported() {
        assertEquals(Outcome.INVALID_COMMAND,
                run(world(List.of()), constant(Double.NaN, 0), new Vec2(18, 5), EpisodeConfig.defaults()).outcome());
        EpisodeResult crash = run(world(List.of()), scripted(() -> {
            throw new IllegalStateException("boom");
        }), new Vec2(18, 5), EpisodeConfig.defaults());
        assertEquals(Outcome.CONTROLLER_ERROR, crash.outcome());
        assertTrue(crash.error().contains("boom"));
    }

    @Test
    void completionReportsAreCheckedAgainstTheGoal() {
        EpisodeResult early = run(world(List.of()), scripted(Command::complete), new Vec2(18, 5),
                EpisodeConfig.defaults().withAutoComplete(false));
        assertEquals(Outcome.FALSE_COMPLETION, early.outcome());
        assertEquals(16.0, early.reportedDoneDistance(), 1e-9);
    }

    @Test
    void commandsBeyondTheLimitsAreCountedAndSaturated() {
        EpisodeResult r = run(world(List.of()), constant(2.0, 0), new Vec2(12, 5), EpisodeConfig.defaults());
        assertTrue(r.commandBoundViolations() > 0);
        assertEquals(2.0, r.maxCommandedSpeed(), 1e-12);
        assertEquals(Outcome.REACHED_GOAL, r.outcome());
        assertEquals(10.2, r.time(), 0.06, "actuators saturate at the robot maximum");
    }

    @Test
    void sensorReadingsMatchGeometryWithoutNoise() {
        double[][] seen = new double[1][];
        Controller probe = new Controller() {
            @Override
            public String id() {
                return "probe";
            }

            @Override
            public boolean usesSensors() {
                return true;
            }

            @Override
            public void reset(ControllerSetup setup) {
            }

            @Override
            public Command step(Observation o) {
                if (seen[0] == null) {
                    seen[0] = o.ranges();
                }
                return Command.stop();
            }
        };
        EpisodeConfig cfg = EpisodeConfig.defaults().withMaxSteps(1);
        run(world(List.of(new Circle(5, 5, 1))), probe, new Vec2(18, 5), cfg);
        assertEquals(2.0, seen[0][0], 1e-9, "ray 0 looks ahead to the circle");
        assertEquals(ROBOT.sensorRange(), seen[0][ROBOT.sensorRays() / 4], 1e-9, "ray at +90 deg: border 5 m away, capped at range");
        assertEquals(2.0, seen[0][ROBOT.sensorRays() / 2], 1e-9, "ray at 180 deg hits the border");
    }

    @Test
    void episodesAreDeterministicIncludingNoiseAndDropout() {
        World w = world(List.of(new Circle(8, 5.4, 0.6), new Rect(12, 3, 0.5, 1.5)));
        EpisodeConfig cfg = EpisodeConfig.defaults().withAutoComplete(false).withSensing(0.03, 0.3, 77).recording(1, true);
        EpisodeResult a = run(w, ControllerRegistry.create("reactive"), new Vec2(18, 5), cfg);
        EpisodeResult b = run(w, ControllerRegistry.create("reactive"), new Vec2(18, 5), cfg);
        assertEquals(a.outcome(), b.outcome());
        assertEquals(a.steps(), b.steps());
        assertEquals(a.droppedScans(), b.droppedScans());
        assertEquals(a.finalPose(), b.finalPose());
        EpisodeResult other = run(w, ControllerRegistry.create("reactive"), new Vec2(18, 5),
                cfg.withSensing(0.03, 0.3, 78));
        assertNotEquals(a.finalPose(), other.finalPose(), "a different noise seed changes the episode");
    }

    @Test
    void commonRandomNumbersMakeDropoutMonotone() {
        World w = world(List.of());
        int previous = -1;
        for (double p : new double[] {0.0, 0.1, 0.2, 0.4, 0.6}) {
            EpisodeConfig cfg = EpisodeConfig.defaults().withMaxSteps(400).withAutoComplete(false).withSensing(0, p, 5);
            Controller c = new Controller() {
                @Override
                public String id() {
                    return "idle";
                }

                @Override
                public boolean usesSensors() {
                    return true;
                }

                @Override
                public void reset(ControllerSetup setup) {
                }

                @Override
                public Command step(Observation o) {
                    return Command.drive(0, 0.1);
                }
            };
            EpisodeConfig noDeadlock = new EpisodeConfig(cfg.dt(), cfg.maxSteps(), false, true, 0, 0.1, 0, p, 1, 5,
                    false, 1, false);
            int dropped = run(w, c, new Vec2(18, 5), noDeadlock).droppedScans();
            assertTrue(dropped >= previous, "dropped scans must not decrease when p grows");
            previous = dropped;
        }
        assertTrue(previous > 150);
    }
}
