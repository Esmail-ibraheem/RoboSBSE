import java.nio.file.Path;

import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.core.Params;
import org.robosbse.experiments.Problems;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.Scenario;

/**
 * Varies one factor at a time for a recorded collision scenario of the correct reactive controller: sensor
 * noise, scan dropout, both, 64 instead of 24 rays, and the controller before the perception fix
 * (docs/experiments/perception-fix.md).
 *
 * <pre>
 *   java -cp backend/java-engine/build/libs/robosbse-engine.jar scripts/perception/ScenarioFactors.java \
 *       examples/faulty-controllers/known-issues/reactive-residual-19.json
 * </pre>
 */
public class ScenarioFactors {

    static ControllerTestingProblem testing(Params controllerParams, RobotSpec robot) {
        return new ControllerTestingProblem(MapLibrary.builtin("testing-arena"),
                Params.of("controller", "reactive", "properties", "safety", "fitness", "collision", "maxSteps", 1200,
                        "simulationSeed", 1), controllerParams, robot);
    }

    static String run(ControllerTestingProblem t, Scenario s) {
        var ex = t.execute(s, "reactive", false);
        return ex.violated().isEmpty() ? "ok (" + ex.result().outcome().name().toLowerCase() + ")"
                : ex.violated() + String.format(" clearance %.4f m", ex.result().minObstacleClearance());
    }

    public static void main(String[] args) {
        for (String file : args) {
            Scenario s = Json.convert(Json.read(Path.of(file)).get("scenario"), Scenario.class);
            Scenario quiet = new Scenario(s.start(), s.goal(), 0, s.scanDropout(), s.battery(), s.speedLimit(), s.freeDistance(), s.obstacles());
            Scenario noDrop = new Scenario(s.start(), s.goal(), s.sensorNoise(), 0, s.battery(), s.speedLimit(), s.freeDistance(), s.obstacles());
            Scenario neither = new Scenario(s.start(), s.goal(), 0, 0, s.battery(), s.speedLimit(), s.freeDistance(), s.obstacles());
            ObjectNode rays64 = Json.object();
            rays64.put("sensorRays", 64);
            ControllerTestingProblem fixed = testing(Params.empty(), RobotSpec.DEFAULT);
            System.out.println(Path.of(file).getFileName());
            System.out.println("  as recorded              : " + run(fixed, s));
            System.out.println("  no sensor noise          : " + run(fixed, quiet));
            System.out.println("  no scan dropout          : " + run(fixed, noDrop));
            System.out.println("  neither                  : " + run(fixed, neither));
            System.out.println("  64 rays                  : " + run(testing(Params.empty(), Problems.robot(rays64)), s));
            System.out.println("  before the fix           : " + run(testing(Params.of("cornerGuard", false, "pointMemory", 1),
                    RobotSpec.DEFAULT), s));
        }
    }
}
