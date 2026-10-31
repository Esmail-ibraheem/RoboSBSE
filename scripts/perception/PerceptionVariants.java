import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.node.TextNode;

import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Params;
import org.robosbse.core.Rng;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.representation.RealVector;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.EpisodeConfig;
import org.robosbse.simulation.EpisodeResult;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.Simulator;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.simulation.controllers.ControllerSetup;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.Scenario;

/**
 * Compares perception configurations of the correct reactive controller (docs/experiments/perception-fix.md):
 * the recorded collision scenarios, 400 random arena scenarios (seed 2024), start-to-goal navigation on 13 maps,
 * and a collision-guided GA (population 50, 2000 scenarios) over a number of seeds.
 *
 * <p>Run from the repository root after {@code gradlew engineJar} (Java 21 single-file launch):
 * <pre>
 *   java -cp backend/java-engine/build/libs/robosbse-engine.jar scripts/perception/PerceptionVariants.java 20 false:1 true:4
 * </pre>
 * Each variant is {@code cornerGuard:pointMemory}; {@code false:1} is the controller before the fix.
 */
public class PerceptionVariants {

    public static void main(String[] args) throws Exception {
        int searchSeeds = Integer.parseInt(args[0]);
        List<Path> cases = new ArrayList<>();
        try (var s = Files.list(Path.of("examples/faulty-controllers/fixed-issues"))) {
            s.filter(f -> f.toString().endsWith(".json")).sorted().forEach(cases::add);
        }
        String[] maps = {"empty-field", "obstacle-course", "warehouse", "maze", "testing-arena",
                "generated:scatter:1", "generated:scatter:2", "generated:scatter:101", "generated:scatter:102",
                "generated:scatter:103", "generated:scatter:104", "generated:scatter:105", "generated:scatter:106"};
        for (int a = 1; a < args.length; a++) {
            String[] v = args[a].split(":");
            Params cp = Params.of("cornerGuard", Boolean.parseBoolean(v[0]), "pointMemory", Integer.parseInt(v[1]));
            ControllerTestingProblem p = new ControllerTestingProblem(MapLibrary.builtin("testing-arena"),
                    Params.of("controller", "reactive", "properties", "safety", "fitness", "collision", "maxSteps", 1200,
                            "simulationSeed", 1), cp, RobotSpec.DEFAULT);
            StringBuilder known = new StringBuilder();
            int colliding = 0;
            for (Path f : cases) {
                Scenario sc = Json.convert(Json.read(f).get("scenario"), Scenario.class);
                boolean bad = !p.execute(sc, "reactive", false).violated().isEmpty();
                colliding += bad ? 1 : 0;
                known.append(bad ? 'X' : '.');
            }
            int valid = 0;
            int violations = 0;
            int reached = 0;
            Rng rng = new Rng(2024);
            for (int i = 0; i < 400; i++) {
                Scenario s = p.decode(p.space().random(rng));
                if (p.invalidity(s) > 0) {
                    continue;
                }
                valid++;
                var ex = p.execute(s, "reactive", false);
                violations += ex.violated().isEmpty() ? 0 : 1;
                reached += ex.result().success() ? 1 : 0;
            }
            int ok = 0;
            StringBuilder perMap = new StringBuilder();
            for (String m : maps) {
                WorldMap map = MapLibrary.resolve(TextNode.valueOf(m), null);
                EpisodeConfig cfg = EpisodeConfig.defaults().withMaxSteps(2400);
                ControllerSetup setup = new ControllerSetup(RobotSpec.DEFAULT, cfg.dt(), map.goal(), map.goalTolerance(), List.of(), cp);
                EpisodeResult r = Simulator.run(new World(map), RobotSpec.DEFAULT, map.start(), map.goal(), map.goalTolerance(),
                        ControllerRegistry.create("reactive"), setup, cfg);
                ok += r.success() ? 1 : 0;
                perMap.append(r.success() ? String.format("%.0f", r.time()) : r.outcome().name().substring(0, 4)).append(' ');
            }
            int found = 0;
            StringBuilder hits = new StringBuilder();
            for (long seed = 1; seed <= searchSeeds; seed++) {
                SearchContext<RealVector> ctx = SearchContext.builder(p, Budget.evaluations(2000), seed)
                        .stopOnTarget(true).maxTargets(1).threads(4).build();
                SearchResult<RealVector> r = SearchAlgorithm.execute(
                        AlgorithmRegistry.create("genetic-algorithm", Params.of("populationSize", 50)), ctx);
                if (r.targetCount() > 0) {
                    found++;
                    hits.append(seed).append('@').append(r.firstTargetEvaluation()).append(' ');
                }
            }
            System.out.printf("cornerGuard=%s pointMemory=%s | recorded scenarios %s (%d of %d collide) | arena: %d of %d reach "
                            + "the goal, %d safety violations | maps %d/13: %s| GA finds a collision in %d of %d seeds %s%n",
                    v[0], v[1], known, colliding, cases.size(), reached, valid, violations, ok, perMap, found, searchSeeds, hits);
        }
    }
}
