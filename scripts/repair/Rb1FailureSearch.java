import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.core.Budget;
import org.robosbse.core.Candidate;
import org.robosbse.core.Evaluation;
import org.robosbse.core.Params;
import org.robosbse.core.Problem;
import org.robosbse.core.SearchAlgorithm;
import org.robosbse.core.SearchContext;
import org.robosbse.core.SearchResult;
import org.robosbse.core.SolutionSpace;
import org.robosbse.core.representation.RealVector;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.WorldMap;
import org.robosbse.testing.ControllerTestingProblem;
import org.robosbse.testing.Scenario;
import org.robosbse.testing.Verdict;

/**
 * Finds failing tests for repair subject RB1 by search-based failure discovery against the faulty subject
 * (GA, collision robustness), keeping scenarios that the reference controller and the correct baseline pass.
 *
 * <p>Build and run from the repository root (Java 21, after {@code gradlew engineJar}; ';' is the Windows
 * class-path separator):
 * <pre>
 *   javac -cp backend/java-engine/build/libs/robosbse-engine.jar -d build/rb1
 *       examples/faulty-controllers/repair/src/subjects/StoppingDistanceBug.java
 *       examples/faulty-controllers/repair/src/subjects/ReactiveBaseline.java scripts/repair/Rb1FailureSearch.java
 *   java -cp "backend/java-engine/build/libs/robosbse-engine.jar;build/rb1" Rb1FailureSearch examples/faulty-controllers/repair/tests
 * </pre>
 */
public class Rb1FailureSearch {
    public static void main(String[] args) throws Exception {
        WorldMap arena = MapLibrary.builtin("testing-arena");
        ControllerTestingProblem testing = new ControllerTestingProblem(arena,
                Params.of("controller", "reactive", "properties", "safety", "maxSteps", 1200, "simulationSeed", 1),
                Params.empty(), RobotSpec.DEFAULT);
        Problem<RealVector> problem = new Problem<>() {
            public String type() { return "rb1-search"; }
            public SolutionSpace<RealVector> space() { return testing.space(); }
            public Evaluation evaluate(RealVector g) {
                Scenario s = testing.decode(g);
                if (testing.invalidity(s) > 0) {
                    return Evaluation.of(2 + testing.invalidity(s));
                }
                var ex = testing.execute(s, "rb1", new subjects.StoppingDistanceBug(), false);
                double rho = 1;
                for (Verdict v : ex.verdicts()) {
                    if (v.property().equals("collision") && v.applicable()) {
                        rho = v.robustness();
                    }
                }
                return new Evaluation(rho, 0, Map.of("violated", ex.violated().isEmpty() ? 0.0 : 1.0));
            }
            public boolean isTarget(Evaluation e) { return e.metric("violated", 0) > 0; }
        };
        Path out = Path.of(args[0]);
        Files.createDirectories(out);
        int saved = 0;
        for (long seed = 1; seed <= 40 && saved < 12; seed++) {
            SearchContext<RealVector> ctx = SearchContext.builder(problem, Budget.evaluations(2000), seed)
                    .stopOnTarget(true).maxTargets(1).threads(8).build();
            SearchResult<RealVector> r = SearchAlgorithm.execute(
                    AlgorithmRegistry.create("genetic-algorithm", Params.of("populationSize", 50)), ctx);
            if (r.targetCount() == 0) {
                System.out.println("seed " + seed + ": no failure");
                continue;
            }
            Scenario s = testing.decode(r.targets().get(0).solution());
            var faulty = testing.execute(s, "rb1", new subjects.StoppingDistanceBug(), false);
            var reference = testing.execute(s, "reactive", false);
            var fixed = testing.execute(s, "baseline", new subjects.ReactiveBaseline(), false);
            boolean usable = !faulty.violated().isEmpty() && reference.violated().isEmpty() && fixed.violated().isEmpty();
            System.out.println("seed " + seed + ": faulty " + faulty.violated() + " reference " + reference.violated()
                    + " fixed " + fixed.violated() + " at evaluation " + r.firstTargetEvaluation() + (usable ? " SAVED" : ""));
            if (usable) {
                saved++;
                Map<String, Object> rec = new LinkedHashMap<>();
                rec.put("schema", "robosbse.failure/1");
                rec.put("controller", "subject-rb1-stopping-distance");
                rec.put("violated", faulty.violated());
                rec.put("outcome", faulty.result().outcome().name());
                rec.put("time", faulty.result().time());
                rec.put("scenario", s);
                rec.put("provenance", Map.of("tool", "search-based failure discovery (GA, collision robustness)",
                        "seed", seed, "evaluation", r.firstTargetEvaluation()));
                Json.write(out.resolve("rb1-collision-" + saved + ".json"), rec);
            }
        }
        System.out.println("saved " + saved);
    }
}
