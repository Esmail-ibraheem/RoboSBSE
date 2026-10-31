package org.robosbse.protocol;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.algorithms.AlgorithmRegistry;
import org.robosbse.arm.ArmScene;
import org.robosbse.algorithms.evolutionary.Selection;
import org.robosbse.algorithms.local.CoolingSchedule;
import org.robosbse.core.representation.RealVectorSpace;
import org.robosbse.experiments.ExperimentRunner;
import org.robosbse.experiments.Problems;
import org.robosbse.patching.Subject;
import org.robosbse.simulation.MapLibrary;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.controllers.ControllerRegistry;
import org.robosbse.testing.Property;
import org.robosbse.testing.coverage.Branch;
import org.robosbse.testing.coverage.Instrumented;

/**
 * Self-description of the engine: protocol version, algorithms with parameter declarations,
 * problem types, controllers (including the documented faulty variants), properties and built-in
 * maps. The UI builds its forms from this message, so new algorithms appear without UI changes.
 */
public final class Capabilities {

    public static final int PROTOCOL_VERSION = 1;

    private Capabilities() {
    }

    public static ObjectNode describe() {
        ObjectNode root = Json.object();
        root.put("engine", "robosbse-engine");
        root.put("version", ExperimentRunner.ENGINE_VERSION);
        root.put("protocol", PROTOCOL_VERSION);
        root.put("java", System.getProperty("java.version"));
        ArrayNode algorithms = root.putArray("algorithms");
        for (AlgorithmRegistry.Descriptor d : AlgorithmRegistry.all()) {
            ObjectNode a = algorithms.addObject();
            a.put("id", d.id());
            a.put("name", d.name());
            a.put("family", d.family());
            a.put("summary", d.summary());
            a.set("params", Json.tree(d.params()));
            a.set("requires", Json.tree(d.requires()));
            a.set("representations", Json.tree(d.representations()));
            a.put("multiObjective", d.multiObjective());
        }
        ArrayNode problems = root.putArray("problems");
        for (Problems.Descriptor d : Problems.DESCRIPTORS) {
            problems.add(Json.tree(d));
        }
        ArrayNode controllers = root.putArray("controllers");
        for (ControllerRegistry.Descriptor d : ControllerRegistry.all()) {
            ObjectNode c = controllers.addObject();
            c.put("id", d.id());
            c.put("name", d.name());
            c.put("kind", d.kind());
            c.put("summary", d.summary());
            c.put("faultyOf", d.faultyOf());
            c.put("defect", d.defect());
            c.put("expectedProperty", d.expectedProperty());
            c.set("params", Json.tree(d.params()));
            if (ControllerRegistry.create(d.id()) instanceof Instrumented inst) {
                ArrayNode branches = c.putArray("branches");
                for (Branch b : inst.branches()) {
                    ObjectNode n = branches.addObject();
                    n.put("id", b.id());
                    n.put("description", b.description());
                    n.put("parent", b.parent() == null ? null : Branch.outcome(b.parent(), b.parentOutcome()));
                }
                c.put("instrumentation", inst.instrumentation());
            }
        }
        ArrayNode subjects = root.putArray("subjects");
        for (String id : Subject.BUILTIN) {
            Subject s = Subject.resolve(id, null);
            ObjectNode n = subjects.addObject();
            n.put("id", s.id());
            n.put("name", s.name());
            n.put("purpose", s.purpose());
            n.put("defectKind", s.defectKind());
            n.put("defect", s.defect());
            n.put("description", s.description());
        }
        ArrayNode properties = root.putArray("properties");
        for (Property p : Property.values()) {
            ObjectNode n = properties.addObject();
            n.put("id", p.id());
            n.put("safety", p.safety());
            n.put("description", p.description());
        }
        root.set("maps", Json.tree(MapLibrary.BUILTIN));
        ArrayNode armScenes = root.putArray("armScenes");
        for (String name : ArmScene.BUILTIN) {
            armScenes.add(Json.tree(ArmScene.builtin(name)));
        }
        root.set("taskSets", Json.tree(org.robosbse.problems.warehouse.TaskLibrary.BUILTIN));
        root.set("fleets", Json.tree(org.robosbse.problems.fleet.Fleet.BUILTIN));
        root.set("robot", Json.tree(RobotSpec.DEFAULT));
        ObjectNode operators = root.putObject("operators");
        operators.set("mutation", Json.tree(RealVectorSpace.MUTATIONS));
        operators.set("crossover", Json.tree(RealVectorSpace.CROSSOVERS));
        operators.set("selection", Json.tree(Selection.NAMES));
        operators.set("cooling", Json.tree(CoolingSchedule.NAMES));
        return root;
    }
}
