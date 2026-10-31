package org.robosbse.arm;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.databind.JsonNode;

import org.robosbse.geometry.Shape;
import org.robosbse.geometry.Vec2;
import org.robosbse.protocol.Json;

/**
 * A planar robotic-arm workspace ({@code robosbse.arm/1}): an arm with revolute joints mounted at
 * {@code base}, static obstacles, a reach target and optional pick-and-place objects. Angles are
 * radians; joint {@code i} is relative to link {@code i-1} (joint 0 relative to the x axis), so the
 * absolute angle of link {@code i} is the sum of the first {@code i+1} joint angles.
 *
 * @param links          link lengths (m); two or more
 * @param jointMin       lower joint limits (rad)
 * @param jointMax       upper joint limits (rad)
 * @param linkRadius     half-thickness of every link for collision checks (m)
 * @param maxJointSpeed  joint speed limit (rad/s) used by the timing model
 * @param home           starting configuration
 * @param tasks          pick-and-place objects (may be empty)
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public record ArmScene(String schema, String name, String description, double width, double height, Vec2 base,
                       List<Double> links, List<Double> jointMin, List<Double> jointMax, double linkRadius,
                       double maxJointSpeed, List<Double> home, List<Shape> obstacles, Vec2 target,
                       List<Task> tasks) {

    public static final String SCHEMA = "robosbse.arm/1";
    public static final List<String> BUILTIN = List.of("arm-reach", "arm-cluttered", "arm-pick-place", "arm-three-link");

    /** An object to pick up at {@code pick} and put down at {@code place}. */
    public record Task(String label, Vec2 pick, Vec2 place) {
    }

    public ArmScene {
        if (links == null || links.size() < 2) {
            throw new IllegalArgumentException("an arm needs at least two links");
        }
        int n = links.size();
        if (jointMin == null || jointMax == null || jointMin.size() != n || jointMax.size() != n) {
            throw new IllegalArgumentException("jointMin and jointMax need one entry per link");
        }
        for (int i = 0; i < n; i++) {
            if (!(links.get(i) > 0) || !(jointMin.get(i) < jointMax.get(i))) {
                throw new IllegalArgumentException("link " + i + ": length must be positive and jointMin < jointMax");
            }
        }
        if (home == null || home.size() != n) {
            throw new IllegalArgumentException("home needs one angle per joint");
        }
        schema = schema == null ? SCHEMA : schema;
        name = name == null ? "arm" : name;
        links = List.copyOf(links);
        jointMin = List.copyOf(jointMin);
        jointMax = List.copyOf(jointMax);
        home = List.copyOf(home);
        obstacles = obstacles == null ? List.of() : List.copyOf(obstacles);
        tasks = tasks == null ? List.of() : List.copyOf(tasks);
        linkRadius = linkRadius > 0 ? linkRadius : 0.04;
        maxJointSpeed = maxJointSpeed > 0 ? maxJointSpeed : 1.0;
        if (target == null) {
            target = new Vec2(base.x() + links.get(0), base.y() + links.get(1));
        }
    }

    public int joints() {
        return links.size();
    }

    public double reach() {
        double r = 0;
        for (double l : links) {
            r += l;
        }
        return r;
    }

    public double[] homeArray() {
        return home.stream().mapToDouble(Double::doubleValue).toArray();
    }

    public ArmScene withTarget(Vec2 newTarget) {
        return new ArmScene(schema, name, description, width, height, base, links, jointMin, jointMax, linkRadius,
                maxJointSpeed, home, obstacles, newTarget, tasks);
    }

    /** {@code builtin:<name>}, a bare built-in name, a file path, or an inline scene object. */
    public static ArmScene resolve(JsonNode reference, Path baseDir) {
        if (reference == null || reference.isNull() || reference.isMissingNode()) {
            return builtin("arm-reach");
        }
        if (reference.isObject()) {
            return Json.convert(reference, ArmScene.class);
        }
        String ref = reference.asText();
        String name = ref.startsWith("builtin:") ? ref.substring("builtin:".length()) : ref;
        if (BUILTIN.contains(name)) {
            return builtin(name);
        }
        Path p = Path.of(ref);
        if (!p.isAbsolute() && baseDir != null) {
            p = baseDir.resolve(p);
        }
        return Json.convert(Json.read(p), ArmScene.class);
    }

    public static ArmScene builtin(String name) {
        try (InputStream in = ArmScene.class.getResourceAsStream("/robosbse/arm/" + name + ".json")) {
            if (in == null) {
                throw new IllegalArgumentException("unknown built-in arm scene '" + name + "', available: " + BUILTIN);
            }
            return Json.MAPPER.readValue(in, ArmScene.class);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
