package org.robosbse.problems.fleet;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.databind.JsonNode;

import org.robosbse.geometry.Vec2;
import org.robosbse.protocol.Json;

/**
 * A multi-robot delivery instance ({@code robosbse.fleet/1}): a map, the robots' depots, and transport
 * tasks from a pickup point to a delivery point. Each robot carries one item at a time.
 *
 * @param map         map reference ({@code builtin:<name>}, a file, or an inline map)
 * @param robots      depot of every robot (its start and, with {@code returnHome}, end point)
 * @param speed       planning speed (m/s)
 * @param serviceTime time to load or unload (s)
 * @param returnHome  whether robots end at their depot
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public record Fleet(String schema, String name, String description, JsonNode map, List<Vec2> robots, double speed,
                    double serviceTime, boolean returnHome, List<Task> tasks) {

    public static final String SCHEMA = "robosbse.fleet/1";
    public static final List<String> BUILTIN = List.of("deliveries-12");

    /** One transport job. */
    public record Task(String label, Vec2 pickup, Vec2 delivery) {
    }

    public Fleet {
        if (robots == null || robots.isEmpty()) {
            throw new IllegalArgumentException("a fleet needs at least one robot");
        }
        if (tasks == null || tasks.isEmpty()) {
            throw new IllegalArgumentException("a fleet instance needs at least one task");
        }
        schema = schema == null ? SCHEMA : schema;
        robots = List.copyOf(robots);
        tasks = List.copyOf(tasks);
        speed = speed > 0 ? speed : 0.8;
        serviceTime = Math.max(0, serviceTime);
    }

    public static Fleet resolve(JsonNode reference, Path baseDir) {
        if (reference == null || reference.isNull() || reference.isMissingNode()) {
            return builtin("deliveries-12");
        }
        if (reference.isObject()) {
            return Json.convert(reference, Fleet.class);
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
        return Json.convert(Json.read(p), Fleet.class);
    }

    public static Fleet builtin(String name) {
        try (InputStream in = Fleet.class.getResourceAsStream("/robosbse/fleet/" + name + ".json")) {
            if (in == null) {
                throw new IllegalArgumentException("unknown built-in fleet '" + name + "', available: " + BUILTIN);
            }
            return Json.MAPPER.readValue(in, Fleet.class);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
