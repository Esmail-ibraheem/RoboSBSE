package org.robosbse.simulation;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;

import org.robosbse.protocol.Json;

/**
 * Resolves map references used in configuration files:
 * <ul>
 *   <li>an inline map object;</li>
 *   <li>{@code "builtin:<name>"} or a bare name — a map bundled with the engine
 *       (copied from {@code examples/maps} at build time);</li>
 *   <li>a file path, relative to a base directory.</li>
 * </ul>
 */
public final class MapLibrary {

    public static final List<String> BUILTIN = List.of(
            "empty-field", "obstacle-course", "warehouse", "maze", "testing-arena", "dynamic-obstacle-course", "delivery-center");

    private MapLibrary() {
    }

    public static WorldMap resolve(JsonNode reference, Path baseDir) {
        if (reference == null || reference.isNull() || reference.isMissingNode()) {
            throw new IllegalArgumentException("problem.map is required");
        }
        if (reference.isObject()) {
            return Json.convert(reference, WorldMap.class);
        }
        String ref = reference.asText();
        if (ref.startsWith("generated:")) {
            return generated(ref);
        }
        if (ref.startsWith("builtin:")) {
            return builtin(ref.substring("builtin:".length()));
        }
        if (BUILTIN.contains(ref)) {
            return builtin(ref);
        }
        Path path = Path.of(ref);
        if (!path.isAbsolute() && baseDir != null) {
            path = baseDir.resolve(path);
        }
        return Json.convert(Json.read(path), WorldMap.class);
    }

    /**
     * Procedurally generated maps by reference: {@code generated:scatter:<seed>[:<obstacles>]} (20 x 20 m,
     * default 15 obstacles, solvability checked) or {@code generated:maze:<seed>[:<cells>]}. The same
     * reference always gives the same map, so configurations stay reproducible.
     */
    public static WorldMap generated(String ref) {
        String[] parts = ref.split(":");
        if (parts.length < 3) {
            throw new IllegalArgumentException("generated maps look like generated:scatter:<seed> or generated:maze:<seed>");
        }
        long seed = Long.parseLong(parts[2]);
        return switch (parts[1]) {
            case "scatter" -> MapGenerator.scatter(seed, 20, 20, parts.length > 3 ? Integer.parseInt(parts[3]) : 15, 0.25);
            case "maze" -> {
                int cells = parts.length > 3 ? Integer.parseInt(parts[3]) : 6;
                yield MapGenerator.maze(seed, cells, cells, 2.5, 0.2);
            }
            default -> throw new IllegalArgumentException("unknown generator '" + parts[1] + "' (scatter, maze)");
        };
    }

    public static WorldMap builtin(String name) {
        String resource = "/robosbse/maps/" + name + ".json";
        try (InputStream in = MapLibrary.class.getResourceAsStream(resource)) {
            if (in == null) {
                throw new IllegalArgumentException("unknown built-in map '" + name + "', available: " + BUILTIN);
            }
            return Json.MAPPER.readValue(in, WorldMap.class);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
