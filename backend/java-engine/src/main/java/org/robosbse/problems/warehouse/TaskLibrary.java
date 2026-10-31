package org.robosbse.problems.warehouse;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;

import org.robosbse.protocol.Json;

/** Resolves task-set references: inline objects, {@code builtin:<name>} (bundled) or files. */
public final class TaskLibrary {

    public static final List<String> BUILTIN = List.of("orders-4", "orders-8");

    private TaskLibrary() {
    }

    public static TaskSet resolve(JsonNode reference, Path baseDir) {
        if (reference == null || reference.isNull() || reference.isMissingNode()) {
            throw new IllegalArgumentException("warehouse scheduling needs a 'tasks' parameter");
        }
        if (reference.isObject()) {
            return Json.convert(reference, TaskSet.class);
        }
        String ref = reference.asText();
        String name = ref.startsWith("builtin:") ? ref.substring("builtin:".length()) : ref;
        if (BUILTIN.contains(name)) {
            try (InputStream in = TaskLibrary.class.getResourceAsStream("/robosbse/tasks/" + name + ".json")) {
                if (in == null) {
                    throw new IllegalArgumentException("bundled task set '" + name + "' missing from the engine jar");
                }
                return Json.MAPPER.readValue(in, TaskSet.class);
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
        Path path = Path.of(ref);
        if (!path.isAbsolute() && baseDir != null) {
            path = baseDir.resolve(path);
        }
        return Json.convert(Json.read(path), TaskSet.class);
    }
}
