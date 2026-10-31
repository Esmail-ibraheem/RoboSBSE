package org.robosbse.patching;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;

import org.robosbse.protocol.Json;
import org.robosbse.testing.FailureRecord;
import org.robosbse.testing.Scenario;

/**
 * A Java controller used as the subject of program repair or genetic improvement
 * ({@code robosbse.repair-subject/1}): its source, class name, mutation scope and, for repair
 * subjects, the defect description, a reference fix (used only to judge results, never by the search)
 * and saved failure records (paths relative to the specification) whose scenarios seed the failing search
 * tests ({@code failureRecords}) and the held-out tests ({@code heldOutRecords}).
 *
 * <p>Built-in subjects live in {@code examples/faulty-controllers/repair} and are bundled into the
 * engine; {@code builtin:<id>} or a bare id selects them, any other reference is a file path.
 */
public record Subject(String id, String name, String purpose, String defectKind, String defect, String className,
                      String source, List<String> scope, String fixFind, String fixReplace,
                      List<Scenario> failureScenarios, List<Scenario> heldOutScenarios, String description) {

    public static final List<String> BUILTIN = List.of("rb1-stopping-distance", "rb2-comparison", "rb3-turn-direction",
            "rb4-boundary-check", "rb5-sensor-mirror", "gi-reactive-baseline");

    public static Subject resolve(String reference, Path baseDir) {
        String ref = reference.startsWith("builtin:") ? reference.substring("builtin:".length()) : reference;
        if (BUILTIN.contains(ref)) {
            JsonNode spec = Json.parse(resource("/robosbse/repair/" + ref + ".json"));
            Loader bundled = path -> resource("/" + Path.of("robosbse/repair", path).normalize().toString().replace('\\', '/'));
            return fromSpec(spec, bundled, bundled);
        }
        Path file = Path.of(ref);
        if (!file.isAbsolute() && baseDir != null) {
            file = baseDir.resolve(file);
        }
        Path dir = file.toAbsolutePath().getParent();
        JsonNode spec = Json.read(file);
        Loader files = path -> read(dir.resolve(path).normalize());
        return fromSpec(spec, files, files);
    }

    private interface Loader {
        String load(String path);
    }

    private static Subject fromSpec(JsonNode spec, Loader sources, Loader records) {
        List<String> scope = new ArrayList<>();
        spec.path("scope").forEach(s -> scope.add(s.asText()));
        List<Scenario> failures = scenarios(spec.path("failureRecords"), records);
        List<Scenario> heldOut = scenarios(spec.path("heldOutRecords"), records);
        JsonNode fix = spec.path("referenceFix");
        return new Subject(spec.path("id").asText(), spec.path("name").asText(), spec.path("purpose").asText("program-repair"),
                spec.path("defectKind").asText(null), spec.path("defect").asText(null), spec.path("className").asText(),
                sources.load(spec.path("source").asText()), List.copyOf(scope),
                fix.isObject() ? fix.path("find").asText() : null, fix.isObject() ? fix.path("replace").asText() : null,
                List.copyOf(failures), List.copyOf(heldOut), spec.path("description").asText(spec.path("defect").asText("")));
    }

    private static List<Scenario> scenarios(JsonNode paths, Loader records) {
        List<Scenario> out = new ArrayList<>();
        for (JsonNode path : paths) {
            out.add(Json.convert(Json.parse(records.load(path.asText())), FailureRecord.class).scenario());
        }
        return out;
    }

    /** The source with the reference fix applied, or null when the subject has none. */
    public String fixedSource() {
        if (fixFind == null || !source.contains(fixFind)) {
            return null;
        }
        return source.replace(fixFind, fixReplace);
    }

    private static String resource(String path) {
        try (InputStream in = Subject.class.getResourceAsStream(path)) {
            if (in == null) {
                throw new IllegalArgumentException("missing bundled resource " + path);
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private static String read(Path path) {
        try {
            return Files.readString(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException("cannot read " + path, e);
        }
    }
}
