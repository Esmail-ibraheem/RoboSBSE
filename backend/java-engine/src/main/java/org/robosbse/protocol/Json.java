package org.robosbse.protocol;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.module.SimpleModule;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * Shared Jackson configuration. Non-finite floating-point values are written as {@code null}
 * because JSON has no NaN/Infinity; consumers treat {@code null} as "not available".
 */
public final class Json {

    public static final ObjectMapper MAPPER = create();

    private Json() {
    }

    private static ObjectMapper create() {
        ObjectMapper mapper = new ObjectMapper();
        mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        mapper.configure(SerializationFeature.FAIL_ON_EMPTY_BEANS, false);
        SimpleModule module = new SimpleModule("robosbse-finite");
        JsonSerializer<Double> finite = new JsonSerializer<>() {
            @Override
            public void serialize(Double value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
                if (value == null || !Double.isFinite(value)) {
                    gen.writeNull();
                } else {
                    gen.writeNumber(value);
                }
            }
        };
        module.addSerializer(Double.class, finite);
        module.addSerializer(double.class, finite);
        module.addSerializer(double[].class, new JsonSerializer<>() {
            @Override
            public void serialize(double[] values, JsonGenerator gen, SerializerProvider serializers) throws IOException {
                gen.writeStartArray();
                for (double v : values) {
                    if (Double.isFinite(v)) {
                        gen.writeNumber(v);
                    } else {
                        gen.writeNull();
                    }
                }
                gen.writeEndArray();
            }
        });
        module.addSerializer(float[].class, new JsonSerializer<>() {
            @Override
            public void serialize(float[] values, JsonGenerator gen, SerializerProvider serializers) throws IOException {
                gen.writeStartArray();
                for (float v : values) {
                    if (Float.isFinite(v)) {
                        gen.writeNumber(v);
                    } else {
                        gen.writeNull();
                    }
                }
                gen.writeEndArray();
            }
        });
        mapper.registerModule(module);
        return mapper;
    }

    public static ObjectNode object() {
        return MAPPER.createObjectNode();
    }

    /**
     * Puts a number, or {@code null} when it is NaN or infinite. Tree nodes bypass the finite-number
     * serialiser above, so code that builds {@link ObjectNode}s by hand must use this.
     */
    public static void putFinite(ObjectNode node, String field, double value) {
        if (Double.isFinite(value)) {
            node.put(field, value);
        } else {
            node.putNull(field);
        }
    }

    public static JsonNode tree(Object value) {
        return MAPPER.valueToTree(value);
    }

    public static <T> T convert(Object value, Class<T> type) {
        return MAPPER.convertValue(value, type);
    }

    public static Map<String, Object> toMap(JsonNode node) {
        if (node == null || node.isNull() || node.isMissingNode()) {
            return Map.of();
        }
        return MAPPER.convertValue(node, new TypeReference<Map<String, Object>>() {
        });
    }

    public static String compact(Object value) {
        try {
            return MAPPER.writeValueAsString(value);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    public static String pretty(Object value) {
        try {
            return MAPPER.writerWithDefaultPrettyPrinter().writeValueAsString(value);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    public static JsonNode parse(String text) {
        try {
            return MAPPER.readTree(text);
        } catch (IOException e) {
            throw new IllegalArgumentException("invalid JSON: " + e.getMessage(), e);
        }
    }

    public static JsonNode read(Path path) {
        try {
            return MAPPER.readTree(path.toFile());
        } catch (IOException e) {
            throw new IllegalArgumentException("cannot read JSON file " + path + ": " + e.getMessage(), e);
        }
    }

    public static void write(Path path, Object value) {
        try {
            if (path.getParent() != null) {
                Files.createDirectories(path.getParent());
            }
            Files.writeString(path, pretty(value));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
