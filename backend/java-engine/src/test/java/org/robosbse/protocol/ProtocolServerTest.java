package org.robosbse.protocol;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import com.fasterxml.jackson.databind.JsonNode;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Drives the real server over piped streams, exactly as the UI does over stdin/stdout. */
class ProtocolServerTest {

    private PrintStream toServer;
    private ByteArrayOutputStream fromServer;
    private Thread serverThread;

    @BeforeEach
    void start() throws IOException {
        PipedInputStream in = new PipedInputStream(1 << 16);
        toServer = new PrintStream(new PipedOutputStream(in), true, StandardCharsets.UTF_8);
        fromServer = new ByteArrayOutputStream();
        ProtocolServer server = new ProtocolServer(in, fromServer, Path.of("."));
        serverThread = new Thread(() -> {
            try {
                server.serve();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        });
        serverThread.start();
    }

    @AfterEach
    void stop() throws InterruptedException {
        toServer.close();
        serverThread.join(5000);
    }

    private List<JsonNode> messages() {
        List<JsonNode> out = new ArrayList<>();
        String text;
        synchronized (fromServer) {
            text = fromServer.toString(StandardCharsets.UTF_8);
        }
        text = text.substring(0, text.lastIndexOf('\n') + 1); // ignore a line still being written
        for (String line : text.split("\n")) {
            if (!line.isBlank()) {
                out.add(Json.parse(line));
            }
        }
        return out;
    }

    private JsonNode await(String type, String id) throws InterruptedException {
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(30);
        while (System.nanoTime() < deadline) {
            for (JsonNode m : messages()) {
                if (m.path("type").asText().equals(type) && (id == null || m.path("id").asText().equals(id))) {
                    return m;
                }
            }
            Thread.sleep(10);
        }
        throw new AssertionError("no '" + type + "' message for " + id + "; got " + messages());
    }

    private void send(String json) {
        toServer.println(json);
    }

    @Test
    void announcesCapabilitiesOnStartAndOnHello() throws Exception {
        JsonNode ready = await("ready", null);
        assertEquals(1, ready.path("protocol").asInt());
        assertTrue(ready.path("algorithms").size() >= 5);
        assertTrue(ready.path("controllers").size() >= 8);
        send("{\"v\":1,\"type\":\"hello\",\"id\":\"h\"}");
        assertEquals("robosbse-engine", await("hello", "h").path("engine").asText());
    }

    @Test
    void streamsProgressAndFinishesARun() throws Exception {
        send("""
                {"v":1,"type":"start","id":"r1","experiment":{"name":"p","seed":1,"budget":{"maxEvaluations":600},
                 "problem":{"type":"navigation","map":"builtin:warehouse","params":{"waypoints":3}},
                 "algorithm":{"type":"genetic-algorithm","params":{"populationSize":20}}}}""".replace("\n", ""));
        assertEquals(600, await("started", "r1").path("maxEvaluations").asLong());
        JsonNode finished = await("finished", "r1");
        assertEquals(600, finished.path("result").path("evaluations").asLong());
        long progress = messages().stream().filter(m -> m.path("type").asText().equals("progress")).count();
        assertTrue(progress >= 1);
        assertTrue(messages().stream().anyMatch(m -> m.path("type").asText().equals("best")
                && m.path("solution").path("route").size() == 4));
    }

    @Test
    void cancelEndsARunEarly() throws Exception {
        send("""
                {"v":1,"type":"start","id":"long","experiment":{"name":"p","seed":1,"budget":{"maxEvaluations":10000000},
                 "problem":{"type":"navigation","map":"builtin:maze","params":{"waypoints":6}},
                 "algorithm":{"type":"random-search"}}}""".replace("\n", ""));
        await("progress", "long");
        send("{\"v\":1,\"type\":\"pause\",\"id\":\"long\"}");
        assertEquals("paused", await("state", "long").path("state").asText());
        send("{\"v\":1,\"type\":\"cancel\",\"id\":\"long\"}");
        JsonNode finished = await("finished", "long");
        assertEquals("CANCELLED", finished.path("result").path("stopReason").asText());
    }

    @Test
    void reportsErrorsWithoutDying() throws Exception {
        send("this is not json");
        assertEquals("invalid_json", await("error", null).path("code").asText());
        send("{\"v\":1,\"type\":\"teleport\",\"id\":\"x\"}");
        assertEquals("unknown_command", await("error", "x").path("code").asText());
        send("{\"v\":2,\"type\":\"hello\",\"id\":\"y\"}");
        assertEquals("unsupported_version", await("error", "y").path("code").asText());
        send("{\"v\":1,\"type\":\"start\",\"id\":\"bad\",\"experiment\":{\"budget\":{\"maxEvaluations\":5},"
                + "\"problem\":{\"type\":\"navigation\",\"map\":\"builtin:warehouse\"},\"algorithm\":{\"type\":\"nope\"}}}");
        assertTrue(await("error", "bad").path("message").asText().contains("unknown algorithm"));
        send("{\"v\":1,\"type\":\"hello\",\"id\":\"alive\"}");
        await("hello", "alive");
    }

    @Test
    void servesMapsPlansAndSimulations() throws Exception {
        send("{\"v\":1,\"type\":\"get_map\",\"id\":\"m\",\"map\":\"builtin:maze\"}");
        assertEquals("maze", await("map", "m").path("map").path("name").asText());
        send("{\"v\":1,\"type\":\"generate_map\",\"id\":\"g\",\"params\":{\"kind\":\"scatter\",\"seed\":4,\"count\":8}}");
        assertTrue(await("map", "g").path("map").path("obstacles").size() > 0);
        send("{\"v\":1,\"type\":\"plan\",\"id\":\"p\",\"problem\":{\"type\":\"navigation\",\"map\":\"builtin:warehouse\"}}");
        assertTrue(await("plan_result", "p").path("reference").path("astar").path("found").asBoolean());
        send("{\"v\":1,\"type\":\"simulate\",\"id\":\"s\",\"problem\":{\"type\":\"navigation\",\"map\":\"builtin:empty-field\"},"
                + "\"route\":[[10,10]]}");
        JsonNode sim = await("simulation", "s");
        assertEquals("REACHED_GOAL", sim.path("outcome").asText());
        assertTrue(sim.path("trajectory").path("x").size() > 10);
    }

    @Test
    void reproducesASavedFailure() throws Exception {
        Path record = Path.of(System.getProperty("robosbse.repoRoot", "../.."))
                .resolve("examples/faulty-controllers/regressions/bug4-completion-false-report.json");
        JsonNode failure = Json.read(record);
        send("{\"v\":1,\"type\":\"reproduce\",\"id\":\"rep\",\"failure\":" + Json.compact(failure) + "}");
        JsonNode r = await("reproduction", "rep");
        assertTrue(r.path("reproduced").asBoolean());
        assertEquals(0, r.path("referenceViolated").size());
    }
    @Test
    void replaysAnyProblemFromAGenome() throws Exception {
        send("{\"v\":1,\"type\":\"simulate\",\"id\":\"w\",\"problem\":{\"type\":\"warehouse-scheduling\","
                + "\"params\":{\"tasks\":\"builtin:orders-4\"}},\"genome\":[0,1,2,3,4,5,6,7]}");
        JsonNode sim = await("simulation", "w");
        int served = 0;
        for (JsonNode v : sim.path("replay").path("schedule")) {
            String kind = v.path("kind").asText();
            if (kind.equals("pickup") || kind.equals("delivery")) {
                served++;
            }
        }
        assertEquals(8, served, "every pickup and delivery appears once (charging stops may be added)");
        assertTrue(sim.path("replay").path("trajectory").path("x").size() > 10);
        assertEquals(3, sim.path("objectives").size());
    }

    @Test
    void capabilitiesDescribeRepresentationsAndObjectives() throws Exception {
        JsonNode ready = await("ready", null);
        boolean sawPermutationProblem = false;
        for (JsonNode p : ready.path("problems")) {
            if (p.path("representation").asText().equals("permutation")) {
                sawPermutationProblem = true;
                assertTrue(p.path("operators").path("crossover").toString().contains("pmx"));
            }
        }
        assertTrue(sawPermutationProblem);
        for (JsonNode a : ready.path("algorithms")) {
            if (a.path("id").asText().equals("nsga2")) {
                assertTrue(a.path("multiObjective").asBoolean());
            }
        }
    }
}
