package org.robosbse.patching.sandbox;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.robosbse.protocol.Json;

/**
 * Parent side of candidate execution: a pool of {@link CandidateWorker} child JVMs.
 *
 * <p>Isolation: every candidate runs in a separate process started with a heap limit
 * ({@code -Xmx256m}), a single-CPU hint and the serial collector. The parent waits at most
 * {@code timeoutMillis} for a result; on timeout the worker is killed and replaced, and the candidate
 * is reported as timed out. A worker that dies (exit, out of memory) is reported as a crash and
 * replaced. Java 21 has no security manager, so the processes are not file-system sandboxes; the
 * restricted patch operators, which cannot introduce new calls, are what limits candidate behaviour.
 *
 * <p>Results are cached by source text, so re-evaluating a patch the search already tried costs no
 * compilation. Thread-safe: concurrent evaluations use different workers.
 */
public final class WorkerPool implements AutoCloseable {

    private static final Set<Process> LIVE = ConcurrentHashMap.newKeySet();
    private static final String EOF = "\u0000EOF";

    static {
        Runtime.getRuntime().addShutdownHook(new Thread(() -> LIVE.forEach(Process::destroyForcibly)));
    }

    private final String setupLine;
    private final int maxWorkers;
    private final long timeoutMillis;
    private final BlockingQueue<Worker> idle = new LinkedBlockingQueue<>();
    private int created;
    private boolean closed;
    private final List<Worker> all = new ArrayList<>();
    private final Map<String, JsonNode> cache = new ConcurrentHashMap<>();
    private final AtomicLong evaluated = new AtomicLong();
    private final AtomicLong cacheHits = new AtomicLong();
    private final AtomicLong timeouts = new AtomicLong();
    private final AtomicLong crashes = new AtomicLong();
    private final AtomicLong compileErrors = new AtomicLong();

    public WorkerPool(ObjectNode setup, int maxWorkers, long timeoutMillis) {
        ObjectNode s = setup.deepCopy();
        s.put("type", "setup");
        s.put("id", "setup");
        this.setupLine = Json.compact(s);
        this.maxWorkers = Math.max(1, maxWorkers);
        this.timeoutMillis = timeoutMillis;
    }

    /**
     * Compiles and tests a candidate. The result has {@code compiled}, {@code tests[]} (name, passed,
     * loss, violated, outcome), optional {@code episodes[]}, and {@code timeout}/{@code crashed} flags.
     */
    public JsonNode evaluate(String className, String source, boolean withEpisodes) {
        String key = digest(className + "\n" + withEpisodes + "\n" + source);
        JsonNode cached = cache.get(key);
        if (cached != null) {
            cacheHits.incrementAndGet();
            return cached;
        }
        ObjectNode request = Json.object();
        request.put("type", "evaluate");
        request.put("id", key.substring(0, 12));
        request.put("className", className);
        request.put("source", source);
        request.put("episodes", withEpisodes);
        Worker worker = acquire();
        JsonNode result;
        try {
            String line = worker.request(Json.compact(request), timeoutMillis);
            if (line == null) {
                timeouts.incrementAndGet();
                worker.kill();
                result = failure("timeout", "no result within " + timeoutMillis + " ms (worker killed)");
            } else if (line.equals(EOF)) {
                crashes.incrementAndGet();
                worker.kill();
                result = failure("crashed", "the worker process died: " + worker.lastError());
            } else {
                result = Json.parse(line);
                if (!result.path("compiled").asBoolean(false)) {
                    compileErrors.incrementAndGet();
                }
            }
        } finally {
            release(worker);
        }
        evaluated.incrementAndGet();
        cache.put(key, result);
        return result;
    }

    private static ObjectNode failure(String flag, String message) {
        ObjectNode out = Json.object();
        out.put("compiled", true);
        out.put(flag, true);
        out.put("error", message);
        return out;
    }

    private Worker acquire() {
        synchronized (this) {
            if (closed) {
                throw new IllegalStateException("worker pool is closed");
            }
            Worker w = idle.poll();
            if (w != null) {
                return w;
            }
            if (created < maxWorkers) {
                created++;
                Worker fresh = start();
                all.add(fresh);
                return fresh;
            }
        }
        try {
            return idle.take();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted while waiting for a candidate worker", e);
        }
    }

    private void release(Worker w) {
        if (w.dead()) {
            Worker replacement;
            synchronized (this) {
                all.remove(w);
                if (closed) {
                    return;
                }
                replacement = start();
                all.add(replacement);
            }
            idle.add(replacement);
        } else {
            idle.add(w);
        }
    }

    private Worker start() {
        String java = ProcessHandle.current().info().command().orElse("java");
        List<String> cmd = List.of(java, "-Xmx256m", "-Xss4m", "-XX:+UseSerialGC", "-XX:ActiveProcessorCount=1",
                "-XX:TieredStopAtLevel=1", "-Djava.awt.headless=true", "-Dfile.encoding=UTF-8",
                "-cp", System.getProperty("java.class.path"), CandidateWorker.class.getName());
        Worker w = new Worker(cmd);
        String ready = w.request(setupLine, 120_000);
        if (ready == null || ready.equals(EOF) || !Json.parse(ready).path("type").asText().equals("ready")) {
            w.kill();
            throw new IllegalStateException("candidate worker failed to start: "
                    + (ready == null ? "timeout" : ready.equals(EOF) ? w.lastError() : ready));
        }
        return w;
    }

    public Map<String, Long> stats() {
        return Map.of("evaluated", evaluated.get(), "cacheHits", cacheHits.get(), "timeouts", timeouts.get(),
                "crashes", crashes.get(), "compileErrors", compileErrors.get());
    }

    @Override
    public void close() {
        synchronized (this) {
            closed = true;
            for (Worker w : all) {
                w.kill();
            }
            all.clear();
            idle.clear();
        }
    }

    private static String digest(String text) {
        try {
            return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8)));
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
    }

    /** One child JVM with a reader thread that turns its stdout into a queue of lines. */
    private static final class Worker {
        private final Process process;
        private final BufferedWriter in;
        private final BlockingQueue<String> lines = new LinkedBlockingQueue<>();
        private final StringBuilder stderr = new StringBuilder();
        private volatile boolean dead;

        Worker(List<String> cmd) {
            try {
                process = new ProcessBuilder(cmd).start();
            } catch (IOException e) {
                throw new IllegalStateException("cannot start a candidate worker: " + e.getMessage(), e);
            }
            LIVE.add(process);
            in = new BufferedWriter(new OutputStreamWriter(process.getOutputStream(), StandardCharsets.UTF_8));
            Thread out = new Thread(() -> {
                try (BufferedReader r = new BufferedReader(new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8))) {
                    String line;
                    while ((line = r.readLine()) != null) {
                        lines.add(line);
                    }
                } catch (IOException ignored) {
                    // Process killed.
                }
                lines.add(EOF);
            }, "robosbse-worker-out");
            out.setDaemon(true);
            out.start();
            Thread err = new Thread(() -> {
                try (BufferedReader r = new BufferedReader(new InputStreamReader(process.getErrorStream(), StandardCharsets.UTF_8))) {
                    String line;
                    while ((line = r.readLine()) != null) {
                        synchronized (stderr) {
                            if (stderr.length() < 4000) {
                                stderr.append(line).append('\n');
                            }
                        }
                    }
                } catch (IOException ignored) {
                    // Process killed.
                }
            }, "robosbse-worker-err");
            err.setDaemon(true);
            err.start();
        }

        /** Sends one line and waits for one reply; null on timeout, {@link #EOF} if the process ended. */
        String request(String line, long timeoutMillis) {
            if (dead) {
                return EOF;
            }
            try {
                in.write(line);
                in.write('\n');
                in.flush();
                String reply = lines.poll(timeoutMillis, TimeUnit.MILLISECONDS);
                if (reply == null) {
                    dead = true;
                } else if (reply.equals(EOF)) {
                    dead = true;
                }
                return reply;
            } catch (IOException e) {
                dead = true;
                return EOF;
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                dead = true;
                return null;
            }
        }

        boolean dead() {
            return dead;
        }

        String lastError() {
            synchronized (stderr) {
                String s = stderr.toString().trim();
                return s.isEmpty() ? "(no output)" : s.substring(Math.max(0, s.length() - 500));
            }
        }

        void kill() {
            dead = true;
            process.destroyForcibly();
            LIVE.remove(process);
        }
    }
}
