package org.robosbse.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.robosbse.protocol.Json;
import org.robosbse.simulation.World;
import org.robosbse.simulation.WorldMap;

class GeometryTest {

    @Test
    void circleSignedDistanceAndRaycast() {
        Circle c = new Circle(5, 5, 1);
        assertEquals(2.0, c.signedDistance(8, 5), 1e-12);
        assertEquals(-1.0, c.signedDistance(5, 5), 1e-12);
        assertEquals(3.0, c.raycast(1, 5, 1, 0), 1e-12);
        assertEquals(Double.POSITIVE_INFINITY, c.raycast(1, 5, -1, 0));
        assertEquals(Double.POSITIVE_INFINITY, c.raycast(1, 7, 1, 0));
        assertEquals(0.0, c.raycast(5, 5.5, 1, 0));
    }

    @Test
    void rectSignedDistanceAndRaycast() {
        Rect r = new Rect(2, 2, 4, 2);
        assertEquals(1.0, r.signedDistance(1, 3), 1e-12);
        assertEquals(Math.sqrt(2), r.signedDistance(1, 1), 1e-12);
        assertEquals(-0.5, r.signedDistance(4, 2.5), 1e-12);
        assertEquals(2.0, r.raycast(0, 3, 1, 0), 1e-12);
        double d = Math.sqrt(0.5);
        assertEquals(Math.sqrt(2), r.raycast(1, 1, d, d), 1e-9);
        assertEquals(Double.POSITIVE_INFINITY, r.raycast(0, 5, 1, 0));
    }

    @Test
    void angleWrapping() {
        assertEquals(Math.PI, Angles.wrap(-Math.PI), 1e-12);
        assertEquals(-Math.PI / 2, Angles.wrap(3 * Math.PI / 2), 1e-12);
        assertEquals(0.5, Angles.wrap(0.5 + 4 * Math.PI), 1e-12);
    }

    @Test
    void worldRaycastSeesBorderAndObstacles() {
        World w = new World(new WorldMap(null, "t", null, 10, 6, null, null, 0.3,
                List.of(new Circle(5, 3, 1))));
        assertEquals(3.0, w.raycast(1, 3, 0, 10), 1e-9);
        assertEquals(3.0, w.raycast(1, 3, Math.PI / 2, 10), 1e-9);
        assertEquals(1.0, w.raycast(1, 3, Math.PI, 10), 1e-9);
        assertEquals(2.0, w.raycast(1, 3, Math.PI, 2.0) + 1.0, 1e-9, "range is capped");
        assertTrue(w.collides(4.2, 3, 0.25));
        assertTrue(w.collides(0.1, 3, 0.25), "the border is a wall");
    }

    @Test
    void shapesRoundTripThroughJsonWithTypeTags() {
        WorldMap map = new WorldMap(null, "t", null, 10, 10, null, null, 0.3,
                List.of(new Circle(1, 2, 0.5), new Rect(3, 3, 1, 2)));
        String json = Json.compact(map);
        assertTrue(json.contains("\"type\":\"circle\""));
        assertTrue(json.contains("\"type\":\"rect\""));
        WorldMap back = Json.MAPPER.convertValue(Json.parse(json), WorldMap.class);
        assertEquals(map.obstacles(), back.obstacles());
    }
}
