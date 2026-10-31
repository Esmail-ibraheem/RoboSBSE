package org.robosbse.geometry;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

/**
 * Static obstacle geometry. Shapes are serialised with a {@code "type"} discriminator
 * ({@code "circle"} or {@code "rect"}), matching the map file format shared with the UI.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, property = "type")
@JsonSubTypes({
        @JsonSubTypes.Type(value = Circle.class, name = "circle"),
        @JsonSubTypes.Type(value = Rect.class, name = "rect")
})
public sealed interface Shape permits Circle, Rect {

    /** Signed distance from the point to the shape boundary: negative inside, positive outside. */
    double signedDistance(double px, double py);

    /**
     * Distance along the unit direction (dx, dy) from (ox, oy) to the first boundary hit;
     * 0 if the origin is inside; {@link Double#POSITIVE_INFINITY} if the ray misses.
     */
    double raycast(double ox, double oy, double dx, double dy);

    /** Centre of the bounding circle. */
    Vec2 boundingCentre();

    /** Radius of the bounding circle. */
    double boundingRadius();
}
