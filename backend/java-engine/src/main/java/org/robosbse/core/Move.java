package org.robosbse.core;

/**
 * A neighbourhood move for tabu search.
 *
 * @param result    the neighbouring solution
 * @param attribute what the move changes (e.g. "gene 3 up", "swap elements 2 and 7")
 * @param inverse   attribute of the move that undoes this one; it becomes tabu after the move
 */
public record Move<S>(S result, Object attribute, Object inverse) {
}
