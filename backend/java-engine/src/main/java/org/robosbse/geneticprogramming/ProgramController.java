package org.robosbse.geneticprogramming;

import org.robosbse.geometry.Angles;
import org.robosbse.simulation.Command;
import org.robosbse.simulation.Observation;
import org.robosbse.simulation.RobotSpec;
import org.robosbse.simulation.controllers.Controller;
import org.robosbse.simulation.controllers.ControllerSetup;

/**
 * Runs a {@link Program} as a robot controller: once per control tick the program is interpreted
 * with the current sensor features and its action becomes the command.
 *
 * <p>Actions: {@code forward} = (speed, 0), {@code turnLeft}/{@code turnRight} = (0, +-turn),
 * {@code veerLeft}/{@code veerRight} = (speed/2, +-turn/2), {@code stop} = (0, 0), with
 * {@code speed = 0.7 maxSpeed} and {@code turn = 0.8 maxTurnRate}; every command is therefore within
 * the robot's limits. Sensor features of a lost scan are those of the last valid scan.
 *
 * <p><b>Execution limit:</b> interpretation counts visited nodes and stops at {@code stepBudget} per
 * tick, returning {@code stop}. The language has no loops, so a program within the size limit never
 * reaches it; the guard exists so that no program, however it was produced, can run unbounded.
 */
public final class ProgramController implements Controller {

    public static final double OBSTACLE_AHEAD = 0.7;
    public static final double GOAL_AHEAD = 0.25;

    private final Program program;
    private final int stepBudget;
    private RobotSpec robot;
    private double goalX;
    private double goalY;
    private double speed;
    private double turn;
    private double front;
    private double left;
    private double right;
    private double bearing;
    private double distance;
    private int steps;
    private long budgetExhausted;

    public ProgramController(Program program, int stepBudget) {
        this.program = program;
        this.stepBudget = stepBudget;
    }

    @Override
    public String id() {
        return "gp-program";
    }

    @Override
    public boolean usesSensors() {
        return true;
    }

    @Override
    public void reset(ControllerSetup setup) {
        robot = setup.robot();
        goalX = setup.goal().x();
        goalY = setup.goal().y();
        speed = 0.7 * robot.maxSpeed();
        turn = 0.8 * robot.maxTurnRate();
        front = robot.sensorRange();
        left = robot.sensorRange();
        right = robot.sensorRange();
        budgetExhausted = 0;
    }

    @Override
    public Command step(Observation obs) {
        if (obs.scanValid()) {
            front = robot.sensorRange();
            left = robot.sensorRange();
            right = robot.sensorRange();
            for (int i = 0; i < obs.rays(); i++) {
                double a = Angles.wrap(robot.rayAngle(i));
                double r = obs.range(i);
                double deg = Math.toDegrees(a);
                if (Math.abs(deg) <= 30) {
                    front = Math.min(front, r);
                } else if (deg > 30 && deg <= 120) {
                    left = Math.min(left, r);
                } else if (deg < -30 && deg >= -120) {
                    right = Math.min(right, r);
                }
            }
        }
        double dx = goalX - obs.x();
        double dy = goalY - obs.y();
        bearing = Angles.wrap(Math.atan2(dy, dx) - obs.theta());
        distance = Math.hypot(dx, dy);
        steps = 0;
        Op action = action(program.root());
        if (steps > stepBudget) {
            budgetExhausted++;
            action = Op.STOP;
        }
        return switch (action) {
            case FORWARD -> Command.drive(speed, 0);
            case TURN_LEFT -> Command.drive(0, turn);
            case TURN_RIGHT -> Command.drive(0, -turn);
            case VEER_LEFT -> Command.drive(speed / 2, turn / 2);
            case VEER_RIGHT -> Command.drive(speed / 2, -turn / 2);
            default -> Command.drive(0, 0);
        };
    }

    /** Ticks in which the step budget was exhausted (0 for programs within the limits). */
    public long budgetExhausted() {
        return budgetExhausted;
    }

    private Op action(Node n) {
        if (++steps > stepBudget) {
            return Op.STOP;
        }
        if (n.op() == Op.IF) {
            return bool(n.children().get(0)) ? action(n.children().get(1)) : action(n.children().get(2));
        }
        return n.op();
    }

    private boolean bool(Node n) {
        if (++steps > stepBudget) {
            return false;
        }
        return switch (n.op()) {
            case OBSTACLE_AHEAD -> front < OBSTACLE_AHEAD;
            case GOAL_LEFT -> bearing > 0;
            case GOAL_AHEAD -> Math.abs(bearing) < GOAL_AHEAD;
            case LT -> num(n.children().get(0)) < num(n.children().get(1));
            case GT -> num(n.children().get(0)) > num(n.children().get(1));
            case AND -> bool(n.children().get(0)) && bool(n.children().get(1));
            case OR -> bool(n.children().get(0)) || bool(n.children().get(1));
            case NOT -> !bool(n.children().get(0));
            default -> throw new IllegalStateException("not a boolean: " + n.op());
        };
    }

    private double num(Node n) {
        if (++steps > stepBudget) {
            return 0;
        }
        return switch (n.op()) {
            case FRONT -> front;
            case LEFT -> left;
            case RIGHT -> right;
            case GOAL_BEARING -> bearing;
            case GOAL_DISTANCE -> distance;
            case CONST -> n.value();
            case ADD -> num(n.children().get(0)) + num(n.children().get(1));
            case SUB -> num(n.children().get(0)) - num(n.children().get(1));
            case MUL -> num(n.children().get(0)) * num(n.children().get(1));
            default -> throw new IllegalStateException("not a number: " + n.op());
        };
    }
}
