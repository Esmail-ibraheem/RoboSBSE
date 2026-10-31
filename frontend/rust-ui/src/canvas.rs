//! The 2D world view: draws maps, routes, trajectories, the robot and its sensor rays, and hosts
//! the interactive environment editor. World coordinates are metres with y pointing up.

use eframe::egui::{self, Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, Vec2};

use crate::model::{Point, Pose, Shape, Trajectory, WorldMap};

#[derive(Clone, Debug)]
pub struct View {
    pub zoom: f32,
    pub pan: Vec2,
}

impl Default for View {
    fn default() -> Self {
        View { zoom: 1.0, pan: Vec2::ZERO }
    }
}

impl View {
    pub fn reset(&mut self) {
        *self = View::default();
    }
}

/// World-to-screen transform for one frame.
#[derive(Clone, Copy, Debug)]
pub struct Xf {
    centre: Pos2,
    scale: f32,
    w: f64,
    h: f64,
}

impl Xf {
    pub fn to_screen(&self, x: f64, y: f64) -> Pos2 {
        Pos2::new(
            self.centre.x + ((x - self.w / 2.0) as f32) * self.scale,
            self.centre.y - ((y - self.h / 2.0) as f32) * self.scale,
        )
    }

    pub fn to_world(&self, p: Pos2) -> (f64, f64) {
        (
            ((p.x - self.centre.x) / self.scale) as f64 + self.w / 2.0,
            (-(p.y - self.centre.y) / self.scale) as f64 + self.h / 2.0,
        )
    }

    pub fn len(&self, metres: f64) -> f32 {
        metres as f32 * self.scale
    }
}

/// Kind of a task marker (warehouse scheduling).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarkerKind {
    Pickup,
    Delivery,
    Depot,
    Charger,
}

/// A labelled point of interest drawn on the map.
#[derive(Clone, Debug)]
pub struct Marker {
    pub x: f64,
    pub y: f64,
    pub label: String,
    pub kind: MarkerKind,
}

/// One robot of a fleet replay (multi-robot task allocation).
#[derive(Clone, Debug, Default)]
pub struct FleetTrack {
    pub label: String,
    pub trajectory: Trajectory,
    pub route: Vec<[f64; 2]>,
}

/// Two robots closer than allowed during `[start, end]` (detected after simulation, not resolved).
#[derive(Clone, Debug)]
pub struct Conflict {
    pub robots: (usize, usize),
    pub x: f64,
    pub y: f64,
    pub start: f64,
    pub end: f64,
}

/// Distinct colours for the robots of a fleet.
pub const FLEET: [Color32; 6] = [
    Color32::from_rgb(56, 189, 222),
    Color32::from_rgb(236, 120, 190),
    Color32::from_rgb(120, 200, 90),
    Color32::from_rgb(170, 130, 250),
    Color32::from_rgb(250, 160, 70),
    Color32::from_rgb(90, 140, 255),
];

/// Things drawn on top of the map.
#[derive(Default)]
pub struct Overlay<'a> {
    /// Task markers (pickups, deliveries, depot, charger).
    pub markers: &'a [Marker],
    /// Straight-line visiting order (e.g. a live schedule), drawn dashed.
    pub sequence: Option<&'a [[f64; 2]]>,
    /// Number the intermediate points of `route` (navigation waypoints).
    pub number_waypoints: bool,
    /// Hide the map's own start pose and goal (problems that define their own, e.g. scheduling).
    pub hide_start_goal: bool,
    /// Obstacles added by a test scenario (drawn in a warning colour).
    pub extra_obstacles: &'a [Shape],
    /// Start/goal overrides (test scenarios define their own).
    pub start: Option<&'a Pose>,
    pub goal: Option<&'a Point>,
    /// Conventional reference route (A*).
    pub reference: Option<&'a [[f64; 2]]>,
    /// Faint alternative candidate routes.
    pub candidates: &'a [Vec<[f64; 2]>],
    /// The candidate route being shown (including start and goal).
    pub route: Option<&'a [[f64; 2]]>,
    pub trajectory: Option<&'a Trajectory>,
    pub time: Option<f64>,
    pub show_rays: bool,
    pub robot_radius: f64,
    pub sensor_range: f64,
    pub caption: Option<String>,
    /// Several robots replayed together (multi-robot task allocation).
    pub fleet: &'a [FleetTrack],
    pub conflicts: &'a [Conflict],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Select,
    Rect,
    Circle,
    Start,
    Goal,
    Delete,
}

impl Tool {
    pub const ALL: [Tool; 6] = [Tool::Select, Tool::Rect, Tool::Circle, Tool::Start, Tool::Goal, Tool::Delete];

    pub fn label(self) -> &'static str {
        match self {
            Tool::Select => "Select / move",
            Tool::Rect => "Rectangle",
            Tool::Circle => "Circle",
            Tool::Start => "Robot start",
            Tool::Goal => "Goal",
            Tool::Delete => "Delete",
        }
    }

    pub fn hint(self) -> &'static str {
        match self {
            Tool::Select => "Drag obstacles, the start or the goal. Del removes the selection.",
            Tool::Rect => "Drag to draw a rectangle.",
            Tool::Circle => "Drag from the centre to set the radius.",
            Tool::Start => "Click to place the robot, drag to set its heading.",
            Tool::Goal => "Click to place the goal.",
            Tool::Delete => "Click an obstacle to remove it.",
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Drag {
    Obstacle { index: usize, last: (f64, f64) },
    Start,
    Goal,
    Create { origin: (f64, f64) },
    Heading,
}

pub struct Editor {
    pub tool: Tool,
    pub selected: Option<usize>,
    pub snap: bool,
    drag: Option<Drag>,
}

impl Default for Editor {
    fn default() -> Self {
        Editor { tool: Tool::Select, selected: None, snap: true, drag: None }
    }
}

pub struct CanvasOutput {
    pub changed: bool,
    /// Transform and area of this frame, for pages that draw their own scene on top.
    pub xf: Xf,
    pub rect: Rect,
    /// World position of a primary click (without an editor).
    pub clicked: Option<(f64, f64)>,
}

pub struct Palette {
    pub background: Color32,
    pub floor: Color32,
    pub grid: Color32,
    pub grid_major: Color32,
    pub obstacle: Color32,
    pub obstacle_edge: Color32,
    pub scenario_obstacle: Color32,
    pub text: Color32,
}

impl Palette {
    pub fn for_mode(dark: bool) -> Palette {
        if dark {
            Palette {
                background: Color32::from_rgb(14, 17, 22),
                floor: Color32::from_rgb(24, 29, 37),
                grid: Color32::from_rgb(33, 39, 49),
                grid_major: Color32::from_rgb(48, 56, 69),
                obstacle: Color32::from_rgb(86, 95, 110),
                obstacle_edge: Color32::from_rgb(136, 146, 163),
                scenario_obstacle: Color32::from_rgb(150, 82, 62),
                text: Color32::from_rgb(200, 207, 218),
            }
        } else {
            Palette {
                background: Color32::from_rgb(226, 230, 236),
                floor: Color32::from_rgb(250, 251, 252),
                grid: Color32::from_rgb(232, 235, 240),
                grid_major: Color32::from_rgb(208, 213, 221),
                obstacle: Color32::from_rgb(120, 129, 143),
                obstacle_edge: Color32::from_rgb(70, 78, 92),
                scenario_obstacle: Color32::from_rgb(196, 110, 84),
                text: Color32::from_rgb(40, 46, 56),
            }
        }
    }
}

pub const ROUTE: Color32 = Color32::from_rgb(56, 189, 222);
pub const REFERENCE: Color32 = Color32::from_rgb(232, 170, 60);
pub const TRAIL: Color32 = Color32::from_rgb(90, 150, 255);
pub const START: Color32 = Color32::from_rgb(70, 200, 120);
pub const GOAL: Color32 = Color32::from_rgb(240, 190, 60);
pub const DANGER: Color32 = Color32::from_rgb(235, 80, 70);
pub const RAY: Color32 = Color32::from_rgba_premultiplied(60, 160, 90, 110);
pub const RESTRICTED: Color32 = Color32::from_rgb(220, 70, 70);
pub const TERRAIN: Color32 = Color32::from_rgb(176, 132, 72);
pub const MOVING: Color32 = Color32::from_rgb(230, 120, 40);

fn shape_centre(shape: &Shape) -> (f64, f64) {
    match *shape {
        Shape::Circle { x, y, .. } => (x, y),
        Shape::Rect { x, y, w, h } => (x + w / 2.0, y + h / 2.0),
    }
}

fn dashed_shape(painter: &egui::Painter, xf: &Xf, shape: &Shape, stroke: Stroke) {
    let pts: Vec<Pos2> = match *shape {
        Shape::Circle { x, y, r } => (0..=48)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 48.0;
                xf.to_screen(x + r * a.cos(), y + r * a.sin())
            })
            .collect(),
        Shape::Rect { x, y, w, h } => [(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)]
            .iter()
            .map(|p| xf.to_screen(p.0, p.1))
            .collect(),
    };
    painter.extend(egui::Shape::dashed_line(&pts, stroke, 6.0, 4.0));
}

/// Zones, timed obstacles and moving obstacles of a dynamic map at time `t`.
fn draw_dynamic(painter: &egui::Painter, xf: &Xf, map: &WorldMap, t: f64, palette: &Palette) {
    for z in &map.zones {
        let restricted = z.kind != "terrain";
        let color = if restricted { RESTRICTED } else { TERRAIN };
        draw_shape(painter, xf, &z.shape, color.gamma_multiply(0.14), Stroke::NONE);
        dashed_shape(painter, xf, &z.shape, Stroke::new(1.5, color.gamma_multiply(0.8)));
        let (cx, cy) = shape_centre(&z.shape);
        let text = match (&z.label, restricted) {
            (Some(l), true) => format!("{l} (restricted)"),
            (None, true) => "restricted".to_string(),
            (Some(l), false) => format!("{l} (x{:.2} speed)", z.speed_factor),
            (None, false) => format!("terrain x{:.2}", z.speed_factor),
        };
        painter.text(xf.to_screen(cx, cy), Align2::CENTER_CENTER, text, FontId::proportional(11.0), color);
    }
    for o in &map.timed_obstacles {
        let window = match o.until {
            Some(u) => format!("{:.0}-{:.0} s", o.from, u),
            None => format!("from {:.0} s", o.from),
        };
        let (cx, cy) = shape_centre(&o.shape);
        if o.active(t) {
            draw_shape(painter, xf, &o.shape, MOVING.gamma_multiply(0.55), Stroke::new(1.5, MOVING));
        } else {
            dashed_shape(painter, xf, &o.shape, Stroke::new(1.0, MOVING.gamma_multiply(0.6)));
        }
        let label = format!("{} {window}", o.label.as_deref().unwrap_or("timed"));
        painter.text(xf.to_screen(cx, cy), Align2::CENTER_CENTER, label, FontId::proportional(10.0), palette.text);
    }
    for m in &map.moving_obstacles {
        let mut pts: Vec<Pos2> = m.path.iter().map(|p| xf.to_screen(p.x, p.y)).collect();
        if m.mode == "loop" && m.path.len() > 1 {
            pts.push(pts[0]);
        }
        if pts.len() > 1 {
            painter.extend(egui::Shape::dashed_line(&pts, Stroke::new(1.0, MOVING.gamma_multiply(0.45)), 4.0, 4.0));
        }
        let (x, y) = m.position(t);
        let c = xf.to_screen(x, y);
        painter.circle(c, xf.len(m.radius).max(3.0), MOVING.gamma_multiply(0.75), Stroke::new(1.5, MOVING));
        if let Some(l) = &m.label {
            painter.text(c + Vec2::new(0.0, -xf.len(m.radius) - 3.0), Align2::CENTER_BOTTOM, l,
                FontId::proportional(10.0), MOVING);
        }
    }
}

fn snap(v: f64, on: bool) -> f64 {
    if on {
        (v * 10.0).round() / 10.0
    } else {
        v
    }
}

fn draw_shape(painter: &egui::Painter, xf: &Xf, shape: &Shape, fill: Color32, edge: Stroke) {
    match *shape {
        Shape::Circle { x, y, r } => {
            painter.circle(xf.to_screen(x, y), xf.len(r), fill, edge);
        }
        Shape::Rect { x, y, w, h } => {
            let rect = Rect::from_two_pos(xf.to_screen(x, y), xf.to_screen(x + w, y + h));
            painter.rect(rect, 0.0, fill, edge, egui::StrokeKind::Inside);
        }
    }
}

fn polyline(painter: &egui::Painter, xf: &Xf, pts: &[[f64; 2]], stroke: Stroke) {
    if pts.len() >= 2 {
        painter.line(pts.iter().map(|p| xf.to_screen(p[0], p[1])).collect(), stroke);
    }
}

fn robot(painter: &egui::Painter, xf: &Xf, x: f64, y: f64, theta: f64, radius: f64, body: Color32, edge: Color32) {
    let c = xf.to_screen(x, y);
    let r = xf.len(radius).max(3.0);
    painter.circle(c, r, body, Stroke::new(1.5, edge));
    let tip = xf.to_screen(x + radius * theta.cos(), y + radius * theta.sin());
    painter.line_segment([c, tip], Stroke::new(2.0, edge));
}

/// Draws the world and handles editing. `map` is only modified when `editor` is given.
pub fn show(
    ui: &mut egui::Ui,
    view: &mut View,
    map: &mut WorldMap,
    overlay: &Overlay,
    mut editor: Option<&mut Editor>,
    dark: bool,
) -> CanvasOutput {
    let palette = Palette::for_mode(dark);
    // Clamp to the visible area: a too-wide sibling row can widen the parent's max rect, which would
    // otherwise push the canvas under neighbouring panels.
    let rect = ui.available_rect_before_wrap().intersect(ui.clip_rect());
    let response = ui.allocate_rect(rect, Sense::click_and_drag());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, palette.background);

    // Zoom around the cursor, pan with the right or middle mouse button.
    let base = (rect.width() / map.width as f32).min(rect.height() / map.height as f32) * 0.94;
    if response.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll != 0.0 {
            let factor = (scroll * 0.0015).exp();
            let new_zoom = (view.zoom * factor).clamp(0.2, 20.0);
            if let Some(mouse) = response.hover_pos() {
                let anchor = mouse - (rect.center() + view.pan);
                view.pan -= anchor * (new_zoom / view.zoom - 1.0);
            }
            view.zoom = new_zoom;
        }
    }
    if response.dragged_by(egui::PointerButton::Secondary) || response.dragged_by(egui::PointerButton::Middle) {
        view.pan += response.drag_delta();
    }
    let xf = Xf { centre: rect.center() + view.pan, scale: base * view.zoom, w: map.width, h: map.height };
    let painter = painter.with_clip_rect(rect);

    // Floor and grid.
    let floor = Rect::from_two_pos(xf.to_screen(0.0, 0.0), xf.to_screen(map.width, map.height));
    painter.rect_filled(floor, 0.0, palette.floor);
    if xf.scale >= 6.0 {
        let mut gx = 0.0;
        while gx <= map.width + 1e-9 {
            let major = (gx as i64) % 5 == 0;
            painter.line_segment(
                [xf.to_screen(gx, 0.0), xf.to_screen(gx, map.height)],
                Stroke::new(1.0, if major { palette.grid_major } else { palette.grid }),
            );
            gx += 1.0;
        }
        let mut gy = 0.0;
        while gy <= map.height + 1e-9 {
            let major = (gy as i64) % 5 == 0;
            painter.line_segment(
                [xf.to_screen(0.0, gy), xf.to_screen(map.width, gy)],
                Stroke::new(1.0, if major { palette.grid_major } else { palette.grid }),
            );
            gy += 1.0;
        }
    }
    painter.rect_stroke(floor, 0.0, Stroke::new(2.0, palette.obstacle_edge), egui::StrokeKind::Outside);

    let now = overlay.time.unwrap_or(0.0);
    draw_dynamic(&painter, &xf, map, now, &palette);
    let selected = editor.as_ref().and_then(|e| e.selected);
    for (i, shape) in map.obstacles.iter().enumerate() {
        let edge = if Some(i) == selected {
            Stroke::new(2.5, GOAL)
        } else {
            Stroke::new(1.0, palette.obstacle_edge)
        };
        draw_shape(&painter, &xf, shape, palette.obstacle, edge);
    }
    for shape in overlay.extra_obstacles {
        draw_shape(&painter, &xf, shape, palette.scenario_obstacle, Stroke::new(1.0, DANGER));
    }

    // Goal and start.
    let goal = overlay.goal.cloned().unwrap_or_else(|| map.goal.clone());
    let start = overlay.start.cloned().unwrap_or_else(|| map.start.clone());
    let radius = if overlay.robot_radius > 0.0 { overlay.robot_radius } else { 0.25 };
    if !overlay.hide_start_goal {
        let gp = xf.to_screen(goal.x, goal.y);
        painter.circle(gp, xf.len(map.goal_tolerance).max(4.0), GOAL.gamma_multiply(0.18), Stroke::new(2.0, GOAL));
        painter.line_segment([gp - Vec2::new(5.0, 0.0), gp + Vec2::new(5.0, 0.0)], Stroke::new(1.5, GOAL));
        painter.line_segment([gp - Vec2::new(0.0, 5.0), gp + Vec2::new(0.0, 5.0)], Stroke::new(1.5, GOAL));
        robot(&painter, &xf, start.x, start.y, start.theta, radius, START.gamma_multiply(0.35), START);
    }

    // Task markers and visiting order.
    if let Some(seq) = overlay.sequence {
        let pts: Vec<Pos2> = seq.iter().map(|p| xf.to_screen(p[0], p[1])).collect();
        painter.extend(egui::Shape::dashed_line(&pts, Stroke::new(1.5, ROUTE.gamma_multiply(0.7)), 6.0, 4.0));
    }
    for m in overlay.markers {
        let c = xf.to_screen(m.x, m.y);
        let (color, text_offset) = match m.kind {
            MarkerKind::Pickup => (Color32::from_rgb(80, 150, 255), Vec2::new(7.0, -7.0)),
            MarkerKind::Delivery => (Color32::from_rgb(240, 150, 60), Vec2::new(7.0, 7.0)),
            MarkerKind::Depot => (palette.text, Vec2::new(8.0, -8.0)),
            MarkerKind::Charger => (START, Vec2::new(8.0, -8.0)),
        };
        match m.kind {
            MarkerKind::Pickup => {
                painter.rect_filled(Rect::from_center_size(c, Vec2::splat(9.0)), 1.0, color);
            }
            MarkerKind::Delivery => {
                painter.add(egui::Shape::convex_polygon(
                    vec![c + Vec2::new(0.0, -6.0), c + Vec2::new(6.0, 5.0), c + Vec2::new(-6.0, 5.0)],
                    color,
                    Stroke::NONE,
                ));
            }
            MarkerKind::Depot => {
                painter.rect_stroke(Rect::from_center_size(c, Vec2::splat(14.0)), 2.0, Stroke::new(2.0, color),
                    egui::StrokeKind::Middle);
            }
            MarkerKind::Charger => {
                painter.circle(c, 8.0, color.gamma_multiply(0.3), Stroke::new(2.0, color));
            }
        }
        let anchor = if text_offset.y > 0.0 { Align2::LEFT_TOP } else { Align2::LEFT_BOTTOM };
        painter.text(c + text_offset, anchor, &m.label, FontId::proportional(11.0), color);
    }

    // Routes.
    for c in overlay.candidates {
        polyline(&painter, &xf, c, Stroke::new(1.0, ROUTE.gamma_multiply(0.25)));
    }
    if let Some(reference) = overlay.reference {
        let pts: Vec<Pos2> = reference.iter().map(|p| xf.to_screen(p[0], p[1])).collect();
        painter.extend(egui::Shape::dashed_line(&pts, Stroke::new(2.0, REFERENCE), 8.0, 5.0));
    }
    if let Some(route) = overlay.route {
        polyline(&painter, &xf, route, Stroke::new(2.0, ROUTE));
        let numbered = if overlay.number_waypoints { route.len().saturating_sub(2) } else { 0 };
        for (k, p) in route.iter().enumerate().skip(1).take(numbered) {
            let s = xf.to_screen(p[0], p[1]);
            painter.circle_filled(s, 4.0, ROUTE);
            painter.text(s + Vec2::new(6.0, -6.0), Align2::LEFT_BOTTOM, format!("{k}"), FontId::monospace(11.0), ROUTE);
        }
    }

    // Executed trajectory, robot and rays.
    if let Some(traj) = overlay.trajectory {
        let n = traj.len();
        if n >= 2 {
            let all: Vec<Pos2> = (0..n).map(|i| xf.to_screen(traj.x[i], traj.y[i])).collect();
            painter.line(all.clone(), Stroke::new(1.0, TRAIL.gamma_multiply(0.35)));
            if let Some(frame) = overlay.time.and_then(|t| traj.frame_at(t)) {
                let mut done: Vec<Pos2> = all[..=frame.index].to_vec();
                done.push(xf.to_screen(frame.x, frame.y));
                painter.line(done, Stroke::new(2.5, TRAIL));
                for (i, valid) in traj.scan_valid.iter().enumerate().take(frame.index + 1) {
                    if !valid {
                        painter.circle_filled(all[i], 2.0, DANGER.gamma_multiply(0.8));
                    }
                }
                if overlay.show_rays {
                    if let Some(rays) = traj.rays_at(frame.index) {
                        let count = rays.len().max(1);
                        for (k, r) in rays.iter().enumerate() {
                            if !r.is_finite() {
                                continue;
                            }
                            let a = frame.theta + std::f64::consts::TAU * k as f64 / count as f64;
                            let hit = (*r as f64) < overlay.sensor_range - 1e-6;
                            let end = xf.to_screen(frame.x + *r as f64 * a.cos(), frame.y + *r as f64 * a.sin());
                            let color = if hit { RAY } else { RAY.gamma_multiply(0.35) };
                            painter.line_segment([xf.to_screen(frame.x, frame.y), end], Stroke::new(1.0, color));
                            if hit {
                                painter.circle_filled(end, 2.5, Color32::from_rgb(90, 220, 120));
                            }
                        }
                    }
                }
                let body = if frame.clearance.is_finite() && frame.clearance < 0.0 { DANGER } else { palette.text };
                robot(&painter, &xf, frame.x, frame.y, frame.theta, radius, body.gamma_multiply(0.85), body);
                if !frame.scan_valid {
                    let c = xf.to_screen(frame.x, frame.y);
                    painter.circle_stroke(c, xf.len(radius) + 6.0, Stroke::new(2.0, DANGER));
                    painter.text(c + Vec2::new(0.0, -xf.len(radius) - 10.0), Align2::CENTER_BOTTOM, "SCAN LOST",
                        FontId::proportional(12.0), DANGER);
                }
            }
        }
    }

    // Fleet replay: every robot's route, its trail so far and its body; conflicts as rings.
    for (k, track) in overlay.fleet.iter().enumerate() {
        let color = FLEET[k % FLEET.len()];
        polyline(&painter, &xf, &track.route, Stroke::new(1.0, color.gamma_multiply(0.35)));
        let traj = &track.trajectory;
        if let Some(frame) = traj.frame_at(now) {
            let mut done: Vec<Pos2> = (0..=frame.index.min(traj.len().saturating_sub(1)))
                .map(|i| xf.to_screen(traj.x[i], traj.y[i]))
                .collect();
            done.push(xf.to_screen(frame.x, frame.y));
            painter.line(done, Stroke::new(2.0, color));
            robot(&painter, &xf, frame.x, frame.y, frame.theta, radius, color.gamma_multiply(0.6), color);
            painter.text(xf.to_screen(frame.x, frame.y) + Vec2::new(0.0, xf.len(radius) + 3.0), Align2::CENTER_TOP,
                &track.label, FontId::proportional(11.0), color);
        }
    }
    for c in overlay.conflicts {
        let p = xf.to_screen(c.x, c.y);
        let active = now >= c.start - 0.05 && now <= c.end + 0.5;
        let r = xf.len(radius * 2.0).max(8.0);
        if active {
            painter.circle_stroke(p, r, Stroke::new(3.0, DANGER));
            painter.text(p + Vec2::new(0.0, -r - 3.0), Align2::CENTER_BOTTOM,
                format!("CONFLICT R{}-R{}", c.robots.0 + 1, c.robots.1 + 1), FontId::proportional(12.0), DANGER);
        } else {
            painter.circle_stroke(p, r * 0.6, Stroke::new(1.0, DANGER.gamma_multiply(0.6)));
        }
    }

    // Editing.
    let mut changed = false;
    let pointer = response.interact_pointer_pos().or(response.hover_pos());
    let world_pointer = pointer.map(|p| xf.to_world(p));
    if let Some(ed) = editor.as_deref_mut() {
        if let Some((wx, wy)) = world_pointer {
            let (sx, sy) = (snap(wx, ed.snap), snap(wy, ed.snap));
            if response.drag_started_by(egui::PointerButton::Primary) {
                // egui reports a drag only after the pointer moved past a threshold; anchor the
                // operation where the button went down, not where the drag was recognised.
                let (px, py) = ui.input(|i| i.pointer.press_origin()).map(|p| xf.to_world(p)).unwrap_or((wx, wy));
                let (ox, oy) = (snap(px, ed.snap), snap(py, ed.snap));
                let near = |qx: f64, qy: f64| (px - qx).hypot(py - qy) < (radius * 1.5).max(8.0 / xf.scale as f64);
                ed.drag = match ed.tool {
                    Tool::Select if near(map.start.x, map.start.y) => Some(Drag::Start),
                    Tool::Select if near(map.goal.x, map.goal.y) => Some(Drag::Goal),
                    Tool::Select => map.obstacle_at(px, py).map(|i| {
                        ed.selected = Some(i);
                        Drag::Obstacle { index: i, last: (ox, oy) }
                    }),
                    Tool::Rect | Tool::Circle => Some(Drag::Create { origin: (ox, oy) }),
                    Tool::Start => {
                        map.start.x = ox;
                        map.start.y = oy;
                        changed = true;
                        Some(Drag::Heading)
                    }
                    _ => None,
                };
            }
            if response.dragged_by(egui::PointerButton::Primary) {
                match ed.drag {
                    Some(Drag::Obstacle { index, last }) => {
                        if let Some(shape) = map.obstacles.get_mut(index) {
                            shape.translate(sx - last.0, sy - last.1);
                            changed = true;
                        }
                        ed.drag = Some(Drag::Obstacle { index, last: (sx, sy) });
                    }
                    Some(Drag::Start) => {
                        map.start.x = sx.clamp(0.0, map.width);
                        map.start.y = sy.clamp(0.0, map.height);
                        changed = true;
                    }
                    Some(Drag::Goal) => {
                        map.goal.x = sx.clamp(0.0, map.width);
                        map.goal.y = sy.clamp(0.0, map.height);
                        changed = true;
                    }
                    Some(Drag::Heading) => {
                        if (wx - map.start.x).hypot(wy - map.start.y) > 0.2 {
                            map.start.theta = (wy - map.start.y).atan2(wx - map.start.x);
                            changed = true;
                        }
                    }
                    Some(Drag::Create { origin }) => {
                        let preview = preview_shape(ed.tool, origin, (sx, sy));
                        if let Some(p) = preview {
                            draw_shape(&painter, &xf, &p, ROUTE.gamma_multiply(0.25), Stroke::new(1.5, ROUTE));
                        }
                    }
                    None => {}
                }
            }
            if response.drag_stopped() {
                if let Some(Drag::Create { origin }) = ed.drag {
                    if let Some(shape) = preview_shape(ed.tool, origin, (sx, sy)) {
                        map.obstacles.push(shape);
                        ed.selected = Some(map.obstacles.len() - 1);
                        changed = true;
                    }
                }
                ed.drag = None;
            }
            if response.clicked() {
                match ed.tool {
                    Tool::Select => ed.selected = map.obstacle_at(wx, wy),
                    Tool::Goal => {
                        map.goal = Point { x: sx, y: sy };
                        changed = true;
                    }
                    Tool::Start => {
                        map.start.x = sx;
                        map.start.y = sy;
                        changed = true;
                    }
                    Tool::Delete => {
                        if let Some(i) = map.obstacle_at(wx, wy) {
                            map.obstacles.remove(i);
                            ed.selected = None;
                            changed = true;
                        }
                    }
                    _ => {}
                }
            }
        }
        if let Some(i) = ed.selected {
            if ui.input(|inp| inp.key_pressed(egui::Key::Delete)) && i < map.obstacles.len() {
                map.obstacles.remove(i);
                ed.selected = None;
                changed = true;
            }
        }
    }

    // Caption, cursor read-out and scale bar.
    if let Some(caption) = &overlay.caption {
        painter.text(rect.left_top() + Vec2::new(10.0, 8.0), Align2::LEFT_TOP, caption, FontId::proportional(13.0),
            palette.text);
    }
    if let Some((wx, wy)) = response.hover_pos().map(|p| xf.to_world(p)) {
        painter.text(rect.left_bottom() + Vec2::new(10.0, -8.0), Align2::LEFT_BOTTOM,
            format!("x = {wx:6.2} m   y = {wy:6.2} m"), FontId::monospace(12.0), palette.text);
    }
    let bar_m = if xf.scale > 40.0 { 1.0 } else { 5.0 };
    let bar = xf.len(bar_m);
    let base_pt = rect.right_bottom() + Vec2::new(-16.0 - bar, -14.0);
    painter.line_segment([base_pt, base_pt + Vec2::new(bar, 0.0)], Stroke::new(2.0, palette.text));
    painter.text(base_pt + Vec2::new(bar / 2.0, -4.0), Align2::CENTER_BOTTOM, format!("{bar_m} m"),
        FontId::monospace(11.0), palette.text);

    let clicked = if editor.is_none() && response.clicked() { response.interact_pointer_pos().map(|p| xf.to_world(p)) } else { None };
    CanvasOutput { changed, xf, rect, clicked }
}

fn preview_shape(tool: Tool, a: (f64, f64), b: (f64, f64)) -> Option<Shape> {
    match tool {
        Tool::Rect => {
            let (x0, x1) = (a.0.min(b.0), a.0.max(b.0));
            let (y0, y1) = (a.1.min(b.1), a.1.max(b.1));
            (x1 - x0 >= 0.1 && y1 - y0 >= 0.1).then_some(Shape::Rect { x: x0, y: y0, w: x1 - x0, h: y1 - y0 })
        }
        Tool::Circle => {
            let r = (b.0 - a.0).hypot(b.1 - a.1);
            (r >= 0.1).then_some(Shape::Circle { x: a.0, y: a.1, r })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_round_trips_and_flips_y() {
        let xf = Xf { centre: Pos2::new(400.0, 300.0), scale: 20.0, w: 20.0, h: 10.0 };
        let p = xf.to_screen(15.0, 2.0);
        let (x, y) = xf.to_world(p);
        assert!((x - 15.0).abs() < 1e-4 && (y - 2.0).abs() < 1e-4);
        assert!(xf.to_screen(0.0, 10.0).y < xf.to_screen(0.0, 0.0).y, "y must point up");
    }

    /// Drives the real canvas headlessly: one frame per event batch, 800 x 600 points.
    struct Harness {
        ctx: egui::Context,
        view: View,
        editor: Editor,
        map: WorldMap,
        time: f64,
    }

    impl Harness {
        fn new(tool: Tool) -> Harness {
            Harness {
                ctx: egui::Context::default(),
                view: View::default(),
                editor: Editor { tool, ..Editor::default() },
                map: WorldMap::empty("t", 20.0, 20.0),
                time: 0.0,
            }
        }

        /// Screen position of a world point (canvas fills the screen; 0.94 * 600 / 20 px per metre).
        fn at(x: f64, y: f64) -> Pos2 {
            let s = 0.94 * 600.0 / 20.0;
            Pos2::new(400.0 + ((x - 10.0) * s) as f32, 300.0 - ((y - 10.0) * s) as f32)
        }

        fn frame(&mut self, events: Vec<egui::Event>) -> bool {
            self.time += 0.1;
            let raw = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0))),
                time: Some(self.time),
                events,
                ..Default::default()
            };
            let mut changed = false;
            let (view, map, editor) = (&mut self.view, &mut self.map, &mut self.editor);
            let mut output = self.ctx.run_ui(raw, |ui| {
                let overlay = Overlay::default();
                changed |= show(ui, view, map, &overlay, Some(editor), true).changed;
            });
            output.textures_delta.clear(); // no renderer in tests
            changed
        }

        fn button(pos: Pos2, pressed: bool) -> egui::Event {
            egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed, modifiers: Default::default() }
        }

        fn drag(&mut self, from: (f64, f64), to: (f64, f64)) {
            let (a, b) = (Self::at(from.0, from.1), Self::at(to.0, to.1));
            self.frame(vec![egui::Event::PointerMoved(a)]);
            self.frame(vec![Self::button(a, true)]);
            for k in 1..=4 {
                let p = a + (b - a) * (k as f32 / 4.0);
                self.frame(vec![egui::Event::PointerMoved(p)]);
            }
            self.frame(vec![Self::button(b, false)]);
            self.frame(vec![]);
        }

        fn click(&mut self, at: (f64, f64)) {
            let p = Self::at(at.0, at.1);
            self.frame(vec![egui::Event::PointerMoved(p)]);
            self.frame(vec![Self::button(p, true)]);
            self.frame(vec![Self::button(p, false)]);
            self.frame(vec![]);
        }
    }

    #[test]
    fn dragging_with_the_rectangle_tool_adds_an_obstacle() {
        let mut h = Harness::new(Tool::Rect);
        h.drag((2.0, 2.0), (5.0, 4.0));
        assert_eq!(h.map.obstacles.len(), 1);
        match h.map.obstacles[0] {
            Shape::Rect { x, y, w, h } => {
                assert!((x - 2.0).abs() < 0.11 && (y - 2.0).abs() < 0.11, "corner ({x}, {y})");
                assert!((w - 3.0).abs() < 0.11 && (h - 2.0).abs() < 0.11, "size {w} x {h}");
            }
            ref other => panic!("expected a rectangle, got {other:?}"),
        }
    }

    #[test]
    fn select_tool_moves_and_delete_tool_removes_obstacles() {
        let mut h = Harness::new(Tool::Select);
        h.map.obstacles.push(Shape::Circle { x: 10.0, y: 10.0, r: 1.0 });
        h.drag((10.0, 10.0), (12.0, 11.0));
        match h.map.obstacles[0] {
            Shape::Circle { x, y, .. } => assert!((x - 12.0).abs() < 0.11 && (y - 11.0).abs() < 0.11, "moved to ({x}, {y})"),
            _ => unreachable!(),
        }
        h.editor.tool = Tool::Delete;
        h.click((12.0, 11.0));
        assert!(h.map.obstacles.is_empty());
    }

    #[test]
    fn start_and_goal_tools_place_the_task() {
        let mut h = Harness::new(Tool::Goal);
        h.click((15.0, 6.0));
        assert!((h.map.goal.x - 15.0).abs() < 0.11 && (h.map.goal.y - 6.0).abs() < 0.11);
        h.editor.tool = Tool::Start;
        h.drag((4.0, 4.0), (4.0, 8.0));
        assert!((h.map.start.x - 4.0).abs() < 0.11 && (h.map.start.y - 4.0).abs() < 0.11);
        assert!((h.map.start.theta - std::f64::consts::FRAC_PI_2).abs() < 0.05, "heading {}", h.map.start.theta);
    }

    #[test]
    fn rectangles_are_normalised_and_tiny_shapes_rejected() {
        assert_eq!(
            preview_shape(Tool::Rect, (3.0, 4.0), (1.0, 1.0)),
            Some(Shape::Rect { x: 1.0, y: 1.0, w: 2.0, h: 3.0 })
        );
        assert_eq!(preview_shape(Tool::Rect, (1.0, 1.0), (1.05, 3.0)), None);
        assert_eq!(preview_shape(Tool::Circle, (1.0, 1.0), (1.0, 1.05)), None);
    }
}
