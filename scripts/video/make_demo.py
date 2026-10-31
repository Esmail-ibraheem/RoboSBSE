"""Records the RoboSBSE feature tour from the running application and assembles a silent video.

Every segment is a real run: the UI starts the engine, runs the demo experiment, and records frames
(`robosbse --record`): a time lapse of the search, then the result replayed frame by frame. ffmpeg adds a
caption to each segment and joins them into docs/media/robosbse-demo.mp4 and a smaller GIF for the README.

Requirements: the release UI (`cargo build --release` in frontend/rust-ui), the engine jar, ffmpeg on PATH,
and ROBOSBSE_JAVA (or JAVA_HOME) pointing at a JDK 21.

Usage, from the repository root:  python scripts/video/make_demo.py [--only N,N,...] [--skip-record]
"""
import argparse
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
UI = os.path.join(ROOT, "frontend", "rust-ui", "target", "release", "robosbse.exe" if os.name == "nt" else "robosbse")
WORK = os.path.join(ROOT, "frontend", "rust-ui", "target", "video")
OUT = os.path.join(ROOT, "docs", "media")
FONT_REGULAR = "C:/Windows/Fonts/segoeui.ttf" if os.name == "nt" else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "C:/Windows/Fonts/segoeuib.ttf" if os.name == "nt" else "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FPS = 30
W, H = 1280, 720

# (caption, page, demo); demo None records the page as it opens.
SEGMENTS = [
    ("Dashboard: five phases, all implemented", "dashboard", None),
    ("Route optimisation: genetic algorithm, fitness from simulation", "lab", "lab"),
    ("Multi-objective routes: NSGA-II Pareto front", "lab", "pareto"),
    ("Warehouse scheduling: capacity, dependencies, deadlines, charging", "lab", "warehouse"),
    ("Controller tuning across several maps (PSO)", "lab", "tuning"),
    ("Novelty search on the maze", "lab", "novelty"),
    ("Search-based testing: a GA finds BUG-1 in the reactive controller", "testing", "testing"),
    ("Coverage-guided testing: branch distance reaches BUG-3", "testing", "coverage"),
    ("Genetic programming: evolved controllers on unseen maps", "gp", "gp"),
    ("Automated program repair of a faulty Java controller", "repair", "repair"),
    ("Genetic improvement: faster controller, same tests", "repair", "improve"),
    ("Benchmarks: equal budgets, Mann-Whitney U, A12", "benchmark", "benchmark"),
    ("Robotic arm: pick-and-place planning", "arm", "arm"),
    ("Robotic arm: trajectory around a post (vs C-space A*)", "arm", "arm-trajectory"),
    ("Multi-robot task allocation with conflict detection", "lab", "fleet"),
    ("Dynamic environment: moving guards, timed pallet, zones", "lab", "dynamic"),
    ("Environment editor", "editor", None),
]


def run(cmd, **kw):
    print("+", " ".join(cmd[:6]), "..." if len(cmd) > 6 else "")
    subprocess.run(cmd, check=True, **kw)


def record(i, page, demo):
    frames = os.path.join(WORK, f"seg{i:02d}")
    shutil.rmtree(frames, ignore_errors=True)
    cmd = [UI, "--page", page, "--record", frames]
    if demo:
        cmd += ["--demo", demo]
    run(cmd, cwd=ROOT)
    if not os.path.isdir(frames) or not os.listdir(frames):
        sys.exit(f"segment {i} ({page} {demo}) recorded no frames")


def caption_filter(i, caption, total):
    # Text from files avoids escaping; fonts are copied next to them (relative paths need no escaping).
    with open(os.path.join(WORK, f"cap{i:02d}.txt"), "w", encoding="utf-8") as f:
        f.write(caption)
    with open(os.path.join(WORK, f"num{i:02d}.txt"), "w", encoding="utf-8") as f:
        f.write(f"{i + 1}/{total}")
    return (f"scale={W}:{H},"
            f"drawbox=x=0:y=ih-56:w=iw:h=56:color=black@0.72:t=fill,"
            f"drawtext=fontfile=bold.ttf:textfile=cap{i:02d}.txt:fontcolor=white:fontsize=26:x=24:y=h-41,"
            f"drawtext=fontfile=regular.ttf:textfile=num{i:02d}.txt:fontcolor=0x9aa4b2:fontsize=20:x=w-tw-24:y=h-38,"
            f"tpad=stop_mode=clone:stop_duration=0.5")


def card(name, lines, seconds):
    texts = []
    for k, (text, size, color, y) in enumerate(lines):
        path = f"{name}{k}.txt"
        with open(os.path.join(WORK, path), "w", encoding="utf-8") as f:
            f.write(text)
        font = "bold.ttf" if k == 0 else "regular.ttf"
        texts.append(f"drawtext=fontfile={font}:textfile={path}:fontcolor={color}:fontsize={size}:x=(w-tw)/2:y={y}")
    run(["ffmpeg", "-y", "-loglevel", "error", "-f", "lavfi", "-i", f"color=c=0x0e1116:s={W}x{H}:d={seconds}:r={FPS}",
         "-vf", ",".join(texts), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", f"{name}.mp4"], cwd=WORK)
    return f"{name}.mp4"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", help="comma-separated segment numbers (1-based) to re-record")
    ap.add_argument("--skip-record", action="store_true", help="assemble from existing frames")
    args = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    shutil.copy(FONT_BOLD, os.path.join(WORK, "bold.ttf"))
    shutil.copy(FONT_REGULAR, os.path.join(WORK, "regular.ttf"))
    only = {int(x) - 1 for x in args.only.split(",")} if args.only else set(range(len(SEGMENTS)))
    if not args.skip_record:
        for i, (_, page, demo) in enumerate(SEGMENTS):
            if i in only:
                record(i, page, demo)
    clips = [card("intro", [("RoboSBSE", 64, "white", "h/2-90"),
                            ("Search-Based Robotics and Software Engineering Laboratory", 30, "0xc8cfda", "h/2-5"),
                            ("Java 21 engine  |  Rust/egui UI  |  every number from real simulation", 22, "0x8a94a3", "h/2+50")], 2.5)]
    for i, (caption, _, _) in enumerate(SEGMENTS):
        clip = f"clip{i:02d}.mp4"
        run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", os.path.join(f"seg{i:02d}", "f%05d.png"),
             "-vf", caption_filter(i, caption, len(SEGMENTS)), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
             "-r", str(FPS), clip], cwd=WORK)
        clips.append(clip)
    clips.append(card("outro", [("RoboSBSE", 56, "white", "h/2-60"),
                                ("Metaheuristics, search-based testing, GP, program repair, robotic arm, multi-robot", 24,
                                 "0xc8cfda", "h/2+10")], 2.5))
    with open(os.path.join(WORK, "clips.txt"), "w", encoding="utf-8") as f:
        for c in clips:
            f.write(f"file '{c}'\n")
    mp4 = os.path.join(OUT, "robosbse-demo.mp4")
    run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "clips.txt", "-an", "-c:v", "libx264",
         "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-movflags", "+faststart", mp4], cwd=WORK)
    gif = os.path.join(OUT, "robosbse-demo.gif")
    run(["ffmpeg", "-y", "-loglevel", "error", "-i", mp4, "-vf",
         "fps=8,scale=760:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];"
         "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle", gif])
    for path in (mp4, gif):
        print(f"{path}: {os.path.getsize(path) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
