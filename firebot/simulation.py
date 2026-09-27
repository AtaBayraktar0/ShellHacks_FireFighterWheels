"""Algorithm-only demo with a pretend room and perfect sensor data."""
import json
from pathlib import Path
import numpy as np
from .mission import Mission, save_log


def build_demo_world(grid):
    """Create hidden ground truth that the robot cannot read directly."""
    h, w = grid.blocked.shape
    truth = np.zeros_like(grid.blocked)
    objects = [
        ("table", 20, 18, 15, 8),
        ("shelf", 40, 9, 6, 19),
        ("box", 18, 36, 7, 6),
        ("box", 42, 36, 6, 5),
        ("flame", 30, 45, 5, 6),
    ]
    # Two blocked cells form each outside wall.
    truth[:2, :] = True
    truth[-2:, :] = True
    truth[:, :2] = True
    truth[:, -2:] = True
    for _, x, y, width, height in objects:
        truth[y:y+height, x:x+width] = True
    return truth, objects


def scan_demo_sensor(grid, truth, now):
    """Let a pretend full-room scan update the robot's map."""
    occupied = {(int(x), int(y)) for y, x in np.argwhere(truth)}
    free = {(int(x), int(y)) for y, x in np.argwhere(~truth)}
    grid.observe(free, occupied, now)


def simulate(grid, config, output, block_return=False):
    """Test the mission logic without opening real hardware."""
    truth, objects = build_demo_world(grid)
    mission = Mission(config["start_cell"], config["goal_cell"])
    cell, samples = mission.visited[0], []
    new_block = None
    status = "stopped"
    try:
        for tick in range(10000):
            if block_return and cell == mission.goal:
                # Add an unseen test obstacle to the hidden world.
                previous = mission.visited[-1]
                truth[previous[1], previous[0]] = True
                new_block = previous
            # The planner learns obstacles only through this sensor step.
            scan_demo_sensor(grid, truth, tick)
            safe = grid.safe(tick, config["map_max_age_s"], config["robot_radius_m"])
            samples.append({"cell": cell, "returning": mission.returning})
            target = mission.next_target(cell, safe)
            if target is None:
                status = "complete"
                break
            cell = target
        else:
            raise RuntimeError("Demo step limit reached")
    except RuntimeError as exc:
        status = str(exc)
    finally:
        output.mkdir(parents=True, exist_ok=True)
        save_log(output / "mission.json", mission, samples, status)
        np.savez_compressed(output / "map.npz", blocked=grid.blocked, seen=grid.seen)
        draw_route(output / "route.svg", grid, mission.visited, objects, new_block)
        traveled = [tuple(sample["cell"]) for sample in samples]
        draw_animation(output / "simulation.html", grid, mission.visited, objects,
                       traveled, status, new_block)
    return status


def route_corners(route):
    """Remove straight-line points so the route picture is clean."""
    if len(route) < 3:
        return route
    corners = [route[0]]
    old_direction = None
    for previous, current in zip(route, route[1:]):
        direction = current[0] - previous[0], current[1] - previous[1]
        if old_direction is not None and direction != old_direction:
            corners.append(previous)
        old_direction = direction
    corners.append(route[-1])
    return corners


def room_shapes(grid, route, objects, new_block=None):
    """Create the room shapes shared by both demo views."""
    h, w = grid.blocked.shape
    pieces = [
        f'<rect width="{w}" height="{h}" fill="#f8fafc"/>',
        f'<path d="M0 0H{w}V{h}H0Z" fill="none" stroke="#334155" stroke-width="3"/>',
    ]
    colors = {"table": "#92400e", "shelf": "#64748b", "box": "#d97706"}
    for name, x, y, width, height in objects:
        sy = h - y - height
        if name == "flame":
            cx, cy = x + width / 2, sy + height / 2
            pieces.append(f'<circle cx="{cx}" cy="{cy}" r="3.4" fill="#ef4444"/>')
            pieces.append(f'<circle cx="{cx}" cy="{cy+.5}" r="1.5" fill="#facc15"/>')
        else:
            pieces.append(f'<rect x="{x}" y="{sy}" width="{width}" height="{height}" '
                          f'rx=".7" fill="{colors[name]}"/>')
            pieces.append(f'<text x="{x+width/2}" y="{sy+height/2+.7}" text-anchor="middle" '
                          f'font-size="2.2" fill="white">{name.title()}</text>')
    if new_block is not None:
        # Make the new return obstacle large enough to notice easily.
        cx, cy = new_block[0] + .5, h - new_block[1] - .5
        pieces.append(f'<g id="new-block"><rect x="{cx-2.5}" y="{cy-2.5}" width="5" height="5" '
                      'rx=".5" fill="#dc2626" stroke="white" stroke-width=".4"/>')
        pieces.append(f'<text x="{cx}" y="{cy-3.2}" text-anchor="middle" font-size="1.8" '
                      'font-weight="bold" fill="#dc2626">NEW BLOCK</text></g>')
    points = " ".join(f"{x+.5},{h-y-.5}" for x, y in route_corners(route))
    pieces.append(f'<polyline points="{points}" fill="none" stroke="#2563eb" '
                  'stroke-width=".7" stroke-linecap="round" stroke-linejoin="round"/>')
    for cell, color, label in ((route[0], "#16a34a", "START"),
                               (route[-1], "#7c3aed", "GOAL")):
        cx, cy = cell[0] + .5, h - cell[1] - .5
        pieces.append(f'<circle cx="{cx}" cy="{cy}" r="1.2" fill="{color}"/>')
        pieces.append(f'<text x="{cx}" y="{cy-1.8}" text-anchor="middle" '
                      f'font-size="2" font-weight="bold">{label}</text>')
    return "\n".join(pieces)


def draw_route(path, grid, route, objects, new_block=None):
    """Save a still picture of the algorithm's route."""
    h, w = grid.blocked.shape
    body = room_shapes(grid, route, objects, new_block)
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-2 -3 {w+4} {h+5}">\n'
           f'<text x="0" y="-1" font-size="2.5" font-weight="bold">Algorithm-only demo</text>\n'
           f'{body}\n</svg>')
    path.write_text(svg, encoding="utf-8")


def draw_animation(path, grid, route, objects, traveled, final_status, new_block=None):
    """Save a playable animation of the pretend mission."""
    h, w = grid.blocked.shape
    body = room_shapes(grid, route, objects, new_block)
    # Animate only cells that the demo actually reached.
    points = [{"x": x + .5, "y": h - y - .5} for x, y in traveled]
    page = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Algorithm Demo</title>
<style>
body{font-family:Arial,sans-serif;background:#e2e8f0;margin:0;color:#0f172a}
main{max-width:700px;margin:20px auto;padding:18px}section{background:white;padding:18px;border-radius:12px}
svg{width:100%;height:auto}button{padding:9px 15px;margin-right:8px;border:0;border-radius:7px;background:#334155;color:white}
#play{background:#2563eb}#status{font-weight:bold;margin-left:10px}
#new-block{display:none}
</style></head><body><main><h1>Algorithm-only demo</h1><p>This uses pretend perfect sensor data.</p>
<section><svg viewBox="-2 -2 __VIEW_W__ __VIEW_H__">__ROOM__
<circle id="robot" r="1.3" fill="#2563eb" stroke="white" stroke-width=".35"/></svg>
<button id="play">Play</button><button id="pause">Pause</button><button id="restart">Restart</button>
<span id="status">Ready</span></section></main>
<script>
const points=__POINTS__,outbound=__OUTBOUND__,finalStatus=__FINAL_STATUS__;let index=0,playing=false;
const robot=document.getElementById('robot'),status=document.getElementById('status');
function show(){const p=points[index];robot.setAttribute('cx',p.x);robot.setAttribute('cy',p.y);
const block=document.getElementById('new-block');if(block){block.style.display=index>=outbound-1?'block':'none'}
if(index===points.length-1){status.textContent=finalStatus==='complete'?'Complete: returned home':'Stopped: '+finalStatus}
else{status.textContent=index<outbound-1?'Driving to goal':'Returning home'}}
setInterval(()=>{if(playing&&index<points.length-1){index++;show()}else if(index===points.length-1){playing=false}},100);
document.getElementById('play').onclick=()=>playing=true;
document.getElementById('pause').onclick=()=>playing=false;
document.getElementById('restart').onclick=()=>{playing=false;index=0;show()};show();
</script></body></html>"""
    page = (page.replace("__VIEW_W__", str(w + 4))
                .replace("__VIEW_H__", str(h + 4))
                .replace("__ROOM__", body)
                .replace("__POINTS__", json.dumps(points))
                .replace("__OUTBOUND__", str(len(route)))
                .replace("__FINAL_STATUS__", json.dumps(final_status)))
    path.write_text(page, encoding="utf-8")
