# Procedural laptop mission behavior

The presentation runs a separate laptop simulator. It never arms motors, sends Arduino commands, or treats actor responses as detector results. Its API rejects hardware mode. Robot paths come from the observed-grid frontier explorer and return planner.

## New missions and repeatable replay

PresentationSimulator() creates a randomized_mission with a fresh seed. Each world independently samples people, response fixtures, fire origins, fire growth parameters and a possible sensing interruption. It does not choose an old fixed scenario. There can be zero through three people and zero through three fires, bounded further in tiny spaces. Missing objects do not prevent exploration or return.

The randomize action accepts an optional integer seed from 0 through 2147483647. The seed controls geometry and environmental conditions. Reset/restart repeats the seed and area exactly; identical elapsed-time controls yield identical logical evolution. Boundary changes preserve the seed and reset to paused-ready. Reports record the seed, generation version and layout hash. Different generation versions may produce different worlds for the same seed.

World geometry varies among branching rooms, corridor apartments, open plans and irregular suites, with varied doors, furniture and launch sides. People and fire origins use shuffled near/middle/far distance strata across usable space. Hidden counts, origins and response fixtures are not published in the operator snapshot. Found counts mean observations so far.

Width and height each accept 1-50 metres. The adaptive grid has at most 80 cells per axis. Free centers include a 0.20 m rover footprint inside the exact rectangle and conservative clearance around objects. Physical geometry and navigation configuration space are separate representations.

## Search and evolving fire

The procedural mission's 2D navigation fixture supplies observed occupancy to the planner; the 3D acquisition is a forward camera view. Unknown cells block planning. The rover searches reachable observed frontiers and returns when none remain, a budget ends, an actor response requires return, or recovered sensing faults require a partial-survey return. A missing known return path produces a stopped/blocked outcome, never a teleport.

Fire is a bounded synthetic hazard model, not a fire dynamics or building safety prediction. Each origin has an ignition tick, intensity increment and spread period. Intensity saturates at one. At most one cardinal neighboring physical-free cell is added per spread interval, with a bounded maximum size. Conservative physical-cell occupancy prevents thin-wall crossings; exclusion buffers also check wall visibility. Fire may close routes. Generated missions are not guaranteed to return.

Environment evolution continues during response assessment, follower holds and sensing interruptions. Pausing playback freezes the entire simulation clock. Hidden fire changes produce no operator event. Perimeters and intensity are updated only after sensing; out-of-view observations retain their last measured state and are labeled not visible. Navigation uses observed occupancy, not hidden origins.

The operator spread_cells field is the observed clearance-expanded exclusion footprint, not the exact physical flame boundary. Post-run diagnostics separately list synthetic physical fire cells.

## Human response assessment and decisions

A person begins unobserved. Five visible logical samples are required to progress from observing to one of these generated responses:

| Simulated response | Executed decision |
| --- | --- |
| mobile | Escort toward launch while verifying following. |
| assistance_needed | Store the observed location, return and deliver an alert only at launch. |
| no_response | Report no response across five visible samples and return for assistance; medical status remains unknown. |

Posture is an independently generated observed attribute. Seated or prone posture does not establish consciousness, age, injury or willingness to evacuate. These fixtures supply an interaction source for software testing, not a trained medical classifier.

Assistance reporting has priority. At most one actor is escorted at a time. Other observed people remain unresolved and are included with their last observed locations in the return report. One verified escort never implies everyone present was evacuated.

Measured actor positions create planning exclusions for a 0.20 m rover plus a 0.18 m actor radius. A separate simulation-physics interlock refuses unsafe attempted moves without choosing an alternative route using hidden positions. This guard is simulator collision handling, not physical validation.

## Following verification

The actor moves at most one cardinal cell per logical cycle through observed free space, with at least 0.38 m combined body clearance and a normal 0.40 m or two-cell gap. Generated pauses can remove current visibility. The rover holds while fire keeps evolving.

Following requires line of sight and a distance within the greater of 0.60 m and three grid cells. Failed reacquisition is bounded to 50 logical ticks. It then returns with the last observed location and no escort-success claim. Current person position and distance are null in invisible following-log entries. Snapshot markers retain last-seen locations, never hidden actor motion.

Arrival requires the rover at launch and the actor visible within the greater of 0.60 m and two cells. Pausing/resuming while the rover is home but still waiting preserves that pending escort.

## Acquired 3D scan

SyntheticDepthScan raycasts continuous floor, wall, cylinder and oriented furniture surfaces through an 87° forward horizontal field of view. It accumulates bounded acquired triangles without revealing the hidden complete world mesh. Scanning starts with playback and refreshes after every two rover moves; a changed heading is a new acquisition. Unchanged snapshot revisions reuse cached geometry.

The scan_mesh fields include vertices, triangles, colors, revision, scan_origin and recent returns in world metres: x right, y up, z following grid rows. The source is synthetic forward raycast depth. It is not a D435i sensor model. Fire and people are separate observed overlays; the mesh does not simulate smoke attenuation or reconstruct people.

## API and outcome evidence

Authenticated endpoints are GET /api/presentation, POST /api/presentation/control, and GET /api/presentation/report. Controls are play, pause, reset/restart, speed, boundary and randomize. Speed accepts 0.25-4; boundary requires width_m and height_m; randomize optionally accepts seed. There is no scenario-selection action. Public health identifies procedural-v2 so the launcher cannot reuse an older server.

Snapshots preserve grid, pose, route, trail, events, observed entities, metrics, follower and alerts, and add procedural mission metadata plus scan_mesh. Phases include idle, exploring, assessing, returning, sensor_hold, escort_wait, complete and blocked. Behavior identifies assessment, assistance return and follower waiting. last_move records checked simulation movement evidence, not a physical certificate.

Terminal exports may add simulation_diagnostics explicitly labeled hidden post-run ground truth for software auditing. These include undetected counts, response fixtures, actor trajectories and physical fire cells. They are omitted before termination and never feed navigation. Runtime snapshots never expose hidden totals.

Coverage keeps the initial reachable configuration-space denominator even when fire closes passages. Returning to launch never means all-clear. Unobserved space and unresolved observed people remain in the report. Blocked missions never mark alerts delivered at launch.

## Regression and physical integration

Explicit legacy IDs remain available through PresentationSimulator("empty_search") and other offline regression fixtures. Historical fixed-scenario reports do not establish that the new procedural dynamics passed. Procedural tests cover seed diversity/replay, repeated response samples, observed fire growth, route changes, actor clearance, bounds and truthful outcomes.

Physical use would need calibrated localization and mapping, current occupancy, verified actor interaction and tracking, uncertainty/data-age handling, motion feedback, suitable environmental hardware and a verified exit. This simulator does not validate those capabilities. Physical autonomous operation remains disabled.
