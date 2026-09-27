# Run our small room in Isaac Sim 6.1

The root `run_isaac.py` now launches `FIRE_ROBOT_CODEX/run_isaac.py` with
`--layout room`. Keep the whole FIRE_ROBOT_CODEX folder alongside it.
No external USD models or isaac_config.json are needed for this mode.
This supersedes the external-USD instructions in README.md.

From the project folder, use your actual Isaac installation path:

```powershell
& 'C:\isaacsim\python.bat' .\run_isaac.py --smoke-test
& 'C:\isaacsim\python.bat' .\run_isaac.py --hold --save-stage
```

The first command checks sensors, forward motion, and turning. The second
attempts the goal and return mission and keeps the window open after success.
Outputs default to FIRE_ROBOT_CODEX/outputs/isaac; inspect result.json and
error.txt if a run fails. Use --output to choose a different folder.

The generated room follows the original 60-by-60-cell drawing: 5 cm cells,
10 cm thick walls, a solid table-shaped block, shelf, two boxes, and a solid
orange flame prop. Start is (0.525, 0.525) and goal (2.425, 2.425) in drawing
metres. Isaac subtracts the start position so home is (0,0).
Furniture uses neutral colors to avoid being mistaken for the flame.

This uses the CODEX edition's rescue mapping, A*, and mission implementation,
not the separate root firebot mission. Its default allows planning through
unknown cells and checks segments while driving. It is not equivalent to the
root firebot planner's strict unknown-is-blocked behavior. Scene geometry is
used for physics and clearance checks; default RGB-D mapping reads rendered
camera data. --sensor raycast is an idealized diagnostic mode.

The robot is an approximate four-wheel model. Camera pan and heading are
idealized. No Isaac physics or RTX run has been validated on this workstation.
Passing CPU tests does not establish that the full mission succeeds in Isaac.

GitHub files to update: root run_isaac.py, this document, README.md,
FIRE_ROBOT_CODEX/run_isaac.py, and isaac/room_layout.py, isaac/scene.py,
isaac/sensors.py inside FIRE_ROBOT_CODEX.
