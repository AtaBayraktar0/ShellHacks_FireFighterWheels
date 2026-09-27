#!/usr/bin/env python3
"""Created by OpenAI Codex: standalone Isaac Sim 6.1 entry point.

Use Isaac Sim's python.sh/python.bat, not the system Python. Original hardware
runner is unchanged. New adapters and scene are isolated under isaac/.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import traceback

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'pi'))


def parser():
    p=argparse.ArgumentParser(description='CODEX fire robot — Isaac Sim 6.1')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--sensor',choices=['rgbd','raycast'],default='rgbd')
    p.add_argument('--layout',choices=['demo','flame'],default='demo')
    p.add_argument('--config',type=Path)
    p.add_argument('--goal',type=float,nargs=2,metavar=('X','Y'))
    p.add_argument('--speed',type=float,default=.15,help='timed-drive speed in m/s')
    p.add_argument('--max-seconds',type=float,default=300.,help='simulation time limit')
    p.add_argument('--smoke-test',action='store_true',help='check camera, forward drive and turn, then exit')
    p.add_argument('--hold',action='store_true',help='keep GUI open after a successful run')
    p.add_argument('--save-stage',action='store_true')
    p.add_argument('--output',type=Path,default=ROOT/'outputs'/'isaac')
    return p


def main():
    args=parser().parse_args()
    if not 0 < args.speed <= .3:
        raise SystemExit('--speed must be greater than 0 and at most 0.3 m/s')
    if not math.isfinite(args.max_seconds) or args.max_seconds <= 0:
        raise SystemExit('--max-seconds must be positive and finite')
    if args.headless and args.hold:
        raise SystemExit('--hold requires the GUI; omit it with --headless')
    args.output.mkdir(parents=True,exist_ok=True)
    # Bootstrap Kit before importing any omni/pxr/Isaac extensions.
    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        raise SystemExit('Run this script using Isaac Sim 6.1 python.sh (Linux) or python.bat (Windows).') from exc
    app=SimulationApp({'headless':args.headless,'width':1280,'height':800})
    rt=base=camera=mission=None
    result=dict(created_by='OpenAI Codex',target_isaac_version='6.1',
                sensor=args.sensor,layout=args.layout,mode='smoke' if args.smoke_test else 'mission',
                passed=False,mission_completed=False,error=None)
    code=1
    try:
        import numpy as np
        import isaacsim.core.experimental.utils.app as app_utils
        # Explicitly enable runtime dependencies before importing them.
        for extension in ['isaacsim.core.experimental.prims','isaacsim.core.simulation_manager',
                          'isaacsim.sensors.experimental.rtx']:
            app_utils.enable_extension(extension)
        from rescue.config import Config
        from rescue.geometry import dist,wrap
        from rescue.mission import Mission
        from isaac.runtime import Runtime
        from isaac.control import IsaacBase
        from isaac.sensors import RgbdCamera,RaycastCamera
        cfg=Config.load(args.config)
        if not cfg.use_imu:
            raise ValueError('Isaac runner requires use_imu=True; ideal physics yaw supplies the simulated gyro')
        if not cfg.edge_is_wall:
            raise ValueError('Isaac tabletop runner requires edge_is_wall=True')
        cfg.speed_mps=args.speed
        if args.goal:
            cfg.goal_xy=tuple(args.goal)
        result['config']=__import__('dataclasses').asdict(cfg)
        rt=Runtime(app,cfg,args.layout,args.max_seconds)
        rt.stage.GetRootLayer().customLayerData={'creator':'OpenAI Codex','target':'Isaac Sim 6.1'}
        base=IsaacBase(rt,cfg)
        camera=RgbdCamera(rt,cfg) if args.sensor=='rgbd' else RaycastCamera(rt,cfg)
        base.camera=camera
        if not args.headless:
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=[3.4,-3.6,3.2],target=[1.,0.,0.])
        if args.save_stage:
            rt.stage.GetRootLayer().Export(str((args.output/'codex_scene.usda').resolve()))
        def log(message):
            print('[CODEX]',message,flush=True)
            with (args.output/'mission.log').open('a',encoding='utf-8') as stream:
                stream.write(message+'\n')
        (args.output/'mission.log').write_text('',encoding='utf-8')
        print('CODEX | Isaac Sim 6.1 | Physical wheel-joint control | '+args.sensor,flush=True)
        if args.smoke_test:
            obs=camera.observe(0.)
            if len(obs.floor)==0 or len(obs.obstacles)==0:
                raise RuntimeError('Sensor smoke test failed: expected floor and obstacle observations')
            p0=rt.pose()
            moved,stopped=base.move(.12)
            p1=rt.pose()
            forward=(p1[0]-p0[0])*math.cos(p0[2])+(p1[1]-p0[1])*math.sin(p0[2])
            if stopped or not .05 < forward < .19:
                raise RuntimeError(f'Forward smoke test failed: true displacement {forward:.3f} m')
            h0=base.heading()
            base.turn(math.pi/4)
            delta=wrap(base.heading()-h0)
            if abs(delta-math.pi/4) > math.radians(5):
                raise RuntimeError(f'Turn smoke test failed: yaw change {math.degrees(delta):.1f} deg')
            result.update(passed=True,forward_displacement_m=forward,turn_degrees=math.degrees(delta))
        else:
            mission=Mission(cfg,base,camera,log=log)
            mission.scan_and_plan()
            mission.outbound()
            result['true_goal_error_m']=dist(rt.pose()[:2],cfg.goal_xy)
            mission.return_trip()
            mission.phase='done'
            result['mission_completed']=True
            result['true_home_error_m']=dist(rt.pose()[:2],(0,0))
            result['estimated_home_error_m']=dist(mission.pose.xy,(0,0))
            result['replans']=mission.replans
            result['passed']=(result['true_goal_error_m']<=cfg.goal_tolerance_m and
                              result['true_home_error_m']<=cfg.goal_tolerance_m)
            if not result['passed']:
                result['error']='Mission finished, but true arrival error exceeded configured tolerance'
            log('Evaluation passed' if result['passed'] else result['error'])
        code=0 if result['passed'] else 2
    except KeyboardInterrupt:
        result['error']='Interrupted by user'
        code=130
    except Exception as exc:
        result['error']=str(exc)
        (args.output/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        print('CODEX run failed:',exc,file=sys.stderr)
        code=1
    finally:
        # Startup, perception, control, mission and serialization errors all close Kit.
        try:
            if base is not None:
                try:
                    base.stop()
                except Exception as stop_error:
                    # A closed/stopped Isaac timeline can invalidate the tensor
                    # controller. Preserve the report and still close Kit.
                    result['stop_error']=str(stop_error)
                    result['passed']=False
                    code=1
            if rt is not None:
                result['minimum_conservative_clearance_m']={k:v if math.isfinite(v) else None for k,v in rt.min_gaps.items()}
                result['footprint_violation']=rt.violation
                (args.output/'true_trajectory.json').write_text(json.dumps(rt.samples)+'\n')
            if camera is not None and getattr(camera,'last_rgb',None) is not None:
                import cv2
                cv2.imwrite(str(args.output/'camera_rgb.png'),camera.last_rgb[...,::-1])
                np.save(args.output/'camera_depth.npy',camera.last_depth)
            (args.output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
            print(json.dumps(result,indent=2),flush=True)
            if args.hold and result['passed']:
                # Pause physics while leaving the scene available for inspection.
                rt.app_utils.pause()
                while app.is_running():
                    app.update()
        finally:
            try:
                if rt is not None:
                    rt.shutdown()
            finally:
                app.close()
    return code


if __name__=='__main__':
    sys.exit(main())
