"""Created by OpenAI Codex: procedural 4-wheel proxy and arena, no asset downloads.

This is a simplified skid-steer model, not calibrated Elegoo CAD or dynamics.
All dimensions are metres, masses kg; velocity control uses four revolute joints.
"""
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, UsdLux, PhysxSchema

ROBOT = '/World/CodexRobot'
CHASSIS = ROBOT + '/chassis'
WHEEL_RADIUS = .033
TRACK = .166
CHASSIS_Z = .073
BODY_RADIUS = .145  # encloses chassis and wheel outer corners in XY
JOINTS = ['left_front_joint', 'left_rear_joint', 'right_front_joint', 'right_rear_joint']
DEMO_BOXES = [(.55, -.30, .75, .15), (.95, .40, 1.10, .60),
              (1.20, -.65, 1.40, -.30), (1.65, .05, 1.80, .25)]


def cube(stage, path, center, size, color, collision=True):
    obj = UsdGeom.Cube.Define(stage, path)
    obj.CreateSizeAttr(1.)
    obj.AddTranslateOp().Set(Gf.Vec3d(*center))
    obj.AddScaleOp().Set(Gf.Vec3f(*size))
    obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    return obj


def material(stage, path, friction):
    mat = UsdShade.Material.Define(stage, path)
    physics = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    physics.CreateStaticFrictionAttr(friction)
    physics.CreateDynamicFrictionAttr(friction)
    physics.CreateRestitutionAttr(0.)
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, 'physics')


def create_scene(stage, cfg, layout):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr(800.)
    ground = cube(stage, '/World/Table',
                  ((cfg.x_min+cfg.x_max)/2, (cfg.y_min+cfg.y_max)/2, -.05),
                  (cfg.x_max-cfg.x_min, cfg.y_max-cfg.y_min, .1), (.55,.59,.62))
    mat = material(stage, '/World/FloorMaterial', .6)
    bind(ground.GetPrim(), mat)
    boxes = DEMO_BOXES if layout == 'demo' else []
    flame = (1.30,.20) if layout == 'demo' else (1.,0.)
    for i, (x0,y0,x1,y1) in enumerate(boxes):
        cube(stage, f'/World/Box_{i}', ((x0+x1)/2,(y0+y1)/2,.13),
             (x1-x0,y1-y0,.26), (.23,.25,.28))
    prop = UsdGeom.Cylinder.Define(stage, '/World/FlameProp')
    prop.CreateRadiusAttr(.05)
    prop.CreateHeightAttr(.24)
    prop.CreateAxisAttr('Z')
    prop.AddTranslateOp().Set(Gf.Vec3d(*flame,.12))
    prop.CreateDisplayColorAttr([Gf.Vec3f(1.,.15,0.)])
    UsdPhysics.CollisionAPI.Apply(prop.GetPrim())
    for name, xy, color in [('Home',(0.,0.),(.1,.6,.9)), ('Goal',cfg.goal_xy,(.1,.8,.2))]:
        cube(stage, '/World/'+name, (*xy,.001), (.09,.09,.002), color, collision=False)
    UsdGeom.Xform.Define(stage, ROBOT)
    # Unscaled rigid-body Xform keeps camera mount and joint offsets in metres.
    chassis = UsdGeom.Xform.Define(stage, CHASSIS)
    chassis.AddTranslateOp().Set(Gf.Vec3d(0,0,CHASSIS_Z))
    UsdPhysics.RigidBodyAPI.Apply(chassis.GetPrim())
    UsdPhysics.MassAPI.Apply(chassis.GetPrim()).CreateMassAttr(.8)
    UsdPhysics.ArticulationRootAPI.Apply(chassis.GetPrim())
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(chassis.GetPrim())
    articulation.CreateEnabledSelfCollisionsAttr(False)
    articulation.CreateSolverPositionIterationCountAttr(16)
    cube(stage, CHASSIS+'/body', (0,0,0), (.18,.14,.045), (.04,.32,.65))
    # Visible blue front mark identifies the forward +X direction.
    cube(stage, CHASSIS+'/front_mark', (.085,0,.026), (.015,.10,.006), (.2,.8,1.), False)
    tire_mat = material(stage, '/World/TireMaterial', .6)
    wheel_z = WHEEL_RADIUS + .002
    for name, x, y in [('left_front',.065,TRACK/2), ('left_rear',-.065,TRACK/2),
                       ('right_front',.065,-TRACK/2), ('right_rear',-.065,-TRACK/2)]:
        path = ROBOT+'/'+name
        wheel = UsdGeom.Cylinder.Define(stage,path)
        wheel.CreateRadiusAttr(WHEEL_RADIUS)
        wheel.CreateHeightAttr(.018)
        wheel.CreateAxisAttr('Y')
        wheel.AddTranslateOp().Set(Gf.Vec3d(x,y,wheel_z))
        wheel.CreateDisplayColorAttr([Gf.Vec3f(.04,.04,.04)])
        UsdPhysics.CollisionAPI.Apply(wheel.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(wheel.GetPrim())
        UsdPhysics.MassAPI.Apply(wheel.GetPrim()).CreateMassAttr(.05)
        bind(wheel.GetPrim(), tire_mat)
        joint = UsdPhysics.RevoluteJoint.Define(stage, ROBOT+'/'+name+'_joint')
        joint.CreateBody0Rel().SetTargets([CHASSIS])
        joint.CreateBody1Rel().SetTargets([path])
        joint.CreateAxisAttr('Y')
        joint.CreateLocalPos0Attr(Gf.Vec3f(x,y,wheel_z-CHASSIS_Z))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(10.)
        drive.CreateMaxForceAttr(2.)
        drive.CreateTargetVelocityAttr(0.)
    return boxes, flame
