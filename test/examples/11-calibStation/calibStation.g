world: {}

### table

origin (world): { Q: [0, 0, .6], shape: marker, size: [.03] }
table (origin): { Q: [0, 0, -.05], shape: ssBox, size: [2.3, 1.24, .1, .02], color: [.3, .3, .3], contact, logical:{ } }

## two pandas with robotiq grippers (and optitrack markers)
Prefix: "l_"
Include: <../../../rai-robotModels/scenarios/panda_fixRobotiq.g>
Prefix: "r_"
Include: <../../../rai-robotModels/scenarios/panda_fixRobotiq.g>
Prefix: False

## position them on the table
Edit l_panda_base (origin): { Q: "t(-.4 -.3 .0) d(90 0 0 1)", motors, joint: rigid }
Edit r_panda_base (origin): { Q: "t( .4 -.3 .0) d(90 0 0 1)", motors, joint: rigid }

### camera

camera(world): {
 Q: "t(-0.01 -.2 2.) d(-150 1 0 0)",
 shape: camera, size: [.1],
 focalLength: 0.895, width: 640, height: 360, zRange: [.5, 100]
}
