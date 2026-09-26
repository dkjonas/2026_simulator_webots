# WildWillyR5 camera calibration

`calib.json` is loaded beside this PROTO using Webots' built-in `wbfile` API.
The client writes it after **Calculate calibration** (intrinsics) and **Calc cam pos**
(RGB pose), as well as updating `D:\calib.json`.
No camera settings need to be pasted into the PROTO.

The JSON contains:

- `ImageWidth` and `ImageHeight`: the resolution used for calibration.
- `CameraMatrix`: the original OpenCV 3-by-3 pixel-space intrinsic matrix.
- `DistCoeffs`: the complete original OpenCV distortion coefficient array.
- Optional `CameraPoseInRobot`: the RGB optical camera's pose relative to the robot.
  `TranslationMillimeters` is `[x, y, z]` in mm. `RotationMatrix` is a 3-by-3
  column-vector rotation: `pointRobot = RotationMatrix * pointOptical + translation`.
  Robot axes are X forward, Y left, Z up; optical axes are X right, Y down, Z forward.

The initial file uses the existing 768-by-432 calibration. Future calibrations
record the actual input-image resolution; mixed-resolution input is rejected.
Missing or invalid JSON causes an explicit template error rather than silently
using a different calibration.

Reset or reload the world after saving. The `nonDeterministic` PROTO tag forces
template reevaluation on reset because its output depends on this external file.
The file is not watched continuously during simulation.

## Conversion and limitations

The template computes horizontal field of view as `2 * atan(width / (2 * fx))`,
the lens center as `(cx / width, cy / height)`, and copies `k1, k2, p1, p2` into
the Lens fields. This preserves the previous C# text export's approximation.

It is **not an exact OpenCV camera model**: Webots Lens uses normalized image UV
coordinates, whereas OpenCV distortion uses normalized camera coordinates;
Webots has no `k3` field and its Camera cannot independently represent `fx`, `fy`
and skew. The distortion center is not a general off-axis projection parameter.
The template warns about these approximations and ignores additional distortion
coefficients without removing them from the JSON.

## Calibrating the RGB position and orientation

1. Use the real RGB camera, with intrinsics calibrated at the same image resolution.
2. Place the 80 mm cube with 60 mm markers (IDs 5, 8, 32) at the known robot-relative
   pose in `CameraPositionCalculator.DefaultCubePoseWorld`: position
   `(209.01, 0, 100)` mm and the existing cube orientation. The rotations are
   Z=135 degrees followed by Y=-210 degrees in System.Numerics row-vector order.
   The reference origin is the floor-level PROTO Robot origin (X forward, Y left,
   Z up), not the bottom of the chassis or an arbitrary world origin. The cube
   corner is 100 mm above the floor. Incorrect target placement becomes an error
   in camera pose.
3. Click **Calc cam pos**. The client saves the pose alongside the existing intrinsics
   to both JSON paths and reports any save failures. A missing cube or mismatched
   resolution does not overwrite the saved pose.
4. Reset or reload the Webots world. The template converts mm to metres and converts
   the optical rotation to Webots' X-forward, Y-left, Z-up camera convention.

The RGB pose is absolute relative to the Robot, not an offset from the old mount.
There is no additional fixed tilt after applying a calibration. `RGB_CAMERA_FRAME`
and `TOF_CAMERA_FRAME` are independent sibling Poses; ToF retains its original
position and orientation relative to the chassis. Mesh files are unchanged.

### Floor-level robot origin

Z=0 is the horizontal plane at the lowest wheel-cylinder surface in the saved
initial pose. This is 22.62819049 mm below the former chassis-based origin.
Root-relative geometry, steering anchors, collision shapes, IMU, arm and ToF
have been translated up by that amount. Nested coordinates and rotations are
unchanged; there is no additional whole-robot wrapper transform.

The saved wheel poses are slightly asymmetric: their bottoms are now at 0,
0.5711 and 1.0901 mm. These small pre-existing differences are preserved rather
than changing wheel geometry. The robot can settle slightly under physics;
its local frame is body-fixed, not dynamically pinned to the floor.

The world now spawns the robot at Z=0. The former Z=21 mm spawn had the lowest
wheel about 1.63 mm below the floor, so initial chassis geometry is now 1.63 mm
higher to eliminate that penetration. Other worlds using this PROTO should
also place its origin at their floor height.

Calibrated positions already measured from the floor are used unchanged.
Only the uncalibrated default RGB mount was translated, from Z=94 mm to
Z=116.62819049 mm. For the currently saved Z=118.8913 mm calibration, the height
difference from the old physical default is therefore about 2.26 mm, not 24.89 mm.

An absent/null pose preserves the original RGB mount with a warning. A malformed
pose fails explicitly (finite coordinates, orthonormal rotation, determinant +1).
Recalculating intrinsics clears the old pose; run **Calc cam pos** again afterwards.
The client also restores the saved pose on startup.

This aligns the simulated camera's geometric pose with the estimated real pose;
it does not correct the lens approximation, lighting, or errors in physical target
placement/calibration. The fixture image provides a regression check, not an
independent measurement of the camera's true pose.

References:

- [Webots JavaScript templates and wbfile](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/javascript-procedural-proto.md)
- [Webots Lens shader](https://github.com/cyberbotics/webots/blob/R2025a/resources/wren/shaders/lens_distortion.frag)

## Tests

From the Webots project root:

    node --test tests/camera-calibration.test.cjs

These tests execute the actual JavaScript block and substitution expressions
from the PROTO, with a mocked Webots file API. They cover loading, conversion,
resolution changes, Windows paths, cache policy, invalid files, optical-frame
conversion (including 180-degree rotations), legacy mounts and ToF independence.
