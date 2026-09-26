// Run with: node --test tests/camera-calibration.test.cjs
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');

const protoDirectory = path.join(__dirname, '..', 'protos', 'robots', 'WildWillyR5');
const proto = fs.readFileSync(path.join(protoDirectory, 'WildWillyR5.proto'), 'utf8');
const fixture = JSON.parse(fs.readFileSync(path.join(protoDirectory, 'calib.json'), 'utf8'));
const script = proto.match(/%<([\s\S]*?)>%/)[1];

function evaluate(data = fixture, options = {}) {
  const warnings = [];
  const protoPath = options.protoPath || 'D:/fixture/robots/WildWillyR5/WildWillyR5.proto';
  const expectedPath = protoPath.replace(/\\/g, '/').replace(/[^/]+$/, 'calib.json');
  let readCount = 0;
  const context = vm.createContext({
    context: { proto: protoPath },
    console: { warn: message => warnings.push(message) },
    wbfile: {
      fileExists(file) {
        assert.equal(file, expectedPath);
        return options.exists !== false;
      },
      readTextFile(file) {
        assert.equal(file, expectedPath);
        readCount++;
        return options.raw ?? JSON.stringify(data);
      }
    }
  });
  vm.runInContext(script, context);
  const settings = JSON.parse(vm.runInContext('JSON.stringify(cameraSettings)', context));
  const rendered = proto.replace(/%<[\s\S]*?>%/, '').replace(/%<=([\s\S]*?)>%/g,
    (_, expression) => String(vm.runInContext(expression, context)));
  const rgbPose = JSON.parse(vm.runInContext('JSON.stringify(rgbPose)', context));
  return { settings, rgbPose, rendered, warnings, readCount };
}

function axisAngleMatrix([x, y, z, angle]) {
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
    [t*x*y+s*z, t*y*y+c, t*y*z-s*x], [t*x*z-s*y, t*y*z+s*x, t*z*z+c]];
}
function multiply(a, b) {
  return a.map(row => b[0].map((_, j) => row.reduce((sum, value, k) => sum + value * b[k][j], 0)));
}
function assertMatrixClose(actual, expected, tolerance = 1e-7) {
  actual.forEach((row, i) => row.forEach((value, j) =>
    assert.ok(Math.abs(value - expected[i][j]) < tolerance, `matrix[${i}][${j}]: ${value} vs ${expected[i][j]}`)));
}
// Columns: optical right, down, forward expressed in a level forward-facing Webots camera.
const opticalToWebots = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]];
function calibrationWithPose(webotsRotation, translation = [104, -2, 119]) {
  return { ...structuredClone(fixture), CameraPoseInRobot: {
    TranslationMillimeters: translation,
    RotationMatrix: multiply(webotsRotation, opticalToWebots)
  }};
}

test('legacy calibration preserves the original RGB mount transform', () => {
  const legacy = structuredClone(fixture);
  delete legacy.CameraPoseInRobot;
  const { rgbPose, warnings } = evaluate(legacy);
  const original = multiply(axisAngleMatrix([0, 0, 1, -0.0007963267948966]),
    axisAngleMatrix([0, 1, 0, 0.2268]));
  assert.deepEqual(rgbPose.translation, [.101, 0, .11662819049]);
  assertMatrixClose(axisAngleMatrix(rgbPose.rotation), original);
  assert.ok(warnings.some(message => message.includes('no CameraPoseInRobot')));
});

for (const [name, axisAngle] of [
  ['level forward camera', [0, 0, 1, 0]],
  ['downward pitch', [0, 1, 0, .23]],
  ['roll', [1, 0, 0, -.17]],
  ['yaw', [0, 0, 1, .31]],
  ['180 degree roll', [1, 0, 0, Math.PI]],
  ['180 degree pitch', [0, 1, 0, Math.PI]],
  ['180 degree yaw', [0, 0, 1, Math.PI]],
  ['near 180 degree rotation', [1/Math.sqrt(3), 1/Math.sqrt(3), 1/Math.sqrt(3), Math.PI-1e-8]]
]) {
  test('optical-to-Webots pose conversion: ' + name, () => {
    const expected = axisAngleMatrix(axisAngle);
    const { rgbPose, rendered } = evaluate(calibrationWithPose(expected));
    assert.deepEqual(rgbPose.translation, [.104, -.002, .119]);
    assertMatrixClose(axisAngleMatrix(rgbPose.rotation), expected);
    assert.ok(rendered.includes('translation ' + rgbPose.translation.join(' ')));
    assert.ok(rendered.includes('rotation ' + rgbPose.rotation.join(' ')));
  });
}

test('combined tilt, yaw and roll preserve every optical basis vector', () => {
  const expected = multiply(multiply(axisAngleMatrix([0, 0, 1, .31]),
    axisAngleMatrix([0, 1, 0, -.23])), axisAngleMatrix([1, 0, 0, .17]));
  const data = calibrationWithPose(expected);
  const actual = axisAngleMatrix(evaluate(data).rgbPose.rotation);
  assertMatrixClose(multiply(actual, opticalToWebots), data.CameraPoseInRobot.RotationMatrix);
});

test('RGB pose changes do not affect ToF mount or lens intrinsics', () => {
  const before = evaluate();
  const after = evaluate(calibrationWithPose(axisAngleMatrix([0, 1, 0, .5]), [150, 10, 130]));
  const tof = rendered => rendered.match(/DEF TOF_CAMERA_FRAME Pose \{[\s\S]*?\n    \}/)[0];
  assert.equal(tof(before.rendered), tof(after.rendered));
  assert.match(tof(after.rendered), /translation 0\.101 0 0\.11662819049/);
  assert.match(tof(after.rendered), /rotation 0 0 1 -0\.0007963267948966/);
  assert.match(tof(after.rendered), /translation 0 0 -0\.04/);
  assert.equal((after.rendered.match(/name "camera_rgb"/g) || []).length, 1);
  assert.equal((after.rendered.match(/name "rangefinder_tof"/g) || []).length, 1);
  assert.deepEqual(before.settings, after.settings);
});

test('floor origin is tangent to the lowest wheel cylinder without altering saved wheel asymmetry', () => {
  const { rendered } = evaluate();
  const pattern = /endPoint Solid \{\s+translation ([^\n]+)\s+rotation ([^\n]+)[\s\S]*?endPoint Solid \{\s+translation ([^\n]+)\s+rotation ([^\n]+)[\s\S]*?geometry DEF wheel_tire Cylinder \{\s+height (\S+)\s+radius (\S+)[\s\S]*?name "wheel_(\d)_tire"/g;
  const wheels = Array.from(rendered.matchAll(pattern));
  assert.equal(wheels.length, 3);
  const vector = text => text.trim().split(/\s+/).map(Number);
  const expectedBottoms = [0, .00057106276014, .00109008394820];
  for (const [, hubTranslation, hubRotation, tireTranslation, tireRotation, height, radius, index] of wheels) {
    const hubR = axisAngleMatrix(vector(hubRotation));
    const tireR = multiply(hubR, axisAngleMatrix(vector(tireRotation)));
    const local = vector(tireTranslation);
    const centerZ = vector(hubTranslation)[2] + hubR[2].reduce((sum, value, j) => sum + value * local[j], 0);
    // Webots Cylinder's axis is local Z; include both the radius and axial extent.
    const axisZ = tireR[2][2];
    const bottom = centerZ - Number(radius) * Math.sqrt(1 - axisZ * axisZ) - Number(height)/2 * Math.abs(axisZ);
    assert.ok(Math.abs(bottom - expectedBottoms[Number(index)]) < 1e-10, `wheel ${index} bottom ${bottom}`);
  }
});

test('steering anchors, chassis collision shapes and world spawn use the floor origin', () => {
  const { rendered } = evaluate();
  const anchors = Array.from(rendered.matchAll(/axis 0 0 -1\s+anchor ([^\n]+)/g));
  assert.equal(anchors.length, 3);
  for (const [, value] of anchors)
    assert.equal(Number(value.trim().split(/\s+/)[2]), .02262819049);
  const bounding = rendered.slice(rendered.indexOf('boundingObject Group'));
  const translations = Array.from(bounding.matchAll(/translation ([^\n]+)/g));
  assert.equal(translations.length, 6);
  for (const [, value] of translations)
    assert.equal(Number(value.trim().split(/\s+/)[2]), .03962819049);
  const world = fs.readFileSync(path.join(__dirname, '..', 'worlds', 'robocup.dk_2026.wbt'), 'utf8');
  const spawn = world.match(/WildWillyR5 \{\s+translation ([^\n]+)/)[1];
  assert.equal(Number(spawn.trim().split(/\s+/)[2]), 0);
});

for (const [name, mutate, message] of [
  ['translation size', p => p.TranslationMillimeters = [1, 2], /TranslationMillimeters/],
  ['non-numeric translation', p => p.TranslationMillimeters[0] = '100', /TranslationMillimeters/],
  ['matrix shape', p => p.RotationMatrix = [[1]], /3 by 3/],
  ['scaled matrix', p => p.RotationMatrix[0][2] = 2, /orthonormal/],
  ['reflected matrix', p => p.RotationMatrix[0][2] = -1, /determinant/]
]) {
  test('rejects invalid pose: ' + name, () => {
    const data = calibrationWithPose(axisAngleMatrix([0, 0, 1, 0]));
    mutate(data.CameraPoseInRobot);
    assert.throws(() => evaluate(data), message);
  });
}

test('actual PROTO loads JSON and reproduces the former exporter mapping', () => {
  const { settings, rendered, warnings, readCount } = evaluate();
  assert.equal(readCount, 1);
  assert.equal(settings.width, fixture.ImageWidth);
  assert.equal(settings.height, fixture.ImageHeight);
  assert.equal(settings.fieldOfView, 2 * Math.atan(fixture.ImageWidth / (2 * fixture.CameraMatrix[0][0])));
  assert.deepEqual(settings.center, [
    fixture.CameraMatrix[0][2] / fixture.ImageWidth,
    fixture.CameraMatrix[1][2] / fixture.ImageHeight
  ]);
  assert.deepEqual(settings.radialCoefficients, fixture.DistCoeffs.slice(0, 2));
  assert.deepEqual(settings.tangentialCoefficients, fixture.DistCoeffs.slice(2, 4));
  assert.ok(rendered.includes('width ' + fixture.ImageWidth));
  assert.ok(rendered.includes('height ' + fixture.ImageHeight));
  assert.ok(rendered.includes('fieldOfView ' + settings.fieldOfView));
  assert.ok(rendered.includes('center ' + settings.center.join(' ')));
  assert.ok(!rendered.includes('%<'));
  assert.ok(warnings.some(message => message.includes('approximate')));
});

test('Windows backslashes and UTF-8 BOM are accepted', () => {
  assert.deepEqual(evaluate(fixture, {
    protoPath: 'D:\\folder with spaces\\WildWillyR5.proto',
    raw: '\uFEFF' + JSON.stringify(fixture)
  }).settings, evaluate().settings);
});

test('scaled image and camera matrix preserve field of view and lens center', () => {
  const scaled = structuredClone(fixture);
  scaled.ImageWidth /= 2;
  scaled.ImageHeight /= 2;
  scaled.CameraMatrix[0] = scaled.CameraMatrix[0].map(value => value / 2);
  scaled.CameraMatrix[1] = scaled.CameraMatrix[1].map(value => value / 2);
  const before = evaluate().settings;
  const after = evaluate(scaled).settings;
  assert.equal(after.width, before.width / 2);
  assert.equal(after.fieldOfView, before.fieldOfView);
  assert.deepEqual(after.center, before.center);
});

test('external file changes are read on each evaluation and template opts out of caching', () => {
  assert.match(proto, /^# tags: nonDeterministic$/m);
  const changed = structuredClone(fixture);
  changed.CameraMatrix[0][0] *= 2;
  assert.notEqual(evaluate(changed).settings.fieldOfView, evaluate().settings.fieldOfView);
});

test('missing and malformed files fail with descriptive errors', () => {
  assert.throws(() => evaluate(fixture, { exists: false }), /missing camera calibration/);
  assert.throws(() => evaluate(fixture, { raw: '{ broken' }), /invalid calibration JSON/);
  assert.throws(() => evaluate(null), /calibration must be an object/);
});

for (const [name, mutate, expected] of [
  ['missing resolution', data => { delete data.ImageWidth; }, /ImageWidth/],
  ['zero height', data => { data.ImageHeight = 0; }, /ImageHeight/],
  ['fractional width', data => { data.ImageWidth = 768.5; }, /ImageWidth/],
  ['wrong matrix shape', data => { data.CameraMatrix = [[1]]; }, /3 by 3/],
  ['non-numeric matrix', data => { data.CameraMatrix[0][0] = '326'; }, /finite/],
  ['zero focal length', data => { data.CameraMatrix[0][0] = 0; }, /focal lengths/],
  ['missing coefficients', data => { data.DistCoeffs = []; }, /DistCoeffs/],
  ['principal point outside image', data => { data.CameraMatrix[0][2] = -1; }, /principal point/]
]) {
  test('rejects ' + name, () => {
    const data = structuredClone(fixture);
    mutate(data);
    assert.throws(() => evaluate(data), expected);
  });
}
