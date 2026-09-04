export const SUPPORTED_URDF_VERSIONS = ["1.0", "1.1", "1.2"];

export function emptyRobot(name = "robot", version = "1.0") {
  return {
    name,
    version: SUPPORTED_URDF_VERSIONS.includes(version) ? version : "1.0",
    versionExplicit: true,
    materials: new Map(),
    links: new Map(),
    joints: new Map(),
    ros2Controls: [],
    rootExtensions: [],
    rootAttributes: {}
  };
}

export function cloneVec3(v = [0, 0, 0]) {
  return [Number(v[0]) || 0, Number(v[1]) || 0, Number(v[2]) || 0];
}

export function defaultOrigin() {
  return {
    xyz: [0, 0, 0],
    rpy: [0, 0, 0],
    quat: [0, 0, 0, 1],
    orientationFormat: "rpy"
  };
}

export function setOriginRPY(origin, rpy) {
  if (!origin) return;
  origin.rpy = cloneVec3(rpy);
  origin.quat = rpyToQuat(origin.rpy);
  origin.orientationFormat = "rpy";
}

export function setOriginQuat(origin, quat) {
  if (!origin) return;
  origin.quat = normaliseQuat(quat);
  origin.rpy = quatToRpy(origin.quat);
  origin.orientationFormat = "quat";
}

export function normaliseQuat(q = [0, 0, 0, 1]) {
  const values = [0, 1, 2, 3].map(i => Number(q[i]) || 0);
  const length = Math.hypot(...values);
  if (length < 1e-12) return [0, 0, 0, 1];
  return values.map(v => v / length);
}

export function rpyToQuat(rpy = [0, 0, 0]) {
  const [roll, pitch, yaw] = cloneVec3(rpy);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  return normaliseQuat([
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy
  ]);
}

export function quatToRpy(quat = [0, 0, 0, 1]) {
  const [x, y, z, w] = normaliseQuat(quat);
  const sinr = 2 * (w * x + y * z);
  const cosr = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr, cosr);

  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

  const siny = 2 * (w * z + x * y);
  const cosy = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny, cosy);
  return [roll, pitch, yaw];
}

export function rotateVectorByQuat(vector, quat) {
  const [vx, vy, vz] = cloneVec3(vector);
  const [qx, qy, qz, qw] = normaliseQuat(quat);
  const ix = qw * vx + qy * vz - qz * vy;
  const iy = qw * vy + qz * vx - qx * vz;
  const iz = qw * vz + qx * vy - qy * vx;
  const iw = -qx * vx - qy * vy - qz * vz;
  return [
    ix * qw + iw * -qx + iy * -qz - iz * -qy,
    iy * qw + iw * -qy + iz * -qx - ix * -qz,
    iz * qw + iw * -qz + ix * -qy - iy * -qx
  ];
}

export function defaultMaterial() {
  return {
    name: "",
    color: [0.48, 0.78, 0.88, 1],
    texture: "",
    attributes: {},
    extensions: []
  };
}

export function createLink(name, geometry = { type: "box", size: [0.4, 0.4, 0.4] }, mass = 1) {
  return {
    name,
    visuals: [{ name: "", origin: defaultOrigin(), geometry: structuredClone(geometry), material: defaultMaterial(), extensions: [] }],
    collisions: [{ name: "", origin: defaultOrigin(), geometry: structuredClone(geometry), extensions: [] }],
    inertial: {
      origin: defaultOrigin(),
      mass,
      inertia: automaticInertia(geometry, mass),
      extensions: []
    },
    extensions: []
  };
}

export function createJoint(name, type, parent, child) {
  return {
    name,
    type,
    parent,
    child,
    origin: defaultOrigin(),
    axis: [0, 0, 1],
    limit: type === "continuous"
      ? { lower: null, upper: null, effort: 20, velocity: 2, acceleration: null, deceleration: null, jerk: null }
      : { lower: -1.5708, upper: 1.5708, effort: 20, velocity: 2, acceleration: null, deceleration: null, jerk: null },
    value: 0,
    dynamics: null,
    mimic: null,
    extensions: []
  };
}

export function automaticInertia(geometry, mass) {
  const m = Math.max(0, Number(mass) || 0);
  if (!geometry) return { ixx: 0, ixy: 0, ixz: 0, iyy: 0, iyz: 0, izz: 0 };

  if (geometry.type === "box") {
    const [x, y, z] = geometry.size.map(Number);
    return {
      ixx: m * (y * y + z * z) / 12, ixy: 0, ixz: 0,
      iyy: m * (x * x + z * z) / 12, iyz: 0,
      izz: m * (x * x + y * y) / 12
    };
  }

  if (geometry.type === "cylinder") {
    const r = Number(geometry.radius);
    const l = Number(geometry.length);
    return {
      ixx: m * (3 * r * r + l * l) / 12, ixy: 0, ixz: 0,
      iyy: m * (3 * r * r + l * l) / 12, iyz: 0,
      izz: 0.5 * m * r * r
    };
  }

  if (geometry.type === "sphere") {
    const r = Number(geometry.radius);
    const i = 0.4 * m * r * r;
    return { ixx: i, ixy: 0, ixz: 0, iyy: i, iyz: 0, izz: i };
  }

  if (geometry.type === "capsule") {
    const r = Math.max(0, Number(geometry.radius) || 0);
    const l = Math.max(0, Number(geometry.length) || 0);
    const cylinderVolume = Math.PI * r * r * l;
    const capsVolume = 4 / 3 * Math.PI * r ** 3;
    const totalVolume = cylinderVolume + capsVolume;
    if (!(totalVolume > 0)) return { ixx: 0, ixy: 0, ixz: 0, iyy: 0, iyz: 0, izz: 0 };

    const mc = m * cylinderVolume / totalVolume;
    const mh = (m - mc) / 2;
    const ixxCylinder = mc * (3 * r * r + l * l) / 12;
    const izzCylinder = 0.5 * mc * r * r;
    const hemisphereTransverseAtCOM = (83 / 320) * mh * r * r;
    const hemisphereCOMOffset = l / 2 + 3 * r / 8;
    const hemispheresTransverse = 2 * (hemisphereTransverseAtCOM + mh * hemisphereCOMOffset ** 2);
    const hemispheresAxial = 2 * (2 / 5) * mh * r * r;

    return {
      ixx: ixxCylinder + hemispheresTransverse, ixy: 0, ixz: 0,
      iyy: ixxCylinder + hemispheresTransverse, iyz: 0,
      izz: izzCylinder + hemispheresAxial
    };
  }

  return { ixx: 0, ixy: 0, ixz: 0, iyy: 0, iyz: 0, izz: 0 };
}

export function roots(robot) {
  const children = new Set([...robot.joints.values()].map(j => j.child));
  return [...robot.links.keys()].filter(name => !children.has(name));
}

export function childJoints(robot, linkName) {
  return [...robot.joints.values()].filter(joint => joint.parent === linkName);
}

export function parentJoint(robot, linkName) {
  return [...robot.joints.values()].find(joint => joint.child === linkName) || null;
}

export function descendants(robot, linkName) {
  const result = new Set();
  const visit = name => {
    for (const joint of childJoints(robot, name)) {
      if (!result.has(joint.child)) {
        result.add(joint.child);
        visit(joint.child);
      }
    }
  };
  visit(linkName);
  return result;
}

export function uniqueName(base, existing) {
  if (!existing.has(base)) return base;
  let i = 2;
  while (existing.has(`${base}_${i}`)) i += 1;
  return `${base}_${i}`;
}
