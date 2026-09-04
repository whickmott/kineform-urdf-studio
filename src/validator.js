import { roots, SUPPORTED_URDF_VERSIONS } from "./model.js";

function issue(level, message, target = null) { return { level, message, target }; }
function isFiniteVec(v, length = 3) { return Array.isArray(v) && v.length >= length && v.slice(0, length).every(Number.isFinite); }

function validateROS2Control(robot, issues) {
  const names = new Set();
  for (const control of robot.ros2Controls || []) {
    if (!control.name?.trim()) issues.push(issue("error", "A ros2_control block has no name."));
    else if (names.has(control.name)) issues.push(issue("error", `Duplicate ros2_control name "${control.name}".`));
    else names.add(control.name);

    if (!["system", "actuator", "sensor"].includes(control.type)) issues.push(issue("error", `ros2_control "${control.name}" has unsupported type "${control.type}".`));
    if (!control.plugin?.trim()) issues.push(issue("warning", `ros2_control "${control.name}" has no hardware plugin.`));
    if (control.rwRate !== "" && (!Number.isInteger(Number(control.rwRate)) || Number(control.rwRate) < 0)) issues.push(issue("error", `ros2_control "${control.name}" has invalid rw_rate.`));

    for (const component of control.components || []) {
      if (!component.name?.trim()) issues.push(issue("error", `A ${component.kind} in ros2_control "${control.name}" has no name.`));
      if (component.kind === "joint" && !robot.joints.has(component.name)) {
        issues.push(issue("error", `ros2_control "${control.name}" references joint "${component.name}" which is not in the URDF.`, { type: "joint", name: component.name }));
      }
      if (!component.interfaces?.length && component.kind === "gpio") issues.push(issue("warning", `GPIO "${component.name}" in ros2_control "${control.name}" has no interfaces.`));
      for (const intf of component.interfaces || []) {
        if (!intf.name?.trim()) issues.push(issue("error", `${component.kind} "${component.name}" has an unnamed ${intf.kind} interface.`));
        if (!(Number.isInteger(Number(intf.size)) && Number(intf.size) > 0)) issues.push(issue("error", `Interface "${intf.name}" on "${component.name}" has invalid size.`));
      }
    }
  }
}

export function validateRobot(robot) {
  const issues = [];
  const version = robot.version || "1.0";
  if (!SUPPORTED_URDF_VERSIONS.includes(version)) issues.push(issue("error", `Unsupported URDF version "${version}".`));
  if (!robot.name?.trim()) issues.push(issue("warning", "Robot has no name."));
  if (robot.links.size === 0) { issues.push(issue("error", "Robot has no links.")); return issues; }

  const childOwners = new Map();
  for (const joint of robot.joints.values()) {
    if (!robot.links.has(joint.parent)) issues.push(issue("error", `Joint "${joint.name}" references missing parent link "${joint.parent}".`, { type: "joint", name: joint.name }));
    if (!robot.links.has(joint.child)) issues.push(issue("error", `Joint "${joint.name}" references missing child link "${joint.child}".`, { type: "joint", name: joint.name }));
    if (joint.parent === joint.child) issues.push(issue("error", `Joint "${joint.name}" connects "${joint.parent}" to itself.`, { type: "joint", name: joint.name }));
    if (childOwners.has(joint.child)) issues.push(issue("error", `Link "${joint.child}" has more than one parent joint.`, { type: "link", name: joint.child }));
    else childOwners.set(joint.child, joint.name);

    if (!isFiniteVec(joint.origin?.xyz) || !isFiniteVec(joint.origin?.rpy)) issues.push(issue("error", `Joint "${joint.name}" has an invalid origin.`, { type: "joint", name: joint.name }));
    if (!["fixed", "floating"].includes(joint.type)) {
      const axis = joint.axis || [];
      if (!isFiniteVec(axis) || Math.hypot(...axis) < 1e-10) issues.push(issue("error", `Joint "${joint.name}" has a zero or invalid axis.`, { type: "joint", name: joint.name }));
    }

    if (joint.mimic) {
      if (!joint.mimic.joint?.trim()) {
        issues.push(issue("error", `Joint "${joint.name}" has a mimic element with no target joint.`, { type: "joint", name: joint.name }));
      } else if (joint.mimic.joint === joint.name) {
        issues.push(issue("error", `Joint "${joint.name}" cannot mimic itself.`, { type: "joint", name: joint.name }));
      } else if (!robot.joints.has(joint.mimic.joint)) {
        issues.push(issue("error", `Joint "${joint.name}" mimics missing joint "${joint.mimic.joint}".`, { type: "joint", name: joint.name }));
      }
      for (const key of ["multiplier", "offset"]) {
        if (joint.mimic[key] != null && !Number.isFinite(joint.mimic[key])) {
          issues.push(issue("error", `Joint "${joint.name}" has invalid mimic ${key}.`, { type: "joint", name: joint.name }));
        }
      }
    }

    if (joint.dynamics) {
      for (const key of ["damping", "friction"]) {
        if (joint.dynamics[key] != null && !Number.isFinite(joint.dynamics[key])) {
          issues.push(issue("error", `Joint "${joint.name}" has invalid dynamics ${key}.`, { type: "joint", name: joint.name }));
        }
      }
    }

    if (["revolute", "prismatic"].includes(joint.type) && !joint.limit) issues.push(issue("error", `Joint "${joint.name}" requires a limit element.`, { type: "joint", name: joint.name }));
    const limit = joint.limit;
    if (limit) {
      if (limit.lower != null && limit.upper != null && limit.lower > limit.upper) issues.push(issue("error", `Joint "${joint.name}" has lower limit above upper limit.`, { type: "joint", name: joint.name }));
      if (version === "1.2") {
        if (["revolute", "prismatic"].includes(joint.type) && (limit.lower == null || limit.upper == null)) issues.push(issue("error", `URDF 1.2 requires lower and upper limits on ${joint.type} joint "${joint.name}".`, { type: "joint", name: joint.name }));
        for (const key of ["effort", "velocity", "acceleration", "deceleration", "jerk"]) {
          if (limit[key] != null && (!(Number.isFinite(limit[key])) || limit[key] < 0)) issues.push(issue("error", `URDF 1.2 ${key} limit on "${joint.name}" must be non-negative.`, { type: "joint", name: joint.name }));
        }
      } else {
        if (["revolute", "prismatic", "continuous"].includes(joint.type)) {
          if (limit.effort == null) issues.push(issue("error", `URDF ${version} requires effort on joint "${joint.name}".`, { type: "joint", name: joint.name }));
          if (limit.velocity == null) issues.push(issue("error", `URDF ${version} requires velocity on joint "${joint.name}".`, { type: "joint", name: joint.name }));
        }
        if (["acceleration", "deceleration", "jerk"].some(key => limit[key] != null)) issues.push(issue("warning", `Joint "${joint.name}" contains URDF 1.2-only extended limits while the document targets ${version}.`, { type: "joint", name: joint.name }));
      }
    }
  }

  const mimicVisiting = new Set();
  const mimicVisited = new Set();
  const visitMimic = name => {
    if (mimicVisiting.has(name)) {
      issues.push(issue("error", `Mimic cycle detected at joint "${name}".`, { type: "joint", name }));
      return;
    }
    if (mimicVisited.has(name)) return;
    mimicVisiting.add(name);
    const target = robot.joints.get(name)?.mimic?.joint;
    if (target && robot.joints.has(target)) visitMimic(target);
    mimicVisiting.delete(name);
    mimicVisited.add(name);
  };
  for (const name of robot.joints.keys()) visitMimic(name);

  for (const link of robot.links.values()) {
    if (!link.visuals?.length) issues.push(issue("info", `Link "${link.name}" has no visual geometry.`, { type: "link", name: link.name }));
    if (!link.collisions?.length) issues.push(issue("info", `Link "${link.name}" has no collision geometry.`, { type: "link", name: link.name }));
    if (version === "1.0" && [...(link.visuals || []), ...(link.collisions || [])].some(item => item.geometry?.type === "capsule")) issues.push(issue("error", `Link "${link.name}" uses capsule geometry, which requires URDF 1.1 or newer.`, { type: "link", name: link.name }));
    if (!link.inertial) issues.push(issue("warning", `Link "${link.name}" has no inertial definition.`, { type: "link", name: link.name }));
    else {
      if (!(Number.isFinite(link.inertial.mass) && link.inertial.mass >= 0)) issues.push(issue("error", `Link "${link.name}" has invalid mass.`, { type: "link", name: link.name }));
      const i = link.inertial.inertia;
      if (!i || !["ixx", "ixy", "ixz", "iyy", "iyz", "izz"].every(k => Number.isFinite(i[k]))) issues.push(issue("error", `Link "${link.name}" has an invalid inertia tensor.`, { type: "link", name: link.name }));
      else if (i.ixx < 0 || i.iyy < 0 || i.izz < 0) issues.push(issue("error", `Link "${link.name}" has a negative principal inertia term.`, { type: "link", name: link.name }));
    }
  }

  for (const link of robot.links.values()) {
    for (const visual of link.visuals || []) {
      const material = visual.material;
      if (!material?.name || material.color) continue;

      if (!robot.materials?.has?.(material.name)) {
        issues.push(issue(
          "warning",
          `Link "${link.name}" references undefined global material "${material.name}".`,
          { type: "link", name: link.name }
        ));
      }
    }
  }

  if (version === "1.0") {
    const quaternionOrigin = [...robot.joints.values()].some(j => j.origin?.orientationFormat === "quat") || [...robot.links.values()].some(link => [link.inertial, ...(link.visuals || []), ...(link.collisions || [])].some(item => item?.origin?.orientationFormat === "quat"));
    if (quaternionOrigin) issues.push(issue("error", "Quaternion origins require URDF 1.1 or newer."));
  }

  const rootLinks = roots(robot);
  if (rootLinks.length === 0) issues.push(issue("error", "No root link found. The kinematic graph probably contains a cycle."));
  else if (rootLinks.length > 1) issues.push(issue("warning", `Robot has ${rootLinks.length} disconnected root links: ${rootLinks.join(", ")}.`));

  const visiting = new Set(), visited = new Set(), byParent = new Map();
  for (const joint of robot.joints.values()) { if (!byParent.has(joint.parent)) byParent.set(joint.parent, []); byParent.get(joint.parent).push(joint.child); }
  const visit = name => {
    if (visiting.has(name)) { issues.push(issue("error", `Kinematic cycle detected at link "${name}".`, { type: "link", name })); return; }
    if (visited.has(name)) return;
    visiting.add(name); for (const child of byParent.get(name) || []) visit(child); visiting.delete(name); visited.add(name);
  };
  for (const root of rootLinks) visit(root);
  for (const name of robot.links.keys()) if (!visited.has(name)) visit(name);

  validateROS2Control(robot, issues);
  if (!issues.length) issues.push(issue("ok", `No structural URDF ${version} problems detected.`));
  return issues;
}
