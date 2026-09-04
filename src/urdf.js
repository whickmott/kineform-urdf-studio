import {
  emptyRobot,
  defaultOrigin,
  defaultMaterial,
  SUPPORTED_URDF_VERSIONS,
  setOriginQuat,
  setOriginRPY,
  rpyToQuat,
  rotateVectorByQuat
} from "./model.js";
import { parseROS2ControlElement, serialiseROS2Control } from "./ros2-control.js";

export { SUPPORTED_URDF_VERSIONS };

function numbers(value, length, fallback = 0) {
  if (!value) return Array(length).fill(fallback);
  const values = value.trim().split(/\s+/).map(Number);
  return Array.from({ length }, (_, i) => Number.isFinite(values[i]) ? values[i] : fallback);
}

function attr(element, name, fallback = "") {
  return element?.getAttribute(name) ?? fallback;
}

function directChildren(element, tag) {
  if (!element) return [];
  return [...element.children].filter(child => child.tagName === tag);
}

function unknownAttributes(element, known = new Set()) {
  const result = {};
  for (const attribute of [...(element?.attributes || [])]) {
    if (!known.has(attribute.name)) result[attribute.name] = attribute.value;
  }
  return result;
}

function firstDirect(element, tag) {
  return directChildren(element, tag)[0] || null;
}

function serialiseUnknownChildren(element, knownTags, serializer) {
  if (!element) return [];

  return [...element.childNodes]
    .filter(node => {
      if (node.nodeType === Node.TEXT_NODE) return Boolean(node.textContent?.trim());
      if (node.nodeType === Node.ELEMENT_NODE) return !knownTags.has(node.tagName);
      return node.nodeType === Node.COMMENT_NODE || node.nodeType === Node.CDATA_SECTION_NODE;
    })
    .map(node => serializer.serializeToString(node));
}

function parseOrigin(parent) {
  const originElement = firstDirect(parent, "origin");
  const origin = defaultOrigin();
  if (!originElement) return origin;
  origin.xyz = numbers(attr(originElement, "xyz"), 3);
  if (originElement.hasAttribute("quat_xyzw")) {
    setOriginQuat(origin, numbers(attr(originElement, "quat_xyzw"), 4, 0));
  } else {
    setOriginRPY(origin, numbers(attr(originElement, "rpy"), 3));
  }
  return origin;
}

function parseGeometry(parent) {
  const geometry = firstDirect(parent, "geometry");
  if (!geometry) return null;

  const box = firstDirect(geometry, "box");
  if (box) return { type: "box", size: numbers(attr(box, "size"), 3, 1) };

  const cylinder = firstDirect(geometry, "cylinder");
  if (cylinder) return { type: "cylinder", radius: Number(attr(cylinder, "radius", 0.5)), length: Number(attr(cylinder, "length", 1)) };

  const sphere = firstDirect(geometry, "sphere");
  if (sphere) return { type: "sphere", radius: Number(attr(sphere, "radius", 0.5)) };

  const capsule = firstDirect(geometry, "capsule");
  if (capsule) return { type: "capsule", radius: Number(attr(capsule, "radius", 0.5)), length: Number(attr(capsule, "length", 1)) };

  const mesh = firstDirect(geometry, "mesh");
  if (mesh) return { type: "mesh", filename: attr(mesh, "filename"), scale: numbers(attr(mesh, "scale"), 3, 1) };

  return null;
}

function parseMaterial(visual, serializer) {
  const material = firstDirect(visual, "material");
  if (!material) return null;

  const color = firstDirect(material, "color");
  const texture = firstDirect(material, "texture");

  return {
    name: attr(material, "name"),
    color: color ? numbers(attr(color, "rgba"), 4, 1) : null,
    texture: texture ? attr(texture, "filename") : "",
    attributes: unknownAttributes(material, new Set(["name"])),
    extensions: serialiseUnknownChildren(material, new Set(["color", "texture"]), serializer)
  };
}

function parseGlobalMaterial(material, serializer) {
  const name = attr(material, "name");
  const color = firstDirect(material, "color");
  const texture = firstDirect(material, "texture");

  return {
    name,
    color: color ? numbers(attr(color, "rgba"), 4, 1) : null,
    texture: texture ? attr(texture, "filename") : "",
    attributes: unknownAttributes(material, new Set(["name"])),
    extensions: serialiseUnknownChildren(material, new Set(["color", "texture"]), serializer)
  };
}

function parseInertial(link, serializer) {
  const inertial = firstDirect(link, "inertial");
  if (!inertial) return null;
  const massElement = firstDirect(inertial, "mass");
  const inertiaElement = firstDirect(inertial, "inertia");
  return {
    origin: parseOrigin(inertial),
    mass: Number(attr(massElement, "value", 0)),
    inertia: {
      ixx: Number(attr(inertiaElement, "ixx", 0)), ixy: Number(attr(inertiaElement, "ixy", 0)), ixz: Number(attr(inertiaElement, "ixz", 0)),
      iyy: Number(attr(inertiaElement, "iyy", 0)), iyz: Number(attr(inertiaElement, "iyz", 0)), izz: Number(attr(inertiaElement, "izz", 0))
    },
    extensions: serialiseUnknownChildren(inertial, new Set(["origin", "mass", "inertia"]), serializer)
  };
}

function parseVersion(robotElement) {
  const explicit = robotElement.hasAttribute("version");
  const version = explicit ? attr(robotElement, "version") : "1.0";
  if (!/^\d+\.\d+$/.test(version)) throw new Error(`Invalid URDF version "${version}". Expected major.minor, for example 1.0.`);
  if (!SUPPORTED_URDF_VERSIONS.includes(version)) throw new Error(`URDF ${version} is not supported. KineForm supports 1.0, 1.1 and 1.2.`);
  return { version, explicit };
}


function losslessError(message) {
  throw new Error(`${message} No changes were applied.`);
}

function requireNonEmptyAttribute(element, name, context) {
  if (!element?.hasAttribute(name) || !element.getAttribute(name).trim()) {
    losslessError(`${context} is missing required attribute "${name}".`);
  }
  return element.getAttribute(name).trim();
}

function validateNumberText(value, context) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) losslessError(`${context} must be a finite number; received "${value}".`);
  return parsed;
}

function validateVectorAttribute(element, name, count, context) {
  if (!element?.hasAttribute(name)) return;
  const raw = element.getAttribute(name).trim();
  const parts = raw ? raw.split(/\s+/) : [];
  if (parts.length !== count) {
    losslessError(`${context} "${name}" must contain exactly ${count} numbers; received "${raw}".`);
  }
  for (const part of parts) validateNumberText(part, `${context} "${name}"`);
}

function validateOptionalNumberAttribute(element, name, context) {
  if (element?.hasAttribute(name)) validateNumberText(element.getAttribute(name), `${context} "${name}"`);
}

function validateOriginElement(parent, context) {
  const origin = firstDirect(parent, "origin");
  if (!origin) return;

  validateVectorAttribute(origin, "xyz", 3, `${context} origin`);
  validateVectorAttribute(origin, "rpy", 3, `${context} origin`);
  validateVectorAttribute(origin, "quat_xyzw", 4, `${context} origin`);

  if (origin.hasAttribute("rpy") && origin.hasAttribute("quat_xyzw")) {
    losslessError(`${context} origin contains both "rpy" and "quat_xyzw"; KineForm will not guess which orientation should win.`);
  }
}

function validateGeometryElement(parent, context) {
  const geometry = firstDirect(parent, "geometry");
  if (!geometry) losslessError(`${context} has no <geometry> element.`);

  const children = [...geometry.children];
  if (children.length !== 1) {
    losslessError(`${context} geometry must contain exactly one geometry primitive; found ${children.length}.`);
  }

  const primitive = children[0];
  const type = primitive.tagName;

  if (!["box", "cylinder", "sphere", "capsule", "mesh"].includes(type)) {
    losslessError(`${context} uses unsupported geometry <${type}>. KineForm will not delete or replace it.`);
  }

  if (type === "box") {
    requireNonEmptyAttribute(primitive, "size", `${context} box`);
    validateVectorAttribute(primitive, "size", 3, `${context} box`);
  } else if (type === "cylinder") {
    validateNumberText(requireNonEmptyAttribute(primitive, "radius", `${context} cylinder`), `${context} cylinder radius`);
    validateNumberText(requireNonEmptyAttribute(primitive, "length", `${context} cylinder`), `${context} cylinder length`);
  } else if (type === "sphere") {
    validateNumberText(requireNonEmptyAttribute(primitive, "radius", `${context} sphere`), `${context} sphere radius`);
  } else if (type === "capsule") {
    validateNumberText(requireNonEmptyAttribute(primitive, "radius", `${context} capsule`), `${context} capsule radius`);
    validateNumberText(requireNonEmptyAttribute(primitive, "length", `${context} capsule`), `${context} capsule length`);
  } else if (type === "mesh") {
    requireNonEmptyAttribute(primitive, "filename", `${context} mesh`);
    validateVectorAttribute(primitive, "scale", 3, `${context} mesh`);
  }
}

function validateInertialElement(linkElement, linkName) {
  const inertial = firstDirect(linkElement, "inertial");
  if (!inertial) return;

  validateOriginElement(inertial, `Link "${linkName}" inertial`);

  const mass = firstDirect(inertial, "mass");
  if (!mass) losslessError(`Link "${linkName}" inertial has no <mass> element.`);
  validateNumberText(requireNonEmptyAttribute(mass, "value", `Link "${linkName}" mass`), `Link "${linkName}" mass value`);

  const inertia = firstDirect(inertial, "inertia");
  if (!inertia) losslessError(`Link "${linkName}" inertial has no <inertia> element.`);
  for (const key of ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]) {
    validateNumberText(requireNonEmptyAttribute(inertia, key, `Link "${linkName}" inertia`), `Link "${linkName}" inertia ${key}`);
  }
}

function validateROS2ControlForLosslessParse(element) {
  const name = element.getAttribute("name") || "(unnamed)";
  if (element.hasAttribute("is_async") && !["true", "false"].includes(element.getAttribute("is_async"))) {
    losslessError(`ros2_control "${name}" has invalid is_async="${element.getAttribute("is_async")}".`);
  }

  for (const component of [...element.children].filter(child => ["joint", "sensor", "gpio"].includes(child.tagName))) {
    if (component.hasAttribute("mimic") && !["true", "false"].includes(component.getAttribute("mimic"))) {
      losslessError(`ros2_control "${name}" component "${component.getAttribute("name") || "(unnamed)"}" has invalid mimic="${component.getAttribute("mimic")}".`);
    }

    for (const intf of [...component.children].filter(child => ["command_interface", "state_interface"].includes(child.tagName))) {
      if (intf.hasAttribute("size")) {
        const size = Number(intf.getAttribute("size"));
        if (!Number.isInteger(size) || size < 1) {
          losslessError(`ros2_control "${name}" interface "${intf.getAttribute("name") || "(unnamed)"}" has invalid size="${intf.getAttribute("size")}".`);
        }
      }
    }
  }
}

function validateKnownXMLForLosslessParse(robotElement) {
  const linkNames = new Set();
  const jointNames = new Set();
  const materialNames = new Set();

  directChildren(robotElement, "material").forEach((materialElement, index) => {
    const name = requireNonEmptyAttribute(materialElement, "name", `Global material ${index + 1}`);
    if (materialNames.has(name)) losslessError(`Duplicate global material name "${name}" would overwrite an earlier material.`);
    materialNames.add(name);

    const color = firstDirect(materialElement, "color");
    validateVectorAttribute(color, "rgba", 4, `Global material "${name}" colour`);

    const texture = firstDirect(materialElement, "texture");
    if (texture) requireNonEmptyAttribute(texture, "filename", `Global material "${name}" texture`);
  });

  const links = directChildren(robotElement, "link");
  links.forEach((linkElement, index) => {
    const name = requireNonEmptyAttribute(linkElement, "name", `Link ${index + 1}`);
    if (linkNames.has(name)) losslessError(`Duplicate link name "${name}" would overwrite an earlier link.`);
    linkNames.add(name);

    validateInertialElement(linkElement, name);

    directChildren(linkElement, "visual").forEach((visual, visualIndex) => {
      validateOriginElement(visual, `Link "${name}" visual ${visualIndex + 1}`);
      validateGeometryElement(visual, `Link "${name}" visual ${visualIndex + 1}`);

      const material = firstDirect(visual, "material");
      if (material) {
        const color = firstDirect(material, "color");
        validateVectorAttribute(color, "rgba", 4, `Link "${name}" visual ${visualIndex + 1} material colour`);

        const texture = firstDirect(material, "texture");
        if (texture) requireNonEmptyAttribute(texture, "filename", `Link "${name}" visual ${visualIndex + 1} material texture`);
      }
    });

    directChildren(linkElement, "collision").forEach((collision, collisionIndex) => {
      validateOriginElement(collision, `Link "${name}" collision ${collisionIndex + 1}`);
      validateGeometryElement(collision, `Link "${name}" collision ${collisionIndex + 1}`);
    });
  });

  const joints = directChildren(robotElement, "joint");
  joints.forEach((jointElement, index) => {
    const name = requireNonEmptyAttribute(jointElement, "name", `Joint ${index + 1}`);
    if (jointNames.has(name)) losslessError(`Duplicate joint name "${name}" would overwrite an earlier joint.`);
    jointNames.add(name);

    validateOriginElement(jointElement, `Joint "${name}"`);

    const parent = firstDirect(jointElement, "parent");
    const child = firstDirect(jointElement, "child");
    if (!parent) losslessError(`Joint "${name}" has no <parent> element.`);
    if (!child) losslessError(`Joint "${name}" has no <child> element.`);
    requireNonEmptyAttribute(parent, "link", `Joint "${name}" parent`);
    requireNonEmptyAttribute(child, "link", `Joint "${name}" child`);

    const axis = firstDirect(jointElement, "axis");
    validateVectorAttribute(axis, "xyz", 3, `Joint "${name}" axis`);

    const limit = firstDirect(jointElement, "limit");
    if (limit) {
      for (const key of ["lower", "upper", "effort", "velocity", "acceleration", "deceleration", "jerk"]) {
        validateOptionalNumberAttribute(limit, key, `Joint "${name}" limit`);
      }
    }

    const dynamics = firstDirect(jointElement, "dynamics");
    if (dynamics) {
      validateOptionalNumberAttribute(dynamics, "damping", `Joint "${name}" dynamics`);
      validateOptionalNumberAttribute(dynamics, "friction", `Joint "${name}" dynamics`);
    }

    const mimic = firstDirect(jointElement, "mimic");
    if (mimic) {
      requireNonEmptyAttribute(mimic, "joint", `Joint "${name}" mimic`);
      validateOptionalNumberAttribute(mimic, "multiplier", `Joint "${name}" mimic`);
      validateOptionalNumberAttribute(mimic, "offset", `Joint "${name}" mimic`);
    }
  });

  directChildren(robotElement, "ros2_control").forEach(validateROS2ControlForLosslessParse);
}

export function parseURDF(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, "application/xml");
  const parserError = doc.querySelector("parsererror");
  if (parserError) {
    throw new Error(`XML syntax error: ${parserError.textContent.trim().replace(/\s+/g, " ")} No changes were applied.`);
  }

  const robotElement = doc.documentElement;
  if (!robotElement || robotElement.tagName !== "robot") {
    throw new Error("The document does not contain a <robot> root element. No changes were applied.");
  }

  validateKnownXMLForLosslessParse(robotElement);

  const { version, explicit } = parseVersion(robotElement);
  const serializer = new XMLSerializer();
  const robot = emptyRobot(attr(robotElement, "name", "robot"), version);
  robot.versionExplicit = explicit;
  robot.rootAttributes = unknownAttributes(robotElement, new Set(["name", "version"]));

  for (const materialElement of directChildren(robotElement, "material")) {
    const material = parseGlobalMaterial(materialElement, serializer);
    robot.materials.set(material.name, material);
  }

  for (const linkElement of directChildren(robotElement, "link")) {
    const name = attr(linkElement, "name");

    const visuals = directChildren(linkElement, "visual").map(visual => ({
      name: attr(visual, "name"),
      origin: parseOrigin(visual),
      geometry: parseGeometry(visual),
      material: parseMaterial(visual, serializer),
      extensions: serialiseUnknownChildren(visual, new Set(["origin", "geometry", "material"]), serializer)
    })).filter(item => item.geometry);

    const collisions = directChildren(linkElement, "collision").map(collision => ({
      name: attr(collision, "name"),
      origin: parseOrigin(collision),
      geometry: parseGeometry(collision),
      extensions: serialiseUnknownChildren(collision, new Set(["origin", "geometry"]), serializer)
    })).filter(item => item.geometry);

    robot.links.set(name, {
      name,
      visuals,
      collisions,
      inertial: parseInertial(linkElement, serializer),
      extensions: serialiseUnknownChildren(linkElement, new Set(["visual", "collision", "inertial"]), serializer)
    });
  }

  for (const jointElement of directChildren(robotElement, "joint")) {
    const name = attr(jointElement, "name");
    const parent = firstDirect(jointElement, "parent");
    const child = firstDirect(jointElement, "child");
    const axis = firstDirect(jointElement, "axis");
    const limit = firstDirect(jointElement, "limit");
    const dynamics = firstDirect(jointElement, "dynamics");
    const mimic = firstDirect(jointElement, "mimic");
    const type = attr(jointElement, "type", "fixed");
    const axisValues = numbers(attr(axis, "xyz"), 3, 0);

    robot.joints.set(name, {
      name,
      type,
      parent: attr(parent, "link"),
      child: attr(child, "link"),
      origin: parseOrigin(jointElement),
      axis: axisValues.every(v => v === 0) ? [1, 0, 0] : axisValues,
      limit: limit ? {
        lower: limit.hasAttribute("lower") ? Number(attr(limit, "lower")) : null,
        upper: limit.hasAttribute("upper") ? Number(attr(limit, "upper")) : null,
        effort: limit.hasAttribute("effort") ? Number(attr(limit, "effort")) : null,
        velocity: limit.hasAttribute("velocity") ? Number(attr(limit, "velocity")) : null,
        acceleration: limit.hasAttribute("acceleration") ? Number(attr(limit, "acceleration")) : null,
        deceleration: limit.hasAttribute("deceleration") ? Number(attr(limit, "deceleration")) : null,
        jerk: limit.hasAttribute("jerk") ? Number(attr(limit, "jerk")) : null
      } : null,
      dynamics: dynamics ? {
        damping: dynamics.hasAttribute("damping") ? Number(attr(dynamics, "damping")) : null,
        friction: dynamics.hasAttribute("friction") ? Number(attr(dynamics, "friction")) : null,
        attributes: unknownAttributes(dynamics, new Set(["damping", "friction"])),
        extensions: serialiseUnknownChildren(dynamics, new Set(), serializer)
      } : null,
      mimic: mimic ? {
        joint: attr(mimic, "joint"),
        multiplier: mimic.hasAttribute("multiplier") ? Number(attr(mimic, "multiplier")) : null,
        offset: mimic.hasAttribute("offset") ? Number(attr(mimic, "offset")) : null,
        attributes: unknownAttributes(mimic, new Set(["joint", "multiplier", "offset"])),
        extensions: serialiseUnknownChildren(mimic, new Set(), serializer)
      } : null,
      value: 0,
      extensions: serialiseUnknownChildren(jointElement, new Set(["parent", "child", "origin", "axis", "limit", "dynamics", "mimic"]), serializer)
    });
  }

  robot.ros2Controls = directChildren(robotElement, "ros2_control").map(element => parseROS2ControlElement(element, serializer));
  robot.rootExtensions = serialiseUnknownChildren(robotElement, new Set(["material", "link", "joint", "ros2_control"]), serializer);
  return robot;
}

function n(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "0";
  if (Math.abs(number) < 1e-12) return "0";
  return Number(number.toPrecision(10)).toString();
}

function vec(values) { return values.map(n).join(" "); }

function esc(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll('"', "&quot;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function originXML(origin, indent, version) {
  if (!origin) return "";
  const xyz = origin.xyz || [0, 0, 0];
  if (version !== "1.0" && origin.orientationFormat === "quat") {
    return `${indent}<origin xyz="${vec(xyz)}" quat_xyzw="${vec(origin.quat || rpyToQuat(origin.rpy))}"/>\n`;
  }
  return `${indent}<origin xyz="${vec(xyz)}" rpy="${vec(origin.rpy || [0, 0, 0])}"/>\n`;
}

function geometryXML(geometry, indent, version) {
  if (!geometry) return "";
  if (geometry.type === "box") return `${indent}<geometry>\n${indent}  <box size="${vec(geometry.size)}"/>\n${indent}</geometry>\n`;
  if (geometry.type === "cylinder") return `${indent}<geometry>\n${indent}  <cylinder radius="${n(geometry.radius)}" length="${n(geometry.length)}"/>\n${indent}</geometry>\n`;
  if (geometry.type === "sphere") return `${indent}<geometry>\n${indent}  <sphere radius="${n(geometry.radius)}"/>\n${indent}</geometry>\n`;
  if (geometry.type === "capsule" && version !== "1.0") return `${indent}<geometry>\n${indent}  <capsule radius="${n(geometry.radius)}" length="${n(geometry.length)}"/>\n${indent}</geometry>\n`;
  if (geometry.type === "mesh") {
    const scale = geometry.scale ? ` scale="${vec(geometry.scale)}"` : "";
    return `${indent}<geometry>\n${indent}  <mesh filename="${esc(geometry.filename || "")}"${scale}/>\n${indent}</geometry>\n`;
  }
  return "";
}

function rawExtensions(items, indent) {
  return (items || []).map(raw => `${indent}${raw}\n`).join("");
}

function materialAttributes(material) {
  return Object.entries(material?.attributes || {})
    .map(([key, value]) => ` ${key}="${esc(value)}"`)
    .join("");
}

function materialXML(material, indent) {
  if (!material) return "";

  const name = material.name ? ` name="${esc(material.name)}"` : "";
  const extraAttributes = materialAttributes(material);
  const extensions = material.extensions || [];
  const hasBody = Boolean(material.color) || Boolean(material.texture) || extensions.length > 0;

  if (!hasBody) {
    return `${indent}<material${name}${extraAttributes}/>\n`;
  }

  let xml = `${indent}<material${name}${extraAttributes}>\n`;
  if (material.color) xml += `${indent}  <color rgba="${vec(material.color)}"/>\n`;
  if (material.texture) xml += `${indent}  <texture filename="${esc(material.texture)}"/>\n`;
  xml += rawExtensions(extensions, `${indent}  `);
  xml += `${indent}</material>\n`;
  return xml;
}

function globalMaterialXML(material) {
  return materialXML(material, "  ");
}

function visualXML(visual, version) {
  const name = visual.name ? ` name="${esc(visual.name)}"` : "";
  let xml = `  <visual${name}>\n`;
  xml += originXML(visual.origin, "    ", version);
  xml += geometryXML(visual.geometry, "    ", version);
  xml += materialXML(visual.material, "    ");
  xml += rawExtensions(visual.extensions, "    ");
  xml += `  </visual>\n`;
  return xml;
}

function collisionXML(collision, version) {
  const name = collision.name ? ` name="${esc(collision.name)}"` : "";
  let xml = `  <collision${name}>\n`;
  xml += originXML(collision.origin, "    ", version);
  xml += geometryXML(collision.geometry, "    ", version);
  xml += rawExtensions(collision.extensions, "    ");
  xml += `  </collision>\n`;
  return xml;
}

function inertialXML(inertial, version) {
  if (!inertial) return "";
  const i = inertial.inertia || {};
  let xml = `  <inertial>\n`;
  xml += originXML(inertial.origin || defaultOrigin(), "    ", version);
  xml += `    <mass value="${n(inertial.mass)}"/>\n`;
  xml += `    <inertia ixx="${n(i.ixx)}" ixy="${n(i.ixy)}" ixz="${n(i.ixz)}" iyy="${n(i.iyy)}" iyz="${n(i.iyz)}" izz="${n(i.izz)}"/>\n`;
  xml += rawExtensions(inertial.extensions, "    ");
  xml += `  </inertial>\n`;
  return xml;
}

function limitXML(joint, version) {
  if (!joint.limit) return "";
  const limit = joint.limit;
  const attrs = [];
  if (limit.lower != null) attrs.push(`lower="${n(limit.lower)}"`);
  if (limit.upper != null) attrs.push(`upper="${n(limit.upper)}"`);
  if (limit.effort != null) attrs.push(`effort="${n(limit.effort)}"`);
  if (limit.velocity != null) attrs.push(`velocity="${n(limit.velocity)}"`);
  if (version === "1.2") {
    if (limit.acceleration != null) attrs.push(`acceleration="${n(limit.acceleration)}"`);
    if (limit.deceleration != null) attrs.push(`deceleration="${n(limit.deceleration)}"`);
    if (limit.jerk != null) attrs.push(`jerk="${n(limit.jerk)}"`);
  }
  return attrs.length ? `    <limit ${attrs.join(" ")}/>\n` : "";
}

function attributesXML(attributes) {
  return Object.entries(attributes || {})
    .map(([key, value]) => ` ${key}="${esc(value)}"`)
    .join("");
}

function dynamicsXML(dynamics) {
  if (!dynamics) return "";
  const attrs = [];
  if (dynamics.damping != null) attrs.push(`damping="${n(dynamics.damping)}"`);
  if (dynamics.friction != null) attrs.push(`friction="${n(dynamics.friction)}"`);
  const extra = attributesXML(dynamics.attributes);
  const extensions = dynamics.extensions || [];
  if (!extensions.length) return `    <dynamics${attrs.length ? ` ${attrs.join(" ")}` : ""}${extra}/>\n`;
  let xml = `    <dynamics${attrs.length ? ` ${attrs.join(" ")}` : ""}${extra}>\n`;
  xml += rawExtensions(extensions, "      ");
  xml += `    </dynamics>\n`;
  return xml;
}

function mimicXML(mimic) {
  if (!mimic) return "";
  const attrs = [`joint="${esc(mimic.joint || "")}"`];
  if (mimic.multiplier != null) attrs.push(`multiplier="${n(mimic.multiplier)}"`);
  if (mimic.offset != null) attrs.push(`offset="${n(mimic.offset)}"`);
  const extra = attributesXML(mimic.attributes);
  const extensions = mimic.extensions || [];
  if (!extensions.length) return `    <mimic ${attrs.join(" ")}${extra}/>\n`;
  let xml = `    <mimic ${attrs.join(" ")}${extra}>\n`;
  xml += rawExtensions(extensions, "      ");
  xml += `    </mimic>\n`;
  return xml;
}

export function serialiseURDF(robot) {
  const version = SUPPORTED_URDF_VERSIONS.includes(robot.version) ? robot.version : "1.0";
  const extraAttributes = Object.entries(robot.rootAttributes || {}).map(([key, value]) => ` ${key}="${esc(value)}"`).join("");
  let xml = `<?xml version="1.0"?>\n<robot name="${esc(robot.name || "robot")}" version="${version}"${extraAttributes}>\n`;

  for (const material of robot.materials?.values?.() || []) {
    xml += globalMaterialXML(material);
  }

  for (const link of robot.links.values()) {
    xml += `  <link name="${esc(link.name)}">\n`;
    xml += inertialXML(link.inertial, version);
    for (const visual of link.visuals || []) xml += visualXML(visual, version);
    for (const collision of link.collisions || []) xml += collisionXML(collision, version);
    xml += rawExtensions(link.extensions, "  ");
    xml += `  </link>\n`;
  }

  for (const joint of robot.joints.values()) {
    xml += `  <joint name="${esc(joint.name)}" type="${esc(joint.type)}">\n`;
    xml += `    <parent link="${esc(joint.parent)}"/>\n`;
    xml += `    <child link="${esc(joint.child)}"/>\n`;
    xml += originXML(joint.origin, "    ", version);
    if (!["fixed", "floating"].includes(joint.type)) xml += `    <axis xyz="${vec(joint.axis || [1, 0, 0])}"/>\n`;
    xml += limitXML(joint, version);
    xml += dynamicsXML(joint.dynamics);
    xml += mimicXML(joint.mimic);
    xml += rawExtensions(joint.extensions, "    ");
    xml += `  </joint>\n`;
  }

  for (const control of robot.ros2Controls || []) xml += serialiseROS2Control(control, "  ");
  xml += rawExtensions(robot.rootExtensions, "  ");
  xml += `</robot>\n`;
  return xml;
}

function allOrigins(robot) {
  const origins = [];
  for (const link of robot.links.values()) {
    if (link.inertial?.origin) origins.push(link.inertial.origin);
    for (const visual of link.visuals || []) origins.push(visual.origin);
    for (const collision of link.collisions || []) origins.push(collision.origin);
  }
  for (const joint of robot.joints.values()) origins.push(joint.origin);
  return origins;
}

function capsuleCount(robot) {
  let count = 0;
  for (const link of robot.links.values()) {
    for (const item of [...(link.visuals || []), ...(link.collisions || [])]) if (item.geometry?.type === "capsule") count += 1;
  }
  return count;
}

function extendedLimitCount(robot) {
  let count = 0;
  for (const joint of robot.joints.values()) {
    if (["acceleration", "deceleration", "jerk"].some(key => joint.limit?.[key] != null)) count += 1;
  }
  return count;
}

export function analyseVersionConversion(robot, targetVersion) {
  if (!SUPPORTED_URDF_VERSIONS.includes(targetVersion)) throw new Error(`Unsupported URDF version ${targetVersion}.`);
  const current = robot.version || "1.0";
  const notes = [];
  const warnings = [];
  if (current === targetVersion) return { current, targetVersion, notes, warnings, changes: false };

  if (targetVersion === "1.0") {
    const quats = allOrigins(robot).filter(origin => origin?.orientationFormat === "quat").length;
    const capsules = capsuleCount(robot);
    if (quats) notes.push(`${quats} quaternion origin${quats === 1 ? "" : "s"} will be converted to equivalent RPY.`);
    if (capsules) notes.push(`${capsules} capsule primitive${capsules === 1 ? "" : "s"} will be expanded into a cylinder and two spheres.`);
  }

  if (targetVersion !== "1.2") {
    const extended = extendedLimitCount(robot);
    if (extended) warnings.push(`Acceleration/deceleration/jerk limits on ${extended} joint${extended === 1 ? "" : "s"} are URDF 1.2-only and will be removed.`);
  }

  if (Number(targetVersion) > Number(current)) notes.push(`The document version will be upgraded from ${current} to ${targetVersion}; existing features remain unchanged.`);
  if (!notes.length && !warnings.length) notes.push(`The document version will be changed from ${current} to ${targetVersion}.`);
  notes.push("ROS 2 control blocks and preserved extension XML are retained unchanged.");
  return { current, targetVersion, notes, warnings, changes: true };
}

function shiftedOrigin(origin, localOffset) {
  const next = structuredClone(origin || defaultOrigin());
  const quat = next.quat || rpyToQuat(next.rpy || [0, 0, 0]);
  const offset = rotateVectorByQuat(localOffset, quat);
  next.xyz = [0, 1, 2].map(i => Number(next.xyz?.[i] || 0) + offset[i]);
  return next;
}

function expandCapsuleItem(item) {
  const radius = Number(item.geometry.radius);
  const length = Number(item.geometry.length);
  const baseName = item.name || "";
  const body = structuredClone(item);
  body.name = baseName ? `${baseName}_body` : "";
  body.geometry = { type: "cylinder", radius, length };
  const capA = structuredClone(item);
  capA.name = baseName ? `${baseName}_cap_a` : "";
  capA.geometry = { type: "sphere", radius };
  capA.origin = shiftedOrigin(item.origin, [0, 0, length / 2]);
  const capB = structuredClone(item);
  capB.name = baseName ? `${baseName}_cap_b` : "";
  capB.geometry = { type: "sphere", radius };
  capB.origin = shiftedOrigin(item.origin, [0, 0, -length / 2]);
  return [body, capA, capB];
}

export function convertRobotVersion(robot, targetVersion) {
  const report = analyseVersionConversion(robot, targetVersion);
  if (!report.changes) return report;

  if (targetVersion === "1.0") {
    for (const origin of allOrigins(robot)) {
      if (!origin) continue;
      origin.orientationFormat = "rpy";
      origin.quat = rpyToQuat(origin.rpy || [0, 0, 0]);
    }
    for (const link of robot.links.values()) {
      link.visuals = (link.visuals || []).flatMap(item => item.geometry?.type === "capsule" ? expandCapsuleItem(item) : [item]);
      link.collisions = (link.collisions || []).flatMap(item => item.geometry?.type === "capsule" ? expandCapsuleItem(item) : [item]);
    }
  }

  if (targetVersion !== "1.2") {
    for (const joint of robot.joints.values()) {
      if (!joint.limit) continue;
      joint.limit.acceleration = null;
      joint.limit.deceleration = null;
      joint.limit.jerk = null;
    }
  }

  robot.version = targetVersion;
  robot.versionExplicit = true;
  return report;
}

export function formatXML(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, "application/xml");
  if (doc.querySelector("parsererror")) return xmlText;
  const serializer = new XMLSerializer();
  const raw = serializer.serializeToString(doc);
  const tokens = raw.replace(/>\s*</g, "><").split(/(?=<)|(?<=>)/g).filter(Boolean);
  let depth = 0;
  const lines = [];
  for (const token of tokens) {
    if (!token.startsWith("<")) {
      if (token.trim()) lines.push(`${"  ".repeat(depth)}${token.trim()}`);
      continue;
    }
    if (/^<\//.test(token)) depth = Math.max(0, depth - 1);
    lines.push(`${"  ".repeat(depth)}${token}`);
    if (/^<[^!?/][^>]*[^/]?>$/.test(token) && !token.includes("</")) depth += 1;
  }
  return lines.join("\n").replace(/^\s*<\?xml[^>]*>\s*\n?/, '<?xml version="1.0"?>\n') + "\n";
}
