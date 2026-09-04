function attr(element, name, fallback = "") {
  return element?.getAttribute(name) ?? fallback;
}

function directChildren(element, tag) {
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

function paramsFrom(element) {
  return directChildren(element, "param").map(param => ({
    name: attr(param, "name"),
    value: param.textContent?.trim() || ""
  }));
}

function unknownChildren(element, known, serializer) {
  return [...element.children]
    .filter(child => !known.has(child.tagName))
    .map(child => serializer.serializeToString(child));
}

function parseInterface(element, serializer) {
  return {
    kind: element.tagName === "command_interface" ? "command" : "state",
    name: attr(element, "name"),
    dataType: attr(element, "data_type", "double"),
    size: Math.max(1, Number(attr(element, "size", 1)) || 1),
    params: paramsFrom(element),
    limitsEnabled: firstDirect(element, "limits")?.getAttribute("enable") ?? null,
    extensions: unknownChildren(element, new Set(["param", "limits"]), serializer)
  };
}

function parseComponent(element, serializer) {
  const interfaces = [
    ...directChildren(element, "command_interface").map(item => parseInterface(item, serializer)),
    ...directChildren(element, "state_interface").map(item => parseInterface(item, serializer))
  ];

  return {
    kind: element.tagName,
    name: attr(element, "name"),
    mimic: element.hasAttribute("mimic") ? attr(element, "mimic") === "true" : null,
    limitsEnabled: firstDirect(element, "limits")?.getAttribute("enable") ?? null,
    params: paramsFrom(element),
    interfaces,
    extensions: unknownChildren(element, new Set(["command_interface", "state_interface", "param", "limits"]), serializer)
  };
}

export function parseROS2ControlElement(element, serializer = new XMLSerializer()) {
  const hardware = firstDirect(element, "hardware");
  const plugin = firstDirect(hardware, "plugin")?.textContent?.trim() || "";
  const group = firstDirect(hardware, "group")?.textContent?.trim() || "";
  const components = [...element.children]
    .filter(child => ["joint", "sensor", "gpio"].includes(child.tagName))
    .map(child => parseComponent(child, serializer));

  return {
    name: attr(element, "name"),
    type: attr(element, "type", "system"),
    rwRate: element.hasAttribute("rw_rate") ? attr(element, "rw_rate") : "",
    isAsync: element.hasAttribute("is_async") ? attr(element, "is_async") === "true" : false,
    threadPriority: element.hasAttribute("thread_priority") ? attr(element, "thread_priority") : "",
    attributes: unknownAttributes(element, new Set(["name", "type", "rw_rate", "is_async", "thread_priority"])),
    plugin,
    group,
    hardwareParams: hardware ? paramsFrom(hardware) : [],
    hardwareExtensions: hardware ? unknownChildren(hardware, new Set(["plugin", "group", "param"]), serializer) : [],
    components,
    extensions: unknownChildren(element, new Set(["hardware", "joint", "sensor", "gpio"]), serializer)
  };
}

export function createROS2Control(
  name = "RobotSystem",
  {
    type = "system",
    plugin = "",
    group = "",
    rwRate = "",
    isAsync = false
  } = {}
) {
  return {
    name,
    type,
    rwRate,
    isAsync,
    threadPriority: "",
    attributes: {},
    plugin,
    group,
    hardwareParams: [],
    hardwareExtensions: [],
    components: [],
    extensions: []
  };
}

export function createROS2Component(kind = "joint", name = "") {
  return {
    kind,
    name,
    mimic: null,
    limitsEnabled: null,
    params: [],
    interfaces: [],
    extensions: []
  };
}

export function createROS2Interface(kind = "state", name = "position") {
  return {
    kind,
    name,
    dataType: "double",
    size: 1,
    params: [],
    limitsEnabled: null,
    extensions: []
  };
}

function esc(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll('"', "&quot;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function paramXML(param, indent) {
  return `${indent}<param name="${esc(param.name)}">${esc(param.value)}</param>\n`;
}

function interfaceXML(item, indent) {
  const tag = item.kind === "command" ? "command_interface" : "state_interface";
  const dataType = item.dataType && item.dataType !== "double" ? ` data_type="${esc(item.dataType)}"` : "";
  const size = Number(item.size) > 1 ? ` size="${Math.round(Number(item.size))}"` : "";
  const hasChildren = (item.params?.length || item.extensions?.length || item.limitsEnabled != null);
  if (!hasChildren) return `${indent}<${tag} name="${esc(item.name)}"${dataType}${size}/>\n`;

  let xml = `${indent}<${tag} name="${esc(item.name)}"${dataType}${size}>\n`;
  for (const param of item.params || []) xml += paramXML(param, `${indent}  `);
  if (item.limitsEnabled != null) xml += `${indent}  <limits enable="${esc(item.limitsEnabled)}"/>\n`;
  for (const raw of item.extensions || []) xml += `${indent}  ${raw}\n`;
  xml += `${indent}</${tag}>\n`;
  return xml;
}

function componentXML(component, indent) {
  const mimic = component.kind === "joint" && component.mimic != null ? ` mimic="${component.mimic ? "true" : "false"}"` : "";
  let xml = `${indent}<${component.kind} name="${esc(component.name)}"${mimic}>\n`;
  if (component.limitsEnabled != null) xml += `${indent}  <limits enable="${esc(component.limitsEnabled)}"/>\n`;
  for (const item of component.interfaces || []) xml += interfaceXML(item, `${indent}  `);
  for (const param of component.params || []) xml += paramXML(param, `${indent}  `);
  for (const raw of component.extensions || []) xml += `${indent}  ${raw}\n`;
  xml += `${indent}</${component.kind}>\n`;
  return xml;
}

export function serialiseROS2Control(control, indent = "  ") {
  const rwRate = control.rwRate !== "" && control.rwRate != null ? ` rw_rate="${esc(control.rwRate)}"` : "";
  const asyncAttr = control.isAsync ? ` is_async="true"` : "";
  const threadPriority = control.isAsync && control.threadPriority !== "" && control.threadPriority != null
    ? ` thread_priority="${esc(control.threadPriority)}"` : "";
  const extraAttributes = Object.entries(control.attributes || {}).map(([key, value]) => ` ${key}="${esc(value)}"`).join("");
  let xml = `${indent}<ros2_control name="${esc(control.name)}" type="${esc(control.type || "system")}"${rwRate}${asyncAttr}${threadPriority}${extraAttributes}>\n`;
  xml += `${indent}  <hardware>\n`;
  if (control.plugin) xml += `${indent}    <plugin>${esc(control.plugin)}</plugin>\n`;
  if (control.group) xml += `${indent}    <group>${esc(control.group)}</group>\n`;
  for (const param of control.hardwareParams || []) xml += paramXML(param, `${indent}    `);
  for (const raw of control.hardwareExtensions || []) xml += `${indent}    ${raw}\n`;
  xml += `${indent}  </hardware>\n`;
  for (const component of control.components || []) xml += componentXML(component, `${indent}  `);
  for (const raw of control.extensions || []) xml += `${indent}  ${raw}\n`;
  xml += `${indent}</ros2_control>\n`;
  return xml;
}
