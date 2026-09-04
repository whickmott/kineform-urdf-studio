import { RobotRenderer } from "./renderer.js";
import { parseURDF, serialiseURDF, formatXML, analyseVersionConversion, convertRobotVersion, SUPPORTED_URDF_VERSIONS } from "./urdf.js";
import {
  emptyRobot,
  createLink,
  createJoint,
  automaticInertia,
  defaultOrigin,
  defaultMaterial,
  roots,
  childJoints,
  parentJoint,
  descendants,
  uniqueName,
  setOriginRPY
} from "./model.js";
import { validateRobot } from "./validator.js";
import { saveMeshFiles, loadMeshFiles, removeMeshFile } from "./mesh-store.js";
import { JointAnimationController } from "./animation.js";
import { WorkspaceLayout } from "./layout.js";
import { createROS2Control, createROS2Component, createROS2Interface } from "./ros2-control.js";
import { RobotHistory } from "./history.js";
import { loadExampleRegistry, exampleById } from "./examples.js";

const $ = selector => document.querySelector(selector);

const elements = {
  viewport: $("#viewport"),
  robotTitle: $("#robot-title"),
  tree: $("#tree"),
  meshLibrary: $("#mesh-library"),
  meshLibraryMeta: $("#mesh-library-meta"),
  meshLibraryAdd: $("#mesh-library-add"),
  jointControls: $("#joint-controls"),
  inspector: $("#inspector"),
  inspectorTitle: $("#inspector-title"),
  deleteSelected: $("#delete-selected"),
  xmlEditor: $("#xml-editor"),
  xmlStatus: $("#xml-status"),
  diagnostics: $("#diagnostics"),
  validationSummary: $("#validation-summary"),
  doctorTab: $("#bottom-tab-diagnostics"),
  doctorErrorCount: $("#doctor-error-count"),
  doctorWarningCount: $("#doctor-warning-count"),
  urdfFile: $("#urdf-file"),
  meshFiles: $("#mesh-files"),
  newRobot: $("#new-robot"),
  exampleSelect: $("#example-select"),
  exportURDF: $("#export-urdf"),
  undoEdit: $("#undo-edit"),
  redoEdit: $("#redo-edit"),
  aboutApp: $("#about-app"),
  aboutDialog: $("#about-dialog"),
  mobileNoticeDialog: $("#mobile-notice-dialog"),
  urdfVersion: $("#urdf-version"),
  urdfVersionDialog: $("#urdf-version-dialog"),
  urdfVersionSummary: $("#urdf-version-summary"),
  urdfVersionDialogTitle: $("#urdf-version-dialog-title"),
  confirmURDFVersion: $("#confirm-urdf-version"),
  ros2Editor: $("#ros2-editor"),
  materialsEditor: $("#materials-editor"),
  materialsMeta: $("#materials-meta"),
  addMaterial: $("#add-material"),
  ros2AddControl: $("#ros2-add-control"),
  ros2AddControlDialog: $("#ros2-add-control-dialog"),
  ros2AddControlForm: $("#ros2-add-control-form"),
  confirmROS2AddControl: $("#confirm-ros2-add-control"),
  ros2NewName: $("#ros2-new-name"),
  ros2NewType: $("#ros2-new-type"),
  ros2NewPlugin: $("#ros2-new-plugin"),
  ros2NewGroup: $("#ros2-new-group"),
  ros2NewRate: $("#ros2-new-rate"),
  ros2NewAsync: $("#ros2-new-async"),
  addLink: $("#add-link"),
  addJoint: $("#add-joint"),
  formatXML: $("#format-xml"),
  applyXML: $("#apply-xml"),
  selectMode: $("#select-mode"),
  moveMode: $("#move-mode"),
  rotateMode: $("#rotate-mode"),
  toggleVisual: $("#toggle-visual"),
  toggleCollision: $("#toggle-collision"),
  toggleFrames: $("#toggle-frames"),
  toggleAxes: $("#toggle-axes"),
  toggleCom: $("#toggle-com"),
  frameRobot: $("#frame-robot"),
  toggleAnimation: $("#toggle-animation"),
  animationPanel: $("#animation-panel"),
  animationPlay: $("#animation-play"),
  animationStop: $("#animation-stop"),
  animationCapture: $("#animation-capture"),
  animationDeleteKey: $("#animation-delete-key"),
  animationClear: $("#animation-clear"),
  animationTimeline: $("#animation-timeline"),
  animationMarkers: $("#animation-markers"),
  animationDuration: $("#animation-duration"),
  animationLoop: $("#animation-loop"),
  animationSummary: $("#animation-summary"),
  animationReadout: $("#animation-readout"),
  animationExport: $("#animation-export"),
  animationImport: $("#animation-import"),
  toggleViewportSettings: $("#toggle-viewport-settings"),
  viewportSettings: $("#viewport-settings"),
  resetViewportSettings: $("#reset-viewport-settings"),
  viewportBackground: $("#viewport-background"),
  viewportFloorVisible: $("#viewport-floor-visible"),
  viewportFloorColour: $("#viewport-floor-colour"),
  viewportGridVisible: $("#viewport-grid-visible"),
  viewportGridCentre: $("#viewport-grid-centre"),
  viewportGridLines: $("#viewport-grid-lines"),
  viewportGridSize: $("#viewport-grid-size"),
  viewportGridDivisions: $("#viewport-grid-divisions"),
  viewportGridOpacity: $("#viewport-grid-opacity"),
  viewportWorldAxes: $("#viewport-world-axes"),
  viewportShadows: $("#viewport-shadows"),
  workspace: $("#workspace"),
  leftDock: $("#left-dock"),
  rightDock: $("#right-dock"),
  bottomDock: $("#bottom-dock"),
  leftSplitter: $("#left-splitter"),
  rightSplitter: $("#right-splitter"),
  bottomSplitter: $("#bottom-splitter"),
  toggleLeftDock: $("#toggle-left-dock"),
  toggleRightDock: $("#toggle-right-dock"),
  toggleBottomDock: $("#toggle-bottom-dock"),
  collapseRightDock: $("#collapse-right-dock"),
  collapseBottomDock: $("#collapse-bottom-dock"),
  resetLayout: $("#reset-layout"),
  dropOverlay: $("#drop-overlay"),
  addLinkDialog: $("#add-link-dialog"),
  addLinkForm: $("#add-link-form"),
  linkGeometryFields: $("#link-geometry-fields"),
  confirmAddLink: $("#confirm-add-link"),
  addJointDialog: $("#add-joint-dialog"),
  addJointForm: $("#add-joint-form"),
  confirmAddJoint: $("#confirm-add-joint")
};

let robot = emptyRobot("untitled_robot");
let selection = null;
let sourceDirty = false;
let sourceApplyIssue = null;
let draggedTreeLink = null;
let meshAssets = [];
let exampleAssets = [];
let currentExample = null;
let pendingURDFVersion = null;
let applyingHistory = false;

const DEFAULT_VIEWPORT_SETTINGS = {
  background: "#0c0f12",
  floorVisible: true,
  floorColour: "#11161a",
  gridVisible: true,
  gridCentre: "#7894a8",
  gridLines: "#34414b",
  gridSize: 20,
  gridDivisions: 40,
  gridOpacity: 0.48,
  worldAxes: true,
  shadows: true
};

let viewportSettings = loadViewportSettings();

const renderer = new RobotRenderer(elements.viewport, {
  onSelect(next) {
    select(next);
  },
  onTransformPreview() {
    sourceDirty = true;
    setXMLStatus("Joint transform changed. Release the gizmo to commit.", "dirty");
  },
  onTransformCommit(jointName) {
    selection = { type: "joint", name: jointName };
    synchroniseFromModel({ rebuild: false });
  }
});


const animation = new JointAnimationController({
  getRobot: () => robot,
  renderer,
  elements: {
    toggle: elements.toggleAnimation,
    panel: elements.animationPanel,
    play: elements.animationPlay,
    stop: elements.animationStop,
    capture: elements.animationCapture,
    deleteKey: elements.animationDeleteKey,
    clear: elements.animationClear,
    timeline: elements.animationTimeline,
    markers: elements.animationMarkers,
    duration: elements.animationDuration,
    loop: elements.animationLoop,
    summary: elements.animationSummary,
    readout: elements.animationReadout,
    exportButton: elements.animationExport,
    importInput: elements.animationImport
  },
  onPoseChanged() {
    updateJointControlInputs();
  }
});

const workspaceLayout = new WorkspaceLayout({
  workspace: elements.workspace,
  leftDock: elements.leftDock,
  rightDock: elements.rightDock,
  bottomDock: elements.bottomDock,
  leftSplitter: elements.leftSplitter,
  rightSplitter: elements.rightSplitter,
  bottomSplitter: elements.bottomSplitter,
  toggleLeft: elements.toggleLeftDock,
  toggleRight: elements.toggleRightDock,
  toggleBottom: elements.toggleBottomDock,
  collapseRight: elements.collapseRightDock,
  collapseBottom: elements.collapseBottomDock,
  reset: elements.resetLayout
});

const history = new RobotHistory({
  limit: 120,
  onChange(state) {
    if (!elements.undoEdit || !elements.redoEdit) return;
    elements.undoEdit.disabled = !state.canUndo;
    elements.redoEdit.disabled = !state.canRedo;
    elements.undoEdit.title = state.canUndo ? `Undo ${state.undoLabel}` : "Undo";
    elements.redoEdit.title = state.canRedo ? `Redo ${state.redoLabel}` : "Redo";
  }
});



function escapeHTML(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function num(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

let generatedFormField = 0;

function ensureFormFieldAccessibility(root = document) {
  const selector = "input, select, textarea";
  const fields = [];

  if (root instanceof Element && root.matches(selector)) fields.push(root);
  if (root.querySelectorAll) fields.push(...root.querySelectorAll(selector));

  for (const control of fields) {
    if (!control.id) control.id = `urdf-field-${++generatedFormField}`;
    if (!control.name) control.name = control.id;

    if (control instanceof HTMLInputElement) {
      const type = (control.type || "text").toLowerCase();
      if (["text", "search", "email", "url", "tel", "password"].includes(type) && !control.hasAttribute("autocomplete")) {
        control.setAttribute("autocomplete", "off");
      }
    }

    const hasLabel = Boolean(
      control.closest("label") ||
      document.querySelector(`label[for="${CSS.escape(control.id)}"]`) ||
      control.hasAttribute("aria-label") ||
      control.hasAttribute("aria-labelledby")
    );

    if (hasLabel) continue;

    const nearby =
      control.closest(".input-with-label")?.querySelector("span")?.textContent?.trim() ||
      control.closest(".field")?.querySelector(".field-label")?.textContent?.trim() ||
      control.closest(".joint-control")?.querySelector("label")?.textContent?.trim() ||
      control.closest(".viewport-setting-row")?.querySelector("span")?.textContent?.trim() ||
      control.closest(".viewport-setting-pair label")?.querySelector("span")?.textContent?.trim() ||
      control.closest("fieldset")?.querySelector("legend")?.textContent?.trim() ||
      control.getAttribute("title") ||
      control.name ||
      control.id;

    control.setAttribute("aria-label", nearby || "URDF editor field");
  }
}

const formAccessibilityObserver = new MutationObserver(records => {
  for (const record of records) {
    for (const node of record.addedNodes) {
      if (node instanceof Element) ensureFormFieldAccessibility(node);
    }
  }
});

formAccessibilityObserver.observe(document.body, { childList: true, subtree: true });
ensureFormFieldAccessibility(document);

function vectorInput(values, names = ["X", "Y", "Z"], className = "triple") {
  return `
    <div class="${className}">
      ${names.map((name, i) => `
        <div class="input-with-label">
          <span>${name}</span>
          <input type="number" step="any" data-index="${i}" value="${Number(values?.[i] ?? 0)}">
        </div>
      `).join("")}
    </div>
  `;
}

function field(label, content) {
  return `<div class="field"><div class="field-label">${label}</div>${content}</div>`;
}

function section(title, content) {
  return `<section class="form-section"><div class="form-section-title">${title}</div>${content}</section>`;
}

function actionSection(title, actionId, actionLabel, content) {
  return `<section class="form-section">
    <div class="form-section-title form-section-title-row">
      <span>${title}</span>
      <button type="button" class="mini-section-action" id="${actionId}">${actionLabel}</button>
    </div>
    ${content}
  </section>`;
}

async function loadRobot(nextRobot, {
  frame = true,
  sourceText = null,
  sourceStatus = null,
  historyMode = "reset",
  historyLabel = "Loaded robot"
} = {}) {
  robot = nextRobot;
  animation.resetForRobot();
  selection = roots(robot)[0] ? { type: "link", name: roots(robot)[0] } : null;
  sourceDirty = false;
  sourceApplyIssue = null;
  elements.urdfVersion.value = robot.version || "1.0";

  if (sourceText === null) {
    elements.xmlEditor.value = serialiseURDF(robot);
  } else {
    elements.xmlEditor.value = sourceText;
  }

  setXMLStatus(sourceStatus || "Model and source are synchronised.");
  renderAll();
  await renderer.setRobot(robot, !frame);
  renderer.setSelection(selection);
  if (frame) renderer.frameRobot();

  if (!applyingHistory) {
    if (historyMode === "record") history.record(robot, selection, historyLabel);
    else history.reset(robot, selection, historyLabel);
  }
}

function renderAll() {
  elements.robotTitle.textContent = robot.name || "Robot";
  renderTree();
  renderMeshLibrary();
  renderJointControls();
  renderInspector();
  renderDiagnostics();
  renderMaterials();
  renderROS2Control();
}

function renderTree() {
  const rootNames = roots(robot);
  const seen = new Set();

  const renderLink = (linkName, stack = new Set()) => {
    const visible = renderer.isLinkVisible(linkName);
    const selected = selection?.type === "link" && selection.name === linkName ? " selected" : "";

    if (stack.has(linkName)) {
      return `
        <li class="tree-node">
          <div class="tree-row link${selected}" draggable="true" data-drag-link="${escapeHTML(linkName)}" data-drop-link="${escapeHTML(linkName)}">
            <button class="tree-select" type="button" data-type="link" data-name="${escapeHTML(linkName)}">
              <span class="tree-kind">L</span>
              <span class="tree-label">${escapeHTML(linkName)} ↻</span>
            </button>
            <button class="tree-add-child" type="button" data-add-child="${escapeHTML(linkName)}" title="Add child link" aria-label="Add child to ${escapeHTML(linkName)}">+</button>
            <button class="tree-visibility${visible ? "" : " hidden-link"}" type="button" data-toggle-link="${escapeHTML(linkName)}" title="${visible ? "Hide link" : "Show link"}" aria-label="${visible ? "Hide" : "Show"} ${escapeHTML(linkName)}">${visible ? "◉" : "○"}</button>
          </div>
        </li>
      `;
    }

    seen.add(linkName);
    const nextStack = new Set(stack);
    nextStack.add(linkName);

    const joints = childJoints(robot, linkName).map(joint => {
      const jointSelected = selection?.type === "joint" && selection.name === joint.name ? " selected" : "";
      return `
        <li class="tree-node">
          <div class="tree-row joint${jointSelected}">
            <button class="tree-select" type="button" data-type="joint" data-name="${escapeHTML(joint.name)}">
              <span class="tree-kind">J</span>
              <span class="tree-label">${escapeHTML(joint.name)} · ${escapeHTML(joint.type)}</span>
            </button>
          </div>
          <ul>${robot.links.has(joint.child) ? renderLink(joint.child, nextStack) : ""}</ul>
        </li>
      `;
    }).join("");

    return `
      <li class="tree-node">
        <div class="tree-row link${selected}" draggable="true" data-drag-link="${escapeHTML(linkName)}" data-drop-link="${escapeHTML(linkName)}">
          <button class="tree-select" type="button" data-type="link" data-name="${escapeHTML(linkName)}">
            <span class="tree-kind">L</span>
            <span class="tree-label">${escapeHTML(linkName)}</span>
          </button>
          <button class="tree-add-child" type="button" data-add-child="${escapeHTML(linkName)}" title="Add child link" aria-label="Add child to ${escapeHTML(linkName)}">+</button>
          <button class="tree-visibility${visible ? "" : " hidden-link"}" type="button" data-toggle-link="${escapeHTML(linkName)}" title="${visible ? "Hide link" : "Show link"}" aria-label="${visible ? "Hide" : "Show"} ${escapeHTML(linkName)}">${visible ? "◉" : "○"}</button>
        </div>
        ${joints ? `<ul>${joints}</ul>` : ""}
      </li>
    `;
  };

  let html = `
    <div class="tree-root-drop" data-root-drop>
      Drop here to make a link a root
    </div>
    <ul>${rootNames.map(name => renderLink(name)).join("")}</ul>
  `;

  const unreached = [...robot.links.keys()].filter(name => !seen.has(name));
  if (unreached.length) {
    html += `<div class="form-section-title" style="margin:10px 5px 5px">Unconnected / cyclic</div>`;
    html += `<ul>${unreached.map(name => renderLink(name)).join("")}</ul>`;
  }

  elements.tree.innerHTML = robot.links.size ? html : `<div class="empty-state">No links yet.</div>`;

  elements.tree.querySelectorAll("[data-type][data-name]").forEach(button => {
    button.addEventListener("click", () => select({
      type: button.dataset.type,
      name: button.dataset.name
    }));
  });

  elements.tree.querySelectorAll("[data-add-child]").forEach(button => {
    button.addEventListener("click", event => {
      event.stopPropagation();
      openAddLinkDialog(button.dataset.addChild);
    });
  });

  elements.tree.querySelectorAll("[data-toggle-link]").forEach(button => {
    button.addEventListener("click", event => {
      event.stopPropagation();
      const name = button.dataset.toggleLink;
      renderer.setLinkVisibility(name, !renderer.isLinkVisible(name));
      renderTree();
    });
  });

  elements.tree.querySelectorAll("[data-drag-link]").forEach(row => {
    row.addEventListener("dragstart", event => {
      const name = row.dataset.dragLink;
      draggedTreeLink = name;
      event.dataTransfer.effectAllowed = "move";
      event.dataTransfer.setData("text/plain", name);
      row.classList.add("drag-source");
      elements.tree.classList.add("dragging-link");
    });

    row.addEventListener("dragend", () => {
      draggedTreeLink = null;
      row.classList.remove("drag-source");
      elements.tree.classList.remove("dragging-link");
      elements.tree.querySelectorAll(".drag-over").forEach(item => item.classList.remove("drag-over"));
    });
  });

  elements.tree.querySelectorAll("[data-drop-link]").forEach(row => {
    row.addEventListener("dragover", event => {
      const dragged = draggedTreeLink || event.dataTransfer.getData("text/plain");
      if (!dragged || !canReparentLink(dragged, row.dataset.dropLink)) return;
      event.preventDefault();
      event.dataTransfer.dropEffect = "move";
      row.classList.add("drag-over");
    });

    row.addEventListener("dragleave", () => row.classList.remove("drag-over"));

    row.addEventListener("drop", event => {
      event.preventDefault();
      row.classList.remove("drag-over");
      const dragged = draggedTreeLink || event.dataTransfer.getData("text/plain");
      reparentLink(dragged, row.dataset.dropLink);
    });
  });

  const rootDrop = elements.tree.querySelector("[data-root-drop]");
  rootDrop?.addEventListener("dragover", event => {
    const dragged = draggedTreeLink || event.dataTransfer.getData("text/plain");
    if (!dragged || !parentJoint(robot, dragged)) return;
    event.preventDefault();
    event.dataTransfer.dropEffect = "move";
    rootDrop.classList.add("drag-over");
  });

  rootDrop?.addEventListener("dragleave", () => rootDrop.classList.remove("drag-over"));

  rootDrop?.addEventListener("drop", event => {
    event.preventDefault();
    rootDrop.classList.remove("drag-over");
    const dragged = draggedTreeLink || event.dataTransfer.getData("text/plain");
    makeLinkRoot(dragged);
  });
}

function canReparentLink(linkName, newParentName) {
  if (!robot.links.has(linkName) || !robot.links.has(newParentName)) return false;
  if (linkName === newParentName) return false;
  if (descendants(robot, linkName).has(newParentName)) return false;

  const current = parentJoint(robot, linkName);
  return current?.parent !== newParentName;
}

function reparentLink(linkName, newParentName) {
  if (!canReparentLink(linkName, newParentName)) {
    setXMLStatus("That drop would create an invalid or unchanged parent relationship.", "error");
    return;
  }

  const existing = parentJoint(robot, linkName);

  if (existing) {
    existing.parent = newParentName;
    selection = { type: "joint", name: existing.name };
  } else {
    const jointName = uniqueName(`${newParentName}_to_${linkName}`, new Set(robot.joints.keys()));
    const joint = createJoint(jointName, "fixed", newParentName, linkName);
    joint.limit = null;
    robot.joints.set(jointName, joint);
    selection = { type: "joint", name: jointName };
  }

  synchroniseFromModel();
}

function makeLinkRoot(linkName) {
  const joint = parentJoint(robot, linkName);
  if (!joint) return;
  robot.joints.delete(joint.name);
  selection = { type: "link", name: linkName };
  synchroniseFromModel();
}
function formatBytes(bytes) {
  if (!Number.isFinite(bytes) || bytes <= 0) return "0 B";
  const units = ["B", "KB", "MB", "GB"];
  const order = Math.min(Math.floor(Math.log(bytes) / Math.log(1024)), units.length - 1);
  const value = bytes / (1024 ** order);
  return `${value >= 10 || order === 0 ? value.toFixed(0) : value.toFixed(1)} ${units[order]}`;
}

function meshBaseName(filename) {
  return String(filename || "")
    .replace(/^package:\/\/[^/]+\//, "")
    .replace(/^file:\/\//, "")
    .split(/[\\/]/)
    .pop();
}

function isMeshAsset(fileOrName) {
  const name = typeof fileOrName === "string" ? fileOrName : fileOrName?.name;
  return /\.(stl|obj|dae)$/i.test(name || "");
}

function isTextureAsset(fileOrName) {
  const name = typeof fileOrName === "string" ? fileOrName : fileOrName?.name;
  return /\.(png|jpe?g|webp|gif|bmp|tiff?)$/i.test(name || "");
}

function supportedLocalAsset(fileOrName) {
  return isMeshAsset(fileOrName) || isTextureAsset(fileOrName);
}

function allAssets() {
  const byName = new Map();
  for (const file of meshAssets) byName.set(file.name, file);
  for (const file of exampleAssets) byName.set(file.name, file);
  return [...byName.values()];
}

function isBundledAsset(file) {
  return exampleAssets.includes(file);
}

function meshReferenceCount(filename) {
  const target = meshBaseName(filename);
  let count = 0;

  for (const link of robot.links.values()) {
    for (const item of [...(link.visuals || []), ...(link.collisions || [])]) {
      if (item.geometry?.type === "mesh" && meshBaseName(item.geometry.filename) === target) count += 1;
    }
  }

  return count;
}

function renderMeshLibrary() {
  const assets = allAssets();
  const totalSize = assets.reduce((sum, file) => sum + (file.size || 0), 0);
  const selectedLink = selection?.type === "link" ? robot.links.get(selection.name) : null;

  const meshCount = assets.filter(isMeshAsset).length;
  const textureCount = assets.filter(isTextureAsset).length;
  const bundledCount = assets.filter(isBundledAsset).length;

  elements.meshLibraryMeta.textContent = assets.length
    ? `${meshCount} ${meshCount === 1 ? "mesh" : "meshes"} · ${textureCount} ${textureCount === 1 ? "texture" : "textures"} · ${formatBytes(totalSize)}${bundledCount ? ` · ${bundledCount} bundled read-only` : " · local"}`
    : "Stored locally in this browser, not on the website server";

  if (!assets.length) {
    elements.meshLibrary.innerHTML = `<div class="mesh-library-empty">Import STL, OBJ, DAE and PNG/JPG/WebP/TIFF texture dependencies. Local imports remain available after a reload on this device.</div>`;
    return;
  }

  elements.meshLibrary.innerHTML = assets.map(file => {
    const extension = file.name.split(".").pop()?.toUpperCase() || "ASSET";
    const mesh = isMeshAsset(file);
    const refs = mesh ? meshReferenceCount(file.name) : null;
    const disabled = selectedLink ? "" : " disabled";
    const bundled = isBundledAsset(file);

    return `
      <div class="mesh-asset${bundled ? " bundled" : ""}" title="${escapeHTML(file.name)}${bundled ? " · bundled example asset · read-only" : ""}">
        <div class="mesh-asset-main">
          <div class="mesh-asset-name">${escapeHTML(file.name)}</div>
          <div class="mesh-asset-info">${
            mesh
              ? `${extension} · ${formatBytes(file.size)} · ${refs} ${refs === 1 ? "reference" : "references"}`
              : `${extension} texture · ${formatBytes(file.size)} · dependency`
          }</div>
        </div>
        <div class="mesh-asset-actions">
          ${mesh ? `
            <button type="button" data-mesh-preview="${escapeHTML(file.name)}" title="Preview mesh in viewport">P</button>
            <button type="button" data-mesh-visual="${escapeHTML(file.name)}" title="Add as visual to selected link"${disabled}>V</button>
            <button type="button" data-mesh-collision="${escapeHTML(file.name)}" title="Add as collision to selected link"${disabled}>C</button>
          ` : ""}
          ${bundled
            ? `<span class="asset-lock" title="Bundled example asset · read-only">LOCK</span>`
            : `<button type="button" class="mesh-remove" data-mesh-remove="${escapeHTML(file.name)}" title="Remove from local asset library">×</button>`}
        </div>
      </div>
    `;
  }).join("");

  elements.meshLibrary.querySelectorAll("[data-mesh-preview]").forEach(button => {
    button.addEventListener("click", () => previewMeshAsset(button.dataset.meshPreview));
  });

  elements.meshLibrary.querySelectorAll("[data-mesh-visual]").forEach(button => {
    button.addEventListener("click", () => addMeshToSelectedLink(button.dataset.meshVisual, "visual"));
  });

  elements.meshLibrary.querySelectorAll("[data-mesh-collision]").forEach(button => {
    button.addEventListener("click", () => addMeshToSelectedLink(button.dataset.meshCollision, "collision"));
  });

  elements.meshLibrary.querySelectorAll("[data-mesh-remove]").forEach(button => {
    button.addEventListener("click", () => removeMeshFromLibrary(button.dataset.meshRemove));
  });
}

async function previewMeshAsset(filename) {
  const shown = await renderer.previewAsset(filename);
  if (shown) {
    setXMLStatus(`Previewing ${filename}. Select any link or joint to return to the robot.`);
  } else {
    setXMLStatus(`Could not preview ${filename}.`, "error");
  }
}

function addMeshToSelectedLink(filename, kind) {
  if (selection?.type !== "link") return;
  const link = robot.links.get(selection.name);
  if (!link) return;

  const geometry = { type: "mesh", filename, scale: [1, 1, 1] };

  if (kind === "visual") {
    link.visuals ||= [];
    link.visuals.push({
      name: "",
      origin: defaultOrigin(),
      geometry,
      material: defaultMaterial()
    });
  } else {
    link.collisions ||= [];
    link.collisions.push({
      name: "",
      origin: defaultOrigin(),
      geometry
    });
  }

  synchroniseFromModel();
}

async function importMeshes(files) {
  const incoming = [...files].filter(supportedLocalAsset);
  if (!incoming.length) return;

  let persistent = true;
  try {
    await saveMeshFiles(incoming);
    meshAssets = await loadMeshFiles();
  } catch (error) {
    console.warn("Persistent asset storage unavailable; keeping assets for this session.", error);
    persistent = false;
    const byName = new Map(meshAssets.map(file => [file.name, file]));
    for (const file of incoming) byName.set(file.name, file);
    meshAssets = [...byName.values()].sort((a, b) => a.name.localeCompare(b.name));
  }

  await renderer.setAssets(allAssets());
  renderMeshLibrary();
  await renderer.setRobot(robot, true);
  renderer.setSelection(selection);
  const importedMeshes = incoming.filter(isMeshAsset).length;
  const importedTextures = incoming.filter(isTextureAsset).length;
  const parts = [];
  if (importedMeshes) parts.push(`${importedMeshes} ${importedMeshes === 1 ? "mesh" : "meshes"}`);
  if (importedTextures) parts.push(`${importedTextures} ${importedTextures === 1 ? "texture" : "textures"}`);
  setXMLStatus(`${parts.join(" and ")} imported${persistent ? " to this browser's local library" : " for this session"}.`);
}

async function removeMeshFromLibrary(name) {
  try {
    await removeMeshFile(name);
  } catch (error) {
    console.warn("Could not remove asset from IndexedDB.", error);
  }

  meshAssets = meshAssets.filter(file => file.name !== name);
  await renderer.setAssets(allAssets());
  renderMeshLibrary();
  await renderer.setRobot(robot, true);
  renderer.setSelection(selection);
}

function meshAssetOptions(currentFilename) {
  const currentBase = meshBaseName(currentFilename);
  const assets = allAssets();
  const currentLoaded = assets.some(file => file.name === currentFilename || file.name === currentBase);
  let html = `<option value="">Choose from local library…</option>`;

  if (currentFilename && !currentLoaded) {
    html += `<option value="${escapeHTML(currentFilename)}" selected>${escapeHTML(currentFilename)} · not loaded</option>`;
  }

  html += assets.filter(isMeshAsset).map(file => {
    const selected = file.name === currentFilename || file.name === currentBase ? " selected" : "";
    return `<option value="${escapeHTML(file.name)}"${selected}>${escapeHTML(file.name)}</option>`;
  }).join("");

  return html;
}

function renderJointControls() {
  const joints = [...robot.joints.values()].filter(j => ["revolute", "continuous", "prismatic"].includes(j.type));

  if (!joints.length) {
    elements.jointControls.innerHTML = `<div class="empty-state">No movable joints.</div>`;
    return;
  }

  elements.jointControls.innerHTML = joints.map(joint => {
    let min = -Math.PI;
    let max = Math.PI;
    let step = 0.001;

    if (joint.type === "prismatic") {
      min = joint.limit?.lower ?? -0.5;
      max = joint.limit?.upper ?? 0.5;
      step = 0.001;
    } else if (joint.type === "revolute") {
      min = joint.limit?.lower ?? -Math.PI;
      max = joint.limit?.upper ?? Math.PI;
    }

    const safeIndex = joints.indexOf(joint);
    const rangeId = `joint-range-${safeIndex}`;
    const numberId = `joint-number-${safeIndex}`;

    return `
      <div class="joint-control">
        <label for="${rangeId}" title="${escapeHTML(joint.name)}">${escapeHTML(joint.name)}</label>
        <input id="${rangeId}" name="${rangeId}" aria-label="${escapeHTML(joint.name)} joint slider" type="range" data-joint-range="${escapeHTML(joint.name)}" min="${min}" max="${max}" step="${step}" value="${joint.value || 0}">
        <input id="${numberId}" name="${numberId}" aria-label="${escapeHTML(joint.name)} joint value" type="number" data-joint-number="${escapeHTML(joint.name)}" min="${min}" max="${max}" step="${step}" value="${Number(joint.value || 0).toFixed(3)}">
      </div>
    `;
  }).join("");

  for (const joint of joints) {
    const range = elements.jointControls.querySelector(`[data-joint-range="${CSS.escape(joint.name)}"]`);
    const number = elements.jointControls.querySelector(`[data-joint-number="${CSS.escape(joint.name)}"]`);

    const update = (value, { writeNumber = true } = {}) => {
      const parsed = Number(value);
      if (!Number.isFinite(parsed)) return false;

      animation.pause();

      let next = parsed;
      if (joint.type !== "continuous") {
        const min = joint.limit?.lower ?? Number(range.min);
        const max = joint.limit?.upper ?? Number(range.max);
        next = clamp(next, min, max);
      }

      joint.value = next;
      range.value = next;

      if (writeNumber) {
        number.value = next.toFixed(3);
      }

      renderer.updateJointValue(joint.name, next);
      animation.notifyManualPoseChanged();
      return true;
    };

    const commitNumber = () => {
      const raw = number.value.trim();

      if (!raw || number.validity.badInput || !update(raw, { writeNumber: true })) {
        number.value = Number(joint.value || 0).toFixed(3);
      }
    };

    range.addEventListener("input", () => update(range.value));

    number.addEventListener("input", () => {
      const raw = number.value.trim();

      // Empty/intermediate number states such as "-" must remain editable.
      if (!raw || number.validity.badInput) return;

      // Update the pose live, but never rewrite the active field.
      update(raw, { writeNumber: false });
    });

    number.addEventListener("change", commitNumber);

    number.addEventListener("keydown", event => {
      if (event.key !== "Enter") return;
      event.preventDefault();
      commitNumber();
      number.blur();
    });
  }
}

function updateJointControlInputs() {
  for (const joint of robot.joints.values()) {
    if (!["revolute", "continuous", "prismatic"].includes(joint.type)) continue;
    const range = elements.jointControls.querySelector(`[data-joint-range="${CSS.escape(joint.name)}"]`);
    const number = elements.jointControls.querySelector(`[data-joint-number="${CSS.escape(joint.name)}"]`);
    if (range) range.value = joint.value || 0;
    if (number && document.activeElement !== number) number.value = Number(joint.value || 0).toFixed(3);
  }
}

function renderInspector() {
  if (!elements.inspector || !elements.inspectorTitle) return;

  if (!selection) {
    elements.inspectorTitle.textContent = "Nothing selected";
    elements.inspector.innerHTML = `<div class="empty-state">Select a link or joint in the tree or viewport.</div>`;
    elements.deleteSelected.hidden = true;
    return;
  }

  if (selection.type === "link") {
    const link = robot.links.get(selection.name);
    if (!link) {
      selection = null;
      renderInspector();
      return;
    }

    renderLinkInspector(link);
    return;
  }

  if (selection.type === "joint") {
    const joint = robot.joints.get(selection.name);
    if (!joint) {
      selection = null;
      renderInspector();
      return;
    }

    renderJointInspector(joint);
    return;
  }

  selection = null;
  renderInspector();
}

function renderLinkInspector(link) {
  if (!link) {
    select(null);
    return;
  }

  elements.inspectorTitle.textContent = link.name;
  elements.deleteSelected.hidden = false;

  const inertial = link.inertial || {
    origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
    mass: 0,
    inertia: { ixx: 0, ixy: 0, ixz: 0, iyy: 0, iyz: 0, izz: 0 }
  };

  let html = section("Link", field("Name", `<input id="link-name" value="${escapeHTML(link.name)}">`));

  html += actionSection("Visual geometry", "add-visual-geometry", "+ Visual", (link.visuals || []).map((visual, index) =>
    geometryEditor("visual", index, visual.geometry, visual.origin, visual.material, visual.name)
  ).join("") || `<div class="empty-state">No visual geometry.</div>`);

  html += actionSection("Collision geometry", "add-collision-geometry", "+ Collision", (link.collisions || []).map((collision, index) =>
    geometryEditor("collision", index, collision.geometry, collision.origin, null, collision.name)
  ).join("") || `<div class="empty-state">No collision geometry.</div>`);

  html += section("Inertial", `
    ${field("Mass (kg)", `<input id="link-mass" type="number" step="any" min="0" value="${inertial.mass}">`)}
    ${field("COM xyz (m)", `<div id="inertial-xyz">${vectorInput(inertial.origin.xyz)}</div>`)}
    ${field("Inertial rpy (rad)", `<div id="inertial-rpy">${vectorInput(inertial.origin.rpy, ["R", "P", "Y"])}</div>`)}
    ${field("Inertia tensor", `
      <div id="inertia-values" class="six">
        ${["ixx", "ixy", "ixz", "iyy", "iyz", "izz"].map(key => `
          <div class="input-with-label">
            <span>${key.toUpperCase()}</span>
            <input type="number" step="any" data-key="${key}" value="${inertial.inertia[key] ?? 0}">
          </div>
        `).join("")}
      </div>
      <button id="auto-inertia" type="button" style="margin-top:6px">Calculate from visual + mass</button>
    `)}
  `);

  elements.inspector.innerHTML = html;

  $("#link-name").addEventListener("change", event => renameLink(link.name, event.target.value.trim()));

  $("#add-visual-geometry")?.addEventListener("click", () => {
    link.visuals ||= [];
    link.visuals.push({
      name: "",
      origin: defaultOrigin(),
      geometry: { type: "box", size: [0.2, 0.2, 0.2] },
      material: defaultMaterial(),
      extensions: []
    });
    synchroniseFromModel({ historyLabel: "Add visual geometry" });
  });

  $("#add-collision-geometry")?.addEventListener("click", () => {
    link.collisions ||= [];
    link.collisions.push({
      name: "",
      origin: defaultOrigin(),
      geometry: { type: "box", size: [0.2, 0.2, 0.2] },
      extensions: []
    });
    synchroniseFromModel({ historyLabel: "Add collision geometry" });
  });

  wireGeometryEditors(link);

  $("#link-mass").addEventListener("change", event => {
    if (!link.inertial) link.inertial = inertial;
    link.inertial.mass = Math.max(0, num(event.target.value));
    synchroniseFromModel();
  });

  wireVector("#inertial-xyz", values => {
    if (!link.inertial) link.inertial = inertial;
    link.inertial.origin.xyz = values;
    synchroniseFromModel();
  });

  wireVector("#inertial-rpy", values => {
    if (!link.inertial) link.inertial = inertial;
    setOriginRPY(link.inertial.origin, values);
    synchroniseFromModel();
  });

  $("#inertia-values").querySelectorAll("input").forEach(input => {
    input.addEventListener("change", () => {
      if (!link.inertial) link.inertial = inertial;
      link.inertial.inertia[input.dataset.key] = num(input.value);
      synchroniseFromModel({ rebuild: false });
    });
  });

  $("#auto-inertia").addEventListener("click", () => {
    const geometry = link.visuals?.[0]?.geometry;
    if (!geometry) return;
    if (!link.inertial) link.inertial = inertial;
    link.inertial.inertia = automaticInertia(geometry, link.inertial.mass);
    synchroniseFromModel({ rebuild: false });
    renderInspector();
  });
}

function isGlobalMaterialReference(material) {
  return Boolean(
    material?.name &&
    !material.color &&
    !material.texture &&
    !(material.extensions?.length) &&
    robot.materials?.has?.(material.name)
  );
}

function resolvedMaterialColour(material) {
  if (material?.color) return material.color;

  if (material?.name) {
    const globalMaterial = robot.materials?.get?.(material.name);
    if (globalMaterial?.color) return globalMaterial.color;
  }

  return [0.48, 0.78, 0.88, 1];
}

function editableMaterialTarget(item) {
  if (!item.material) {
    item.material = defaultMaterial();
    return item.material;
  }

  if (isGlobalMaterialReference(item.material)) {
    const globalMaterial = robot.materials?.get?.(item.material.name);
    if (globalMaterial) return globalMaterial;
  }

  return item.material;
}

function visualMaterialOptions(material) {
  let current = "__none__";
  if (material) {
    current = isGlobalMaterialReference(material)
      ? material.name
      : "__inline__";
  }

  const options = [
    `<option value="__none__" ${current === "__none__" ? "selected" : ""}>No material</option>`,
    `<option value="__inline__" ${current === "__inline__" ? "selected" : ""}>Inline colour</option>`
  ];

  for (const name of robot.materials?.keys?.() || []) {
    options.push(`<option value="${escapeHTML(name)}" ${current === name ? "selected" : ""}>Global · ${escapeHTML(name)}</option>`);
  }

  return options.join("");
}

function geometryEditor(kind, index, geometry, origin, material, itemName = "") {
  const geometryFields = geometryFieldsHTML(geometry, kind, index);
  const displayColour = resolvedMaterialColour(material);
  const materialHTML = kind === "visual" ? `
    ${field("Material", `<select data-geometry-material="${kind}:${index}">${visualMaterialOptions(material)}</select>`)}
    ${material ? field(material.color ? "Material RGBA" : `Resolved RGBA · ${escapeHTML(material.name || "reference")}`, `
      <div class="swatch-row">
        <input type="color" data-geometry-color="${kind}:${index}" value="${rgbaToHex(displayColour)}">
        <input type="number" data-geometry-alpha="${kind}:${index}" min="0" max="1" step="0.01" value="${displayColour?.[3] ?? 1}">
      </div>
    `) : ""}
  ` : "";

  return `
    <div class="geometry-card" data-geometry-card="${kind}:${index}">
      <div class="geometry-card-head">
        <span>${index + 1}. ${escapeHTML(geometry?.type || "unknown")}</span>
        <div class="geometry-card-actions">
          <button type="button" data-duplicate-geometry="${kind}:${index}">Duplicate</button>
          <button type="button" data-remove-geometry="${kind}:${index}">Remove</button>
        </div>
      </div>
      ${field("Name", `<input data-geometry-name="${kind}:${index}" value="${escapeHTML(itemName || "")}" placeholder="optional">`)}
      ${field("Type", `
        <select data-geometry-type="${kind}:${index}">
          ${["box", "cylinder", "sphere", ...(robot.version !== "1.0" ? ["capsule"] : []), "mesh"].map(type => `<option value="${type}" ${geometry?.type === type ? "selected" : ""}>${type}</option>`).join("")}
        </select>
      `)}
      ${geometryFields}
      ${field("Origin xyz (m)", `<div data-origin-xyz="${kind}:${index}">${vectorInput(origin?.xyz)}</div>`)}
      ${field("Origin rpy (rad)", `<div data-origin-rpy="${kind}:${index}">${vectorInput(origin?.rpy, ["R", "P", "Y"])}</div>`)}
      ${materialHTML}
    </div>
  `;
}

function geometryFieldsHTML(geometry, kind, index) {
  if (geometry?.type === "box") {
    return field("Size xyz (m)", `<div data-geometry-size="${kind}:${index}">${vectorInput(geometry.size)}</div>`);
  }

  if (geometry?.type === "cylinder") {
    return `
      <div class="inline-row">
        ${field("Radius (m)", `<input data-geometry-radius="${kind}:${index}" type="number" step="any" min="0" value="${geometry.radius}">`)}
        ${field("Length (m)", `<input data-geometry-length="${kind}:${index}" type="number" step="any" min="0" value="${geometry.length}">`)}
      </div>
    `;
  }

  if (geometry?.type === "sphere") {
    return field("Radius (m)", `<input data-geometry-radius="${kind}:${index}" type="number" step="any" min="0" value="${geometry.radius}">`);
  }

  if (geometry?.type === "capsule") {
    return `
      <div class="inline-row">
        ${field("Radius (m)", `<input data-geometry-radius="${kind}:${index}" type="number" step="any" min="0" value="${geometry.radius}">`)}
        ${field("Cylinder length (m)", `<input data-geometry-length="${kind}:${index}" type="number" step="any" min="0" value="${geometry.length}">`)}
      </div>
    `;
  }

  if (geometry?.type === "mesh") {
    return `
      ${field("Local mesh", `
        <div class="mesh-picker">
          <select data-geometry-asset="${kind}:${index}">${meshAssetOptions(geometry.filename)}</select>
          <button type="button" data-import-mesh="${kind}:${index}">Import…</button>
        </div>
      `)}
      ${field("URDF filename", `<input data-geometry-filename="${kind}:${index}" value="${escapeHTML(geometry.filename || "")}" placeholder="package://robot/meshes/link.stl">`)}
      ${field("Scale xyz", `<div data-geometry-scale="${kind}:${index}">${vectorInput(geometry.scale || [1, 1, 1])}</div>`)}
    `;
  }

  return "";
}

function wireGeometryEditors(link) {
  const kinds = [
    ["visual", link.visuals],
    ["collision", link.collisions]
  ];

  for (const [kind, collection] of kinds) {
    collection.forEach((item, index) => {
      const key = `${kind}:${index}`;
      const typeSelect = elements.inspector.querySelector(`[data-geometry-type="${CSS.escape(key)}"]`);

      typeSelect?.addEventListener("change", () => {
        const old = item.geometry;
        const type = typeSelect.value;
        if (type === "box") item.geometry = { type, size: old?.size || [0.2, 0.2, 0.2] };
        if (type === "cylinder") item.geometry = { type, radius: old?.radius || 0.1, length: old?.length || 0.3 };
        if (type === "sphere") item.geometry = { type, radius: old?.radius || 0.15 };
        if (type === "capsule") item.geometry = { type, radius: old?.radius || 0.1, length: old?.length || 0.3 };
        if (type === "mesh") item.geometry = { type, filename: "", scale: [1, 1, 1] };
        synchroniseFromModel();
      });

      const assetSelect = elements.inspector.querySelector(`[data-geometry-asset="${CSS.escape(key)}"]`);
      assetSelect?.addEventListener("change", () => {
        if (!assetSelect.value) return;
        item.geometry.filename = assetSelect.value;
        synchroniseFromModel();
      });

      elements.inspector.querySelector(`[data-import-mesh="${CSS.escape(key)}"]`)?.addEventListener("click", () => {
        elements.meshFiles.click();
      });

      wireVector(`[data-origin-xyz="${cssAttr(key)}"]`, values => {
        item.origin.xyz = values;
        synchroniseFromModel();
      });

      wireVector(`[data-origin-rpy="${cssAttr(key)}"]`, values => {
        setOriginRPY(item.origin, values);
        synchroniseFromModel();
      });

      wireVector(`[data-geometry-size="${cssAttr(key)}"]`, values => {
        item.geometry.size = values.map(v => Math.max(0.0001, v));
        synchroniseFromModel();
      });

      wireVector(`[data-geometry-scale="${cssAttr(key)}"]`, values => {
        item.geometry.scale = values;
        synchroniseFromModel();
      });

      for (const prop of ["radius", "length", "filename"]) {
        const input = elements.inspector.querySelector(`[data-geometry-${prop}="${CSS.escape(key)}"]`);
        input?.addEventListener("change", () => {
          item.geometry[prop] = prop === "filename" ? input.value : Math.max(0.0001, num(input.value));
          synchroniseFromModel();
        });
      }

      const nameInput = elements.inspector.querySelector(`[data-geometry-name="${CSS.escape(key)}"]`);
      nameInput?.addEventListener("change", () => {
        item.name = nameInput.value.trim();
        synchroniseFromModel({ historyLabel: `Rename ${kind}` });
      });

      const materialSelect = elements.inspector.querySelector(`[data-geometry-material="${CSS.escape(key)}"]`);
      materialSelect?.addEventListener("change", () => {
        if (materialSelect.value === "__none__") {
          item.material = null;
        } else if (materialSelect.value === "__inline__") {
          item.material = defaultMaterial();
        } else {
          item.material = {
            name: materialSelect.value,
            color: null,
            texture: "",
            attributes: {},
            extensions: []
          };
        }
        synchroniseFromModel({ historyLabel: "Change visual material" });
      });

      const color = elements.inspector.querySelector(`[data-geometry-color="${CSS.escape(key)}"]`);
      const alpha = elements.inspector.querySelector(`[data-geometry-alpha="${CSS.escape(key)}"]`);

      color?.addEventListener("change", () => {
        const target = editableMaterialTarget(item);
        const rgb = hexToRgb(color.value);
        const current = resolvedMaterialColour(item.material);
        target.color = [...rgb, num(alpha?.value, current?.[3] ?? 1)];
        synchroniseFromModel();
      });

      alpha?.addEventListener("change", () => {
        const target = editableMaterialTarget(item);
        const current = target.color || resolvedMaterialColour(item.material);
        target.color = [...current.slice(0, 3), clamp(num(alpha.value, 1), 0, 1)];
        synchroniseFromModel();
      });

      elements.inspector.querySelector(`[data-duplicate-geometry="${CSS.escape(key)}"]`)?.addEventListener("click", () => {
        collection.splice(index + 1, 0, structuredClone(item));
        synchroniseFromModel({ historyLabel: `Duplicate ${kind} geometry` });
      });

      elements.inspector.querySelector(`[data-remove-geometry="${CSS.escape(key)}"]`)?.addEventListener("click", () => {
        collection.splice(index, 1);
        synchroniseFromModel({ historyLabel: `Remove ${kind} geometry` });
      });
    });
  }
}

function cssAttr(value) {
  return CSS.escape(value);
}

function renderJointInspector(joint) {
  if (!joint) {
    select(null);
    return;
  }

  elements.inspectorTitle.textContent = joint.name;
  elements.deleteSelected.hidden = false;

  const limit = joint.limit || { lower: 0, upper: 0, effort: 0, velocity: 0 };
  const dynamics = joint.dynamics;
  const mimic = joint.mimic;
  const mimicTargets = [...robot.joints.keys()]
    .filter(name => name !== joint.name)
    .map(name => `<option value="${escapeHTML(name)}" ${mimic?.joint === name ? "selected" : ""}>${escapeHTML(name)}</option>`)
    .join("");

  elements.inspector.innerHTML =
    section("Joint", `
      ${field("Name", `<input id="joint-name" value="${escapeHTML(joint.name)}">`)}
      ${field("Type", `
        <select id="joint-type">
          ${["fixed", "revolute", "continuous", "prismatic", "planar", "floating"].map(type => `<option value="${type}" ${joint.type === type ? "selected" : ""}>${type}</option>`).join("")}
        </select>
      `)}
      ${field("Parent link", `<select id="joint-parent">${linkOptions(joint.parent)}</select>`)}
      ${field("Child link", `<select id="joint-child">${linkOptions(joint.child)}</select>`)}
    `) +
    section("Joint frame", `
      ${field("Origin xyz (m)", `<div id="joint-xyz">${vectorInput(joint.origin.xyz)}</div>`)}
      ${field("Origin rpy (rad)", `<div id="joint-rpy">${vectorInput(joint.origin.rpy, ["R", "P", "Y"])}</div>`)}
      ${field("Axis xyz", `<div id="joint-axis">${vectorInput(joint.axis)}</div>`)}
    `) +
    section("Limits", `
      <div class="inline-row">
        ${field("Lower", `<input id="joint-lower" type="number" step="any" value="${limit.lower ?? ""}">`)}
        ${field("Upper", `<input id="joint-upper" type="number" step="any" value="${limit.upper ?? ""}">`)}
      </div>
      <div class="inline-row">
        ${field("Effort", `<input id="joint-effort" type="number" step="any" value="${limit.effort ?? ""}">`)}
        ${field("Velocity", `<input id="joint-velocity" type="number" step="any" value="${limit.velocity ?? ""}">`)}
      </div>
      ${robot.version === "1.2" ? `
        <div class="inline-row">
          ${field("Acceleration", `<input id="joint-acceleration" type="number" min="0" step="any" value="${limit.acceleration ?? ""}">`)}
          ${field("Deceleration", `<input id="joint-deceleration" type="number" min="0" step="any" value="${limit.deceleration ?? ""}">`)}
        </div>
        ${field("Jerk", `<input id="joint-jerk" type="number" min="0" step="any" value="${limit.jerk ?? ""}">`)}
      ` : ""}
    `) +
    section("Dynamics", `
      <label class="checkbox-row inspector-check">
        <span>Enable dynamics</span>
        <input id="joint-dynamics-enabled" type="checkbox" ${dynamics ? "checked" : ""}>
      </label>
      <div id="joint-dynamics-fields" ${dynamics ? "" : "hidden"}>
        <div class="inline-row">
          ${field("Damping", `<input id="joint-damping" type="number" step="any" value="${dynamics?.damping ?? ""}" placeholder="optional">`)}
          ${field("Friction", `<input id="joint-friction" type="number" step="any" value="${dynamics?.friction ?? ""}" placeholder="optional">`)}
        </div>
      </div>
    `) +
    section("Mimic", `
      <label class="checkbox-row inspector-check">
        <span>Enable mimic</span>
        <input id="joint-mimic-enabled" type="checkbox" ${mimic ? "checked" : ""} ${robot.joints.size < 2 ? "disabled" : ""}>
      </label>
      <div id="joint-mimic-fields" ${mimic ? "" : "hidden"}>
        ${field("Target joint", `<select id="joint-mimic-target"><option value="">Choose joint…</option>${mimicTargets}</select>`)}
        <div class="inline-row">
          ${field("Multiplier", `<input id="joint-mimic-multiplier" type="number" step="any" value="${mimic?.multiplier ?? ""}" placeholder="default 1">`)}
          ${field("Offset", `<input id="joint-mimic-offset" type="number" step="any" value="${mimic?.offset ?? ""}" placeholder="default 0">`)}
        </div>
      </div>
    `);

  $("#joint-name").addEventListener("change", event => renameJoint(joint.name, event.target.value.trim()));

  $("#joint-type").addEventListener("change", event => {
    joint.type = event.target.value;
    joint.value = 0;
    if (joint.type === "continuous") {
      joint.limit = { lower: null, upper: null, effort: limit.effort ?? 20, velocity: limit.velocity ?? 2, acceleration: null, deceleration: null, jerk: null };
    } else if (["fixed", "floating", "planar"].includes(joint.type)) {
      joint.limit = null;
    } else if (!joint.limit || joint.limit.lower == null || joint.limit.upper == null) {
      joint.limit = { lower: -1.5708, upper: 1.5708, effort: 20, velocity: 2, acceleration: null, deceleration: null, jerk: null };
    }
    synchroniseFromModel({ historyLabel: "Change joint type" });
  });

  $("#joint-parent").addEventListener("change", event => {
    joint.parent = event.target.value;
    synchroniseFromModel({ historyLabel: "Change joint parent" });
  });

  $("#joint-child").addEventListener("change", event => {
    joint.child = event.target.value;
    synchroniseFromModel({ historyLabel: "Change joint child" });
  });

  wireVector("#joint-xyz", values => {
    joint.origin.xyz = values;
    synchroniseFromModel({ historyLabel: "Edit joint origin" });
  });

  wireVector("#joint-rpy", values => {
    setOriginRPY(joint.origin, values);
    synchroniseFromModel({ historyLabel: "Edit joint rotation" });
  });

  wireVector("#joint-axis", values => {
    joint.axis = values;
    synchroniseFromModel({ historyLabel: "Edit joint axis" });
  });

  for (const [id, key] of [
    ["#joint-lower", "lower"],
    ["#joint-upper", "upper"],
    ["#joint-effort", "effort"],
    ["#joint-velocity", "velocity"],
    ["#joint-acceleration", "acceleration"],
    ["#joint-deceleration", "deceleration"],
    ["#joint-jerk", "jerk"]
  ]) {
    const control = $(id);
    if (!control) continue;
    control.addEventListener("change", event => {
      if (!joint.limit) joint.limit = {};
      joint.limit[key] = event.target.value === "" ? null : num(event.target.value);
      synchroniseFromModel({ historyLabel: "Edit joint limits" });
    });
  }

  $("#joint-dynamics-enabled")?.addEventListener("change", event => {
    joint.dynamics = event.target.checked
      ? { damping: null, friction: null, attributes: {}, extensions: [] }
      : null;
    synchroniseFromModel({ historyLabel: event.target.checked ? "Add joint dynamics" : "Remove joint dynamics" });
  });

  for (const [id, key] of [["#joint-damping", "damping"], ["#joint-friction", "friction"]]) {
    $(id)?.addEventListener("change", event => {
      if (!joint.dynamics) joint.dynamics = { damping: null, friction: null, attributes: {}, extensions: [] };
      joint.dynamics[key] = event.target.value === "" ? null : num(event.target.value);
      synchroniseFromModel({ historyLabel: "Edit joint dynamics" });
    });
  }

  $("#joint-mimic-enabled")?.addEventListener("change", event => {
    const firstTarget = [...robot.joints.keys()].find(name => name !== joint.name) || "";
    joint.mimic = event.target.checked
      ? { joint: firstTarget, multiplier: null, offset: null, attributes: {}, extensions: [] }
      : null;
    synchroniseFromModel({ historyLabel: event.target.checked ? "Add joint mimic" : "Remove joint mimic" });
  });

  $("#joint-mimic-target")?.addEventListener("change", event => {
    if (!joint.mimic) joint.mimic = { joint: "", multiplier: null, offset: null, attributes: {}, extensions: [] };
    joint.mimic.joint = event.target.value;
    synchroniseFromModel({ historyLabel: "Change mimic target" });
  });

  for (const [id, key] of [["#joint-mimic-multiplier", "multiplier"], ["#joint-mimic-offset", "offset"]]) {
    $(id)?.addEventListener("change", event => {
      if (!joint.mimic) return;
      joint.mimic[key] = event.target.value === "" ? null : num(event.target.value);
      synchroniseFromModel({ historyLabel: "Edit mimic parameters" });
    });
  }
}

function linkOptions(selected) {
  return [...robot.links.keys()].map(name =>
    `<option value="${escapeHTML(name)}" ${name === selected ? "selected" : ""}>${escapeHTML(name)}</option>`
  ).join("");
}

function wireVector(selector, callback) {
  const root = elements.inspector.querySelector(selector);
  if (!root) return;

  root.querySelectorAll("input").forEach(input => {
    input.addEventListener("change", () => {
      const values = [...root.querySelectorAll("input")].map(field => num(field.value));
      callback(values);
    });
  });
}


function materialReferenceCount(name) {
  let count = 0;
  for (const link of robot.links.values()) {
    for (const visual of link.visuals || []) {
      if (visual.material?.name === name && isGlobalMaterialReference(visual.material)) count += 1;
    }
  }
  return count;
}

function textureAssetOptions(current = "") {
  const currentBase = meshBaseName(current);
  const assets = allAssets().filter(isTextureAsset);
  const currentLoaded = assets.some(file => file.name === current || file.name === currentBase);
  let html = `<option value="">No local texture selected</option>`;
  if (current && !currentLoaded) html += `<option value="${escapeHTML(current)}" selected>${escapeHTML(current)} · not loaded</option>`;
  html += assets.map(file => {
    const selected = file.name === current || file.name === currentBase ? " selected" : "";
    return `<option value="${escapeHTML(file.name)}"${selected}>${escapeHTML(file.name)}${isBundledAsset(file) ? " · bundled" : ""}</option>`;
  }).join("");
  return html;
}

function renderMaterials() {
  if (!elements.materialsEditor) return;
  const materials = [...(robot.materials?.values?.() || [])];
  const referenceTotal = materials.reduce((sum, material) => sum + materialReferenceCount(material.name), 0);
  elements.materialsMeta.textContent = `${materials.length} ${materials.length === 1 ? "material" : "materials"} · ${referenceTotal} ${referenceTotal === 1 ? "reference" : "references"}`;

  if (!materials.length) {
    elements.materialsEditor.innerHTML = `<div class="empty-state">No global materials. Add one here, or use inline visual colours in the Inspector.</div>`;
    return;
  }

  elements.materialsEditor.innerHTML = materials.map((material, index) => {
    const refs = materialReferenceCount(material.name);
    const colour = material.color || [1, 1, 1, 1];
    return `
      <article class="material-card" data-material-index="${index}">
        <div class="material-card-head">
          <div>
            <span class="eyebrow">Global material</span>
            <strong>${escapeHTML(material.name)}</strong>
          </div>
          <span class="material-ref-count">${refs} ${refs === 1 ? "reference" : "references"}</span>
        </div>
        <div class="material-grid">
          ${field("Name", `<input data-material-name="${index}" value="${escapeHTML(material.name)}">`)}
          ${field("Colour", `
            <label class="material-colour-control">
              <input data-material-use-colour="${index}" type="checkbox" ${material.color ? "checked" : ""}>
              <input data-material-colour="${index}" type="color" value="${rgbaToHex(colour)}" ${material.color ? "" : "disabled"}>
              <input data-material-alpha="${index}" type="number" min="0" max="1" step="0.01" value="${colour[3] ?? 1}" ${material.color ? "" : "disabled"}>
            </label>
          `)}
        </div>
        ${field("Texture filename", `<input data-material-texture="${index}" value="${escapeHTML(material.texture || "")}" placeholder="package://robot/textures/material.tif">`)}
        ${field("Local texture", `<select data-material-texture-asset="${index}">${textureAssetOptions(material.texture || "")}</select>`)}
        <div class="material-card-actions">
          <button type="button" data-material-remove="${index}" ${refs ? "disabled" : ""} title="${refs ? "Remove references before deleting this material" : "Delete global material"}">Remove material</button>
        </div>
      </article>
    `;
  }).join("");

  materials.forEach((material, index) => {
    elements.materialsEditor.querySelector(`[data-material-name="${index}"]`)?.addEventListener("change", event => {
      renameGlobalMaterial(material.name, event.target.value.trim());
    });

    elements.materialsEditor.querySelector(`[data-material-use-colour="${index}"]`)?.addEventListener("change", event => {
      material.color = event.target.checked ? [1, 1, 1, 1] : null;
      synchroniseFromModel({ historyLabel: event.target.checked ? "Add material colour" : "Remove material colour" });
    });

    const colourInput = elements.materialsEditor.querySelector(`[data-material-colour="${index}"]`);
    const alphaInput = elements.materialsEditor.querySelector(`[data-material-alpha="${index}"]`);

    colourInput?.addEventListener("change", () => {
      const current = material.color || [1, 1, 1, 1];
      material.color = [...hexToRgb(colourInput.value), clamp(num(alphaInput?.value, current[3]), 0, 1)];
      synchroniseFromModel({ historyLabel: "Edit material colour" });
    });

    alphaInput?.addEventListener("change", () => {
      const current = material.color || [1, 1, 1, 1];
      material.color = [...current.slice(0, 3), clamp(num(alphaInput.value, current[3]), 0, 1)];
      synchroniseFromModel({ historyLabel: "Edit material alpha" });
    });

    elements.materialsEditor.querySelector(`[data-material-texture="${index}"]`)?.addEventListener("change", event => {
      material.texture = event.target.value.trim();
      synchroniseFromModel({ historyLabel: "Edit material texture" });
    });

    elements.materialsEditor.querySelector(`[data-material-texture-asset="${index}"]`)?.addEventListener("change", event => {
      material.texture = event.target.value;
      synchroniseFromModel({ historyLabel: event.target.value ? "Choose material texture" : "Clear material texture" });
    });

    elements.materialsEditor.querySelector(`[data-material-remove="${index}"]`)?.addEventListener("click", () => {
      const refs = materialReferenceCount(material.name);
      if (refs) {
        setXMLStatus(`Material "${material.name}" still has ${refs} visual reference${refs === 1 ? "" : "s"}.`, "error");
        return;
      }
      robot.materials.delete(material.name);
      synchroniseFromModel({ historyLabel: "Remove global material" });
    });
  });
}

function renameGlobalMaterial(oldName, requested) {
  if (!requested || requested === oldName) return;
  const next = uniqueName(requested, new Set([...robot.materials.keys()].filter(name => name !== oldName)));
  const material = robot.materials.get(oldName);
  if (!material) return;

  for (const link of robot.links.values()) {
    for (const visual of link.visuals || []) {
      if (visual.material?.name === oldName && isGlobalMaterialReference(visual.material)) visual.material.name = next;
    }
  }

  robot.materials.delete(oldName);
  material.name = next;
  robot.materials.set(next, material);

  synchroniseFromModel({ historyLabel: "Rename global material" });
}

function addGlobalMaterial() {
  const name = uniqueName("Material", new Set(robot.materials.keys()));
  robot.materials.set(name, {
    name,
    color: [0.7, 0.7, 0.7, 1],
    texture: "",
    attributes: {},
    extensions: []
  });
  workspaceLayout.setBottomTab("materials");
  synchroniseFromModel({ historyLabel: "Add global material" });
}

function ros2ParamRows(params, controlIndex, scope, componentIndex = -1, interfaceIndex = -1) {
  return (params || []).map((param, paramIndex) => `
    <div class="ros2-param-row">
      <input data-ros2-param-name="${controlIndex}:${scope}:${componentIndex}:${interfaceIndex}:${paramIndex}" value="${escapeHTML(param.name || "")}" placeholder="name">
      <input data-ros2-param-value="${controlIndex}:${scope}:${componentIndex}:${interfaceIndex}:${paramIndex}" value="${escapeHTML(param.value || "")}" placeholder="value">
      <button class="ros2-remove" type="button" data-ros2-remove-param="${controlIndex}:${scope}:${componentIndex}:${interfaceIndex}:${paramIndex}" title="Remove parameter">×</button>
    </div>
  `).join("");
}

function ros2InterfaceRow(item, controlIndex, componentIndex, interfaceIndex) {
  const getParam = name => item.params?.find(param => param.name === name)?.value ?? "";
  return `
    <div class="ros2-interface-row">
      <select data-ros2-interface-kind="${controlIndex}:${componentIndex}:${interfaceIndex}">
        <option value="command" ${item.kind === "command" ? "selected" : ""}>command</option>
        <option value="state" ${item.kind === "state" ? "selected" : ""}>state</option>
      </select>
      <input data-ros2-interface-name="${controlIndex}:${componentIndex}:${interfaceIndex}" value="${escapeHTML(item.name || "")}" placeholder="interface">
      <input data-ros2-interface-type="${controlIndex}:${componentIndex}:${interfaceIndex}" value="${escapeHTML(item.dataType || "double")}" placeholder="data type">
      <input data-ros2-interface-size="${controlIndex}:${componentIndex}:${interfaceIndex}" type="number" min="1" step="1" value="${Math.max(1, Number(item.size) || 1)}" title="size">
      <input data-ros2-interface-param="${controlIndex}:${componentIndex}:${interfaceIndex}:min" value="${escapeHTML(getParam("min"))}" placeholder="min">
      <input data-ros2-interface-param="${controlIndex}:${componentIndex}:${interfaceIndex}:max" value="${escapeHTML(getParam("max"))}" placeholder="max">
      <input data-ros2-interface-param="${controlIndex}:${componentIndex}:${interfaceIndex}:initial_value" value="${escapeHTML(getParam("initial_value"))}" placeholder="initial">
      <button class="ros2-remove" type="button" data-ros2-remove-interface="${controlIndex}:${componentIndex}:${interfaceIndex}" title="Remove interface">×</button>
    </div>
  `;
}

function ros2ComponentCard(component, controlIndex, componentIndex) {
  const jointNameControl = component.kind === "joint"
    ? `<select data-ros2-component-name="${controlIndex}:${componentIndex}">${[...robot.joints.keys()].map(name => `<option value="${escapeHTML(name)}" ${component.name === name ? "selected" : ""}>${escapeHTML(name)}</option>`).join("")}</select>`
    : `<input data-ros2-component-name="${controlIndex}:${componentIndex}" value="${escapeHTML(component.name || "")}" placeholder="name">`;
  return `
    <div class="ros2-component">
      <div class="ros2-component-head">
        <select data-ros2-component-kind="${controlIndex}:${componentIndex}">
          ${["joint", "sensor", "gpio"].map(kind => `<option value="${kind}" ${component.kind === kind ? "selected" : ""}>${kind}</option>`).join("")}
        </select>
        ${jointNameControl}
        <button class="ros2-remove" type="button" data-ros2-remove-component="${controlIndex}:${componentIndex}" title="Remove component">×</button>
      </div>
      <div class="ros2-interface-list">
        ${(component.interfaces || []).map((item, interfaceIndex) => ros2InterfaceRow(item, controlIndex, componentIndex, interfaceIndex)).join("") || `<div class="ros2-preserved">No interfaces.</div>`}
      </div>
      <div class="mini-actions">
        <button class="ros2-mini-button" type="button" data-ros2-add-interface="${controlIndex}:${componentIndex}:command">+ command</button>
        <button class="ros2-mini-button" type="button" data-ros2-add-interface="${controlIndex}:${componentIndex}:state">+ state</button>
      </div>
      ${(component.extensions?.length || component.params?.length) ? `<div class="ros2-preserved">${component.params?.length || 0} component parameter(s), ${component.extensions?.length || 0} extension tag(s) preserved in XML.</div>` : ""}
    </div>
  `;
}

function renderROS2Control() {
  if (!elements.ros2Editor) return;
  const controls = robot.ros2Controls || [];
  if (!controls.length) {
    elements.ros2Editor.innerHTML = `<div class="ros2-empty">No <code>&lt;ros2_control&gt;</code> blocks. Add one here or paste one into the URDF XML.</div>`;
    return;
  }

  elements.ros2Editor.innerHTML = controls.map((control, controlIndex) => `
    <article class="ros2-card">
      <header class="ros2-card-head">
        <div class="ros2-card-title"><strong>${escapeHTML(control.name || "Unnamed hardware")}</strong><span class="ros2-badge">${escapeHTML(control.type || "system")}</span></div>
        <button class="danger ghost" type="button" data-ros2-remove-control="${controlIndex}">Delete</button>
      </header>
      <div class="ros2-card-body">
        <div class="ros2-grid">
          <label class="ros2-field"><span>Name</span><input data-ros2-control-name="${controlIndex}" value="${escapeHTML(control.name || "")}"></label>
          <label class="ros2-field"><span>Type</span><select data-ros2-control-type="${controlIndex}">${["system", "actuator", "sensor"].map(type => `<option value="${type}" ${control.type === type ? "selected" : ""}>${type}</option>`).join("")}</select></label>
          <label class="ros2-field"><span>rw_rate (Hz)</span><input data-ros2-control-rate="${controlIndex}" type="number" min="0" step="1" value="${escapeHTML(control.rwRate ?? "")}" placeholder="default"></label>
          <label class="ros2-field"><span>Async</span><select data-ros2-control-async="${controlIndex}"><option value="false" ${!control.isAsync ? "selected" : ""}>false</option><option value="true" ${control.isAsync ? "selected" : ""}>true</option></select></label>
        </div>
        <div class="ros2-grid two">
          <label class="ros2-field"><span>Hardware plugin</span><input data-ros2-control-plugin="${controlIndex}" value="${escapeHTML(control.plugin || "")}" placeholder="package/Class"></label>
          <label class="ros2-field"><span>Hardware group</span><input data-ros2-control-group="${controlIndex}" value="${escapeHTML(control.group || "")}" placeholder="optional"></label>
        </div>
        <section class="ros2-section">
          <div class="ros2-section-head"><span>Hardware parameters</span><button class="ros2-mini-button" type="button" data-ros2-add-param="${controlIndex}:hardware">+ param</button></div>
          <div class="ros2-param-list">${ros2ParamRows(control.hardwareParams, controlIndex, "hardware") || `<div class="ros2-preserved">No hardware parameters.</div>`}</div>
        </section>
        <section class="ros2-section">
          <div class="ros2-section-head"><span>Components</span><div class="mini-actions"><button class="ros2-mini-button" type="button" data-ros2-add-component="${controlIndex}:joint">+ joint</button><button class="ros2-mini-button" type="button" data-ros2-add-component="${controlIndex}:sensor">+ sensor</button><button class="ros2-mini-button" type="button" data-ros2-add-component="${controlIndex}:gpio">+ GPIO</button></div></div>
          <div class="ros2-interface-list">${(control.components || []).map((component, componentIndex) => ros2ComponentCard(component, controlIndex, componentIndex)).join("") || `<div class="ros2-preserved">No components.</div>`}</div>
        </section>
        ${(control.extensions?.length || control.hardwareExtensions?.length) ? `<div class="ros2-preserved">${(control.extensions?.length || 0) + (control.hardwareExtensions?.length || 0)} unrecognised/advanced tag(s) are preserved verbatim on export.</div>` : ""}
      </div>
    </article>
  `).join("");

  wireROS2Control();
}

function setNamedParam(params, name, value) {
  const index = params.findIndex(param => param.name === name);
  if (value === "") {
    if (index >= 0) params.splice(index, 1);
    return;
  }
  if (index >= 0) params[index].value = value;
  else params.push({ name, value });
}

function syncROS2Only({ recordHistory = true, historyLabel = "Edit ROS 2 Control" } = {}) {
  elements.xmlEditor.value = serialiseURDF(robot);
  sourceDirty = false;
  setXMLStatus("Model and source are synchronised.");
  renderDiagnostics();
  if (recordHistory && !applyingHistory) history.record(robot, selection, historyLabel);
}

function wireROS2Control() {
  const controls = robot.ros2Controls || [];
  elements.ros2Editor.querySelectorAll("[data-ros2-control-name]").forEach(input => input.addEventListener("change", () => { controls[Number(input.dataset.ros2ControlName)].name = input.value.trim(); syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-control-type]").forEach(input => input.addEventListener("change", () => { controls[Number(input.dataset.ros2ControlType)].type = input.value; syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-control-rate]").forEach(input => input.addEventListener("change", () => { controls[Number(input.dataset.ros2ControlRate)].rwRate = input.value; syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-control-async]").forEach(input => input.addEventListener("change", () => { controls[Number(input.dataset.ros2ControlAsync)].isAsync = input.value === "true"; syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-control-plugin]").forEach(input => {
    input.addEventListener("input", () => {
      controls[Number(input.dataset.ros2ControlPlugin)].plugin = input.value.trim();
      syncROS2Only({ recordHistory: false });
    });
    input.addEventListener("change", () => syncROS2Only({ historyLabel: "Edit ROS 2 hardware plugin" }));
  });
  elements.ros2Editor.querySelectorAll("[data-ros2-control-group]").forEach(input => input.addEventListener("change", () => { controls[Number(input.dataset.ros2ControlGroup)].group = input.value.trim(); syncROS2Only(); }));

  elements.ros2Editor.querySelectorAll("[data-ros2-remove-control]").forEach(button => button.addEventListener("click", () => { controls.splice(Number(button.dataset.ros2RemoveControl), 1); syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-add-param]").forEach(button => button.addEventListener("click", () => { const [ci] = button.dataset.ros2AddParam.split(":").map(Number); controls[ci].hardwareParams.push({ name: "param", value: "" }); syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-param-name]").forEach(input => input.addEventListener("change", () => { const [ci,, , , pi] = input.dataset.ros2ParamName.split(":").map((v,i)=>i===1?v:Number(v)); controls[ci].hardwareParams[pi].name = input.value.trim(); syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-param-value]").forEach(input => input.addEventListener("change", () => { const [ci,, , , pi] = input.dataset.ros2ParamValue.split(":").map((v,i)=>i===1?v:Number(v)); controls[ci].hardwareParams[pi].value = input.value; syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-remove-param]").forEach(button => button.addEventListener("click", () => { const [ci,, , , pi] = button.dataset.ros2RemoveParam.split(":").map((v,i)=>i===1?v:Number(v)); controls[ci].hardwareParams.splice(pi,1); syncROS2Only(); renderROS2Control(); }));

  elements.ros2Editor.querySelectorAll("[data-ros2-add-component]").forEach(button => button.addEventListener("click", () => { const [ciText, kind] = button.dataset.ros2AddComponent.split(":"); const ci=Number(ciText); const name = kind === "joint" ? ([...robot.joints.keys()][0] || "joint") : kind === "sensor" ? "sensor" : "gpio"; controls[ci].components.push(createROS2Component(kind,name)); syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-remove-component]").forEach(button => button.addEventListener("click", () => { const [ci,coi]=button.dataset.ros2RemoveComponent.split(":").map(Number); controls[ci].components.splice(coi,1); syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-component-kind]").forEach(input => input.addEventListener("change", () => { const [ci,coi]=input.dataset.ros2ComponentKind.split(":").map(Number); const component=controls[ci].components[coi]; component.kind=input.value; if (component.kind === "joint" && !robot.joints.has(component.name)) component.name=[...robot.joints.keys()][0]||""; syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-component-name]").forEach(input => input.addEventListener("change", () => { const [ci,coi]=input.dataset.ros2ComponentName.split(":").map(Number); controls[ci].components[coi].name=input.value.trim(); syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-add-interface]").forEach(button => button.addEventListener("click", () => { const [ci,coi,kind]=button.dataset.ros2AddInterface.split(":"); controls[Number(ci)].components[Number(coi)].interfaces.push(createROS2Interface(kind, kind === "command" ? "position" : "position")); syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-remove-interface]").forEach(button => button.addEventListener("click", () => { const [ci,coi,ii]=button.dataset.ros2RemoveInterface.split(":").map(Number); controls[ci].components[coi].interfaces.splice(ii,1); syncROS2Only(); renderROS2Control(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-interface-kind]").forEach(input => input.addEventListener("change", () => { const [ci,coi,ii]=input.dataset.ros2InterfaceKind.split(":").map(Number); controls[ci].components[coi].interfaces[ii].kind=input.value; syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-interface-name]").forEach(input => input.addEventListener("change", () => { const [ci,coi,ii]=input.dataset.ros2InterfaceName.split(":").map(Number); controls[ci].components[coi].interfaces[ii].name=input.value.trim(); syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-interface-type]").forEach(input => input.addEventListener("change", () => { const [ci,coi,ii]=input.dataset.ros2InterfaceType.split(":").map(Number); controls[ci].components[coi].interfaces[ii].dataType=input.value.trim()||"double"; syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-interface-size]").forEach(input => input.addEventListener("change", () => { const [ci,coi,ii]=input.dataset.ros2InterfaceSize.split(":").map(Number); controls[ci].components[coi].interfaces[ii].size=Math.max(1,Math.round(num(input.value,1))); syncROS2Only(); }));
  elements.ros2Editor.querySelectorAll("[data-ros2-interface-param]").forEach(input => input.addEventListener("change", () => { const [ci,coi,ii,name]=input.dataset.ros2InterfaceParam.split(":"); setNamedParam(controls[Number(ci)].components[Number(coi)].interfaces[Number(ii)].params,name,input.value); syncROS2Only(); }));
}

function requestURDFVersionChange(targetVersion) {
  if (!SUPPORTED_URDF_VERSIONS.includes(targetVersion)) return;
  if (targetVersion === robot.version) return;
  const report = analyseVersionConversion(robot, targetVersion);
  pendingURDFVersion = targetVersion;
  elements.urdfVersionDialogTitle.textContent = `URDF ${report.current} → ${targetVersion}`;
  elements.urdfVersionSummary.innerHTML = [
    ...report.notes.map(text => `<div class="version-conversion-item info"><span>i</span><span>${escapeHTML(text)}</span></div>`),
    ...report.warnings.map(text => `<div class="version-conversion-item warning"><span>!</span><span>${escapeHTML(text)}</span></div>`)
  ].join("");
  elements.urdfVersionDialog.showModal();
}

function renderDiagnostics() {
  let issues = validateRobot(robot);
  if (sourceApplyIssue) {
    issues = issues.filter(item => item.level !== "ok");
    issues.unshift(sourceApplyIssue);
  }

  const errorCount = issues.filter(item => item.level === "error").length;
  const warningCount = issues.filter(item => item.level === "warning").length;

  elements.doctorErrorCount.textContent = String(errorCount);
  elements.doctorErrorCount.hidden = errorCount === 0;
  elements.doctorWarningCount.textContent = String(warningCount);
  elements.doctorWarningCount.hidden = warningCount === 0;

  const doctorParts = ["Doctor"];
  if (errorCount) doctorParts.push(`${errorCount} ${errorCount === 1 ? "error" : "errors"}`);
  if (warningCount) doctorParts.push(`${warningCount} ${warningCount === 1 ? "warning" : "warnings"}`);
  elements.doctorTab.setAttribute("aria-label", doctorParts.join(", "));
  elements.doctorTab.title = doctorParts.join(" · ");

  elements.diagnostics.innerHTML = issues.map(item => `
    <div class="diagnostic ${item.level}" ${item.target ? `data-diag-type="${item.target.type}" data-diag-name="${escapeHTML(item.target.name)}"` : ""}>
      <span class="diagnostic-icon">${item.level === "error" ? "×" : item.level === "warning" ? "!" : item.level === "ok" ? "✓" : "i"}</span>
      <span>${escapeHTML(item.message)}</span>
    </div>
  `).join("");

  const problemCount = issues.filter(i => i.level === "error" || i.level === "warning").length;
  if (problemCount) {
    elements.validationSummary.textContent = `${problemCount} structural ${problemCount === 1 ? "issue" : "issues"} detected`;
    elements.validationSummary.classList.add("visible");
  } else {
    elements.validationSummary.classList.remove("visible");
  }

  elements.diagnostics.querySelectorAll("[data-diag-type]").forEach(item => {
    item.style.cursor = "pointer";
    item.addEventListener("click", () => select({
      type: item.dataset.diagType,
      name: item.dataset.diagName
    }));
  });
}

function select(next) {
  const wasPreviewing = Boolean(renderer.previewingAsset);
  renderer.clearAssetPreview();
  if (wasPreviewing) setXMLStatus("Model and source are synchronised.");
  selection = next;
  renderTree();
  renderMeshLibrary();
  renderInspector();
  renderer.setSelection(selection);
}

async function synchroniseFromModel({ rebuild = true, historyLabel = "Edit robot", recordHistory = true } = {}) {
  elements.urdfVersion.value = robot.version || "1.0";
  elements.xmlEditor.value = serialiseURDF(robot);
  sourceDirty = false;
  sourceApplyIssue = null;
  setXMLStatus("Model and source are synchronised.");
  renderAll();
  if (recordHistory && !applyingHistory) history.record(robot, selection, historyLabel);

  if (rebuild) {
    await renderer.setRobot(robot, true);
  }

  renderer.setSelection(selection);
}

async function applyHistoryEntry(entry, direction) {
  if (!entry) return;
  applyingHistory = true;
  try {
    robot = entry.robot;
    selection = entry.selection;
    sourceDirty = false;
    sourceApplyIssue = null;
    elements.urdfVersion.value = robot.version || "1.0";
    elements.xmlEditor.value = serialiseURDF(robot);
    setXMLStatus(`${direction}: ${entry.label}.`);
    renderAll();
    await renderer.setRobot(robot, true);
    renderer.setSelection(selection);
  } finally {
    applyingHistory = false;
  }
}

async function undoRobotEdit() {
  await applyHistoryEntry(history.undo(), "Undo");
}

async function redoRobotEdit() {
  await applyHistoryEntry(history.redo(), "Redo");
}

function setXMLStatus(message, state = "") {
  elements.xmlStatus.textContent = message;
  elements.xmlStatus.className = `xml-status${state ? ` ${state}` : ""}`;
}

function loadViewportSettings() {
  try {
    const stored = JSON.parse(localStorage.getItem("urdf-studio-viewport") || "null");
    const merged = { ...DEFAULT_VIEWPORT_SETTINGS, ...(stored || {}) };
    if (stored?.gridCentre?.toLowerCase() === "#41505d") merged.gridCentre = DEFAULT_VIEWPORT_SETTINGS.gridCentre;
    return merged;
  } catch {
    return { ...DEFAULT_VIEWPORT_SETTINGS };
  }
}

function saveViewportSettings() {
  try {
    localStorage.setItem("urdf-studio-viewport", JSON.stringify(viewportSettings));
  } catch {
  }
}

function syncViewportSettingsControls() {
  elements.viewportBackground.value = viewportSettings.background;
  elements.viewportFloorVisible.checked = viewportSettings.floorVisible;
  elements.viewportFloorColour.value = viewportSettings.floorColour;
  elements.viewportGridVisible.checked = viewportSettings.gridVisible;
  elements.viewportGridCentre.value = viewportSettings.gridCentre;
  elements.viewportGridLines.value = viewportSettings.gridLines;
  elements.viewportGridSize.value = viewportSettings.gridSize;
  elements.viewportGridDivisions.value = viewportSettings.gridDivisions;
  elements.viewportGridOpacity.value = viewportSettings.gridOpacity;
  elements.viewportWorldAxes.checked = viewportSettings.worldAxes;
  elements.viewportShadows.checked = viewportSettings.shadows;
}

function typedNumber(input) {
  const raw = input.value.trim();
  if (!raw || input.validity.badInput) return null;

  const value = Number(raw);
  return Number.isFinite(value) ? value : null;
}

function applyViewportSettings({ save = true, commitNumbers = false } = {}) {
  const typedGridSize = typedNumber(elements.viewportGridSize);
  const typedGridDivisions = typedNumber(elements.viewportGridDivisions);

  const gridSize = typedGridSize == null
    ? viewportSettings.gridSize
    : Math.max(0.1, typedGridSize);

  const gridDivisions = typedGridDivisions == null
    ? viewportSettings.gridDivisions
    : Math.max(1, Math.round(typedGridDivisions));

  viewportSettings = {
    background: elements.viewportBackground.value,
    floorVisible: elements.viewportFloorVisible.checked,
    floorColour: elements.viewportFloorColour.value,
    gridVisible: elements.viewportGridVisible.checked,
    gridCentre: elements.viewportGridCentre.value,
    gridLines: elements.viewportGridLines.value,
    gridSize,
    gridDivisions,
    gridOpacity: clamp(num(elements.viewportGridOpacity.value, viewportSettings.gridOpacity ?? 0.48), 0, 1),
    worldAxes: elements.viewportWorldAxes.checked,
    shadows: elements.viewportShadows.checked
  };

  if (commitNumbers) {
    elements.viewportGridSize.value = String(viewportSettings.gridSize);
    elements.viewportGridDivisions.value = String(viewportSettings.gridDivisions);
  }

  renderer.setViewportSettings(viewportSettings);
  if (save) saveViewportSettings();
}

function renameLink(oldName, requested) {
  if (!requested || requested === oldName) return;
  const next = uniqueName(requested, new Set([...robot.links.keys()].filter(name => name !== oldName)));
  const link = robot.links.get(oldName);
  robot.links.delete(oldName);
  link.name = next;
  robot.links.set(next, link);

  for (const joint of robot.joints.values()) {
    if (joint.parent === oldName) joint.parent = next;
    if (joint.child === oldName) joint.child = next;
  }

  selection = { type: "link", name: next };
  synchroniseFromModel();
}

function renameJoint(oldName, requested) {
  if (!requested || requested === oldName) return;
  const next = uniqueName(requested, new Set([...robot.joints.keys()].filter(name => name !== oldName)));
  const joint = robot.joints.get(oldName);
  robot.joints.delete(oldName);
  joint.name = next;
  robot.joints.set(next, joint);
  for (const control of robot.ros2Controls || []) {
    for (const component of control.components || []) {
      if (component.kind === "joint" && component.name === oldName) component.name = next;
    }
  }
  for (const other of robot.joints.values()) {
    if (other.mimic?.joint === oldName) other.mimic.joint = next;
  }
  selection = { type: "joint", name: next };
  synchroniseFromModel();
}

function deleteSelection() {
  if (!selection) return;

  if (selection.type === "joint") {
    robot.joints.delete(selection.name);
    selection = null;
    synchroniseFromModel();
    return;
  }

  const linkName = selection.name;
  const doomed = new Set([linkName, ...descendants(robot, linkName)]);

  for (const [name, joint] of [...robot.joints.entries()]) {
    if (doomed.has(joint.parent) || doomed.has(joint.child)) robot.joints.delete(name);
  }

  for (const name of doomed) robot.links.delete(name);
  selection = null;
  synchroniseFromModel();
}

function rgbaToHex(rgba) {
  const values = (rgba || [0.5, 0.5, 0.5]).slice(0, 3).map(v => clamp(Math.round(v * 255), 0, 255));
  return `#${values.map(v => v.toString(16).padStart(2, "0")).join("")}`;
}

function hexToRgb(hex) {
  const value = hex.replace("#", "");
  return [0, 2, 4].map(i => parseInt(value.slice(i, i + 2), 16) / 255);
}

async function populateExamples() {
  if (!elements.exampleSelect) return;

  elements.exampleSelect.disabled = true;
  elements.exampleSelect.innerHTML = `<option value="">Loading…</option>`;

  try {
    const examples = await loadExampleRegistry();

    if (!examples.length) {
      elements.exampleSelect.innerHTML = `<option value="">No examples</option>`;
      return;
    }

    elements.exampleSelect.innerHTML = `
      <option value="">Choose…</option>
      ${examples.map(example => `
        <option value="${escapeHTML(example.id)}" title="${escapeHTML(example.description)}">
          ${escapeHTML(example.name)}
        </option>
      `).join("")}
    `;
    elements.exampleSelect.disabled = false;
  } catch (error) {
    console.warn("Could not load the examples registry.", error);
    elements.exampleSelect.innerHTML = `<option value="">Examples unavailable</option>`;
    elements.exampleSelect.title = error.message;
  }
}

function freshExampleURL(path) {
  const url = new URL(path, document.baseURI);
  url.searchParams.set("_kf", `${Date.now()}`);
  return url.href;
}

async function fetchBundledAsset(path) {
  const response = await fetch(freshExampleURL(path), {
    cache: "no-store"
  });

  if (!response.ok) throw new Error(`Could not load bundled asset ${path} (${response.status}).`);

  const blob = await response.blob();
  const url = new URL(path, document.baseURI);
  const name = decodeURIComponent(url.pathname.split("/").pop() || "asset");

  return new File([blob], name, {
    type: blob.type || "application/octet-stream"
  });
}

async function clearExampleAssets() {
  currentExample = null;
  exampleAssets = [];
  if (elements.exampleSelect) elements.exampleSelect.value = "";
  await renderer.setAssets(allAssets());
  renderMeshLibrary();
}

async function loadExample(exampleId) {
  const example = exampleById(exampleId);
  if (!example) return;

  try {
    const response = await fetch(freshExampleURL(example.urdf), {
      cache: "no-store"
    });

    if (!response.ok) throw new Error(`Could not load example URDF (${response.status}).`);
    const text = await response.text();
    const nextRobot = parseURDF(text);

    const bundledAssets = [];
    for (const assetPath of example.assets || []) bundledAssets.push(await fetchBundledAsset(assetPath));
    exampleAssets = bundledAssets;
    currentExample = example.id;
    await renderer.setAssets(allAssets());

    await loadRobot(nextRobot, {
      sourceText: text,
      sourceStatus: `Loaded bundled example "${example.name}" as an editable working copy. Bundled source files are read-only.`,
      historyMode: "reset",
      historyLabel: `Load example ${example.name}`
    });
    elements.exampleSelect.value = example.id;
  } catch (error) {
    setXMLStatus(`Could not load example: ${error.message}`, "error");
    elements.exampleSelect.value = currentExample || "";
  }
}

function isClearlyMobileWorkspace() {
  const narrow = window.matchMedia("(max-width: 760px)").matches;
  const compactTouch = window.matchMedia("(max-width: 920px) and (pointer: coarse)").matches;
  return narrow || compactTouch;
}

function showFirstEntryMobileNotice() {
  if (!elements.mobileNoticeDialog || !isClearlyMobileWorkspace()) return;
  try {
    if (localStorage.getItem("kineform-mobile-notice-v1") === "seen") return;
  } catch {
  }

  elements.mobileNoticeDialog.showModal();
  elements.mobileNoticeDialog.addEventListener("close", () => {
    try {
      localStorage.setItem("kineform-mobile-notice-v1", "seen");
    } catch {
    }
  }, { once: true });
}

async function openFiles(files) {
  const list = [...files];
  const urdfFile = list.find(file => /\.(urdf|xml)$/i.test(file.name));
  const assets = list.filter(supportedLocalAsset);

  if (assets.length) await importMeshes(assets);

  if (urdfFile) {
    try {
      const text = await urdfFile.text();
      const nextRobot = parseURDF(text);
      await clearExampleAssets();
      await loadRobot(nextRobot, { historyLabel: `Open ${urdfFile.name}` });
    } catch (error) {
      setXMLStatus(`Could not open URDF: ${error.message}`, "error");
    }
  }
}

function updateLinkGeometryDialog() {
  const type = elements.addLinkForm.elements.geometry.value;
  if (type === "box") {
    elements.linkGeometryFields.innerHTML = `
      <fieldset>
        <legend>Size xyz (m)</legend>
        <div class="triple">
          <input name="sx" type="number" step="any" min="0.001" value="0.4">
          <input name="sy" type="number" step="any" min="0.001" value="0.4">
          <input name="sz" type="number" step="any" min="0.001" value="0.4">
        </div>
      </fieldset>
    `;
  } else if (type === "cylinder" || type === "capsule") {
    elements.linkGeometryFields.innerHTML = `
      <div class="inline-row">
        <label>Radius (m)<input name="radius" type="number" step="any" min="0.001" value="0.2"></label>
        <label>${type === "capsule" ? "Cylinder length" : "Length"} (m)<input name="length" type="number" step="any" min="0.001" value="0.4"></label>
      </div>
    `;
  } else {
    elements.linkGeometryFields.innerHTML = `<label>Radius (m)<input name="radius" type="number" step="any" min="0.001" value="0.2"></label>`;
  }
}

function openAddLinkDialog(parentName = null) {
  const names = new Set(robot.links.keys());
  const form = elements.addLinkForm.elements;
  form.name.value = uniqueName("new_link", names);

  const links = [...robot.links.keys()];
  form.parent.innerHTML = `<option value="">Unattached / root</option>${links.map(name => `<option value="${escapeHTML(name)}">${escapeHTML(name)}</option>`).join("")}`;

  const preferredParent = parentName && robot.links.has(parentName)
    ? parentName
    : selection?.type === "link" && robot.links.has(selection.name)
      ? selection.name
      : links[0] || "";

  form.parent.value = preferredParent;
  form.joint_name.value = preferredParent
    ? uniqueName(`${preferredParent}_to_${form.name.value}`, new Set(robot.joints.keys()))
    : "";

  const capsuleOption = form.geometry.querySelector('option[value="capsule"]');
  if (capsuleOption) {
    capsuleOption.disabled = robot.version === "1.0";
    if (robot.version === "1.0" && form.geometry.value === "capsule") form.geometry.value = "box";
  }

  updateLinkGeometryDialog();
  updateAddLinkAttachmentFields();
  elements.addLinkDialog.showModal();
}

function updateAddLinkAttachmentFields() {
  const form = elements.addLinkForm.elements;
  const attached = Boolean(form.parent.value);
  const fields = elements.addLinkForm.querySelectorAll("[data-attachment-field]");

  fields.forEach(field => {
    field.hidden = !attached;
    field.querySelectorAll("input, select").forEach(control => {
      control.disabled = !attached;
    });
  });

  if (attached && !form.joint_name.value.trim()) {
    form.joint_name.value = uniqueName(`${form.parent.value}_to_${form.name.value || "link"}`, new Set(robot.joints.keys()));
  }
}

function addLinkFromDialog() {
  const form = elements.addLinkForm.elements;
  const name = uniqueName(form.name.value.trim() || "link", new Set(robot.links.keys()));
  const type = form.geometry.value;
  let geometry;

  if (type === "box") {
    geometry = { type, size: [num(form.sx.value, 0.4), num(form.sy.value, 0.4), num(form.sz.value, 0.4)] };
  } else if (type === "cylinder" || type === "capsule") {
    geometry = { type, radius: num(form.radius.value, 0.2), length: num(form.length.value, 0.4) };
  } else {
    geometry = { type, radius: num(form.radius.value, 0.2) };
  }

  const link = createLink(name, geometry, Math.max(0, num(form.mass.value, 1)));
  robot.links.set(name, link);

  if (form.parent.value && robot.links.has(form.parent.value)) {
    const suggested = form.joint_name.value.trim() || `${form.parent.value}_to_${name}`;
    const jointName = uniqueName(suggested, new Set(robot.joints.keys()));
    const joint = createJoint(jointName, form.joint_type.value, form.parent.value, name);
    joint.origin.xyz = [num(form.jx.value), num(form.jy.value), num(form.jz.value)];

    if (["fixed", "floating", "planar"].includes(joint.type)) joint.limit = null;
    robot.joints.set(jointName, joint);
  }

  selection = { type: "link", name };
  elements.addLinkDialog.close();
  synchroniseFromModel();
}
function openAddJointDialog() {
  const form = elements.addJointForm.elements;
  const links = [...robot.links.keys()];

  if (links.length < 2) return;

  form.name.value = uniqueName("new_joint", new Set(robot.joints.keys()));
  form.parent.innerHTML = links.map(name => `<option>${escapeHTML(name)}</option>`).join("");
  form.child.innerHTML = links.map(name => `<option>${escapeHTML(name)}</option>`).join("");

  const preferredParent = selection?.type === "link" && robot.links.has(selection.name) ? selection.name : links[0];
  form.parent.value = preferredParent;
  form.child.value = links.find(name => name !== preferredParent) || links[0];
  elements.addJointDialog.showModal();
}

function addJointFromDialog() {
  const form = elements.addJointForm.elements;
  const name = uniqueName(form.name.value.trim() || "joint", new Set(robot.joints.keys()));
  const joint = createJoint(name, form.type.value, form.parent.value, form.child.value);
  joint.origin.xyz = [num(form.x.value), num(form.y.value), num(form.z.value)];

  if (["fixed", "floating", "planar"].includes(joint.type)) joint.limit = null;
  robot.joints.set(name, joint);
  selection = { type: "joint", name };
  elements.addJointDialog.close();
  synchroniseFromModel();
}


elements.urdfVersion.addEventListener("change", () => {
  const requested = elements.urdfVersion.value;
  elements.urdfVersion.value = robot.version || "1.0";
  requestURDFVersionChange(requested);
});

elements.confirmURDFVersion.addEventListener("click", async () => {
  if (!pendingURDFVersion) return;
  convertRobotVersion(robot, pendingURDFVersion);
  pendingURDFVersion = null;
  elements.urdfVersionDialog.close();
  await synchroniseFromModel({ historyLabel: "Change URDF version" });
});

elements.urdfVersionDialog.addEventListener("close", () => {
  if (elements.urdfVersionDialog.returnValue !== "default") {
    pendingURDFVersion = null;
    elements.urdfVersion.value = robot.version || "1.0";
  }
});

elements.ros2AddControl.addEventListener("click", () => {
  const index = (robot.ros2Controls?.length || 0) + 1;
  elements.ros2NewName.value = `RobotSystem${index}`;
  elements.ros2NewType.value = "system";
  elements.ros2NewPlugin.value = "";
  elements.ros2NewGroup.value = "";
  elements.ros2NewRate.value = "";
  elements.ros2NewAsync.value = "false";
  elements.ros2AddControlDialog.showModal();
  requestAnimationFrame(() => elements.ros2NewPlugin.focus());
});

elements.confirmROS2AddControl.addEventListener("click", () => {
  if (!elements.ros2AddControlForm.reportValidity()) return;

  const control = createROS2Control(
    elements.ros2NewName.value.trim(),
    {
      type: elements.ros2NewType.value,
      plugin: elements.ros2NewPlugin.value.trim(),
      group: elements.ros2NewGroup.value.trim(),
      rwRate: elements.ros2NewRate.value,
      isAsync: elements.ros2NewAsync.value === "true"
    }
  );

  robot.ros2Controls.push(control);
  elements.ros2AddControlDialog.close("default");
  syncROS2Only({ historyLabel: "Add ROS 2 hardware" });
  renderROS2Control();

  requestAnimationFrame(() => {
    elements.ros2Editor.scrollTop = elements.ros2Editor.scrollHeight;
  });
});

elements.urdfFile.addEventListener("change", async () => {
  await openFiles(elements.urdfFile.files);
  elements.urdfFile.value = "";
});
elements.meshFiles.addEventListener("change", async () => {
  await openFiles(elements.meshFiles.files);
  elements.meshFiles.value = "";
});
elements.meshLibraryAdd.addEventListener("click", () => elements.meshFiles.click());

elements.newRobot.addEventListener("click", async () => {
  await clearExampleAssets();
  const next = emptyRobot("untitled_robot");
  await loadRobot(next, { historyLabel: "New empty robot" });
});

elements.exampleSelect?.addEventListener("change", async () => {
  if (!elements.exampleSelect.value) return;
  await loadExample(elements.exampleSelect.value);
});

elements.undoEdit?.addEventListener("click", undoRobotEdit);
elements.redoEdit?.addEventListener("click", redoRobotEdit);
elements.addMaterial?.addEventListener("click", addGlobalMaterial);
elements.aboutApp?.addEventListener("click", () => elements.aboutDialog?.showModal());

elements.exportURDF.addEventListener("click", () => {
  const text = serialiseURDF(robot);
  const blob = new Blob([text], { type: "application/xml" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `${(robot.name || "robot").replace(/[^a-zA-Z0-9_-]+/g, "_")}.urdf`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

elements.addLink.addEventListener("click", () => openAddLinkDialog());
elements.addJoint.addEventListener("click", openAddJointDialog);
elements.addLinkForm.elements.geometry.addEventListener("change", updateLinkGeometryDialog);
elements.addLinkForm.elements.parent.addEventListener("change", () => {
  const form = elements.addLinkForm.elements;
  if (form.parent.value) {
    form.joint_name.value = uniqueName(`${form.parent.value}_to_${form.name.value || "link"}`, new Set(robot.joints.keys()));
  }
  updateAddLinkAttachmentFields();
});
elements.addLinkForm.elements.name.addEventListener("input", () => {
  const form = elements.addLinkForm.elements;
  if (form.parent.value) {
    form.joint_name.value = uniqueName(`${form.parent.value}_to_${form.name.value || "link"}`, new Set(robot.joints.keys()));
  }
});
elements.confirmAddLink.addEventListener("click", addLinkFromDialog);
elements.confirmAddJoint.addEventListener("click", addJointFromDialog);
elements.deleteSelected.addEventListener("click", deleteSelection);

elements.xmlEditor.addEventListener("input", () => {
  sourceDirty = true;
  if (sourceApplyIssue) {
    sourceApplyIssue = null;
    renderDiagnostics();
  }
  setXMLStatus("Source has unapplied changes.", "dirty");
});

elements.applyXML.addEventListener("click", async () => {
  const draft = elements.xmlEditor.value;
  const previousRobot = robot;
  const previousSelection = selection;

  try {
    const next = parseURDF(draft);
    const nextIssues = validateRobot(next);
    const errorCount = nextIssues.filter(item => item.level === "error").length;
    const warningCount = nextIssues.filter(item => item.level === "warning").length;

    const summary = [];
    if (errorCount) summary.push(`${errorCount} ${errorCount === 1 ? "error" : "errors"}`);
    if (warningCount) summary.push(`${warningCount} ${warningCount === 1 ? "warning" : "warnings"}`);

    const status = summary.length
      ? `XML applied without rewriting your source. Doctor reports ${summary.join(" and ")}.`
      : "XML applied without rewriting your source.";

    await loadRobot(next, {
      frame: false,
      sourceText: draft,
      sourceStatus: status,
      historyMode: "record",
      historyLabel: "Apply XML"
    });
  } catch (error) {
    robot = previousRobot;
    selection = previousSelection;
    sourceDirty = true;
    sourceApplyIssue = {
      level: "error",
      message: `XML draft not applied: ${error.message}`
    };
    setXMLStatus(`Not applied. Existing robot unchanged. ${error.message}`, "error");
    renderDiagnostics();
  }
});

elements.formatXML.addEventListener("click", () => {
  elements.xmlEditor.value = formatXML(elements.xmlEditor.value);
  sourceDirty = true;
  setXMLStatus("Source formatted. Apply XML to update the model.", "dirty");
});

function setTransformButton(mode) {
  for (const [button, value] of [
    [elements.selectMode, "select"],
    [elements.moveMode, "move"],
    [elements.rotateMode, "rotate"]
  ]) {
    button.classList.toggle("active", value === mode);
  }
  renderer.setTransformMode(mode);
}

elements.selectMode.addEventListener("click", () => setTransformButton("select"));
elements.moveMode.addEventListener("click", () => setTransformButton("move"));
elements.rotateMode.addEventListener("click", () => setTransformButton("rotate"));

for (const [button, kind] of [
  [elements.toggleVisual, "visual"],
  [elements.toggleCollision, "collision"],
  [elements.toggleFrames, "frames"],
  [elements.toggleAxes, "axes"],
  [elements.toggleCom, "com"]
]) {
  button.addEventListener("click", () => {
    button.classList.toggle("active");
    renderer.setVisibility(kind, button.classList.contains("active"));
  });
}

elements.frameRobot.addEventListener("click", () => renderer.frameRobot());

elements.toggleViewportSettings.addEventListener("click", () => {
  const open = elements.viewportSettings.hidden;
  elements.viewportSettings.hidden = !open;
  elements.toggleViewportSettings.classList.toggle("active", open);
});

for (const control of [
  elements.viewportBackground,
  elements.viewportFloorVisible,
  elements.viewportFloorColour,
  elements.viewportGridVisible,
  elements.viewportGridCentre,
  elements.viewportGridLines,
  elements.viewportGridOpacity,
  elements.viewportWorldAxes,
  elements.viewportShadows
]) {
  control.addEventListener("input", () => applyViewportSettings());
  control.addEventListener("change", () => applyViewportSettings());
}

for (const control of [
  elements.viewportGridSize,
  elements.viewportGridDivisions
]) {
  control.addEventListener("input", () => {
    applyViewportSettings({ commitNumbers: false });
  });

  control.addEventListener("change", () => {
    applyViewportSettings({ commitNumbers: true });
  });

  control.addEventListener("keydown", event => {
    if (event.key !== "Enter") return;
    event.preventDefault();
    applyViewportSettings({ commitNumbers: true });
    control.blur();
  });
}

elements.resetViewportSettings.addEventListener("click", () => {
  viewportSettings = { ...DEFAULT_VIEWPORT_SETTINGS };
  syncViewportSettingsControls();
  applyViewportSettings();
});

let dragDepth = 0;

window.addEventListener("dragenter", event => {
  event.preventDefault();
  dragDepth += 1;
  elements.dropOverlay.classList.add("visible");
});

window.addEventListener("dragover", event => event.preventDefault());

window.addEventListener("dragleave", event => {
  event.preventDefault();
  dragDepth -= 1;
  if (dragDepth <= 0) {
    dragDepth = 0;
    elements.dropOverlay.classList.remove("visible");
  }
});

window.addEventListener("drop", event => {
  event.preventDefault();
  dragDepth = 0;
  elements.dropOverlay.classList.remove("visible");
  openFiles(event.dataTransfer.files);
});

window.addEventListener("keydown", event => {
  if (event.target.matches("input, textarea, select") || event.target.isContentEditable) return;

  const modifier = event.ctrlKey || event.metaKey;
  if (modifier && event.key.toLowerCase() === "z") {
    event.preventDefault();
    if (event.shiftKey) redoRobotEdit();
    else undoRobotEdit();
    return;
  }
  if (modifier && event.key.toLowerCase() === "y") {
    event.preventDefault();
    redoRobotEdit();
    return;
  }

  if (event.key === "Delete") deleteSelection();
  if (event.key.toLowerCase() === "f") renderer.frameRobot();
  if (event.key.toLowerCase() === "w") setTransformButton("move");
  if (event.key.toLowerCase() === "e") setTransformButton("rotate");
  if (event.key === "Escape") setTransformButton("select");
});

await populateExamples();
syncViewportSettingsControls();
renderer.setViewportSettings(viewportSettings);

try {
  meshAssets = await loadMeshFiles();
  await renderer.setAssets(allAssets());
  navigator.storage?.persist?.().catch(() => false);
} catch (error) {
  console.warn("Could not restore the local asset library.", error);
}

await renderer.setRobot(robot, false);
renderer.setSelection(selection);
renderer.frameRobot();
elements.xmlEditor.value = serialiseURDF(robot);
renderAll();
history.reset(robot, selection, "Empty robot");
setXMLStatus("Empty workspace. Open a URDF, choose an example, or add a link.");
showFirstEntryMobileNotice();
