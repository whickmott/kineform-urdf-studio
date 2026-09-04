const STORAGE_KEY = "urdf-studio-layout-v1";

const DEFAULT_STATE = {
  leftOpen: true,
  rightOpen: true,
  bottomOpen: true,
  leftWidth: 286,
  rightWidth: 320,
  bottomHeight: 220,
  bottomTab: "source",
  panes: {
    structure: { open: true, grow: 3.2 },
    meshes: { open: false, grow: 1.45 },
    joints: { open: true, grow: 2.35 }
  }
};

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function cloneDefaultState() {
  return JSON.parse(JSON.stringify(DEFAULT_STATE));
}

function mergeState(raw) {
  const state = cloneDefaultState();
  if (!raw || typeof raw !== "object") return state;

  for (const key of ["leftOpen", "rightOpen", "bottomOpen"]) {
    if (typeof raw[key] === "boolean") state[key] = raw[key];
  }

  for (const key of ["leftWidth", "rightWidth", "bottomHeight"]) {
    if (Number.isFinite(Number(raw[key]))) state[key] = Number(raw[key]);
  }

  if (["source", "diagnostics", "materials", "ros2"].includes(raw.bottomTab)) state.bottomTab = raw.bottomTab;

  for (const name of Object.keys(state.panes)) {
    const pane = raw.panes?.[name];
    if (!pane) continue;
    if (typeof pane.open === "boolean") state.panes[name].open = pane.open;
    if (Number.isFinite(Number(pane.grow)) && Number(pane.grow) > 0) state.panes[name].grow = Number(pane.grow);
  }

  return state;
}

export class WorkspaceLayout {
  constructor(elements) {
    this.elements = elements;
    this.state = this.load();
    this.dragCleanup = null;
    this.bind();
    this.apply();
  }

  load() {
    try {
      return mergeState(JSON.parse(localStorage.getItem(STORAGE_KEY)));
    } catch {
      return cloneDefaultState();
    }
  }

  save() {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(this.state));
    } catch {
    }
  }

  reset() {
    this.state = cloneDefaultState();
    this.save();
    this.apply();
  }

  bind() {
    const e = this.elements;

    e.toggleLeft?.addEventListener("click", () => this.toggleDock("left"));
    e.toggleRight?.addEventListener("click", () => this.toggleDock("right"));
    e.toggleBottom?.addEventListener("click", () => this.toggleDock("bottom"));
    e.collapseRight?.addEventListener("click", () => this.setDock("right", false));
    e.collapseBottom?.addEventListener("click", () => this.setDock("bottom", false));
    e.reset?.addEventListener("click", () => this.reset());

    e.workspace?.querySelectorAll("[data-toggle-pane]").forEach(button => {
      button.addEventListener("click", () => this.togglePane(button.dataset.togglePane));
    });

    e.workspace?.querySelectorAll("[data-bottom-tab]").forEach(button => {
      button.addEventListener("click", () => this.setBottomTab(button.dataset.bottomTab));
    });

    this.bindOuterSplitter(e.leftSplitter, "left");
    this.bindOuterSplitter(e.rightSplitter, "right");
    this.bindOuterSplitter(e.bottomSplitter, "bottom");

    e.workspace?.querySelectorAll("[data-stack-splitter]").forEach(splitter => {
      this.bindStackSplitter(splitter);
    });

    window.addEventListener("resize", () => this.applySizes());
  }

  toggleDock(which) {
    const key = `${which}Open`;
    this.state[key] = !this.state[key];
    this.save();
    this.apply();
  }

  setDock(which, open) {
    this.state[`${which}Open`] = Boolean(open);
    this.save();
    this.apply();
  }

  togglePane(name) {
    const pane = this.state.panes[name];
    if (!pane) return;
    pane.open = !pane.open;
    this.save();
    this.applyPanes();
  }

  setBottomTab(tab) {
    if (!["source", "diagnostics", "materials", "ros2"].includes(tab)) return;
    this.state.bottomTab = tab;
    if (!this.state.bottomOpen) this.state.bottomOpen = true;
    this.save();
    this.apply();
  }

  apply() {
    const e = this.elements;

    if (e.leftDock) e.leftDock.hidden = !this.state.leftOpen;
    if (e.leftSplitter) e.leftSplitter.hidden = !this.state.leftOpen;
    if (e.rightDock) e.rightDock.hidden = !this.state.rightOpen;
    if (e.rightSplitter) e.rightSplitter.hidden = !this.state.rightOpen;
    if (e.bottomDock) e.bottomDock.hidden = !this.state.bottomOpen;
    if (e.bottomSplitter) e.bottomSplitter.hidden = !this.state.bottomOpen;

    e.toggleLeft?.classList.toggle("active", this.state.leftOpen);
    e.toggleRight?.classList.toggle("active", this.state.rightOpen);
    e.toggleBottom?.classList.toggle("active", this.state.bottomOpen);

    e.toggleLeft?.setAttribute("aria-pressed", String(this.state.leftOpen));
    e.toggleRight?.setAttribute("aria-pressed", String(this.state.rightOpen));
    e.toggleBottom?.setAttribute("aria-pressed", String(this.state.bottomOpen));

    this.applySizes();
    this.applyPanes();
    this.applyBottomTab();
  }

  applySizes() {
    const e = this.elements;
    if (!e.workspace) return;

    const width = e.workspace.clientWidth || window.innerWidth;
    const height = e.workspace.clientHeight || window.innerHeight;
    const leftMax = Math.max(220, Math.min(560, width * 0.45));
    const rightMax = Math.max(240, Math.min(560, width * 0.45));
    const bottomMax = Math.max(140, Math.min(460, height * 0.62));

    this.state.leftWidth = clamp(this.state.leftWidth, 210, leftMax);
    this.state.rightWidth = clamp(this.state.rightWidth, 240, rightMax);
    this.state.bottomHeight = clamp(this.state.bottomHeight, 120, bottomMax);

    e.workspace.style.setProperty("--left-width", `${this.state.leftWidth}px`);
    e.workspace.style.setProperty("--right-width", `${this.state.rightWidth}px`);
    e.workspace.style.setProperty("--bottom-height", `${this.state.bottomHeight}px`);
  }

  applyPanes() {
    for (const [name, paneState] of Object.entries(this.state.panes)) {
      const pane = this.elements.workspace?.querySelector(`[data-dock-pane="${name}"]`);
      if (!pane) continue;

      pane.classList.toggle("collapsed", !paneState.open);
      pane.style.flexGrow = paneState.open ? String(paneState.grow) : "0";
      pane.style.flexShrink = paneState.open ? "1" : "0";
      pane.style.flexBasis = paneState.open ? "0px" : "37px";

      const button = pane.querySelector(`[data-toggle-pane="${name}"]`);
      if (button) {
        button.textContent = paneState.open ? "▾" : "▸";
        button.setAttribute("aria-expanded", String(paneState.open));
      }
    }

    this.elements.workspace?.querySelectorAll("[data-stack-splitter]").forEach(splitter => {
      const [a, b] = splitter.dataset.stackSplitter.split(":");
      const usable = this.state.panes[a]?.open && this.state.panes[b]?.open;
      splitter.classList.toggle("disabled", !usable);
    });
  }

  applyBottomTab() {
    const tab = this.state.bottomTab;
    this.elements.workspace?.querySelectorAll("[data-bottom-tab]").forEach(button => {
      const active = button.dataset.bottomTab === tab;
      button.classList.toggle("active", active);
      button.setAttribute("aria-selected", String(active));
    });

    this.elements.workspace?.querySelectorAll("[data-bottom-view]").forEach(view => {
      const active = view.dataset.bottomView === tab;
      view.hidden = !active;
      view.classList.toggle("active", active);
    });

    this.elements.workspace?.querySelectorAll("[data-bottom-actions]").forEach(actions => {
      actions.hidden = actions.dataset.bottomActions !== tab;
    });
  }

  bindOuterSplitter(splitter, which) {
    if (!splitter) return;

    splitter.addEventListener("dblclick", () => this.toggleDock(which));
    splitter.addEventListener("pointerdown", event => {
      const openKey = `${which}Open`;
      if (!this.state[openKey]) return;

      event.preventDefault();
      splitter.setPointerCapture?.(event.pointerId);
      document.body.classList.add("resizing-workspace");

      const startX = event.clientX;
      const startY = event.clientY;
      const startValue = which === "left"
        ? this.state.leftWidth
        : which === "right"
          ? this.state.rightWidth
          : this.state.bottomHeight;

      const move = moveEvent => {
        if (which === "left") {
          this.state.leftWidth = startValue + (moveEvent.clientX - startX);
        } else if (which === "right") {
          this.state.rightWidth = startValue - (moveEvent.clientX - startX);
        } else {
          this.state.bottomHeight = startValue - (moveEvent.clientY - startY);
        }
        this.applySizes();
      };

      const end = () => {
        window.removeEventListener("pointermove", move);
        window.removeEventListener("pointerup", end);
        window.removeEventListener("pointercancel", end);
        document.body.classList.remove("resizing-workspace");
        this.save();
      };

      window.addEventListener("pointermove", move);
      window.addEventListener("pointerup", end);
      window.addEventListener("pointercancel", end);
    });
  }

  bindStackSplitter(splitter) {
    splitter.addEventListener("pointerdown", event => {
      const [firstName, secondName] = splitter.dataset.stackSplitter.split(":");
      const firstState = this.state.panes[firstName];
      const secondState = this.state.panes[secondName];
      if (!firstState?.open || !secondState?.open) return;

      const first = this.elements.workspace.querySelector(`[data-dock-pane="${firstName}"]`);
      const second = this.elements.workspace.querySelector(`[data-dock-pane="${secondName}"]`);
      if (!first || !second) return;

      event.preventDefault();
      document.body.classList.add("resizing-workspace");

      const startY = event.clientY;
      const firstHeight = first.getBoundingClientRect().height;
      const secondHeight = second.getBoundingClientRect().height;
      const totalHeight = firstHeight + secondHeight;
      const totalGrow = firstState.grow + secondState.grow;

      const move = moveEvent => {
        const nextFirst = clamp(firstHeight + (moveEvent.clientY - startY), 78, totalHeight - 78);
        const ratio = nextFirst / totalHeight;
        firstState.grow = totalGrow * ratio;
        secondState.grow = totalGrow * (1 - ratio);
        this.applyPanes();
      };

      const end = () => {
        window.removeEventListener("pointermove", move);
        window.removeEventListener("pointerup", end);
        window.removeEventListener("pointercancel", end);
        document.body.classList.remove("resizing-workspace");
        this.save();
      };

      window.addEventListener("pointermove", move);
      window.addEventListener("pointerup", end);
      window.addEventListener("pointercancel", end);
    });
  }
}
