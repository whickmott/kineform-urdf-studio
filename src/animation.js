const MOVABLE_TYPES = new Set(["revolute", "continuous", "prismatic"]);

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export class JointAnimationController {
  constructor({
    getRobot,
    renderer,
    elements,
    onPoseChanged = () => {},
    onStateChanged = () => {}
  }) {
    this.getRobot = getRobot;
    this.renderer = renderer;
    this.elements = elements;
    this.onPoseChanged = onPoseChanged;
    this.onStateChanged = onStateChanged;

    this.duration = 5;
    this.time = 0;
    this.loop = true;
    this.keyframes = [];
    this.selectedKeyframeId = null;
    this.playing = false;
    this.startedAt = 0;
    this.startedFrom = 0;
    this.raf = null;
    this.poseDirty = false;
    this.nextKeyframeId = 1;

    this.bind();
    this.render();
  }

  bind() {
    this.elements.toggle.addEventListener("click", () => {
      this.elements.panel.classList.toggle("visible");
      this.elements.toggle.classList.toggle("active", this.elements.panel.classList.contains("visible"));
    });

    this.elements.play.addEventListener("click", () => this.togglePlayback());
    this.elements.stop.addEventListener("click", () => this.stop());
    this.elements.capture.addEventListener("click", () => this.captureKeyframe());
    this.elements.deleteKey.addEventListener("click", () => this.deleteSelectedKeyframe());
    this.elements.clear.addEventListener("click", () => this.clear());

    this.elements.timeline.addEventListener("input", () => {
      this.pause();
      this.movePlayhead(Number(this.elements.timeline.value));
    });

    const commitDuration = () => {
      const next = Math.max(0.1, Number(this.elements.duration.value) || 5);
      this.duration = next;
      this.time = clamp(this.time, 0, this.duration);

      for (const frame of this.keyframes) frame.time = clamp(frame.time, 0, this.duration);

      this.deduplicateTimes();
      this.sortKeyframes();
      this.elements.duration.value = this.duration.toFixed(2);
      this.render();
      if (!this.poseDirty) this.applyTime(this.time);
      this.onStateChanged();
    };

    this.elements.duration.addEventListener("change", commitDuration);

    this.elements.duration.addEventListener("keydown", event => {
      if (event.key !== "Enter") return;
      event.preventDefault();
      commitDuration();
      this.elements.duration.blur();
    });

    this.elements.loop.addEventListener("change", () => {
      this.loop = this.elements.loop.checked;
      this.onStateChanged();
    });

    this.elements.exportButton.addEventListener("click", () => this.exportJSON());
    this.elements.importInput.addEventListener("change", async () => {
      const [file] = this.elements.importInput.files || [];
      if (file) await this.importJSON(file);
      this.elements.importInput.value = "";
    });
  }

  resetForRobot() {
    this.pause();
    this.time = 0;
    this.keyframes = [];
    this.selectedKeyframeId = null;
    this.poseDirty = false;
    this.nextKeyframeId = 1;
    this.render();
    this.onStateChanged();
  }

  movableJoints() {
    return [...this.getRobot().joints.values()].filter(joint => MOVABLE_TYPES.has(joint.type));
  }

  capturePose() {
    return Object.fromEntries(this.movableJoints().map(joint => [joint.name, Number(joint.value) || 0]));
  }

  selectedFrame() {
    return this.keyframes.find(frame => frame.id === this.selectedKeyframeId) || null;
  }

  epsilon() {
    return Math.max(0.002, this.duration / 5000);
  }

  keyframeAtTime(time = this.time, excludeId = null) {
    const epsilon = this.epsilon();
    return this.keyframes.find(frame => frame.id !== excludeId && Math.abs(frame.time - time) <= epsilon) || null;
  }

  sortKeyframes() {
    this.keyframes.sort((a, b) => a.time - b.time || a.id - b.id);
  }

  deduplicateTimes() {
    this.sortKeyframes();
    const result = [];

    for (const frame of this.keyframes) {
      const existing = result.find(item => Math.abs(item.time - frame.time) <= this.epsilon());
      if (!existing) {
        result.push(frame);
        continue;
      }

      if (frame.id === this.selectedKeyframeId) {
        const index = result.indexOf(existing);
        result[index] = frame;
      }
    }

    this.keyframes = result;
    if (this.selectedKeyframeId != null && !this.selectedFrame()) this.selectedKeyframeId = null;
  }

  movePlayhead(value) {
    const next = clamp(Number(value) || 0, 0, this.duration);
    this.selectedKeyframeId = null;

    if (this.poseDirty) {
      this.time = next;
      this.renderTime();
      this.renderKeyframeSelection();
      this.renderSummary();
      return;
    }

    this.setTime(next, { apply: true, clearSelection: true });
  }

  notifyManualPoseChanged() {
    this.pause();
    const frame = this.selectedFrame();

    if (frame && Math.abs(this.time - frame.time) <= this.epsilon()) {
      frame.joints = this.capturePose();
      this.poseDirty = false;
      this.renderSummary();
      this.onStateChanged();
      return;
    }

    this.selectedKeyframeId = null;
    this.poseDirty = true;
    this.renderKeyframeSelection();
    this.renderSummary();
  }

  captureKeyframe() {
    const pose = this.capturePose();
    if (!Object.keys(pose).length) return;

    const existing = this.keyframeAtTime(this.time);
    if (existing) {
      existing.joints = pose;
      this.selectedKeyframeId = existing.id;
    } else {
      const frame = {
        id: this.nextKeyframeId++,
        time: this.time,
        joints: pose
      };
      this.keyframes.push(frame);
      this.selectedKeyframeId = frame.id;
    }

    this.poseDirty = false;
    this.sortKeyframes();
    this.render();
    this.onStateChanged();
  }

  deleteSelectedKeyframe() {
    const frame = this.selectedFrame();
    if (!frame) return;

    this.keyframes = this.keyframes.filter(item => item.id !== frame.id);
    this.selectedKeyframeId = null;
    this.poseDirty = false;
    this.render();
    if (this.keyframes.length) this.applyTime(this.time);
    this.onStateChanged();
  }

  clear() {
    this.pause();
    this.keyframes = [];
    this.selectedKeyframeId = null;
    this.poseDirty = false;
    this.time = 0;
    this.render();
    this.onStateChanged();
  }

  selectKeyframe(id) {
    const frame = this.keyframes.find(item => item.id === id);
    if (!frame) return;

    this.pause();
    this.poseDirty = false;
    this.selectedKeyframeId = frame.id;
    this.time = frame.time;
    this.applyFrame(frame);
    this.renderTime();
    this.renderKeyframeSelection();
    this.renderSummary();
  }

  beginMarkerDrag(event, id) {
    if (event.button !== 0) return;
    const frame = this.keyframes.find(item => item.id === id);
    if (!frame) return;

    event.preventDefault();
    event.stopPropagation();
    this.selectKeyframe(id);

    const rect = this.elements.markers.getBoundingClientRect();
    const update = clientX => {
      const ratio = clamp((clientX - rect.left) / Math.max(1, rect.width), 0, 1);
      frame.time = ratio * this.duration;
      this.time = frame.time;
      this.sortKeyframes();
      this.renderMarkers();
      this.renderTime();
      this.renderSummary();
    };

    update(event.clientX);

    const move = moveEvent => update(moveEvent.clientX);
    const end = () => {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", end);
      window.removeEventListener("pointercancel", end);
      this.deduplicateTimes();
      this.sortKeyframes();
      const selected = this.selectedFrame();
      if (selected) {
        this.time = selected.time;
        this.applyFrame(selected);
      }
      this.render();
      this.onStateChanged();
    };

    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", end);
    window.addEventListener("pointercancel", end);
  }

  setTime(value, { apply = true, clearSelection = false } = {}) {
    this.time = clamp(Number(value) || 0, 0, this.duration);
    if (clearSelection) this.selectedKeyframeId = null;

    if (apply) {
      this.poseDirty = false;
      this.applyTime(this.time);
    }

    this.renderTime();
    this.renderKeyframeSelection();
    this.renderSummary();
  }

  applyFrame(frame) {
    for (const joint of this.movableJoints()) {
      const value = finite(frame.joints[joint.name], Number(joint.value) || 0);
      joint.value = value;
      this.renderer.updateJointValue(joint.name, value);
    }
    this.onPoseChanged();
  }

  applyTime(time) {
    if (!this.keyframes.length) return;

    const frames = this.keyframes;
    let left = frames[0];
    let right = frames[frames.length - 1];

    if (time <= frames[0].time) {
      left = right = frames[0];
    } else if (time >= frames[frames.length - 1].time) {
      left = right = frames[frames.length - 1];
    } else {
      for (let i = 0; i < frames.length - 1; i += 1) {
        if (time >= frames[i].time && time <= frames[i + 1].time) {
          left = frames[i];
          right = frames[i + 1];
          break;
        }
      }
    }

    const span = Math.max(1e-9, right.time - left.time);
    const alpha = left === right ? 0 : clamp((time - left.time) / span, 0, 1);

    for (const joint of this.movableJoints()) {
      const current = Number(joint.value) || 0;
      const start = finite(left.joints[joint.name], current);
      const end = finite(right.joints[joint.name], start);
      const value = lerp(start, end, alpha);
      joint.value = value;
      this.renderer.updateJointValue(joint.name, value);
    }

    this.onPoseChanged();
  }

  togglePlayback() {
    if (this.playing) {
      this.pause();
      return;
    }

    if (this.keyframes.length < 2) return;
    if (this.time >= this.duration - 1e-6) this.time = 0;

    this.poseDirty = false;
    this.selectedKeyframeId = null;
    this.playing = true;
    this.startedFrom = this.time;
    this.startedAt = performance.now();
    this.renderKeyframeSelection();
    this.renderPlaybackState();
    this.tick();
  }

  tick() {
    if (!this.playing) return;

    const elapsed = (performance.now() - this.startedAt) / 1000;
    let next = this.startedFrom + elapsed;

    if (next > this.duration) {
      if (this.loop) {
        next %= this.duration;
        this.startedFrom = next;
        this.startedAt = performance.now();
      } else {
        next = this.duration;
        this.playing = false;
      }
    }

    this.time = next;
    this.applyTime(this.time);
    this.renderTime();
    this.renderPlaybackState();
    if (this.playing) this.raf = requestAnimationFrame(() => this.tick());
  }

  pause() {
    this.playing = false;
    if (this.raf != null) cancelAnimationFrame(this.raf);
    this.raf = null;
    this.renderPlaybackState();
  }

  stop() {
    this.pause();
    this.poseDirty = false;
    this.selectedKeyframeId = null;
    this.setTime(0, { apply: true, clearSelection: true });
    this.render();
  }

  render() {
    if (document.activeElement !== this.elements.duration) {
      this.elements.duration.value = this.duration.toFixed(2);
    }
    this.elements.loop.checked = this.loop;
    this.elements.timeline.min = 0;
    this.elements.timeline.max = this.duration;
    this.elements.timeline.step = Math.max(0.001, this.duration / 1000);
    this.renderMarkers();
    this.renderSummary();
    this.renderTime();
    this.renderKeyframeSelection();
    this.renderPlaybackState();
  }

  renderMarkers() {
    this.elements.markers.innerHTML = this.keyframes.map((frame, index) => {
      const left = this.duration > 0 ? frame.time / this.duration * 100 : 0;
      const selected = frame.id === this.selectedKeyframeId ? " selected" : "";
      return `<button type="button" class="animation-marker${selected}" data-animation-key="${frame.id}" style="left:${left}%" aria-label="Keyframe ${index + 1} at ${frame.time.toFixed(3)} seconds" title="Keyframe ${index + 1} · ${frame.time.toFixed(3)} s · drag to retime"></button>`;
    }).join("");

    this.elements.markers.querySelectorAll("[data-animation-key]").forEach(button => {
      const id = Number(button.dataset.animationKey);
      button.addEventListener("pointerdown", event => this.beginMarkerDrag(event, id));
      button.addEventListener("keydown", event => {
        if (event.key === "Enter" || event.key === " ") {
          event.preventDefault();
          this.selectKeyframe(id);
        }
      });
    });
  }

  renderSummary() {
    const joints = this.movableJoints().length;
    const selected = this.selectedFrame();
    const mode = selected
      ? ` · key ${this.keyframes.indexOf(selected) + 1} selected`
      : this.poseDirty
        ? " · working pose"
        : "";

    this.elements.summary.textContent = `${this.keyframes.length} ${this.keyframes.length === 1 ? "key" : "keys"} · ${joints} ${joints === 1 ? "joint" : "joints"}${mode}`;
    this.elements.deleteKey.disabled = !selected;
    this.elements.play.disabled = this.keyframes.length < 2;
    this.elements.exportButton.disabled = this.keyframes.length === 0;
  }

  renderKeyframeSelection() {
    this.elements.markers.querySelectorAll("[data-animation-key]").forEach(button => {
      button.classList.toggle("selected", Number(button.dataset.animationKey) === this.selectedKeyframeId);
    });
  }

  renderTime() {
    this.elements.timeline.value = this.time;
    this.elements.readout.textContent = `${this.time.toFixed(2)} / ${this.duration.toFixed(2)} s`;
  }

  renderPlaybackState() {
    this.elements.play.textContent = this.playing ? "Pause" : "Play";
    this.elements.play.classList.toggle("active", this.playing);
  }

  exportJSON() {
    if (!this.keyframes.length) return;

    const payload = {
      format: "urdf-studio-animation",
      version: 2,
      robot: this.getRobot().name || "robot",
      duration: this.duration,
      loop: this.loop,
      interpolation: "linear",
      keyframes: this.keyframes.map(frame => ({
        time: frame.time,
        joints: { ...frame.joints }
      }))
    };

    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${(this.getRobot().name || "robot").replace(/[^a-zA-Z0-9_-]+/g, "_")}.animation.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  async importJSON(file) {
    try {
      const data = JSON.parse(await file.text());
      if (data?.format !== "urdf-studio-animation" || !Array.isArray(data.keyframes)) {
        throw new Error("Not a URDF Studio animation file.");
      }

      this.pause();
      this.duration = Math.max(0.1, Number(data.duration) || 5);
      this.loop = data.loop !== false;
      this.keyframes = data.keyframes
        .filter(frame => Number.isFinite(Number(frame.time)) && frame.joints && typeof frame.joints === "object")
        .map(frame => ({
          id: this.nextKeyframeId++,
          time: clamp(Number(frame.time), 0, this.duration),
          joints: { ...frame.joints }
        }));

      this.deduplicateTimes();
      this.sortKeyframes();
      this.selectedKeyframeId = this.keyframes[0]?.id ?? null;
      this.poseDirty = false;
      this.time = this.keyframes[0]?.time || 0;
      this.render();
      if (this.selectedFrame()) this.applyFrame(this.selectedFrame());
      this.onStateChanged();
    } catch (error) {
      console.error(error);
      alert(`Could not import animation: ${error.message}`);
    }
  }
}
