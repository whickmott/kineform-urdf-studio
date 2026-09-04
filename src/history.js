function cloneState(value) {
  return structuredClone(value);
}

export class RobotHistory {
  constructor({ limit = 100, onChange = null } = {}) {
    this.limit = Math.max(2, Number(limit) || 100);
    this.onChange = onChange;
    this.entries = [];
    this.index = -1;
  }

  snapshot(robot, selection, label = "Edit") {
    return {
      robot: cloneState(robot),
      selection: selection ? { ...selection } : null,
      label
    };
  }

  reset(robot, selection, label = "Loaded") {
    this.entries = [this.snapshot(robot, selection, label)];
    this.index = 0;
    this.emit();
  }

  record(robot, selection, label = "Edit") {
    const next = this.snapshot(robot, selection, label);
    this.entries = this.entries.slice(0, this.index + 1);
    this.entries.push(next);

    if (this.entries.length > this.limit) {
      this.entries.shift();
    } else {
      this.index += 1;
    }

    if (this.entries.length === this.limit && this.index >= this.limit) {
      this.index = this.limit - 1;
    }

    this.emit();
  }

  canUndo() {
    return this.index > 0;
  }

  canRedo() {
    return this.index >= 0 && this.index < this.entries.length - 1;
  }

  undo() {
    if (!this.canUndo()) return null;
    this.index -= 1;
    this.emit();
    return cloneState(this.entries[this.index]);
  }

  redo() {
    if (!this.canRedo()) return null;
    this.index += 1;
    this.emit();
    return cloneState(this.entries[this.index]);
  }

  undoLabel() {
    return this.canUndo() ? this.entries[this.index]?.label || "Edit" : "";
  }

  redoLabel() {
    return this.canRedo() ? this.entries[this.index + 1]?.label || "Edit" : "";
  }

  emit() {
    this.onChange?.({
      canUndo: this.canUndo(),
      canRedo: this.canRedo(),
      undoLabel: this.undoLabel(),
      redoLabel: this.redoLabel()
    });
  }
}
