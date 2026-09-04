import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { TIFFLoader } from "three/addons/loaders/TIFFLoader.js";
import { roots, childJoints } from "./model.js";

const URDF_TO_THREE = new THREE.Matrix4().makeRotationX(-Math.PI / 2);

function setOrigin(object, origin) {
  object.position.set(...(origin?.xyz || [0, 0, 0]));
  object.rotation.set(...(origin?.rpy || [0, 0, 0]), "XYZ");
}

function makeGeometry(geometry) {
  if (!geometry) return null;

  if (geometry.type === "box") {
    const [x, y, z] = geometry.size;
    return new THREE.BoxGeometry(x, y, z);
  }

  if (geometry.type === "cylinder") {
    const g = new THREE.CylinderGeometry(geometry.radius, geometry.radius, geometry.length, 36);
    g.rotateX(Math.PI / 2);
    return g;
  }

  if (geometry.type === "sphere") {
    return new THREE.SphereGeometry(geometry.radius, 32, 20);
  }

  return null;
}

function rgbaMaterial(rgba, options = {}) {
  const [r, g, b, a] = rgba || [0.5, 0.75, 0.85, 1];
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(r, g, b),
    roughness: 0.62,
    metalness: 0.08,
    transparent: Boolean(a < 1 || options.transparent === true),
    opacity: options.opacity ?? a,
    depthWrite: options.depthWrite ?? true,
    side: THREE.DoubleSide
  });
}

export class RobotRenderer {
  constructor(container, callbacks = {}) {
    this.container = container;
    this.callbacks = callbacks;
    this.robot = null;
    this.assetMap = new Map();
    this.objectMap = new Map();
    this.selectables = [];
    this.selected = null;
    this.visualVisible = true;
    this.collisionVisible = false;
    this.framesVisible = false;
    this.axesVisible = false;
    this.comVisible = false;
    this.linkVisibility = new Map();
    this.viewportSettings = {
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
    this.transformMode = "select";
    this.draggedJointName = null;

    this.scene = new THREE.Scene();
    this.scene.matrixAutoUpdate = true;

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 500);
    this.camera.position.set(3.1, 2.4, 2.8);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.setClearColor(this.viewportSettings.background, 1);
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 0.55, 0);

    this.transform = new TransformControls(this.camera, this.renderer.domElement);
    this.transform.setSize(0.75);
    this.scene.add(this.transform.getHelper());
    this.transform.addEventListener("dragging-changed", event => {
      this.controls.enabled = !event.value;
      if (!event.value && this.draggedJointName) {
        this.commitTransform(this.draggedJointName);
      }
    });
    this.transform.addEventListener("objectChange", () => {
      if (this.draggedJointName) this.callbacks.onTransformPreview?.();
    });

    const hemi = new THREE.HemisphereLight(0xd9efff, 0x283038, 2.0);
    this.scene.add(hemi);

    this.keyLight = new THREE.DirectionalLight(0xffffff, 2.5);
    this.keyLight.position.set(3, 5, 4);
    this.keyLight.castShadow = true;
    this.keyLight.shadow.mapSize.set(2048, 2048);
    this.keyLight.shadow.bias = -0.0002;
    this.keyLight.shadow.normalBias = 0.002;
    this.keyLight.shadow.camera.near = 0.01;
    this.keyLight.shadow.camera.far = 50;

    this.keyLightTarget = new THREE.Object3D();
    this.scene.add(this.keyLightTarget);
    this.keyLight.target = this.keyLightTarget;

    this.scene.add(this.keyLight);

    this.floor = null;
    this.grid = null;
    this.rebuildFloorAndGrid();

    this.worldAxes = new THREE.AxesHelper(0.9);
    this.worldAxes.visible = this.viewportSettings.worldAxes;
    this.worldAxes.renderOrder = 1000;
    const axisMaterials = Array.isArray(this.worldAxes.material) ? this.worldAxes.material : [this.worldAxes.material];
    for (const material of axisMaterials) {
      material.depthTest = false;
      material.depthWrite = false;
      material.transparent = true;
      material.opacity = 0.95;
    }
    this.scene.add(this.worldAxes);

    this.robotRoot = new THREE.Group();
    this.robotRoot.matrix.copy(URDF_TO_THREE);
    this.robotRoot.matrixAutoUpdate = false;
    this.scene.add(this.robotRoot);

    this.assetPreviewRoot = new THREE.Group();
    this.assetPreviewRoot.matrix.copy(URDF_TO_THREE);
    this.assetPreviewRoot.matrixAutoUpdate = false;
    this.assetPreviewRoot.visible = false;
    this.previewingAsset = null;
    this.scene.add(this.assetPreviewRoot);

    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this.pointerDown = new THREE.Vector2();

    this.renderer.domElement.addEventListener("pointerdown", event => {
      this.pointerDown.set(event.clientX, event.clientY);
    });

    this.renderer.domElement.addEventListener("pointerup", event => {
      const dx = event.clientX - this.pointerDown.x;
      const dy = event.clientY - this.pointerDown.y;
      if (Math.hypot(dx, dy) < 4 && event.button === 0 && !this.transform.dragging) {
        this.pick(event);
      }
    });

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.container);
    this.resize();

    this.animate();
  }

  disposeObject(object) {
    object.traverse(child => {
      if (child.geometry) child.geometry.dispose?.();
      if (child.material) {
        const materials = Array.isArray(child.material) ? child.material : [child.material];
        for (const material of materials) material.dispose?.();
      }
    });
  }

  clearRobot() {
    this.transform.detach();
    this.draggedJointName = null;
    this.disposeObject(this.robotRoot);
    while (this.robotRoot.children.length) this.robotRoot.remove(this.robotRoot.children[0]);
    this.objectMap.clear();
    this.selectables = [];
  }

  revokeAssetURLs() {
    const urls = new Set();
    for (const previous of this.assetMap.values()) {
      if (previous.url) urls.add(previous.url);
    }
    for (const url of urls) URL.revokeObjectURL(url);
  }

  async textureSafeAssetURL(file) {
    if (!/\.tiff?$/i.test(file.name || "")) return URL.createObjectURL(file);

    try {
      const buffer = await file.arrayBuffer();
      const parsed = new TIFFLoader().parse(buffer);
      const width = Number(parsed.width);
      const height = Number(parsed.height);
      const data = parsed.data;

      if (!(width > 0 && height > 0 && data?.length)) throw new Error("TIFF decoder returned no pixels.");

      const canvas = document.createElement("canvas");
      canvas.width = width;
      canvas.height = height;
      const context = canvas.getContext("2d");
      if (!context) throw new Error("Canvas 2D context is unavailable.");

      let rgba;
      if (data.length === width * height * 4) {
        rgba = new Uint8ClampedArray(data);
      } else if (data.length === width * height * 3) {
        rgba = new Uint8ClampedArray(width * height * 4);
        for (let source = 0, target = 0; source < data.length; source += 3, target += 4) {
          rgba[target] = data[source];
          rgba[target + 1] = data[source + 1];
          rgba[target + 2] = data[source + 2];
          rgba[target + 3] = 255;
        }
      } else {
        throw new Error(`Unsupported TIFF pixel layout (${data.length} values for ${width}×${height}).`);
      }

      context.putImageData(new ImageData(rgba, width, height), 0, 0);
      const png = await new Promise((resolve, reject) => {
        canvas.toBlob(blob => blob ? resolve(blob) : reject(new Error("Could not convert TIFF to PNG.")), "image/png");
      });
      return URL.createObjectURL(png);
    } catch (error) {
      console.warn(`Could not decode TIFF asset "${file.name}".`, error);
      return URL.createObjectURL(file);
    }
  }

  async setAssets(files) {
    this.revokeAssetURLs();
    this.assetMap.clear();

    for (const file of files) {
      const url = await this.textureSafeAssetURL(file);
      const base = file.name.split(/[\\/]/).pop();
      const entry = { file, url };
      this.assetMap.set(file.name, entry);
      this.assetMap.set(base, entry);
    }
  }

  async mergeAssets(files) {
    for (const file of files) {
      const url = await this.textureSafeAssetURL(file);
      const base = file.name.split(/[\\/]/).pop();
      const entry = { file, url };
      this.assetMap.set(file.name, entry);
      this.assetMap.set(base, entry);
    }
  }

  resolveAsset(filename) {
    if (!filename) return null;

    let decoded = String(filename);
    try {
      decoded = decodeURIComponent(decoded);
    } catch {
    }

    const withoutQuery = decoded.split(/[?#]/, 1)[0];
    const normal = withoutQuery
      .replace(/^package:\/\/[^/]+\//, "")
      .replace(/^file:\/\//, "")
      .replace(/^\.\//, "");

    const base = normal.split(/[\\/]/).pop();

    return this.assetMap.get(decoded)
      || this.assetMap.get(withoutQuery)
      || this.assetMap.get(normal)
      || this.assetMap.get(base)
      || null;
  }

  makeAssetLoadingManager() {
    const manager = new THREE.LoadingManager();

    manager.setURLModifier(url => {
      const asset = this.resolveAsset(url);
      return asset?.url || url;
    });

    return manager;
  }

  resolveURDFMaterial(material) {
    if (!material) return { color: null, texture: "", name: "" };

    const globalMaterial = material.name
      ? this.robot?.materials?.get?.(material.name)
      : null;

    return {
      name: material.name || globalMaterial?.name || "",
      color: material.color || globalMaterial?.color || null,
      texture: material.texture || globalMaterial?.texture || ""
    };
  }

  async createDisplayMaterial(material, manager = null) {
    const resolved = this.resolveURDFMaterial(material);
    const rgba = resolved.color || [1, 1, 1, 1];
    const result = rgbaMaterial(rgba);

    if (!resolved.texture) return result;

    const textureAsset = this.resolveAsset(resolved.texture);
    if (!textureAsset) return result;

    try {
      const textureLoader = new THREE.TextureLoader(manager || this.makeAssetLoadingManager());
      const texture = await textureLoader.loadAsync(resolved.texture);
      texture.colorSpace = THREE.SRGBColorSpace;
      result.map = texture;
      result.color.setRGB(
        resolved.color?.[0] ?? 1,
        resolved.color?.[1] ?? 1,
        resolved.color?.[2] ?? 1
      );
      result.needsUpdate = true;
    } catch (error) {
      console.warn(`Could not load URDF texture "${resolved.texture}".`, error);
    }

    return result;
  }

  prepareEmbeddedDAEMaterials(object) {
    object.traverse(child => {
      if (!child.isMesh || !child.material) return;

      const materials = Array.isArray(child.material)
        ? child.material
        : [child.material];

      for (const material of materials) {
        if (material.map) {
          material.map.colorSpace = THREE.SRGBColorSpace;
          material.map.needsUpdate = true;
        }
        material.side = THREE.DoubleSide;
        material.needsUpdate = true;
      }
    });
  }

  shadowBoundsForObject(object) {
    if (!object) return null;

    object.updateWorldMatrix(true, true);

    const box = new THREE.Box3();
    let found = false;

    object.traverse(child => {
      if (!child.isMesh || child.userData.shadowEligible === false) return;
      if (!child.geometry) return;

      if (!child.geometry.boundingBox) {
        child.geometry.computeBoundingBox();
      }

      if (!child.geometry.boundingBox) return;

      const childBox = child.geometry.boundingBox.clone();
      childBox.applyMatrix4(child.matrixWorld);
      box.union(childBox);
      found = true;
    });

    return found && !box.isEmpty() ? box : null;
  }

  fitShadowCamera(object = this.previewingAsset ? this.assetPreviewRoot : this.robotRoot) {
    if (!this.keyLight || !this.keyLightTarget) return;

    const objectBox = this.shadowBoundsForObject(object);
    if (!objectBox) return;

    const fitBox = objectBox.clone();

    // A tightly fitted shadow camera must include the receiver area as well
    // as the objects casting the shadow. Project the object's bounding-box
    // corners along the directional-light rays onto the floor so long
    // shadows cannot be clipped at the edge of the shadow map.
    if (this.floor?.visible && this.floor.receiveShadow) {
      const min = objectBox.min;
      const max = objectBox.max;
      const corners = [
        new THREE.Vector3(min.x, min.y, min.z),
        new THREE.Vector3(min.x, min.y, max.z),
        new THREE.Vector3(min.x, max.y, min.z),
        new THREE.Vector3(min.x, max.y, max.z),
        new THREE.Vector3(max.x, min.y, min.z),
        new THREE.Vector3(max.x, min.y, max.z),
        new THREE.Vector3(max.x, max.y, min.z),
        new THREE.Vector3(max.x, max.y, max.z)
      ];

      const floorY = this.floor.position.y;
      const rayDirection = new THREE.Vector3(3, 5, 4)
        .normalize()
        .negate();

      if (Math.abs(rayDirection.y) > 1e-6) {
        for (const corner of corners) {
          const t = (floorY - corner.y) / rayDirection.y;
          if (t < 0) continue;

          fitBox.expandByPoint(
            corner.clone().addScaledVector(rayDirection, t)
          );
        }
      }
    }

    const centre = fitBox.getCenter(new THREE.Vector3());
    const size = fitBox.getSize(new THREE.Vector3());
    const radius = Math.max(size.length() * 0.5, 0.1);

    // The sphere enclosing fitBox remains inside these orthographic bounds
    // regardless of the directional light's orientation.
    const extent = radius * 1.15;
    const lightDirection = new THREE.Vector3(3, 5, 4).normalize();
    const lightDistance = Math.max(radius * 4, 4);

    this.keyLightTarget.position.copy(centre);
    this.keyLight.position.copy(centre).add(
      lightDirection.multiplyScalar(lightDistance)
    );

    const shadowCamera = this.keyLight.shadow.camera;
    shadowCamera.left = -extent;
    shadowCamera.right = extent;
    shadowCamera.top = extent;
    shadowCamera.bottom = -extent;
    shadowCamera.near = Math.max(0.01, lightDistance - radius * 1.75);
    shadowCamera.far = lightDistance + radius * 1.75;
    shadowCamera.updateProjectionMatrix();

    this.keyLight.shadow.normalBias = THREE.MathUtils.clamp(
      radius * 0.0015,
      0.0005,
      0.02
    );

    this.keyLightTarget.updateMatrixWorld(true);
    this.keyLight.updateMatrixWorld(true);
    this.keyLight.shadow.camera.updateMatrixWorld(true);
    this.renderer.shadowMap.needsUpdate = true;
  }

  async setRobot(robot, preserveCamera = true) {
    this.clearAssetPreview();
    this.robot = robot;

    for (const name of [...this.linkVisibility.keys()]) {
      if (!robot.links.has(name)) this.linkVisibility.delete(name);
    }
    for (const name of robot.links.keys()) {
      if (!this.linkVisibility.has(name)) this.linkVisibility.set(name, true);
    }

    this.clearRobot();

    for (const rootName of roots(robot)) {
      const linkGroup = await this.buildLinkBranch(rootName, new Set());
      this.robotRoot.add(linkGroup);
    }

    this.applyVisibility();
    this.applySelection();
    this.fitShadowCamera(this.robotRoot);

    if (!preserveCamera) this.frameRobot();
  }

  async buildLinkBranch(linkName, stack) {
    if (stack.has(linkName)) return new THREE.Group();
    stack = new Set(stack);
    stack.add(linkName);

    const link = this.robot.links.get(linkName);
    const group = new THREE.Group();
    group.name = `link:${linkName}`;
    group.userData.selection = { type: "link", name: linkName };
    this.objectMap.set(`link:${linkName}`, group);

    const frame = new THREE.AxesHelper(0.18);
    frame.name = "frame-helper";
    frame.userData.linkName = linkName;
    frame.visible = this.framesVisible;
    group.add(frame);

    if (link) {
      for (const visual of link.visuals || []) {
        const object = await this.makeGeometryObject(visual.geometry, visual.material, false);
        if (!object) continue;
        setOrigin(object, visual.origin);
        object.name = `visual:${linkName}`;
        object.userData.selection = { type: "link", name: linkName };
        object.userData.renderKind = "visual";
        object.userData.linkName = linkName;
        group.add(object);
        this.collectSelectable(object);
      }

      for (const collision of link.collisions || []) {
        const object = await this.makeGeometryObject(collision.geometry, null, true);
        if (!object) continue;
        setOrigin(object, collision.origin);
        object.name = `collision:${linkName}`;
        object.userData.selection = { type: "link", name: linkName };
        object.userData.renderKind = "collision";
        object.userData.linkName = linkName;
        group.add(object);
        this.collectSelectable(object);
      }

      if (link.inertial) {
        const com = new THREE.Group();
        setOrigin(com, link.inertial.origin);
        com.name = "com-helper";
        com.userData.renderKind = "com";
        com.userData.linkName = linkName;

        const sphere = new THREE.Mesh(
          new THREE.SphereGeometry(0.035, 18, 12),
          new THREE.MeshBasicMaterial({ color: 0xffe36f, depthTest: false })
        );
        com.add(sphere);

        const ring = new THREE.LineLoop(
          new THREE.BufferGeometry().setFromPoints(
            Array.from({ length: 32 }, (_, i) => {
              const a = i / 32 * Math.PI * 2;
              return new THREE.Vector3(Math.cos(a) * 0.07, Math.sin(a) * 0.07, 0);
            })
          ),
          new THREE.LineBasicMaterial({ color: 0xffe36f, depthTest: false })
        );
        com.add(ring);
        group.add(com);
      }
    }

    for (const joint of childJoints(this.robot, linkName)) {
      const originGroup = new THREE.Group();
      originGroup.name = `joint-origin:${joint.name}`;
      originGroup.userData.selection = { type: "joint", name: joint.name };
      setOrigin(originGroup, joint.origin);

      const motionGroup = new THREE.Group();
      motionGroup.name = `joint-motion:${joint.name}`;
      this.applyJointMotion(joint, motionGroup);

      const axisHelper = this.createJointAxis(joint);
      axisHelper.name = "joint-axis-helper";
      axisHelper.visible = this.axesVisible;
      originGroup.add(axisHelper);

      originGroup.add(motionGroup);
      group.add(originGroup);

      this.objectMap.set(`joint:${joint.name}`, originGroup);
      this.objectMap.set(`motion:${joint.name}`, motionGroup);

      const childGroup = await this.buildLinkBranch(joint.child, stack);
      motionGroup.add(childGroup);
    }

    return group;
  }

  collectSelectable(object) {
    object.traverse(child => {
      if (child.isMesh) {
        let owner = child;
        while (owner && !owner.userData.selection) owner = owner.parent;
        if (owner?.userData.selection) {
          child.userData.selection = owner.userData.selection;
          child.userData.linkName = owner.userData.linkName;
          child.userData.renderKind = owner.userData.renderKind;
          this.selectables.push(child);
        }
      }
    });
  }

  createJointAxis(joint) {
    const axis = new THREE.Vector3(...(joint.axis || [1, 0, 0])).normalize();
    const length = 0.34;
    const start = axis.clone().multiplyScalar(-length * 0.5);
    const end = axis.clone().multiplyScalar(length * 0.5);

    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([start, end]),
      new THREE.LineBasicMaterial({ color: 0xffca69, depthTest: false })
    );

    const arrow = new THREE.ArrowHelper(axis, start, length, 0xffca69, 0.08, 0.04);
    const group = new THREE.Group();
    group.add(line, arrow);
    return group;
  }

  resolveMaterialColor(material) {
    return this.resolveURDFMaterial(material).color || [0.48, 0.78, 0.88, 1];
  }

  async makeGeometryObject(geometry, material, collision) {
    const primitive = makeGeometry(geometry);
    const manager = this.makeAssetLoadingManager();
    const resolvedMaterial = this.resolveURDFMaterial(material);
    const mat = collision
      ? rgbaMaterial([1, 0.36, 0.16, 1], { transparent: true, opacity: 0.18, depthWrite: false })
      : await this.createDisplayMaterial(material, manager);

    if (primitive) {
      const mesh = new THREE.Mesh(primitive, mat);
      mesh.userData.shadowEligible = !collision;
      mesh.castShadow = !collision && this.viewportSettings.shadows;
      mesh.receiveShadow = !collision && this.viewportSettings.shadows;
      return mesh;
    }

    if (geometry?.type === "capsule") {
      const group = new THREE.Group();
      const radius = Math.max(0, Number(geometry.radius) || 0);
      const length = Math.max(0, Number(geometry.length) || 0);
      const cylinder = new THREE.CylinderGeometry(radius, radius, length, 36);
      cylinder.rotateX(Math.PI / 2);
      const body = new THREE.Mesh(cylinder, mat);
      const capGeometry = new THREE.SphereGeometry(radius, 32, 20);
      const capA = new THREE.Mesh(capGeometry, mat.clone());
      const capB = new THREE.Mesh(capGeometry.clone(), mat.clone());
      capA.position.z = length * 0.5;
      capB.position.z = -length * 0.5;
      for (const mesh of [body, capA, capB]) {
        mesh.userData.shadowEligible = !collision;
        mesh.castShadow = !collision && this.viewportSettings.shadows;
        mesh.receiveShadow = !collision && this.viewportSettings.shadows;
        group.add(mesh);
      }
      return group;
    }

    if (geometry?.type !== "mesh") return null;
    const asset = this.resolveAsset(geometry.filename);
    if (!asset) {
      return this.makeMissingMesh(geometry.filename);
    }

    try {
      const extension = asset.file.name.split(".").pop().toLowerCase();
      let object;

      if (extension === "stl") {
        const loader = new STLLoader();
        const loaded = await loader.loadAsync(asset.url);
        object = new THREE.Mesh(loaded, mat);
      } else if (extension === "obj") {
        const loader = new OBJLoader();
        object = await loader.loadAsync(asset.url);
        object.traverse(child => {
          if (child.isMesh) child.material = mat.clone();
        });
      } else if (extension === "dae") {
        const loader = new ColladaLoader(manager);
        const loaded = await loader.loadAsync(asset.url);
        object = loaded.scene;

        const hasURDFOverride = Boolean(
          resolvedMaterial.color || resolvedMaterial.texture
        );

        if (collision || hasURDFOverride) {
          object.traverse(child => {
            if (!child.isMesh) return;

            if (Array.isArray(child.material)) {
              child.material = child.material.map(() => mat.clone());
            } else {
              child.material = mat.clone();
            }
          });
        } else {
          this.prepareEmbeddedDAEMaterials(object);
        }
      } else {
        return this.makeMissingMesh(geometry.filename);
      }

      object.scale.set(...(geometry.scale || [1, 1, 1]));
      object.traverse(child => {
        if (child.isMesh) {
          child.userData.shadowEligible = !collision;
          child.castShadow = !collision && this.viewportSettings.shadows;
          child.receiveShadow = !collision && this.viewportSettings.shadows;
        }
      });
      return object;
    } catch (error) {
      console.error(error);
      return this.makeMissingMesh(geometry.filename);
    }
  }

  makeMissingMesh(filename) {
    const group = new THREE.Group();
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(0.25, 0.25, 0.25)),
      new THREE.LineBasicMaterial({ color: 0xff5d70 })
    );
    group.add(edges);
    group.userData.missingMesh = filename;
    return group;
  }

  async previewAsset(filename) {
    this.clearAssetPreview();
    const object = await this.makeGeometryObject(
      { type: "mesh", filename, scale: [1, 1, 1] },
      null,
      false
    );

    if (!object || object.userData.missingMesh) return false;

    this.assetPreviewRoot.add(object);
    this.assetPreviewRoot.visible = true;
    this.robotRoot.visible = false;
    this.previewingAsset = filename;
    this.fitShadowCamera(this.assetPreviewRoot);
    this.frameObject(this.assetPreviewRoot);
    return true;
  }

  clearAssetPreview(frameRobot = false) {
    if (!this.assetPreviewRoot) return;

    for (const child of [...this.assetPreviewRoot.children]) {
      this.disposeObject(child);
      this.assetPreviewRoot.remove(child);
    }

    this.assetPreviewRoot.visible = false;
    this.robotRoot.visible = true;
    this.previewingAsset = null;
    this.fitShadowCamera(this.robotRoot);
    if (frameRobot) this.frameRobot();
  }

  applyJointMotion(joint, motionGroup = this.objectMap.get(`motion:${joint.name}`)) {
    if (!motionGroup) return;
    motionGroup.position.set(0, 0, 0);
    motionGroup.quaternion.identity();

    const axis = new THREE.Vector3(...(joint.axis || [1, 0, 0]));
    if (axis.lengthSq() < 1e-12) axis.set(1, 0, 0);
    axis.normalize();

    if (joint.type === "revolute" || joint.type === "continuous") {
      motionGroup.quaternion.setFromAxisAngle(axis, Number(joint.value) || 0);
    } else if (joint.type === "prismatic") {
      motionGroup.position.copy(axis.multiplyScalar(Number(joint.value) || 0));
    }
  }

  updateJointValue(name, value) {
    const joint = this.robot?.joints.get(name);
    if (!joint) return;
    joint.value = Number(value) || 0;
    this.applyJointMotion(joint);
  }

  setSelection(selection) {
    this.selected = selection;
    this.applySelection();
  }

  applySelection() {
    this.transform.detach();
    this.draggedJointName = null;

    for (const mesh of this.selectables) {
      if (!mesh.material) continue;
      const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      for (const material of materials) {
        if (!material.userData.baseEmissive && "emissive" in material) {
          material.userData.baseEmissive = material.emissive.clone();
        }
        if ("emissive" in material) {
          const same = this.selected &&
            mesh.userData.selection?.type === this.selected.type &&
            mesh.userData.selection?.name === this.selected.name;
          material.emissive.copy(material.userData.baseEmissive || new THREE.Color(0));
          if (same) material.emissive.add(new THREE.Color(0x5a2c18));
        }
      }
    }

    if (this.selected?.type === "joint" && this.transformMode !== "select") {
      const object = this.objectMap.get(`joint:${this.selected.name}`);
      if (object) {
        this.draggedJointName = this.selected.name;
        this.transform.attach(object);
        this.transform.setMode(this.transformMode === "move" ? "translate" : "rotate");
        this.transform.setSpace("local");
      }
    }
  }

  setTransformMode(mode) {
    this.transformMode = mode;
    this.applySelection();
  }

  commitTransform(jointName) {
    const joint = this.robot?.joints.get(jointName);
    const object = this.objectMap.get(`joint:${jointName}`);
    if (!joint || !object) return;

    joint.origin.xyz = [object.position.x, object.position.y, object.position.z];
    const euler = new THREE.Euler().setFromQuaternion(object.quaternion, "XYZ");
    joint.origin.rpy = [euler.x, euler.y, euler.z];
    this.callbacks.onTransformCommit?.(jointName);
  }

  rebuildFloorAndGrid() {
    if (this.floor) {
      this.scene.remove(this.floor);
      this.floor.geometry?.dispose?.();
      this.floor.material?.dispose?.();
      this.floor = null;
    }

    if (this.grid) {
      this.scene.remove(this.grid);
      this.grid.geometry?.dispose?.();
      if (Array.isArray(this.grid.material)) {
        for (const material of this.grid.material) material.dispose?.();
      } else {
        this.grid.material?.dispose?.();
      }
      this.grid = null;
    }

    const size = Math.max(0.1, Number(this.viewportSettings.gridSize) || 20);
    const divisions = Math.max(1, Math.round(Number(this.viewportSettings.gridDivisions) || 40));

this.floor = new THREE.Mesh(
      new THREE.PlaneGeometry(size, size),
      new THREE.MeshStandardMaterial({
        color: new THREE.Color(this.viewportSettings.floorColour),
        roughness: 1,
        metalness: 0,
        transparent: true,
        opacity: 0.5,
        depthWrite: false,
        side: THREE.DoubleSide
      })
    );
    this.floor.rotation.x = -Math.PI / 2;
    this.floor.position.y = -0.002;
    this.floor.receiveShadow = Boolean(this.viewportSettings.shadows);
    this.floor.castShadow = false;
    this.floor.visible = Boolean(this.viewportSettings.floorVisible);
    this.floor.userData.viewportFloor = true;
    this.scene.add(this.floor);

    this.grid = new THREE.GridHelper(
      size,
      divisions,
      new THREE.Color(this.viewportSettings.gridCentre),
      new THREE.Color(this.viewportSettings.gridLines)
    );
    this.grid.position.y = 0;
    this.grid.material.transparent = true;
    this.grid.material.opacity = Math.min(1, Math.max(0, Number(this.viewportSettings.gridOpacity) || 0));
    this.grid.visible = Boolean(this.viewportSettings.gridVisible);
    this.grid.renderOrder = 2;
    this.scene.add(this.grid);
  }

  setViewportSettings(settings = {}) {
    this.viewportSettings = { ...this.viewportSettings, ...settings };

    this.renderer.setClearColor(this.viewportSettings.background, 1);

    const shadows = Boolean(this.viewportSettings.shadows);
    this.renderer.shadowMap.enabled = shadows;
    if (this.keyLight) this.keyLight.castShadow = shadows;

    for (const root of [this.robotRoot, this.assetPreviewRoot]) {
      root?.traverse(object => {
        if (!object.isMesh) return;
        const eligible = object.userData.shadowEligible !== false && object.userData.renderKind !== "collision";
        object.castShadow = shadows && eligible;
        object.receiveShadow = shadows && eligible;

        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of materials) {
          if (material) material.needsUpdate = true;
        }
      });
    }

    if (this.floor) {
      this.floor.receiveShadow = shadows;
      this.floor.visible = Boolean(this.viewportSettings.floorVisible);
      if (this.floor.material?.color) {
        this.floor.material.color.set(this.viewportSettings.floorColour);
        this.floor.material.needsUpdate = true;
      }
    }

    this.fitShadowCamera();

    this.renderer.shadowMap.needsUpdate = true;

    if (this.worldAxes) {
      this.worldAxes.visible = Boolean(this.viewportSettings.worldAxes);
    }

    this.rebuildFloorAndGrid();
  }

  setVisibility(kind, visible) {
    if (kind === "visual") this.visualVisible = visible;
    if (kind === "collision") this.collisionVisible = visible;
    if (kind === "frames") this.framesVisible = visible;
    if (kind === "axes") this.axesVisible = visible;
    if (kind === "com") this.comVisible = visible;
    this.applyVisibility();
  }

  isLinkVisible(name) {
    return this.linkVisibility.get(name) !== false;
  }

  setLinkVisibility(name, visible) {
    this.linkVisibility.set(name, Boolean(visible));
    this.applyVisibility();
  }

  applyVisibility() {
    this.robotRoot.traverse(object => {
      const linkVisible = object.userData.linkName
        ? this.isLinkVisible(object.userData.linkName)
        : true;

      if (object.userData.renderKind === "visual") object.visible = this.visualVisible && linkVisible;
      if (object.userData.renderKind === "collision") object.visible = this.collisionVisible && linkVisible;
      if (object.name === "frame-helper") object.visible = this.framesVisible && linkVisible;
      if (object.name === "joint-axis-helper") object.visible = this.axesVisible;
      if (object.name === "com-helper") object.visible = this.comVisible && linkVisible;
    });
  }
  pick(event) {
    if (this.previewingAsset) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointer, this.camera);

    const hits = this.raycaster.intersectObjects(this.selectables.filter(o => o.visible), false);
    if (!hits.length) {
      this.callbacks.onSelect?.(null);
      return;
    }

    const selection = hits[0].object.userData.selection;
    this.callbacks.onSelect?.(selection || null);
  }

  frameObject(object) {
    object.updateWorldMatrix(true, true);
    const box = new THREE.Box3().setFromObject(object);

    if (box.isEmpty()) {
      this.controls.target.set(0, 0, 0);
      this.camera.position.set(3, 2, 3);
      this.controls.update();
      return;
    }

    const size = box.getSize(new THREE.Vector3());
    const centre = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.length() * 0.58, 0.6);
    const direction = new THREE.Vector3(1.15, 0.8, 1.05).normalize();

    this.controls.target.copy(centre);
    this.camera.position.copy(centre).add(direction.multiplyScalar(radius * 2.2));
    this.camera.near = Math.max(radius / 1000, 0.001);
    this.camera.far = Math.max(radius * 100, 100);
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  frameRobot() {
    this.frameObject(this.previewingAsset ? this.assetPreviewRoot : this.robotRoot);
  }

  resize() {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
