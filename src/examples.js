const EXAMPLE_INDEX_URL = "./assets/examples/index.json";

let examples = [];
let examplesById = new Map();

function resolveExampleURL(path, baseURL) {
  if (!path || typeof path !== "string") return "";
  return new URL(path, baseURL).href;
}

function normaliseExample(raw, baseURL, index) {
  if (!raw || typeof raw !== "object") {
    throw new Error(`Example entry ${index + 1} is not an object.`);
  }

  const id = String(raw.id || "").trim();
  const name = String(raw.name || "").trim();
  const urdf = String(raw.urdf || "").trim();

  if (!id) throw new Error(`Example entry ${index + 1} has no id.`);
  if (!/^[a-z0-9][a-z0-9._-]*$/i.test(id)) {
    throw new Error(`Example id "${id}" contains unsupported characters.`);
  }
  if (!name) throw new Error(`Example "${id}" has no name.`);
  if (!urdf) throw new Error(`Example "${id}" has no URDF path.`);

  const assets = Array.isArray(raw.assets)
    ? raw.assets.map(path => resolveExampleURL(path, baseURL))
    : [];

  return {
    id,
    name,
    description: String(raw.description || "").trim(),
    urdf: resolveExampleURL(urdf, baseURL),
    assets
  };
}

function freshRequestURL(url) {
  const requestURL = new URL(url, document.baseURI);
  requestURL.searchParams.set("_kf", `${Date.now()}`);
  return requestURL.href;
}

export async function loadExampleRegistry(url = EXAMPLE_INDEX_URL) {
  const response = await fetch(freshRequestURL(url), {
    cache: "no-store"
  });

  if (!response.ok) {
    throw new Error(`Could not load examples index (${response.status}).`);
  }

  const manifest = await response.json();
  const rawExamples = Array.isArray(manifest)
    ? manifest
    : manifest?.examples;

  if (!Array.isArray(rawExamples)) {
    throw new Error("Examples index must contain an examples array.");
  }

  const baseURL = new URL(url, document.baseURI);
  const nextExamples = rawExamples.map((raw, index) =>
    normaliseExample(raw, baseURL, index)
  );

  const nextById = new Map();
  for (const example of nextExamples) {
    if (nextById.has(example.id)) {
      throw new Error(`Duplicate example id "${example.id}".`);
    }
    nextById.set(example.id, example);
  }

  examples = nextExamples;
  examplesById = nextById;

  return [...examples];
}

export function allExamples() {
  return [...examples];
}

export function exampleById(id) {
  return examplesById.get(id) || null;
}
