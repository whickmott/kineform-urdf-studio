const DB_NAME = "urdf-studio";
const DB_VERSION = 1;
const STORE_NAME = "mesh-library";

function openDatabase() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open(DB_NAME, DB_VERSION);

    request.onupgradeneeded = () => {
      const db = request.result;
      if (!db.objectStoreNames.contains(STORE_NAME)) {
        db.createObjectStore(STORE_NAME, { keyPath: "name" });
      }
    };

    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}

function transactionPromise(transaction) {
  return new Promise((resolve, reject) => {
    transaction.oncomplete = () => resolve();
    transaction.onerror = () => reject(transaction.error);
    transaction.onabort = () => reject(transaction.error);
  });
}

export async function saveMeshFiles(files) {
  if (!files.length) return;
  const db = await openDatabase();
  const transaction = db.transaction(STORE_NAME, "readwrite");
  const done = transactionPromise(transaction);
  const store = transaction.objectStore(STORE_NAME);

  for (const file of files) {
    store.put({
      name: file.name,
      type: file.type || "application/octet-stream",
      size: file.size,
      lastModified: file.lastModified || Date.now(),
      blob: file
    });
  }

  await done;
  db.close();
}

export async function loadMeshFiles() {
  const db = await openDatabase();
  const transaction = db.transaction(STORE_NAME, "readonly");
  const done = transactionPromise(transaction);
  const request = transaction.objectStore(STORE_NAME).getAll();

  const records = await new Promise((resolve, reject) => {
    request.onsuccess = () => resolve(request.result || []);
    request.onerror = () => reject(request.error);
  });

  await done;
  db.close();

  return records
    .sort((a, b) => a.name.localeCompare(b.name))
    .map(record => new File([record.blob], record.name, {
      type: record.type,
      lastModified: record.lastModified
    }));
}

export async function removeMeshFile(name) {
  const db = await openDatabase();
  const transaction = db.transaction(STORE_NAME, "readwrite");
  const done = transactionPromise(transaction);
  transaction.objectStore(STORE_NAME).delete(name);
  await done;
  db.close();
}

export async function clearMeshFiles() {
  const db = await openDatabase();
  const transaction = db.transaction(STORE_NAME, "readwrite");
  const done = transactionPromise(transaction);
  transaction.objectStore(STORE_NAME).clear();
  await done;
  db.close();
}
