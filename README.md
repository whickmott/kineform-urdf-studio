# KineForm / URDF Studio

KineForm URDF Studio is a browser-based workspace for creating, inspecting, editing and validating URDF robot models.

Live application: https://whickmott.github.io/urdf/

## Features

- URDF 1.0, 1.1 and 1.2 support
- Visual and collision geometry editing
- STL, OBJ and DAE mesh support
- Texture support including PNG, JPEG, WebP, GIF, BMP and TIFF
- Joint editing and live articulation
- Mimic and dynamics support
- Materials manager
- ROS 2 Control workspace
- URDF Doctor validation
- Transactional XML editing
- Undo and redo
- Animation timeline
- Configurable 3D viewport
- Browser-local asset library
- Manifest-driven bundled examples

## Structure

```text
index.html
styles.css
src/
assets/
```

The application is static and uses relative paths, so the repository itself is served at `/urdf/` on the KineForm website.

## Contributing

Issues and pull requests are welcome.

For changes to the application, keep the existing static structure and avoid committing generated build output or browser-local data.

Bundled examples are registered in:

```text
assets/examples/index.json
```

## Author

William Hickmott  
https://whickmott.github.io/
