# Offline browser viewer

`viewer.bundle.js` is a committed, self-contained ES module built from `viewer.js`.
No CDN, telemetry, remote fonts or runtime npm installation is used.
All network requests made by this viewer load its local URDF and mesh assets.

Dependencies are pinned in `package.json` and `pnpm-lock.yaml`:

- Three.js: rendering, STL loading, camera navigation (MIT).
- urdf-loader: URDF transforms, joint limits and mimic propagation (Apache-2.0).
- esbuild: development-time bundler; not required on the AGX.

Original dependency licenses are preserved in `LICENSES.txt`.
The source viewer is an offline preview tool, not a robot control interface.

To rebuild after editing the JavaScript, use pnpm in this directory:

```sh
pnpm install --frozen-lockfile
pnpm run build
```

From the project root, `python3 tools/description/serve_preview.py` exports the
models and starts a server bound only to `127.0.0.1`. Open its printed URL in a
WebGL-capable browser on the same computer.
