# OmindOS authenticated cloud preview

This adapter exposes the existing workbench through an authenticated HTTPS subpath. It is a **single-operator geometric simulation preview**, not a customer account system or hardware controller.

## Scope

- Existing navigation and dual-arm planning run on the server; the browser renders results.
- All application routes, model assets, and APIs require HTTP Basic authentication.
- The service binds to 127.0.0.1:18885. Only the HTTPS reverse proxy is exposed.
- POST origins must match https://omindos.cn. Authentication credentials are never committed.
- Root-relative fetches, script imports, and assets are scoped to /_cloud-preview/.
- Navigation waits for the current model's assets before starting. Existing motion locks, cancellation, and connection watchdogs remain active.
- State is shared by the preview operator. Do not distribute these credentials to multiple customers.
- There is no ROS runtime URL, live hardware connection, dynamics validation, customer registration, quotas, or billing.

## Runtime

Use an existing Python environment with the workbench dependencies (including NumPy and SciPy). Set:

| Variable | Meaning |
| --- | --- |
| OMINDOS_SOURCE | Read-only checkout containing scripts, config and web |
| OMINDOS_MODEL_BUNDLE | Read-only approved RM75 model bundle |
| OMINDOS_PREVIEW_AUTH_FILE | JSON file containing username and SHA-256 password hash |
| OMINDOS_PREVIEW_STORE | Separate writable state directory (default /data) |
| OMINDOS_PREVIEW_PORT | Loopback port (default 18885) |

Generate a unique high-entropy password outside the repository. The auth file has this shape:
`{"username":"preview","sha256":"<SHA-256 of UTF-8 password>"}`

Keep the actual password in an owner-only runtime secret file. The browser test expects `<preview root>/secrets/access.json` with username and password. Never include either secret file in source control.

Place cloud_preview.py, preview-bootstrap.js and navigation-ready.js together. Run cloud_preview.py with the existing Python environment. The deployed preview reuses an existing container image rather than installing another environment. It has a read-only root filesystem and source/model mounts, a separate writable state mount, 1 CPU, 768 MiB RAM, 128 PIDs, a 32 MiB temporary directory, all Linux capabilities dropped, no-new-privileges, and bounded logs. The existing source permissions require container UID 0; source mounts remain read-only.

The current adapter pins its navigation insertion point; incompatible source returns 503 rather than silently omitting the asset readiness guard. Update and verify the adapter when changing the upstream workbench.

## Reverse proxy and rollback

nginx.conf is a location snippet for the existing omindos.cn HTTPS server. It is not a replacement site configuration.

1. Verify authenticated loopback responses before exposing the proxy.
2. Back up the existing site configuration and record the homepage content hash.
3. Include the snippet in the HTTPS server, validate nginx configuration, then reload.
4. Verify unauthenticated HTTPS requests return 401 and the homepage hash is unchanged.
5. To withdraw the preview, remove only its include, validate and reload nginx, then stop its container. Preserve the data and backup for investigation.

No DNS change or public website navigation entry is required. The eventual cloud.omindos.ai customer deployment remains separate work.

## Validation

`check-preview.cjs` uses Playwright and a real Chrome WebGL renderer. Set OMINDOS_PLAYWRIGHT to the installed module if needed and OMINDOS_PREVIEW_ROOT to the runtime directory. It maps the existing HTTPS host to loopback while retaining TLS verification and authenticates with the runtime secret.

The check drives normal controls for navigation, loads all 20 approved CAD visuals, runs the coupled dual-arm planner and playback, and checks a 390 px mobile viewport. No product state or planning result is injected. Superseded browser fetches may be aborted during model changes; active CAD mesh loading must still finish without failures.

Separate HTTP checks must cover 401 without credentials, 403 for an untrusted Origin, 404 for non-asset filesystem paths, and unchanged production homepage content.

Preview acceptance does not establish customer isolation, concurrency capacity, production availability, GPU throughput, dynamics accuracy, or hardware safety.
