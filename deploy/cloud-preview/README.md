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

## Navigation stability follow-up, 2026-10-12 Beijing time

The live adapter has evolved since this branch: `/_cloud-preview/` now redirects
to `https://cloud.omindos.cn/`, with anonymous experience and authenticated
configuration. Do not overwrite that live app with this branch's older adapter.
The navigation fix is staged separately at
`https://cloud.omindos.cn/_navigation-preview/`, using the existing authorized
operator login, separate runtime/data, and unchanged public root.

`prepare-navigation-preview.py` makes the scoped changes to an isolated copy of
the verified live adapter. It checks all four source hashes before writing,
requires different source/output directories, gates assets and APIs before
dispatch, and isolates Cookie names, local storage and PWA caches. Its packed
mesh route requires the approved read-only `/packed` mount. The candidate keeps
the existing layout, rendering, form login, CSRF checks and origin validation.
`navigation-preview.conf` is a scoped location snippet, never a full nginx site.
Record backups, validate nginx, and wait for candidate loopback readiness after
restart before accepting HTTPS checks. To withdraw it, remove only that include
and stop the candidate container; the existing public container/data stay intact.

Navigation uses `GET /v1/navigation-heartbeat/<task_id>` for a lightweight
500 ms lease heartbeat, independent of full pose/result reads. The server's
three-second disconnect limit remains unchanged; late reads cannot revive an
expired task. Navigation reads have bounded deadlines and one transient retry;
state-changing POSTs are never automatically replayed. Cancellation aborts
pending reads, and ticket/revision checks reject superseded results. The model's
15-second readiness limit precedes task creation; a failed asset rejects start.

Run sequentially against the isolated preview, with approved credentials in a
runtime-only file; never include request headers in the report:

```bash
export OMINDOS_PLAYWRIGHT=/path/to/node_modules/playwright
export CHECK_PREVIEW_URL=https://cloud.omindos.cn/_navigation-preview/
export CHECK_AUTH_FILE=/private/runtime/access.json
CHECK_OUTPUT=/tmp/nav-viewports node deploy/cloud-preview/check-navigation-stability.cjs
CHECK_OUTPUT=/tmp/nav-faults node deploy/cloud-preview/check-navigation-stability.cjs --faults
CHECK_OUTPUT=/tmp/nav-assets node deploy/cloud-preview/check-navigation-assets.cjs
CHECK_OUTPUT=/tmp/nav-access node deploy/cloud-preview/check-navigation-access.cjs
```

The fault suite uses real backend responses with controlled delay/loss, tests
GET retry without duplicate start, cancellation, estop/reset, planner rejection
and browser network loss, including an HTML HTTP503 proxy response. The access
suite uses form login and inherited browser settings/cookies/cache to verify
the preview scope. The asset suite delays approved mesh requests or
makes both packed and fallback geometry unavailable; it never injects poses or
fabricates planner success. `CHECK_BACKEND_URL` is only needed for a loopback TLS
test whose Chrome host mapping does not apply to Node's `route.fetch`.

See `docs/validation/cloud-navigation-stability-20261012.md` for actual results,
deployment hashes, limitations and conditions for a multi-user Cloud release.

## Public navigation release, 2026-10-12 Beijing time

PR #25 and its dependency PR #24 are merged. The navigation fix is now live at
`https://cloud.omindos.cn/`; the protected candidate remains separate. Production
keeps its newer guest/user adapter. Only the three navigation core files,
`navigation-ready.js`, and a one-line guest heartbeat allowlist are promoted.
The older adapter snapshot above must not replace the current public app.

`release-navigation.py` is pinned to the verified source hashes and current
server paths. `--prepare` checks drift and creates before/after backups;
`--apply` rechecks hashes, stops the container, atomically replaces the scoped
files, starts it and verifies the existing root and guest heartbeat route.
An apply failure restores the original files. `--rollback` requires the same
manifest and refuses to overwrite later changes. Authentication, user data,
layout, model/media bundles, nginx and domains are outside its write scope.

Public acceptance uses fresh anonymous guest contexts. Unset CHECK_AUTH_FILE
for the navigation, fault and asset suites. `check-navigation-public-access.cjs`
uses separate guests plus the existing form login to verify task visibility,
estop isolation, origin checks and configuration authentication; supply the
existing runtime-only CHECK_AUTH_FILE to this access suite. The private-only
`check-navigation-access.cjs` checks different cookie/storage/cache names and
must not be run against the public root.

The public deployment, actual request evidence, screenshots, backup and remaining
multi-customer requirements are recorded in
`docs/validation/cloud-navigation-release-20261012.md`.
