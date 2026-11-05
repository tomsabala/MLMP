# The public face of this repository, as an image apps.tom-sabala.dev can run.
#
# WHAT IT SERVES. Three static surfaces under one origin: `site/` (the project
# overview) at the root, `docs/` (the committed-trace player) under `explorer/`
# and `web/dist` (the WebAssembly playground) under `live/`. There is no server
# behind any of them - the planner the playground runs is `mlmp.wasm`, executing
# in the visitor's browser - so this image is Caddy plus a directory.
#
# BUILD INPUT. `web/dist` must exist before `docker build`; it is the Emscripten
# build output, gitignored, and produced by `Dockerfile.wasm`'s `dist` target,
# which compiles OMPL under emsdk and takes minutes.
# `.github/workflows/apps-image.yml` builds it first and then builds this file.
# By hand:
#
#   docker build -f Dockerfile.wasm --target dist --output type=local,dest=web/dist .
#   docker build -f docker/app/Dockerfile -t mlmp-site:dev .
#   docker run --rm -p 8080:8080 mlmp-site:dev
#   curl -sI localhost:8080/a/mlmp/ | head -1
#
# A missing `web/dist` fails the COPY below rather than producing an image that
# serves 404s - but only because `docker/app/Dockerfile.dockerignore` exists:
# the repo-root `.dockerignore` excludes `web/dist/`, and BuildKit would have
# copied nothing at all, silently.

# The caddy binary, stripped of its file capability. NOT plain
# `FROM caddy:2.10-alpine`: that image ships `/usr/bin/caddy` with
# `cap_net_bind_service=ep` so a root-less process can bind port 80, and a
# process whose bounding set is empty cannot exec a file that carries permitted
# capabilities. The broker runs every instance with `CapDrop: ALL`, so this is
# not a corner case - it is how this image will always be started, and the
# failure mode is `exec /usr/bin/caddy: operation not permitted`, container
# unhealthy, every request refused.
#
# `COPY --from` PRESERVES the xattr; `cp` inside the stage does not, so one copy
# here is the whole fix. Nothing in this image needs the capability: the server
# listens on 8080.
FROM caddy:2.10-alpine AS caddybin
RUN cp /usr/bin/caddy /caddy

FROM alpine:3.21
COPY --from=caddybin /caddy /usr/bin/caddy

# Caddy's own image runs as root and writes to /data and /config; both are
# pointed at /tmp and the user is dropped, so this container needs no capability
# and no writable layer of its own. `admin off` and `persist_config off` in the
# Caddyfile are the other half of that.
ENV XDG_CONFIG_HOME=/tmp/caddy-config \
    XDG_DATA_HOME=/tmp/caddy-data

COPY docker/app/Caddyfile /etc/caddy/Caddyfile
COPY site/      /srv/site/
COPY docs/      /srv/site/explorer/
COPY web/dist/  /srv/site/live/

USER 1000:1000

EXPOSE 8080

# The broker waits on the image's own HEALTHCHECK before it probes `readyPath`,
# so an image without one is an instance the broker cannot tell apart from a
# hung one. The interval is the cold start a visitor waits, not a background
# cost; Caddy over a directory of static files has nothing to warm up, so the
# period is short and the retries carry the tolerance.
HEALTHCHECK --interval=2s --timeout=2s --start-period=1s --retries=5 \
  CMD wget -q -O /dev/null http://127.0.0.1:8080/healthz || exit 1

# The title is what distinguishes this artifact from the native CLI image in the
# GHCR package UI: both are tags of ghcr.io/tomsabala/mlmp.
LABEL org.opencontainers.image.title="MLMP — hosted site" \
      org.opencontainers.image.description="Trace Explorer and in-browser WebAssembly planner for MLMP bundle-space motion planning" \
      org.opencontainers.image.source="https://github.com/tomsabala/MLMP" \
      org.opencontainers.image.licenses="Apache-2.0"

CMD ["caddy", "run", "--config", "/etc/caddy/Caddyfile", "--adapter", "caddyfile"]
