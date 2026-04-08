ARG IMAGE
FROM ${IMAGE}

# run original entrypoint to create dockeruser in image
ARG DOCKER_UID
ARG DOCKER_GID
ENV DOCKER_UID=${DOCKER_UID}
ENV DOCKER_GID=${DOCKER_GID}
RUN bash /entrypoint.sh whoami

# install pre-commit via pip while keeping distro packages untouched
RUN if command -v python3 >/dev/null 2>&1 && python3 -m pip --version >/dev/null 2>&1; then \
        python3 -m pip install --no-cache-dir --ignore-installed pre-commit; \
    fi
