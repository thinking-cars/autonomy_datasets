ARG IMAGE
FROM ${IMAGE}

# run original entrypoint to create dockeruser in image
ARG DOCKER_UID
ARG DOCKER_GID
ENV DOCKER_UID=${DOCKER_UID}
ENV DOCKER_GID=${DOCKER_GID}
RUN bash /entrypoint.sh whoami
