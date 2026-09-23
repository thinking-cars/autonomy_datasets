# --- FOR INTERNAL USE ONLY ---
#
# A) Create test data image
#   docker build -f ./Dockerfile -t ghcr.io/thinking-cars/autonomy_datasets_test:latest ${DATASET_DIR}
#   docker push ghcr.io/thinking-cars/autonomy_datasets_test:latest
#
# B) Download test data
#   docker run -itd --rm --name autonomy_data --pull always ghcr.io/thinking-cars/autonomy_datasets_test:latest
#   docker cp autonomy_data:/dataset/. ${DATASET_DIR}
#   docker stop autonomy_data

FROM alpine:3.24

# exclude converted rosbags and hosted datasets
COPY --exclude=*/bags --exclude=nvidia_physicalai_av_dataset/* --exclude=fzi_aura/* . /dataset

WORKDIR /dataset
