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

# add raw dataset files excluding non-relevant files
COPY --exclude=*/bags \
     --exclude=fzi_aura/scenes/* \
     --exclude=!fzi_aura/scenes/2025-06-13-07-09-37_75 \
     --exclude=nuscenes/sweeps \
     --exclude=nuscenes/can_bus/* \
     --exclude=!nuscenes/can_bus/scene-0103* \
     --exclude=!nuscenes/can_bus/scene-0916* \
     --exclude=nuscenes/detection-megvii/megvii_test.json \
     --exclude=nuscenes/detection-megvii/megvii_train.json \
     . /dataset

WORKDIR /dataset
