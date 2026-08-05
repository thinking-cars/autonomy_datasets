# A) Create test data image
#   docker build -f ./Dockerfile -t ghcr.io/thinking-cars/autonomy_datasets_test:latest ${DATASET_DIR}
#   docker push ghcr.io/thinking-cars/autonomy_datasets_test:latest
#
# B) Download test data
#   docker run -it --rm --name autonomy_data ghcr.io/thinking-cars/autonomy_datasets_test:latest
#   docker cp autonomy_data:/dataset/. ${DATASET_DIR}

FROM alpine:3.24

COPY . /dataset

WORKDIR /dataset
