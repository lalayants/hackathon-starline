#!/usr/bin/env bash

# Written by Nikolay Dema <ndema2301@gmail.com>, September 2022

KOB_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

IMG_NAME="hsl_2022_kobuki_sol"
BASE_IMG="nickodema/hsl_2022_kobuki"


### DOCKER BUILD --------------------------------------------------------- #

printf "\n BUILDING DOCKER IMAGE: ${IMG_NAME}"
printf "\n                  FROM: ${BASE_IMG}\n\n"

docker build -t "${IMG_NAME}" \
             -f $KOB_ROOT/docker/Dockerfile $KOB_ROOT \
            --network=host \
            --build-arg base_img=${BASE_IMG} \
            --build-arg workspace=${KOB_ROOT}/workspace
