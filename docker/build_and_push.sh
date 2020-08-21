#!/bin/bash

docker build -f Dockerfile_trl -t terrainrl:latest .
docker tag terrainrl:latest gberseth/terrainrl:latest
docker push gberseth/terrainrl:latest

