#!/bin/sh
# Build and run the AHRS host tests, ESP32-S3 float check, and benchmark.
set -e
cd "$(dirname "$0")"
make -s all ahrs_s3.o
make -s s3check
./test_ahrs
./bench_ahrs
./opcount_host
