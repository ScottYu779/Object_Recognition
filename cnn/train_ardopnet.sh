#!/usr/bin/env sh
set -e

~/caffe/build/tools/caffe train --solver=ardopnet_solver.prototxt $@ | tee log/ardop_net.log
