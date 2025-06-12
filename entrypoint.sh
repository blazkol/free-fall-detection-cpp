#!/bin/bash

./build/detector/detector &
sleep 0.5
./build/simulator/simulator &

wait