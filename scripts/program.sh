#!/usr/bin/env bash
# program.sh — full build + flash for ECP5-5G eval board
# Usage: ./program.sh
# Requires: oss-cad-suite at /opt/oss-cad-suite, openFPGALoader, USB JTAG connected

set -e

OSS_CAD=/opt/oss-cad-suite/bin

echo "==> Synthesis (Yosys)..."
$OSS_CAD/yosys -p "read_verilog -sv fpga/top.v fpga/ADC_Model_TB.v fpga/UWB_Serial_Handler.v; synth_ecp5 -top top -json top.json"

echo "==> Place and route (nextpnr-ecp5)..."
$OSS_CAD/nextpnr-ecp5 \
    --um5g-85k \
    --package CABGA381 \
    --speed 8 \
    --lpf fpga/clock.lpf \
    --json top.json \
    --textcfg top.cfg

echo "==> Pack bitstream (ecppack)..."
$OSS_CAD/ecppack --compress top.cfg top.bit

echo "==> Programming (openFPGALoader)..."
$OSS_CAD/openFPGALoader -b ecp5_evn top.bit

echo "==> Done."
