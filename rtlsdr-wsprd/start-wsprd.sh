#!/bin/sh
set -eu

ORIG_ARGS="$*"

WSPR_BAND="${WSPR_BAND:-20m}"
WSPR_CALLSIGN="${WSPR_CALLSIGN:-N0CALL}"
WSPR_LOCATOR="${WSPR_LOCATOR:-AA00aa}"
WSPR_GAIN="${WSPR_GAIN:-32}"
WSPR_PPM="${WSPR_PPM:-0}"
WSPR_DEVICE_INDEX="${WSPR_DEVICE_INDEX:-0}"
WSPR_RF_AMP="${WSPR_RF_AMP:-}"
WSPR_IF_GAIN="${WSPR_IF_GAIN:-}"
WSPR_BASEBAND_GAIN="${WSPR_BASEBAND_GAIN:-}"

BASE_ARGS="-c $WSPR_CALLSIGN -l $WSPR_LOCATOR -g $WSPR_GAIN -p $WSPR_PPM -i $WSPR_DEVICE_INDEX"

if [ -n "$WSPR_RF_AMP" ]; then
  BASE_ARGS="$BASE_ARGS -A $WSPR_RF_AMP"
fi

if [ -n "$WSPR_IF_GAIN" ]; then
  BASE_ARGS="$BASE_ARGS -L $WSPR_IF_GAIN"
fi

if [ -n "$WSPR_BASEBAND_GAIN" ]; then
  BASE_ARGS="$BASE_ARGS -V $WSPR_BASEBAND_GAIN"
fi

if [ -n "$ORIG_ARGS" ]; then
  # shellcheck disable=SC2086
  exec /usr/local/bin/rtlsdr_wsprd -f "$WSPR_BAND" $BASE_ARGS $ORIG_ARGS
fi

# shellcheck disable=SC2086
exec /usr/local/bin/rtlsdr_wsprd -f "$WSPR_BAND" $BASE_ARGS
