/**
 *   Shim header for StrokeEngine pattern classes
 *   Provides the minimal types, constants, and framework includes that the
 *   vendored pattern.h and PatternMath.h need, WITHOUT pulling in
 *   StrokeEngine.h (which owns motor/homing/task logic).
 *
 *   This file is SlopDrive-32 machinery. Upstream pattern code lives in
 *   pattern.h / PatternMath.h — those must remain byte-identical to OSSM-hardware.
 *
 *   Copyright (C) the respective upstream authors.
 *   Vendored from: KinkyMakers/OSSM-hardware (main branch)
 *   License: MIT (see upstream pattern.h header)
 */

#pragma once

#include <Arduino.h>
#include <math.h>

/* --------------------------------------------------------------------------
 * STRING_LEN — byte size for pattern name char arrays
 * Defined identically to upstream pattern.h to keep everything consistent.
 * -------------------------------------------------------------------------- */
#ifndef STRING_LEN
#define STRING_LEN 64
#endif

/* --------------------------------------------------------------------------
 * motionParameter — the struct patterns return for every stroke target.
 * Copied verbatim from the upstream pattern.h.  The struct uses `int` for
 * stroke/speed/acceleration because OSSM works in stepper *steps*; SlopDrive
 * will reinterpret these at the PatternEngine layer.
 * -------------------------------------------------------------------------- */
typedef struct {
    int stroke;         //!< Absolute target position in steps
    int speed;          //!< Speed in steps/second
    int acceleration;   //!< Acceleration in steps/second²
    bool skip;          //!< No valid stroke this cycle — allows pauses
} motionParameter;