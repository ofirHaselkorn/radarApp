[app]

title = LD2410C Tactical
package.name = ld2410tactical
package.domain = org.tactical

source.dir = .
source.include_exts = py,kv,xml,png,jpg,atlas
# test_*.py: never needed at runtime. inspect_serial.py/configure_sensor.py:
# standalone CLI tools meant to be run directly from a desktop terminal with
# --port args, not useful bundled behind main.py's single entry point.
source.exclude_patterns = test_*.py,inspect_serial.py,configure_sensor.py,README.md,bin/*,.buildozer/*,logs/*

version = 1.0

# usb4a/usbserial4a are the Android USB-serial stack (CH340, CP210x, FTDI,
# CDC-ACM); pyjnius is what they call into the Android framework with.
#
# p4a.branch pins python-for-android to the release contemporary with
# Kivy 2.3.0 (~Jan 2024), not whatever buildozer's default "master" resolves
# to today. This matters a lot: building against today's master (a 2026
# checkout) with Kivy 2.3.0/Python 3.11 hit a stack of real, unrelated
# native-toolchain incompatibilities one at a time -- Cython 0.29 vs Python
# 3.14's changed C API, a GCC-only compiler flag libffi's configure sends
# that every Clang version rejects, Android's Bionic libc missing the
# POSIX group-enumeration functions CPython's grp module assumes exist, and
# newer Clang treating an OpenGL function-pointer mismatch in Kivy's own
# cgl_gl.c as a hard error instead of a warning. Each of those is a real
# version-skew bug, not a config mistake -- pinning the whole toolchain to
# one era avoids hitting all of them by building against the release that
# was actually tested with this Kivy version, the same reasoning as the
# python3==3.11.9 pin below.
p4a.branch = v2024.01.21

# python3==3.11.9 is pinned for the same reason: Kivy 2.3.0's Cython 0.29
# extensions need CPython internal APIs that 3.13+ changed/removed.
#
# hostpython3 must be pinned to the SAME version explicitly: p4a asserts
# python3's version equals hostpython3's, but hostpython3 resolves its own
# default independently -- pinning only python3 left hostpython3 at its
# own default and the build refused to start ("python3 should have same
# version as hostpython3, 3.11.9 != <hostpython3's own default>").
requirements = python3==3.11.9,hostpython3==3.11.9,kivy==2.3.0,pyjnius,usb4a,usbserial4a

orientation = portrait
fullscreen = 0

# android.features (a <uses-feature> manifest entry, mainly for Play Store
# device filtering) is deliberately NOT set here: buildozer 1.6.0's CLI
# glue for it passes a --feature flag that p4a v2024.01.21 (pinned above)
# doesn't recognise ("unrecognized arguments: --feature ..."), a version-
# interface mismatch between the two pinned tools. It only matters for
# store distribution anyway -- this APK is sideloaded, not store-listed,
# and usb4a's actual USB host calls work the same with or without the
# manifest declaration. Runtime access is still gated normally, by the
# per-device permission dialog usb4a raises on first connect.
android.wakelock = True

android.api = 34
android.minapi = 21
android.ndk_api = 21
# NDK 25b: the version contemporary with p4a.branch above, not whatever the
# newest NDK happens to be today.
android.ndk = 25b
# arm64-v8a only: virtually every Android phone from the last ~8-10 years
# is arm64 (64-bit ARM); armeabi-v7a (32-bit) only matters for genuinely
# old hardware. Single-arch roughly halves build time for this app's actual
# target device. (A "jnius.c not found" error during a dual-arch attempt
# turned out to be unrelated/non-fatal noise, not an architecture bug --
# this choice is purely for build speed and matching the real target.)
android.archs = arm64-v8a
android.allow_backup = True

# Launch the app when the adapter is plugged in, and declare which bridges we
# handle. Both files live next to this spec.
android.manifest.intent_filters = usb_intent_filter.xml
# Needs buildozer >= 1.5. On older versions, copy device_filter.xml into
# .buildozer/.../res/xml/ by hand (see README).
android.add_resources = device_filter.xml:xml/device_filter.xml

[buildozer]

log_level = 2
warn_on_root = 1
