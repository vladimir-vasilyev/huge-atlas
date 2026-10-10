#!/usr/bin/env bash
# Builds a Release version of the program, wraps it into huge-atlas.app and packs a disk image for users who do not build
# anything: dist/huge-atlas-<version>-<arch>.dmg (open it, drag the app to Applications).
#   ./scripts/make_dmg.sh            build + app + dmg
#   SKIP_BUILD=1 ./scripts/make_dmg.sh   use the existing build-xcode/Release/vsa_app
# The app is signed ad hoc (no Apple Developer ID): see the README for the first start. With a Developer ID set
# CODESIGN_IDENTITY="Developer ID Application: ..." and (for notarization) run notarytool on the dmg yourself.
set -euo pipefail
cd "$(dirname "$0")/.."

VERSION="$(tr -d '[:space:]' < VERSION)"
ARCH="$(uname -m)"
BUILD=build-xcode
APP="dist/huge-atlas.app"
DMG="dist/huge-atlas-${VERSION}-${ARCH}.dmg"

if [ -z "${SKIP_BUILD:-}" ]; then
    cmake -G Xcode -B "$BUILD" >/dev/null
    cmake --build "$BUILD" --config Release --target vsa_app
fi
BIN="$BUILD/Release/vsa_app"
[ -x "$BIN" ] || { echo "no $BIN: build first"; exit 1; }

rm -rf "$APP" "$DMG" dist/dmg-root
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources/data"
cp "$BIN" "$APP/Contents/MacOS/huge-atlas"
sed "s/@VERSION@/${VERSION}/g" packaging/Info.plist.in > "$APP/Contents/Info.plist"
cp packaging/AppIcon.icns "$APP/Contents/Resources/AppIcon.icns"
cp data/cube.obj data/bunny.ply "$APP/Contents/Resources/data/"
cp LICENSE "$APP/Contents/Resources/LICENSE.txt"
cp -R data/textured_hemisphere "$APP/Contents/Resources/data/"

codesign --force --deep --sign "${CODESIGN_IDENTITY:--}" "$APP"
codesign --verify --deep --strict "$APP"

mkdir -p dist/dmg-root
cp -R "$APP" dist/dmg-root/
ln -s /Applications dist/dmg-root/Applications
hdiutil create -volname "huge-atlas ${VERSION}" -srcfolder dist/dmg-root -ov -format UDZO "$DMG" >/dev/null
rm -rf dist/dmg-root
echo "ready: $DMG ($(du -h "$DMG" | cut -f1))"
