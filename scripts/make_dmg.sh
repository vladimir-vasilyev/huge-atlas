#!/usr/bin/env bash
# Builds a Release version of the program, wraps it into huge-atlas.app and packs a disk image for users who do not build
# anything: dist/huge-atlas-<version>-<arch>.dmg (open it, drag the app to Applications).
#   ./scripts/make_dmg.sh            build + app + dmg
#   SKIP_BUILD=1 ./scripts/make_dmg.sh   use the existing build-xcode/Release/vsa_app
# Without a Developer ID the app is signed ad hoc: see the README for the first start (macOS blocks it once).
# With one, no warning at all:
#   CODESIGN_IDENTITY="Developer ID Application: Your Name (TEAMID)"  sign with the hardened runtime and a timestamp
#   NOTARY_PROFILE=huge-atlas   notarize the app and the disk image and staple the tickets; the profile is made once with
#     xcrun notarytool store-credentials huge-atlas --apple-id <id> --team-id <TEAMID> --password <app-specific password>
# e.g.  CODESIGN_IDENTITY="Developer ID Application: ..." NOTARY_PROFILE=huge-atlas ./scripts/make_dmg.sh
#   FLAVOR=native ./scripts/make_dmg.sh   the native (AppKit) window instead of the ImGui one: dist/huge-atlas-native.app and
#                                         dist/huge-atlas-native-<version>-<arch>.dmg
set -euo pipefail
cd "$(dirname "$0")/.."

VERSION="$(tr -d '[:space:]' < VERSION)"
ARCH="$(uname -m)"
BUILD=build-xcode
if [ "${FLAVOR:-imgui}" = "native" ]; then
    TARGET=huge_atlas_native; NAME=huge-atlas-native
else
    TARGET=vsa_app; NAME=huge-atlas
fi
APP="dist/${NAME}.app"
DMG="dist/${NAME}-${VERSION}-${ARCH}.dmg"

if [ -z "${SKIP_BUILD:-}" ]; then
    cmake -G Xcode -B "$BUILD" >/dev/null
    cmake --build "$BUILD" --config Release --target "$TARGET"
fi
BIN="$BUILD/Release/$TARGET"
[ -x "$BIN" ] || { echo "no $BIN: build first"; exit 1; }

rm -rf "$APP" "$DMG" dist/dmg-root
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources/data"
cp "$BIN" "$APP/Contents/MacOS/huge-atlas"
sed "s/@VERSION@/${VERSION}/g" packaging/Info.plist.in > "$APP/Contents/Info.plist"
if [ "$TARGET" = huge_atlas_native ]; then   # another bundle id and name: it can live next to the ImGui version
    sed -i '' -e 's/io.github.vladimir-vasilyev.huge-atlas/io.github.vladimir-vasilyev.huge-atlas-native/' \
        -e 's|<key>CFBundleName</key><string>huge-atlas</string>|<key>CFBundleName</key><string>huge-atlas native</string>|' \
        -e 's|<key>CFBundleDisplayName</key><string>huge-atlas</string>|<key>CFBundleDisplayName</key><string>huge-atlas native</string>|' \
        "$APP/Contents/Info.plist"
fi
cp packaging/AppIcon.icns "$APP/Contents/Resources/AppIcon.icns"
cp data/cube.obj data/bunny.ply "$APP/Contents/Resources/data/"
cp LICENSE "$APP/Contents/Resources/LICENSE.txt"
if [ "$TARGET" = vsa_app ]; then
    cp packaging/imgui.ini "$APP/Contents/Resources/imgui.ini"   # the layout of the windows for the first start
fi
cp -R data/textured_hemisphere "$APP/Contents/Resources/data/"

IDENTITY="${CODESIGN_IDENTITY:--}"
if [ "$IDENTITY" = "-" ]; then
    codesign --force --deep --sign - "$APP"
else
    codesign --force --deep --options runtime --timestamp --sign "$IDENTITY" "$APP"
fi
codesign --verify --deep --strict "$APP"

notarize() {   # notarize <file>: submit and wait; stops the script when Apple rejects it
    xcrun notarytool submit "$1" --keychain-profile "$NOTARY_PROFILE" --wait
}
if [ -n "${NOTARY_PROFILE:-}" ] && [ "$IDENTITY" != "-" ]; then
    ditto -c -k --keepParent "$APP" dist/app-for-notary.zip
    notarize dist/app-for-notary.zip
    rm -f dist/app-for-notary.zip
    xcrun stapler staple "$APP"   # the ticket travels with the app when it is copied out of the disk image
fi

mkdir -p dist/dmg-root
cp -R "$APP" dist/dmg-root/
ln -s /Applications dist/dmg-root/Applications
hdiutil create -volname "${NAME} ${VERSION}" -srcfolder dist/dmg-root -ov -format UDZO "$DMG" >/dev/null
rm -rf dist/dmg-root
if [ "$IDENTITY" != "-" ]; then
    codesign --force --timestamp --sign "$IDENTITY" "$DMG"
    if [ -n "${NOTARY_PROFILE:-}" ]; then
        notarize "$DMG"
        xcrun stapler staple "$DMG"
        spctl -a -vv -t open --context context:primary-signature "$DMG"
    fi
fi
echo "ready: $DMG ($(du -h "$DMG" | cut -f1))"
