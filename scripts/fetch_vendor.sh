#!/bin/sh
# Download the frontend vendor assets into app/static/vendor.
# Used by the Dockerfile and for local development (the folder is git-ignored).
set -e
DEST="${1:-app/static/vendor}"
mkdir -p "$DEST/js" "$DEST/css" "$DEST/fonts"
curl -fsSL -o "$DEST/js/vue.min.js" "https://cdn.jsdelivr.net/npm/vue@2.7.16/dist/vue.min.js"
curl -fsSL -o "$DEST/js/vuetify.min.js" "https://cdn.jsdelivr.net/npm/vuetify@2.7.2/dist/vuetify.min.js"
curl -fsSL -o "$DEST/css/vuetify.min.css" "https://cdn.jsdelivr.net/npm/vuetify@2.7.2/dist/vuetify.min.css"
curl -fsSL -o "$DEST/css/materialdesignicons.min.css" "https://cdn.jsdelivr.net/npm/@mdi/font@7.4.47/css/materialdesignicons.min.css"
for f in woff2 woff ttf eot; do
  curl -fsSL -o "$DEST/fonts/materialdesignicons-webfont.$f" "https://cdn.jsdelivr.net/npm/@mdi/font@7.4.47/fonts/materialdesignicons-webfont.$f"
done
for w in 300 400 500 700; do
  curl -fsSL -o "$DEST/fonts/roboto-$w.woff2" "https://cdn.jsdelivr.net/npm/@fontsource/roboto@5.1.0/files/roboto-latin-$w-normal.woff2"
done
: > "$DEST/css/roboto.css"
for w in 300 400 500 700; do
  printf '@font-face{font-family:"Roboto";font-weight:%s;font-style:normal;font-display:swap;src:url("../fonts/roboto-%s.woff2") format("woff2")}\n' "$w" "$w" >> "$DEST/css/roboto.css"
done
