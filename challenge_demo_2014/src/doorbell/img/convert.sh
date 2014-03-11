#!/bin/bash



#favicon
inkscape \
	--export-width=32 \
	--export-png=temp.png \
	icon.svg
convert -verbose temp.png -alpha off -colors 256 ../favicon.ico
rm -v temp.png

#apple-touch-icon
inkscape \
	--export-png=../apple-touch-icon-precomposed.png \
	icon.svg

#tile
inkscape \
	--export-width=558 \
	--export-png=../tile.png \
	icon.svg

#tile-wide
inkscape \
	--export-width=558 --export-area=-76:0:228:152 --export-background=white \
	--export-png=../tile-wide.png \
	icon.svg

#startup
inkscape \
	--export-png=../startup.png \
	startup.svg

#optimise images
optipng -zc1-9 -zm8-9 -zs0-3 -f0-5 ../*.png
advpng -z4 ../*.png