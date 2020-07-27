#!/bin/sh

pwd=`pwd`
exe=${pwd}/release/imageTools
icon=${pwd}/image_tools_app/resouces/icon.jpg
newline="\n"

output="image_tools_app.desktop"

if [ -f "$output" ]; then
	rm "$output"
fi

echo "[Desktop Entry]" >> "$output"
echo "Type=Application" >> "$output"
echo "Exec=${exe}" >> "$output"
echo "Name=image_tools" >> "$output"
echo "GenericName=IMAGE_TOOLS" >> "$output"
echo "Icon=${icon}" >> "$output"
echo "Terminal=false" >> "$output"
echo "Categories=Development" >> "$output"

chmod a+x "$output"

if [ -f "~/Desktop/$output" ]; then
	rm "~/Desktop/$output"
fi

cp "$pwd/$output" ~/Desktop/$output

echo "generate desktop shortcut ok."
