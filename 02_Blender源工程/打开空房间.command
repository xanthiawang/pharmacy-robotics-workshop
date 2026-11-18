#!/bin/zsh
TASK_DIR="${0:A:h}"
exec "/Applications/Blender.app/Contents/MacOS/Blender" "$TASK_DIR/pharmacy_empty.blend" --python "$TASK_DIR/tools/workshop_editor.py"
