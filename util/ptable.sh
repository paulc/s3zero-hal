#!/bin/zsh

f=$(mktemp)

trap 'rm -f "$f"' EXIT INT TERM QUIT HUP

echo $f


espflash read-flash 0x8000 0xc00 "$f" && espflash partition-table --to-csv "$f"

