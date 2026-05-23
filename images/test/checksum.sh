#!/usr/bin/env bash
file="$1"
[[ -z "$file" ]] && { echo "Usage: $0 <file>"; exit 1; }
python3 -c "
data = open('$file','rb').read()
total = sum(data) & 0xFFFFFFFF
print(f'Checksum: 0x{total:08X}')
"
echo "SHA1:     $(sha1sum "$file" | awk '{print $1}')"
