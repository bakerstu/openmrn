#!/usr/bin/gawk -f

# This file is used to preprocess the SHA test vectors file.
# Usage: $0 < SHA256LongMsg.rsp

BEGIN {
  idx = 0;
}

/Msg =/ {
  printf("if (index == %d)\n{\n    Payload = hex2str(\"%s\");\n", idx, $3);
  idx = idx + 1;
}

/MD =/ {
  printf("    Hash = hex2str(\"%s\");\n}\n", $3);
}

// {}

