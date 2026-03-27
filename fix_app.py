#!/usr/bin/env python3
"""Fix spacing issues in app.py"""

import re

# Read the file
with open('app.py', 'r') as f:
    content = f. read()

# Fix all spacing issues
fixes = [
    (r'(\d)\. (\d)', r'\1.\2'),      # Fix "1. 0" -> "1. 0"
    (r'\.  (\w)', r'.\1'),             # Fix ". method" -> ". method"
    (r'(\w) \. ', r'\1. '),             # Fix "obj ." -> "obj."
    (r"'\. '", "'.'"),                # Fix '.  ' -> '.'
    (r'-999\. 0', '-999.0'),          # Fix -999. 0
    (r'100\. 0', '100. 0'),            # Fix 100.  0
    (r'0\. 0', '0. 0'),                # Fix 0.  0
    (r'0\. 1', '0.1'),                # Fix 0. 1
    (r'0\. 5', '0.5'),                # Fix 0. 5
    (r'0\. 7', '0.7'),                # Fix 0. 7
    (r'4\. 0', '4. 0'),                # Fix 4.  0
    (r'1\. 0', '1. 0'),                # Fix 1.  0
    (r'1\. 7', '1.7'),                # Fix 1. 7
    (r'2\. 0', '2. 0'),                # Fix 2.  0
    (r'3\. 0', '3. 0'),                # Fix 3.  0
    (r'5\. 0', '5. 0'),                # Fix 5.  0
]

for pattern, replacement in fixes:
    content = re.sub(pattern, replacement, content)

# Write the fixed file
with open('app.py', 'w') as f:
    f.write(content)

print(" Fixed all spacing issues in app.py")