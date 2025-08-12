import os
import re
import sys

def get_header_map(header_files_list):
    header_map = {}
    with open(header_files_list, 'r') as f:
        for line in f:
            full_path = line.strip()
            if not full_path:
                continue
            filename = os.path.basename(full_path)
            if filename not in header_map:
                header_map[filename] = []
            header_map[filename].append(full_path)
    return header_map

def process_files(files_to_process_list, header_map):
    with open(files_to_process_list, 'r') as f:
        for line in f:
            filepath = line.strip()
            if not filepath:
                continue
            process_file(filepath, header_map)

def process_file(filepath, header_map):
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
    except IOError:
        print(f"Could not read file: {filepath}")
        return

    new_lines = []
    changed = False
    for line in lines:
        match = re.match(r'\s*#\s*include\s*"([^"]+)"', line)
        if match:
            include_path = match.group(1)

            # Skip if it's already a relative path with subdirectories
            if '/' in include_path:
                new_lines.append(line)
                continue

            filename = os.path.basename(include_path)

            if filename == 'endian.h' or filename == 'stm32f_hal_conf.hxx':
                new_lines.append(line)
                continue

            if filename in header_map and len(header_map[filename]) == 1:
                new_include_path = header_map[filename][0]

                if 'include/freertos/' in new_include_path:
                    new_lines.append(line)
                    continue

                if new_include_path.startswith('src/'):
                    new_include_path = new_include_path[len('src/'):]
                elif new_include_path.startswith('include/'):
                    new_include_path = new_include_path[len('include/'):]

                new_line = f'#include "{new_include_path}"\n'
                if line.strip() != new_line.strip():
                    new_lines.append(new_line)
                    changed = True
                    print(f'Changed "{include_path}" to "{new_include_path}" in {filepath}')
                else:
                    new_lines.append(line)
            else:
                if filename in header_map:
                    print(f"Skipping ambiguous include '{include_path}' in {filepath}. Candidates: {header_map[filename]}")
                new_lines.append(line)
        else:
            new_lines.append(line)

    if changed:
        try:
            with open(filepath, 'w') as f:
                f.writelines(new_lines)
        except IOError:
            print(f"Could not write to file: {filepath}")

def main():
    # Chunks are passed as arguments now.
    if len(sys.argv) < 2:
        print("Usage: python fix_includes.py <file_list_chunk1> <file_list_chunk2> ...")
        return

    header_map = get_header_map('/tmp/header_files.txt')
    for file_list in sys.argv[1:]:
        process_files(file_list, header_map)


if __name__ == "__main__":
    main()
