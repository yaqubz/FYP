# 9 Jan Source: ChatGPT

import os

def generate_file_structure(directory, prefix=""):
    """
    Recursively generates a visual representation of the directory structure.
    :param directory: Root directory to start the representation.
    :param prefix: Prefix for visual formatting.
    """
    if not os.path.isdir(directory):
        print(f"{directory} is not a valid directory.")
        return

    entries = sorted(os.listdir(directory))
    last_index = len(entries) - 1

    for index, entry in enumerate(entries):
        full_path = os.path.join(directory, entry)
        is_last = index == last_index

        # Tree branch symbols
        connector = "└── " if is_last else "├── "
        print(f"{prefix}{connector}{entry}")

        # Recursive call for directories
        if os.path.isdir(full_path):
            new_prefix = f"{prefix}    " if is_last else f"{prefix}│   "
            generate_file_structure(full_path, new_prefix)

# Replace "workspace" with your desired directory path
generate_file_structure(".")    # works if running from main directory
