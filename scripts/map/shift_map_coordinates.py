#!/usr/bin/env python3
"""
Shift local_x and local_y coordinates in Lanelet2 OSM map.
Subtracts offset from each node's local coordinates.
"""

import re
import sys

def shift_coordinates(input_file, output_file, x_offset, y_offset):
    """
    Shift local_x and local_y values in OSM map.

    Args:
        input_file: Path to input OSM file
        output_file: Path to output OSM file
        x_offset: Value to subtract from local_x
        y_offset: Value to subtract from local_y
    """

    # Regex patterns to match local_x and local_y tags
    local_x_pattern = re.compile(r'(<tag k="local_x" v=")([0-9.-]+)("\s*/>)')
    local_y_pattern = re.compile(r'(<tag k="local_y" v=")([0-9.-]+)("\s*/>)')

    nodes_processed = 0
    x_values_modified = 0
    y_values_modified = 0

    print(f"Reading from: {input_file}")
    print(f"Writing to: {output_file}")
    print(f"X offset: -{x_offset}")
    print(f"Y offset: -{y_offset}")
    print()

    try:
        with open(input_file, 'r', encoding='utf-8') as infile:
            with open(output_file, 'w', encoding='utf-8') as outfile:
                in_node = False

                for line_num, line in enumerate(infile, 1):
                    # Track if we're in a node definition
                    if '<node ' in line and 'id=' in line:
                        in_node = True
                        nodes_processed += 1

                        # Show progress every 1000 nodes
                        if nodes_processed % 1000 == 0:
                            print(f"Processed {nodes_processed} nodes...")

                    if '</node>' in line:
                        in_node = False

                    # Modify local_x values
                    if in_node and 'local_x' in line:
                        match = local_x_pattern.search(line)
                        if match:
                            old_value = float(match.group(2))
                            new_value = old_value - x_offset
                            line = local_x_pattern.sub(r'\g<1>' + f'{new_value:.10f}' + r'\g<3>', line)
                            x_values_modified += 1

                            # Show first modification as example
                            if x_values_modified == 1:
                                print(f"Example X transformation:")
                                print(f"  {old_value:.10f} - {x_offset} = {new_value:.10f}")

                    # Modify local_y values
                    if in_node and 'local_y' in line:
                        match = local_y_pattern.search(line)
                        if match:
                            old_value = float(match.group(2))
                            new_value = old_value - y_offset
                            line = local_y_pattern.sub(r'\g<1>' + f'{new_value:.10f}' + r'\g<3>', line)
                            y_values_modified += 1

                            # Show first modification as example
                            if y_values_modified == 1:
                                print(f"Example Y transformation:")
                                print(f"  {old_value:.10f} - {y_offset} = {new_value:.10f}")
                                print()

                    # Write the (possibly modified) line
                    outfile.write(line)

        print(f"\n✅ Success!")
        print(f"Nodes processed: {nodes_processed}")
        print(f"local_x values modified: {x_values_modified}")
        print(f"local_y values modified: {y_values_modified}")
        print(f"\nOutput saved to: {output_file}")

    except FileNotFoundError:
        print(f"❌ Error: Input file not found: {input_file}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    # Configuration
    input_file = "/home/jetson/AutoSDV/data/COSS-map-planning/lanelet2_map_orig_v4_shifted.osm"
    output_file = "/home/jetson/AutoSDV/data/COSS-map-planning/lanelet2_map_orig_v4.osm"

    x_offset = - 304731.3793
    y_offset = - 2768113.4495

    print("=" * 60)
    print("Lanelet2 Map Coordinate Shifter")
    print("=" * 60)

    shift_coordinates(input_file, output_file, x_offset, y_offset)
