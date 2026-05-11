#!/usr/bin/env python3

import apriltag
import cv2
import numpy as np
import argparse

def generate_apriltag(family, tag_id, size_pixels):
    """
    Generate an AprilTag image using apriltag library data.

    Args:
        family: AprilTag family (e.g., 'tag36h11')
        tag_id: Tag ID number
        size_pixels: Size of the tag in pixels

    Returns:
        numpy array of the tag image
    """
    # Get the tag family
    fam = apriltag.family(family)

    if tag_id >= len(fam):
        raise ValueError(f"Tag ID {tag_id} not available in family {family}")

    # Get the tag code (6x6 for 36h11)
    code = fam[tag_id]

    # Convert to 8x8 with border
    # AprilTags have a white border
    tag = np.ones((8, 8), dtype=np.uint8) * 255  # White background
    tag[1:7, 1:7] = code.reshape(6, 6) * 255  # Black=0, White=255

    # Scale to size_pixels
    scale = size_pixels // 8
    tag_img = np.kron(tag, np.ones((scale, scale), dtype=np.uint8))

    return tag_img

def main():
    parser = argparse.ArgumentParser(description='Generate AprilTag images')
    parser.add_argument('--family', default='tag36h11', help='AprilTag family')
    parser.add_argument('--tag_id', type=int, default=0, help='Tag ID')
    parser.add_argument('--size', type=int, default=200, help='Tag size in pixels')
    parser.add_argument('--output', default='apriltag.png', help='Output file')

    args = parser.parse_args()

    try:
        tag_img = generate_apriltag(args.family, args.tag_id, args.size)
        cv2.imwrite(args.output, tag_img)
        print(f"Generated AprilTag {args.tag_id} ({args.family}) in {args.output}")
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure apriltag library is installed: pip install apriltag")
        print("Alternatively, use online generators like https://chev.me/apriltag.html")

if __name__ == '__main__':
    main()