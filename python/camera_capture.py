"""
Exactly what it sounds like - script to use CV to capture images
Saves to dataset/images directory

Space -> Save current frame
Q -> Quit
"""

import cv2
import os
from datetime import datetime
from pathlib import Path
from utils.config import (
    RAW_IMAGES_DIR,
    CAMERA_INDEX,
    CAMERA_FRAME_WIDTH,
    CAMERA_FRAME_HEIGHT,
    CAMERA_FPS,
    CAPTURE_QUALITY
)

def main():
    print("="*60)
    print("CAMERA CAPTURE TOOL")
    print("="*60)
    print(f"Saving images to {RAW_IMAGES_DIR}")
    print("\nControls:")
    print("  SPACE  - Save current frame")
    print("  Q      - Quit")
    print("="*60)

    # Open Webcam
    cap = cv2.VideoCapture(CAMERA_INDEX)

    # Set camera resolution and FPS
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    if not cap.isOpened():
        print("ERROR: Could not open webcam")
        return

    frame_count = 0
    frame_buffer = 0
    SKIP_FRAMES = 5 # Capture every 5th frame

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("ERROR: Failed to read frame")
                return

            # Increment buffer
            frame_buffer += 1

            # Display the current frame with overlay
            display_frame = frame.copy()
            cv2.putText(
                display_frame,
                f"Frames Saved: {frame_count}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )
            cv2.putText(
                display_frame,
                "Press SPACE to save | Q to Quit",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2
            )

            # Show frame
            cv2.imshow("Camera Feed - Press SPACE to capture", display_frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                if frame_buffer >= SKIP_FRAMES:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                    filename = f"{timestamp}.jpg"
                    filepath = RAW_IMAGES_DIR / filename
                    cv2.imwrite(str(filepath), frame, [cv2.IMWRITE_JPEG_QUALITY, CAPTURE_QUALITY])
                    frame_count += 1
                    frame_buffer = 0
                    print(f"Saved: {filename} (Total: {frame_count})")

            elif key == ord('q'):
                print("Quitting...")
                break

    except KeyboardInterrupt:
        print("\nInterupted by User")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"\n{'='*60}")
        print(f"✓ Capture complete!")
        print(f"✓ Total images saved: {frame_count}")
        print(f"✓ Location: {RAW_IMAGES_DIR}")
        print(f"{'='*60}")

if __name__ == "__main__":
    main()