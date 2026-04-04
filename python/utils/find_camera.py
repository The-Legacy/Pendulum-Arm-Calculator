import cv2

print("Searching for available cameras...")
print("=" * 60)

found_camera = False

for i in range(10):  # Check indices 0-9
    cap = cv2.VideoCapture(i)
    
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"✓ Camera found at index: {i}")
            print(f"  Resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}")
            print(f"  FPS: {cap.get(cv2.CAP_PROP_FPS)}")
            found_camera = True
        cap.release()

if not found_camera:
    print("✗ No cameras found!")
    print("\nTroubleshooting:")
    print("  - Check if camera is connected")
    print("  - Run: ls /dev/video*")
    print("  - Check permission: ls -la /dev/video*")
else:
    print("=" * 60)
