import cv2
import numpy as np

# Load images
target = cv2.imread("images/test1sim.jpg")
scene = cv2.imread("images/test2sim.jpg")

# Create ORB detector
orb = cv2.ORB_create(nfeatures=1000)

# Detect keypoints + descriptors
target_kp, target_desc = orb.detectAndCompute(target, None)
scene_kp, scene_desc = orb.detectAndCompute(scene, None)

# Use KNN matching instead of simple crossCheck matching
matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

matches = matcher.knnMatch(
    target_desc,
    scene_desc,
    k=2
)

# Lowe's ratio test:
# keep a match only if the best match is clearly better than the second-best
good_matches = []

for m, n in matches:
    if m.distance < 0.75 * n.distance:
        good_matches.append(m)

print(f"Good matches: {len(good_matches)}")

# Need enough matches to estimate object position
if len(good_matches) >= 10:

    # Coordinates of matching points in target image
    target_points = np.float32([
        target_kp[m.queryIdx].pt
        for m in good_matches
    ]).reshape(-1, 1, 2)

    # Corresponding coordinates in scene
    scene_points = np.float32([
        scene_kp[m.trainIdx].pt
        for m in good_matches
    ]).reshape(-1, 1, 2)

    # Find transformation from target image -> scene image
    homography, mask = cv2.findHomography(
        target_points,
        scene_points,
        cv2.RANSAC,
        5.0
    )

    # Reference image dimensions
    h, w = target.shape[:2]

    # Corners of target image
    target_corners = np.float32([
        [0, 0],
        [w, 0],
        [w, h],
        [0, h]
    ]).reshape(-1, 1, 2)

    # Transform those corners into the scene
    scene_corners = cv2.perspectiveTransform(
        target_corners,
        homography
    )

    # Draw polygon around detected object
    result = scene.copy()

    cv2.polylines(
        result,
        [np.int32(scene_corners)],
        True,
        (255, 255, 255),
        3
    )

    cv2.imwrite("images/detected.jpg", result)

    print("Object detected!")
    print("Saved to images/detected.jpg")

    inliers = mask.ravel().sum()

    print(f"Good ORB matches: {len(good_matches)}")
    print(f"RANSAC inliers: {inliers}")
    print(f"Inlier ratio: {inliers / len(good_matches):.2f}")

else:
    print("Not enough good matches.")

