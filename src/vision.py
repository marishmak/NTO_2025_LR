from typing import List, Tuple

import cv2
import numpy as np


def image_process(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 90, 90])
    upper_red_1 = np.array([200, 255, 255])
    lower_red_2 = np.array([0, 90, 90])
    upper_red_2 = np.array([200, 255, 255])
    lower_yellow_orange = np.array([0, 90, 90])
    upper_yellow_orange = np.array([200, 255, 255])

    mask_red_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
    mask_red_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
    mask_yellow_orange = cv2.inRange(
        hsv_image, lower_yellow_orange, upper_yellow_orange
    )

    fire_mask = cv2.bitwise_or(mask_red_1, mask_red_2)
    fire_mask = cv2.bitwise_or(fire_mask, mask_yellow_orange)

    processed_img = cv2.bitwise_and(img, img, mask=fire_mask)
    # showInRow([img, mask, processed_img], ['origin', 'mask', 'result'])

    return processed_img


def thresholding(processed_img):
    thresh_val = 90

    img_gray = cv2.cvtColor(processed_img, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (11, 11), 0)
    img_gray = cv2.medianBlur(img_gray, 9)

    ret, final_thresh = cv2.threshold(img_gray, thresh_val, 255, cv2.THRESH_BINARY)
    return final_thresh


def define_kernel(x, y):
    kernel = np.ones((x, y), np.uint8)
    return kernel


def morphological_ops(img_after_thresholding, kernel):
    opening = cv2.morphologyEx(
        img_after_thresholding, cv2.MORPH_OPEN, kernel, iterations=2
    )
    erosion = cv2.erode(opening, kernel, iterations=1)
    dilation = cv2.dilate(erosion, kernel, iterations=2)
    img_morphology = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)

    # removing small noizes
    connectivity = 1
    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        img_morphology, connectivity, cv2.CV_32S
    )

    labeled_image = np.zeros(
        (img_morphology.shape[0], img_morphology.shape[1], 3), dtype=np.uint8
    )
    labeled_image[labels == n_labels - 1, :] = [255, 255, 255]
    img_gray = cv2.cvtColor(labeled_image, cv2.COLOR_BGR2GRAY)

    return img_gray


def contour(fire_mask, image) -> Tuple[np.ndarray, List[Tuple[int, int, int, int]]]:
    contours, _ = cv2.findContours(
        fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    coordinates = []

    for contour in contours:
        pixel_area = cv2.contourArea(contour)
        if pixel_area > 500:
            x, y, w, h = cv2.boundingRect(contour)

            contour = contours[0]
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            print(cx, cy)

            print("Coordinates of fire:", (x + w // 2, y + h // 2))
            coordinates.append((x, y, w, h))

            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            text = "Fire"
            cv2.putText(
                image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
            )

    return image, coordinates


def main_fire_detection(img):
    processed_img = image_process(img)
    img_after_thresholding = thresholding(processed_img)
    kernel = define_kernel(5, 5)
    mask = morphological_ops(img_after_thresholding, kernel)

    final_img, coordinates = contour(mask, img)
    return final_img, coordinates


if __name__ == "__main__":
    image = cv2.imread("pic1.png")
    final_img, coordinates = main_fire_detection(image)
    print(coordinates)
