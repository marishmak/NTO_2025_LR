from typing import List, Tuple

import cv2
import numpy as np


def crop_center(image, percent):
    """Crops image from center by specified percentage.

    Args:
        image: Input image array
        percent: Percentage of original size to keep

    Returns:
        Cropped image array
    """
    h, w = image.shape[:2]
    crop_width = int(w * percent / 100)
    crop_height = int(h * percent / 100)

    start_x = (w - crop_width) // 2
    start_y = (h - crop_height) // 2

    cropped_image = image[
        start_y : start_y + crop_height, start_x : start_x + crop_width
    ]
    return cropped_image


def image_process(img):
    """Processes image to detect fire-like colors using HSV thresholding.

    Args:
        img: Input BGR image array

    Returns:
        Processed image with only fire-colored pixels
    """
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 90, 90])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 90, 90])
    upper_red_2 = np.array([180, 255, 255])
    lower_orange = np.array([10, 90, 90])
    upper_orange = np.array([25, 255, 255])
    lower_yellow = np.array([25, 90, 90])
    upper_yellow = np.array([35, 255, 255])

    mask_red_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
    mask_red_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
    mask_orange = cv2.inRange(hsv_image, lower_orange, upper_orange)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    fire_mask = cv2.bitwise_or(mask_red_1, mask_red_2)
    fire_mask = cv2.bitwise_or(fire_mask, mask_orange)
    fire_mask = cv2.bitwise_or(fire_mask, mask_yellow)

    processed_img = cv2.bitwise_and(img, img, mask=fire_mask)

    return processed_img


def thresholding(processed_img):
    """Applies thresholding to processed image after Gaussian and median blur.

    Args:
        processed_img: Color-filtered image array

    Returns:
        Binary threshold image
    """
    img_gray = cv2.cvtColor(processed_img, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (11, 11), 0)
    img_gray = cv2.medianBlur(img_gray, 9)

    ret, final_thresh = cv2.threshold(img_gray, 90, 255, cv2.THRESH_BINARY)
    return final_thresh


def define_kernel(x, y):
    """Creates kernel for morphological operations.

    Args:
        x: Kernel width
        y: Kernel height

    Returns:
        numpy array kernel of ones
    """
    kernel = np.ones((x, y), np.uint8)
    return kernel


def morphological_ops(img_after_thresholding, kernel):
    """Applies morphological operations to clean up thresholded image.

    Args:
        img_after_thresholding: Binary image array
        kernel: Morphological operation kernel

    Returns:
        Processed binary image after opening, erosion, dilation and closing
    """
    opening = cv2.morphologyEx(
        img_after_thresholding, cv2.MORPH_OPEN, kernel, iterations=2
    )
    erosion = cv2.erode(opening, kernel, iterations=1)
    dilation = cv2.dilate(erosion, kernel, iterations=2)
    img_morphology = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)

    return img_morphology


def contour(fire_mask, image) -> Tuple[np.ndarray, List[Tuple[int, int, int, int]]]:
    """Detects and draws fire contours on image.

    Args:
        fire_mask: Binary image with fire regions
        image: Original image to draw on

    Returns:
        Tuple of:
            - Image with drawn contours and labels
            - List of contour coordinates (x, y, width, height)
    """
    contours, _ = cv2.findContours(
        fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    coordinates = []

    crop_percent_x = 0.25
    crop_percent_y = 0.25

    for contour in contours:
        pixel_area = cv2.contourArea(contour)
        if pixel_area > 500:
            x, y, w, h = cv2.boundingRect(contour)

            center_x = x + w // 2
            center_y = y + h // 2

            if (
                center_x >= crop_percent_x * image.shape[1]
                and center_x <= (1 - crop_percent_x) * image.shape[1]
                and center_y >= crop_percent_y * image.shape[0]
                and center_y <= (1 - crop_percent_y) * image.shape[0]
            ):
                coordinates.append((x, y, w, h))

                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)
                text = "Fire"
                cv2.putText(
                    image,
                    text,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

    return image, coordinates


def main_fire_detection(img):
    """Main fire detection pipeline.

    Args:
        img: Input BGR image array

    Returns:
        Tuple of:
            - Image with detected fires marked
            - List of fire coordinates
            - Binary mask of fire regions
    """
    # cropped_img = crop_center(img, crop_percent)
    processed_img = image_process(img)
    img_after_thresholding = thresholding(processed_img)
    kernel = define_kernel(5, 5)
    mask = morphological_ops(img_after_thresholding, kernel)

    final_img, coordinates = contour(mask, img.copy())

    return final_img, coordinates, mask


if __name__ == "__main__":
    name = "pic1.png"
    image = cv2.imread(name)
    final_img, coordinates, mask = main_fire_detection(image)
