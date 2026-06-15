import cv2 as cv
import numpy as np

# ====== CONFIGURE THESE ======
roi_x_min = 0
roi_x_max = 3320
roi_y_min = 850
roi_y_max = 1800

lower_green = np.array([40, 80, 50])
upper_green = np.array([90, 255, 255])

lower_red1 = np.array([0, 80, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 80, 50])
upper_red2 = np.array([180, 255, 255])

black_hsv_min = np.array([0, 0, 10])
black_hsv_max = np.array([180, 255, 50])

min_tray_size = 10000
min_silver_ball_size = 20
min_black_ball_size = 40000
# =============================

image = cv.imread("30percent.jpg")
image = image[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
output = image.copy()


def draw_detections(mask, min_area, label):
    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv.contourArea(contour)

        if area > min_area:
            x, y, w, h = cv.boundingRect(contour)

            # Red box
            cv.rectangle(output, (x, y), (x + w, y + h), (0, 0, 255), 2)

            cv.putText(
                output,
                label,
                (x, y - 5),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )


# Green tray
green = cv.cvtColor(image, cv.COLOR_BGR2HSV)
green = cv.GaussianBlur(green, (5, 5), 0)
green = cv.inRange(green, lower_green, upper_green)
draw_detections(green, min_tray_size, "GREEN")

# Red tray
red = cv.cvtColor(image, cv.COLOR_BGR2HSV)
red = cv.GaussianBlur(red, (5, 5), 0)
red1 = cv.inRange(red, lower_red1, upper_red1)
red2 = cv.inRange(red, lower_red2, upper_red2)
red = cv.bitwise_or(red1, red2)
draw_detections(red, min_tray_size, "RED")

# Silver victim
# silver = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
# silver = cv.inRange(silver, 245, 255)
# draw_detections(silver, min_silver_ball_size, "SILVER")

# Silver victim
silver = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
silver = cv.inRange(silver, 245, 255)

# Merge bright patches separated by small gaps
kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (100, 100))
silver = cv.morphologyEx(silver, cv.MORPH_CLOSE, kernel)
draw_detections(silver, min_silver_ball_size, "SILVER")
silver_raw = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
silver_raw = cv.inRange(silver_raw, 245, 255)

# Black victim
black = cv.GaussianBlur(image, (5, 5), 0)
black = cv.cvtColor(black, cv.COLOR_BGR2HSV)
black = cv.inRange(black, black_hsv_min, black_hsv_max)
draw_detections(black, min_black_ball_size, "BLACK")

cv.imshow("Detections", output)
cv.imshow("Black", black)
cv.waitKey(0)
cv.destroyAllWindows()