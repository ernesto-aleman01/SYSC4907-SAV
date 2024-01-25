import numpy as np
import cv2
import PIL.Image as Images




# Start a while loop
while (1):

    # Reading the video from the
    # webcam in image frames
    # C:\Users\marcl\PycharmProjects\TrafficLightState\.venv\TL_AllStates.jpg
    # C:\Users\marcl\PycharmProjects\TrafficLightState\.venv\trafficLightColours.jpg
    # C:\Users\marcl\PycharmProjects\TrafficLightState\.venv\TL_green.png
    imageF = cv2.imread(r"C:\Users\marcl\PycharmProjects\TrafficLightState\.venv\TL_AllStates.jpg")
    imageFrame = cv2.resize(imageF, (450, 338))
   # imageFrame = np.asarray(img)
    # Convert the imageFrame in
    # BGR(RGB color space) to
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)


    # Set range for green color and
    # define mask
    green_lower = np.array([40, 70, 80], np.uint8)
    green_upper = np.array([70, 255,255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)



    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernel = np.ones((5, 5), "uint8")



    # For green color
    green_mask = cv2.dilate(green_mask, kernel)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=green_mask)


            # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 0), 2)

            cv2.putText(imageFrame, "Green", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0))


            # Program Termination
    cv2.imshow("Traffic Light State", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break