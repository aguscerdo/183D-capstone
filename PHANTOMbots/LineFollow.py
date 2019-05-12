import cv2
import numpy as np
import time
import cv2.aruco as aruco
import settings

def Follow():
    cap = cv2.VideoCapture(0)
    print(cap.isOpened())

    _, img1 = cap.read()
    gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    minLineLength = 100
    maxLineGap = 10
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
    total = len(lines)
    for x in range(total):
        for rho, theta in lines[x]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))

            cv2.line(img1, (x1, y1), (x2, y2), (0, 0, 255), 4)

    while (1):
        _, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        minLineLength = 100
        maxLineGap = 10

        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters_create()

        if lines is None:
            pass
        else:
            total = len(lines)
            for x in range(total):
                for rho, theta in lines[x]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 4)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        print(corners)

        font = cv2.FONT_HERSHEY_SIMPLEX

        gray = aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 255))
        cv2.imshow('img', img)
        cv2.imshow('gray', gray)
        cv2.imshow('edges', edges)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    
