import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)
print(cap.isOpened())
while(1):
    _, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    minLineLength = 1000
    maxLineGap = 40
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 60, minLineLength, maxLineGap)

    if lines is None:
        pass
    else:
        numlines=len(lines)
        print(numlines)
        for x in range(numlines):
            for x1, y1, x2, y2 in lines[x]:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)


    cv2.imshow('img', img)
    cv2.imshow('gray', gray)
    cv2.imshow('edges', edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()