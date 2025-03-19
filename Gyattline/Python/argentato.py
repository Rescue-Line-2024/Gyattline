import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_silver = np.array([0, 0, 225])
    upper_silver = np.array([180, 50, 255])
    mask = cv2.inRange(hsv, lower_silver, upper_silver)
    blurred_mask = cv2.medianBlur(mask, 5)
    contours, _ = cv2.findContours(blurred_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    threshold = 1000

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > threshold:  
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Silver", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    mask_bgr = cv2.cvtColor(blurred_mask, cv2.COLOR_GRAY2BGR)
    combined_view = np.hstack((frame, mask_bgr))
    
    cv2.imshow("Silver", combined_view)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
