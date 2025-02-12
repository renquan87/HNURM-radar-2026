import cv2

map_source = cv2.imread("map.png")
map_show = cv2.resize(map_source, (1920, 1080))
map_source = cv2.resize(map_source, (2800, 1500))

# 获取鼠标坐标
def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        
        xx = x
        yy = 1080 - y
        # print('x: %d, y: %d' % (xx, yy))
        print('x: %f, y: %f' % (xx/1920*2800, yy/1080*1500))
        # cv2.circle(map_show, (x, y), 10, (0, 0, 255), -1)

cv2.namedWindow("map", cv2.WINDOW_FREERATIO)
cv2.resizeWindow("map", 1920, 1080)
cv2.setMouseCallback("map", on_EVENT_LBUTTONDOWN)


cv2.imshow("map", map_show)
cv2.waitKey(0)