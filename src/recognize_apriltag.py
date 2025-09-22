from maix import camera, display, image
from dredge.apriltagmap import generate_map_dict

# 生成坐标映射字典
coord_map = generate_map_dict(rows=15, cols=20)

cam = camera.Camera()
cam.open(width=640, height=480)

screen = display.Display( width=640, height=480)
screen.open()

roi = [160, 120, 320, 240]
families = image.ApriltagFamilies.TAG36H11
fx = float(-1)
fy = float(-1)
cx = 320
cy = 240

while True:
    img = cam.read()
    apriltags = img.find_apriltags(roi, families, fx, fy, cx, cy)
    
    for a in apriltags:
        tag_id = a.id()
        
        # 使用映射字典查询坐标
        if tag_id in coord_map:
            global_coord = coord_map[tag_id]
            print(f"检测到Tag ID: {tag_id}, 全局坐标: {global_coord}")
            # 在画面上显示坐标信息
            img.draw_string(a.x() + a.w() + 5, a.y() + 50, f"Coord: {global_coord}", image.COLOR_GREEN)
        else:
            print(f"警告: 检测到未知ID的Tag: {tag_id}")
        
        # 绘制Tag边界和信息
        corners = a.corners()
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1], image.COLOR_RED, 2)
        
        rect = a.rect()
        img.draw_rect(rect[0], rect[1], rect[2], rect[3], image.COLOR_BLUE, 2)
        img.draw_string(rect[0] + 5, rect[1] + 5, "ID: " + str(tag_id), image.COLOR_RED)
    
    img.draw_rect(roi[0], roi[1], roi[2], roi[3], image.COLOR_GREEN)
    screen.show(img)