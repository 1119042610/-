import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
import win32gui
from PIL import Image, ImageTk, ImageGrab
import pyautogui
import time
import threading

# -------------------------------
# 区域坐标参数
img_width, img_height = 477, 358
LEFT_REGION = (436, 420, 436+img_width, 420+img_height)
RIGHT_REGION = (1008, 420, 1008+img_width, 420+img_height)
x_offset = 436 - 320    # 相对偏移坐标
y_offset = 420 - 30
img_offset = 1008 - 436     # 图像相对间距
# -------------------------------

def get_window_handle(window_title):
    handle = win32gui.FindWindow(None, window_title)
    if handle == 0:
        print("未找到窗口")
    return handle

def grab_img(region):
    img = ImageGrab.grab(region)
    img = np.array(img)
    if img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img

def try_ecc_align(img_ref_gray, img_mov_gray):
    """使用 ECC 对齐移动图像到参考图像"""
    ref = img_ref_gray.astype(np.float32) / 255.0
    mov = img_mov_gray.astype(np.float32) / 255.0
    sz = ref.shape
    warp_matrix = np.eye(2, 3, dtype=np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1e-6)
    try:
        _, warp_matrix = cv2.findTransformECC(ref, mov, warp_matrix, cv2.MOTION_EUCLIDEAN, criteria)
        aligned = cv2.warpAffine(img_mov_gray, warp_matrix, (sz[1], sz[0]),
                                 flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
        return aligned
    except cv2.error:
        return img_mov_gray

def find_differences(imgA, imgB, min_area=100, kernel_size=5):
    """检测两图差异"""
    grayA = cv2.cvtColor(imgA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imgB, cv2.COLOR_BGR2GRAY)
    grayB_aligned = try_ecc_align(grayA, grayB)

    if kernel_size == 1:
        blurA = grayA
        blurB = grayB_aligned
    else:
        blurA = cv2.GaussianBlur(grayA, (kernel_size,kernel_size), 0)
        blurB = cv2.GaussianBlur(grayB_aligned, (kernel_size,kernel_size), 0)
    diff = cv2.absdiff(blurA, blurB)

    mean, std = diff.mean(), diff.std()
    thresh_val = max(15, int(mean + 1.5 * std))
    _, thresh = cv2.threshold(diff, thresh_val, 255, cv2.THRESH_BINARY)

    kernel_close = np.ones((5,5), np.uint8)
    kernel_open = np.ones((3,3), np.uint8)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_open, iterations=1)
    merged = cv2.dilate(opened, np.ones((7, 7), np.uint8), iterations=1)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(merged, connectivity=8)
    boxes = []
    annotated = imgB.copy()
    for i in range(1, num_labels):
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue
        if w < 4 or h < 4:
            continue
        aspect = w / float(h)
        if aspect > 8 or aspect < 0.125:
            continue
        boxes.append((x, y, w, h))
        cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 0, 255), 2)
    return boxes, merged, annotated

def detect_once():
    """点击按钮时启动后台检测线程"""
    btn_detect.config(state="disabled", text="检测中...")
    # 临时取消置顶并隐藏窗口，防止被截进去
    root.attributes("-topmost", False)
    root.withdraw()
    # 延迟 0.5 秒后启动检测线程
    root.after(150, lambda: threading.Thread(target=run_detection_thread, daemon=True).start())

def run_detection_thread():
    """后台线程执行检测逻辑"""
    try:
        # 获取绝对坐标
        window_handle = get_window_handle("大家来找茬")
        coor = win32gui.GetWindowRect(window_handle)
        # print(coor)
        left_rigion = [coor[0] + x_offset, coor[1] + y_offset, coor[0] + x_offset + img_width, coor[1] + y_offset + img_height]
        right_rigion = [coor[0] + x_offset + img_offset, coor[1] + y_offset, coor[0] + x_offset + img_width + img_offset, coor[1] + y_offset + img_height]
        # print("left rigion:", left_rigion)
        # print("right rigion:", right_rigion)

        left_img = grab_img(left_rigion)
        right_img = grab_img(right_rigion)

        min_area = min_area_var.get()
        kernel_size = kernel_size_var.get()
        auto_click = auto_click_var.get()

        boxes, mask, annotated = find_differences(left_img, right_img,
                                                  min_area=min_area, kernel_size=kernel_size)

        # 合并 result 和 mask
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        highlight = cv2.bitwise_and(annotated, mask_color)
        combined = cv2.addWeighted(annotated, 0.7, highlight, 0.8, 0)

        # 将 combined 作为最终展示图
        disp = combined.copy()

        # disp = annotated.copy()
        click_points = []
        for idx, (x, y, w, h) in enumerate(boxes, start=1):
            cx, cy = x + w // 2, y + h // 2
            cv2.putText(disp, str(idx), (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1)
            screen_x = RIGHT_REGION[0] + cx
            screen_y = RIGHT_REGION[1] + cy
            click_points.append((screen_x, screen_y))

        # 通过主线程更新界面
        root.after(0, lambda: update_display(disp, mask))

    except Exception as e:
        print("检测时出错：", e)
    finally:
        # 检测结束后，重新显示窗口并恢复置顶
        root.after(0, lambda: [
            root.deiconify(),
            root.attributes("-topmost", True),
            btn_detect.config(state="normal", text="🔍 找茬")
        ])

        # 自动点击
        if auto_click and click_points:
            # time.sleep(0.4)
            for (sx, sy) in click_points:
                pyautogui.moveTo(sx, sy)
                time.sleep(0.1)
                # pyautogui.click(sx, sy, button='left')
                pyautogui.mouseDown(sx, sy, button="left")
                # time.sleep(0.3)
                pyautogui.mouseUp(sx, sy, button="left")

def update_display(result_img, mask_img):
    """线程安全地更新界面"""
    show_image("result", result_img)
    # show_image("mask", mask_img, is_gray=True)

def show_image(label_name, img, is_gray=False):
    """更新界面图像显示"""
    if is_gray:
        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    else:
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(img_rgb)
    img_pil = img_pil.resize((475, 356))
    img_tk = ImageTk.PhotoImage(img_pil)
    labels[label_name].configure(image=img_tk)
    labels[label_name].image = img_tk

# -------------------------------
# Tkinter GUI
# -------------------------------
root = tk.Tk()
root.title("帮我找茬")
root.geometry("550x600")
root.configure(bg="#e0f2ff")

root.attributes("-topmost", True)       # 窗口置顶

frame_top = tk.Frame(root, bg="#e0f2ff")
frame_top.pack(pady=5)

btn_detect = ttk.Button(frame_top, text="🔍 找茬", command=detect_once)
btn_detect.grid(row=0, column=0, padx=10)

btn_exit = ttk.Button(frame_top, text="❌ 退出", command=root.destroy)
btn_exit.grid(row=0, column=1, padx=10)

# 参数调节区域
frame_params = tk.LabelFrame(root, text="参数调节", bg="#e0f2ff", padx=10, pady=5)
frame_params.pack(pady=5)

tk.Label(frame_params, text="最小面积阈值:", bg="#e0f2ff").grid(row=1, column=0, sticky="e")
min_area_var = tk.IntVar(value=100)
tk.Scale(frame_params, from_=20, to=500, orient=tk.HORIZONTAL, variable=min_area_var, length=200).grid(row=1, column=1, padx=5)

tk.Label(frame_params, text="高斯核大小:", bg="#e0f2ff").grid(row=2, column=0, sticky="e")
kernel_size_var = tk.IntVar(value=3)
tk.Scale(frame_params, from_=1, to=15, orient=tk.HORIZONTAL, variable=kernel_size_var, length=200).grid(row=2, column=1, padx=5)

auto_click_var = tk.BooleanVar(value=True)
tk.Checkbutton(frame_params, text="启用自动点击", variable=auto_click_var, bg="#e0f2ff").grid(row=2, column=2, columnspan=2, pady=5)

# 图像显示
frame_images = tk.Frame(root, bg="#e0f2ff")
frame_images.pack(pady=10)

labels = {}
for idx, name in enumerate(["result"]):
    lbl = tk.Label(frame_images, bg="#b3e0ff", relief="sunken", width=475, height=356)
    lbl.grid(row=0, column=idx, padx=5)
    labels[name] = lbl

info = tk.Label(root, text="检测结果 / 差异掩码", bg="#e0f2ff", font=("Arial", 12))
info.pack(pady=5)

root.mainloop()
