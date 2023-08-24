### Instruction: Tune colors in R, G, B order. Press q on the image window to proceed.
import cv2
import os
import yaml
import tkinter as tk
from tkinter import ttk

def filter_image(image, lower_threshold, upper_threshold):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv_image, lower_threshold, upper_threshold)

def save_values_to_yaml(filename, values):
    with open(filename, 'w') as file:
        yaml.dump(values, file)

def create_gui_for_color(cap, color, filename):
    root = tk.Tk()
    root.title(f"{color} Color Tuner UI")
    
    with open(filename, 'r') as file:
        data = yaml.safe_load(file)
    
    lower_values = data[f"{color}_lower_threshold"]
    upper_values = data[f"{color}_upper_threshold"]

    sliders = []
    for i, channel in enumerate(['H', 'S', 'V']):
        frame = ttk.Frame(root)
        frame.pack(pady=20)

        l = ttk.Scale(frame, from_=0, to=255, value=lower_values[i], orient=tk.HORIZONTAL)
        l.grid(row=0, column=0)
        u = ttk.Scale(frame, from_=0, to=255, value=upper_values[i], orient=tk.HORIZONTAL)
        u.grid(row=0, column=1)
        
        ttk.Label(frame, text=channel).grid(row=1, columnspan=2)
        
        sliders.append((l, u))

    def update_video():
        ret, frame = cap.read()
        if not ret:
            return
        
        lower = tuple(s[0].get() for s in sliders)
        upper = tuple(s[1].get() for s in sliders)
        mask = filter_image(frame, lower, upper)
        cv2.imshow(f"{color} Filtered Video", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyWindow(f"{color} Filtered Video")
            save_and_exit()

        root.after(10, update_video)

    def save_and_exit():
        lower = [int(s[0].get()) for s in sliders]
        upper = [int(s[1].get()) for s in sliders]

        data[f"{color}_lower_threshold"] = lower
        data[f"{color}_upper_threshold"] = upper
        save_values_to_yaml(filename, data)

        root.destroy()

    button = ttk.Button(root, text="Confirm", command=save_and_exit)
    button.pack(pady=20)

    root.after(10, update_video)
    root.mainloop()

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera is not accessible")
        return

    script_directory = os.path.dirname(os.path.abspath(__file__))
    yaml_file_path = os.path.join(script_directory, '..', 'config', 'HSV_filter_param.yaml')

    for color in ['R', 'G', 'B']:
        create_gui_for_color(cap, color, yaml_file_path)

    cap.release()

if __name__ == "__main__":
    main()
