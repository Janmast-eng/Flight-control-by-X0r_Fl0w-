import tkinter as tk

root = tk.Tk()
root.title("Tkinter测试")
root.geometry("300x200")

label = tk.Label(root, text="Tkinter工作正常!")
label.pack(pady=50)

button = tk.Button(root, text="关闭", command=root.destroy)
button.pack()

root.mainloop()

# x0r_fl0w