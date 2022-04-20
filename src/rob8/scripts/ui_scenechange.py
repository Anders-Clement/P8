#! /bin/python3

import tkinter as tk
from tkinter import ttk
from tkinter.messagebox import showinfo
import random

import rospy
from std_msgs.msg import String, Int16, Int32
  
class App(tk.Tk):
    old_scene_objects = ""
    def __init__(self):
        super().__init__()
        
        self.geometry("200x250")
        self.lbl2 = tk.Label(self,text = "Clients connected: 0")
        self.lbl = tk.Label(self,text = "Double click to change scene")  
        self.listbox = tk.Listbox(self) 
        self.listbox.bind('<Double-1>', self.list_click)

        self.lbl2.pack()
        self.lbl.pack()  
        self.listbox.pack()  

        self.namesub = rospy.Subscriber("/scene/names", String, self.incoming_scenes)
        self.connectionsub = rospy.Subscriber("/client_count", Int32, self.incoming_clinets)
        self.changepub = rospy.Publisher("/scene/change", Int16, queue_size=16)

    def incoming_scenes(self, msg):
        incoming_data = msg.data
        
        if incoming_data == self.old_scene_objects:
            return
        self.old_scene_objects = incoming_data

        array = incoming_data.split(',')
        self.listbox.delete(0, tk.END)
        for text in array:
            name, id = text.split(':')
            self.listbox.insert(1, id + " " + name)

    def incoming_clinets(self, msg):
        self.lbl2.config(text = "Clients connected: " + str(msg.data))

    def list_click(self, event):
        selection = event.widget.curselection()
        if selection:
            index = selection[0]
            data = event.widget.get(index)
            self.lbl.config(text = "Changing to " + data)

            new_msg = Int16()
            new_msg.data = int(data[0])
            self.changepub.publish(new_msg)
            self.after(1000, self.reset_text)

    def reset_text(self):
        self.lbl.config(text = "Double click to change scene")
  
if __name__ == "__main__":
    my_app = App()
    rospy.init_node('talker', anonymous=True)
    my_app.mainloop()
    
